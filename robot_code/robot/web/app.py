#!/usr/bin/env python3
"""Robot dashboard for runtime control and camera workflows."""

from __future__ import annotations

import argparse
import ipaddress
import json
import subprocess
import threading
import time
from dataclasses import asdict
from pathlib import Path
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.request import urlopen

try:
    from flask import Flask, jsonify, redirect, render_template, request, url_for
except ImportError as exc:  # pragma: no cover - depends on target Pi environment
    raise RuntimeError("Flask is required for robot/web/app.py") from exc

from process_control import (
    CalibrationPoseConfig,
    CalibrationStore,
    DEFAULT_DASHBOARD_HOST,
    DEFAULT_DASHBOARD_PORT,
    DEFAULT_RUNTIME_BAUDRATE,
    DEFAULT_RUNTIME_PORT,
    CameraLaunchConfig,
    ManagedProcessManager,
    MOTION_ENGINE_ROOT,
    OverrideStore,
    RuntimeLaunchConfig,
    VISION_ROOT,
    XboxControllerLaunchConfig,
    build_camera_command,
    build_runtime_command,
    build_xbox_command,
    read_xbox_status,
    send_calibration_pose,
)
from dance_recording import DanceRecordingManager
from dance_storage import (
    DanceError,
    DancePresetNotFound,
    DanceStorage,
    DanceValidationError,
    TagConflict,
)
from dance_encoder import (
    DanceEncoder,
    EncoderBusy,
    EncoderError,
    EncoderNotActive,
)
from automated_runner import (
    AutomatedRunner,
    CalibrationIncomplete,
    PresetNotLoaded,
    SerialUnavailable,
    TAG_STOP,
)


APP_ROOT = Path(__file__).resolve().parent
STATIC_ROOT = APP_ROOT / "static"
ASSET_VERSION = str(
    int(
        max(
            (STATIC_ROOT / "robot_ui.css").stat().st_mtime,
            (STATIC_ROOT / "robot_ui.js").stat().st_mtime,
        )
    )
)


def _make_manager_bundle() -> tuple[ManagedProcessManager, ManagedProcessManager, ManagedProcessManager]:
    runtime_manager = ManagedProcessManager(
        "motion-runtime",
        cwd=MOTION_ENGINE_ROOT,
        default_config=RuntimeLaunchConfig(),
        command_builder=build_runtime_command,
    )
    camera_manager = ManagedProcessManager(
        "camera-service",
        cwd=VISION_ROOT,
        default_config=CameraLaunchConfig(),
        command_builder=build_camera_command,
    )
    xbox_manager = ManagedProcessManager(
        "xbox-controller",
        cwd=MOTION_ENGINE_ROOT,
        default_config=XboxControllerLaunchConfig(),
        command_builder=build_xbox_command,
    )
    return runtime_manager, camera_manager, xbox_manager


def _public_host(request_host: str, configured_host: str) -> str:
    request_public = request_host.split(":", 1)[0]
    if configured_host.lower() in {"0.0.0.0", "::", "", "localhost"}:
        return request_public
    try:
        ipaddress.ip_address(configured_host)
    except ValueError:
        return configured_host
    return request_public


def _is_ip_address(value: str) -> bool:
    try:
        ipaddress.ip_address(value)
    except ValueError:
        return False
    return True


def _proxy_host(configured_host: str) -> str:
    if configured_host.lower() in {"0.0.0.0", "::", "", "localhost"} or _is_ip_address(configured_host):
        return "127.0.0.1"
    return configured_host


def _camera_urls(request_host: str, camera_config: dict[str, Any]) -> dict[str, str]:
    public_host = _public_host(request_host, str(camera_config["host"]))
    proxy_host = _proxy_host(str(camera_config["host"]))
    port = int(camera_config["port"])
    return {
        "page_url": f"http://{public_host}:{port}/",
        "stream_url": f"http://{public_host}:{port}/stream.mjpg",
        "raw_stream_url": f"http://{public_host}:{port}/raw.mjpg",
        "health_proxy_url": f"http://{proxy_host}:{port}/health",
        "detections_proxy_url": f"http://{proxy_host}:{port}/detections",
    }


def _active_camera_stream_url(camera_config: dict[str, Any], urls: dict[str, str]) -> str:
    mode = str(camera_config.get("mode") or "detect").lower()
    return urls["raw_stream_url"] if mode == "camera" else urls["stream_url"]


def _resolve_camera_status(
    request_host: str,
    camera_manager: ManagedProcessManager,
    *,
    skip_external_probe: bool = False,
) -> dict[str, Any]:
    camera_status = camera_manager.snapshot()
    camera_status["managed"] = bool(camera_status["running"])

    if not camera_status["running"] and not skip_external_probe:
        urls = _camera_urls(request_host, camera_status["config"])
        payload, status_code = _fetch_json(urls["health_proxy_url"], timeout=0.8)
        if status_code == 200 and payload.get("status") == "ok":
            config = dict(camera_status["config"])
            mode = str(payload.get("mode") or "").lower()
            if mode in {"camera", "detect"}:
                config["mode"] = mode
            detector_info = payload.get("detector")
            if isinstance(detector_info, dict):
                detector_name = str(detector_info.get("name") or "").strip().lower()
                if detector_name:
                    config["detector"] = detector_name
            camera_status = dict(camera_status)
            camera_status["running"] = True
            camera_status["managed"] = False
            camera_status["config"] = config
            if not camera_status["logs"]:
                camera_status["logs"] = [
                    f"[web] detected external camera service at {urls['page_url']}"
                ]

    urls = _camera_urls(request_host, camera_status["config"])
    camera_status.update(
        {
            "page_url": urls["page_url"],
            "stream_url": urls["stream_url"],
            "raw_stream_url": urls["raw_stream_url"],
            "active_stream_url": _active_camera_stream_url(camera_status["config"], urls),
            "detections_proxy_url": urls["detections_proxy_url"],
        }
    )
    return camera_status


def _stop_external_camera_service(camera_config: dict[str, Any]) -> bool:
    urls = _camera_urls("127.0.0.1", camera_config)
    port = int(camera_config["port"])
    pattern = rf"[l]ive.py .* --port {port}\b"
    try:
        subprocess.run(
            ["pkill", "-INT", "-f", pattern],
            cwd=str(VISION_ROOT),
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except OSError:
        return False

    deadline = time.time() + 6.0
    while time.time() < deadline:
        _payload, status_code = _fetch_json(urls["health_proxy_url"], timeout=0.4)
        if status_code >= 500:
            return True
        time.sleep(0.2)
    return False


def _fetch_json(url: str, *, timeout: float = 1.5) -> tuple[dict[str, Any], int]:
    try:
        with urlopen(url, timeout=timeout) as response:
            charset = response.headers.get_content_charset() or "utf-8"
            payload = json.loads(response.read().decode(charset))
            return payload, response.status
    except HTTPError as exc:
        charset = exc.headers.get_content_charset() or "utf-8"
        try:
            payload = json.loads(exc.read().decode(charset))
        except Exception:
            payload = {"error": str(exc)}
        return payload, exc.code
    except (URLError, TimeoutError, ValueError) as exc:
        return {"error": str(exc)}, 503


def _prepare_camera_environment() -> None:
    # PipeWire can hold the camera device on desktop images.
    try:
        subprocess.run(
            ["systemctl", "--user", "stop", "pipewire", "pipewire-pulse", "wireplumber"],
            cwd=str(VISION_ROOT),
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except OSError:
        pass


def _build_ui_defaults(
    runtime_manager: ManagedProcessManager,
    camera_manager: ManagedProcessManager,
    xbox_manager: ManagedProcessManager,
    calibration_store: CalibrationStore,
    dance_storage: DanceStorage,
) -> dict[str, Any]:
    return {
        "runtime": runtime_manager.snapshot()["default_config"],
        "camera": camera_manager.snapshot()["default_config"],
        "xbox": xbox_manager.snapshot()["default_config"],
        "calibration": calibration_store.snapshot()["default_config"],
        "dance": {"presets": dance_storage.list_presets(), "stop_tag_id": TAG_STOP},
    }


def _xbox_status_payload(xbox_manager: ManagedProcessManager) -> dict[str, Any]:
    xbox_process = xbox_manager.snapshot()
    xbox_status = read_xbox_status()
    xbox_status["process_running"] = xbox_process["running"]
    xbox_status["process_pid"] = xbox_process["pid"]
    xbox_status["logs"] = xbox_process["logs"]
    return xbox_status


def _status_payload(
    request_host: str,
    *,
    runtime_manager: ManagedProcessManager,
    camera_manager: ManagedProcessManager,
    xbox_manager: ManagedProcessManager,
    calibration_store: CalibrationStore,
    override_store: OverrideStore,
    dance_state: dict[str, Any],
    recording_manager: DanceRecordingManager,
    automated_runner: AutomatedRunner,
    dance_storage: DanceStorage,
    dance_encoder: DanceEncoder,
    skip_external_probe: bool = False,
) -> dict[str, Any]:
    runtime_status = runtime_manager.snapshot()
    camera_status = _resolve_camera_status(
        request_host, camera_manager, skip_external_probe=skip_external_probe,
    )
    xbox_status = _xbox_status_payload(xbox_manager)
    base_url = f"http://{request_host}"
    return {
        "runtime": runtime_status,
        "camera": camera_status,
        "xbox": xbox_status,
        "calibration": calibration_store.snapshot(),
        "override": override_store.snapshot(),
        "dance": {
            "presets": dance_storage.list_presets(),
            "stop_tag_id": TAG_STOP,
            "last_request": dance_state.get("last_request"),
            **automated_runner.snapshot(),
            "recording": recording_manager.snapshot(),
            "encoder": dance_encoder.snapshot(),
        },
        "pages": {
            "dashboard": f"{base_url}/dashboard",
            "manual": f"{base_url}/manual",
            "auto": f"{base_url}/auto",
            "dance": f"{base_url}/dance",
            "camera": f"{base_url}/camera",
        },
    }


def _render_page(
    *,
    template_name: str,
    page_key: str,
    title: str,
    subtitle: str,
    runtime_manager: ManagedProcessManager,
    camera_manager: ManagedProcessManager,
    xbox_manager: ManagedProcessManager,
    calibration_store: CalibrationStore,
    override_store: OverrideStore,
    dance_state: dict[str, Any],
    recording_manager: DanceRecordingManager,
    automated_runner: AutomatedRunner,
    dance_storage: DanceStorage,
    dance_encoder: DanceEncoder,
) -> str:
    presets = dance_storage.list_presets()
    return render_template(
        f"{template_name}.html",
        page_key=page_key,
        title=title,
        subtitle=subtitle,
        asset_version=ASSET_VERSION,
        initial_status=_status_payload(
            request.host,
            runtime_manager=runtime_manager,
            camera_manager=camera_manager,
            xbox_manager=xbox_manager,
            calibration_store=calibration_store,
            override_store=override_store,
            dance_state=dance_state,
            recording_manager=recording_manager,
            automated_runner=automated_runner,
            dance_storage=dance_storage,
            dance_encoder=dance_encoder,
        ),
        ui_defaults=_build_ui_defaults(
            runtime_manager, camera_manager, xbox_manager, calibration_store, dance_storage,
        ),
        dance_presets=presets,
    )


def _ensure_runtime(
    runtime_manager: ManagedProcessManager,
    payload: MappingLike | None = None,
) -> dict[str, Any]:
    config = RuntimeLaunchConfig.from_mapping(payload)
    snapshot = runtime_manager.snapshot()
    if snapshot["running"]:
        return snapshot
    return runtime_manager.start(config)


def _ensure_camera(
    camera_manager: ManagedProcessManager,
    payload: MappingLike | None = None,
) -> dict[str, Any]:
    config = CameraLaunchConfig.from_mapping(payload)
    snapshot = camera_manager.snapshot()
    current_config = snapshot["config"]
    if snapshot["running"] and current_config == asdict(config):
        return snapshot
    if snapshot["running"]:
        camera_manager.stop()
    _prepare_camera_environment()
    return camera_manager.start(config)


def _ensure_xbox(
    xbox_manager: ManagedProcessManager,
    payload: MappingLike | None = None,
) -> dict[str, Any]:
    config = XboxControllerLaunchConfig.from_mapping(payload)
    snapshot = xbox_manager.snapshot()
    current_config = snapshot["config"]
    if snapshot["running"] and current_config == asdict(config):
        return snapshot
    if snapshot["running"]:
        return snapshot
    return xbox_manager.start(config)


def _ensure_xbox_best_effort(
    xbox_manager: ManagedProcessManager,
    payload: MappingLike | None = None,
) -> None:
    """Try to start the Xbox worker; swallow startup errors.

    The encoder and authoring pages can still load and run without a paired
    gamepad — they just show "Stopped" — so a missing controller is non-fatal.
    """
    try:
        _ensure_xbox(xbox_manager, payload)
    except (RuntimeError, ValueError):
        pass


def _stop_all(
    *,
    runtime_manager: ManagedProcessManager,
    camera_manager: ManagedProcessManager,
    xbox_manager: ManagedProcessManager,
    override_store: OverrideStore,
) -> None:
    """Send stop signals and clean up in the background.

    Returns almost immediately — the heavy ``proc.wait()`` calls happen in
    daemon threads so the HTTP response is never blocked.
    """
    override_store.clear()
    threading.Thread(target=xbox_manager.stop, daemon=True).start()
    threading.Thread(target=camera_manager.stop, daemon=True).start()
    threading.Thread(target=runtime_manager.stop, daemon=True).start()


MappingLike = dict[str, Any] | None


def create_app(
    runtime_manager: ManagedProcessManager | None = None,
    camera_manager: ManagedProcessManager | None = None,
    xbox_manager: ManagedProcessManager | None = None,
    calibration_store: CalibrationStore | None = None,
    override_store: OverrideStore | None = None,
    recording_manager: DanceRecordingManager | None = None,
    automated_runner: AutomatedRunner | None = None,
    dance_storage: DanceStorage | None = None,
    dance_encoder: DanceEncoder | None = None,
) -> Flask:
    if runtime_manager is None or camera_manager is None or xbox_manager is None:
        runtime_manager, camera_manager, xbox_manager = _make_manager_bundle()
    else:
        runtime_manager, camera_manager, xbox_manager = runtime_manager, camera_manager, xbox_manager
    calibration_store = calibration_store or CalibrationStore()
    override_store = override_store or OverrideStore()
    recording_manager = recording_manager or DanceRecordingManager()
    dance_storage = dance_storage or DanceStorage()
    override_store.clear()
    app = Flask(
        __name__,
        template_folder=str(APP_ROOT / "templates"),
        static_folder=str(APP_ROOT / "static"),
    )
    app.config["SEND_FILE_MAX_AGE_DEFAULT"] = 0
    dance_state: dict[str, Any] = {
        "last_request": None,
    }

    if automated_runner is None:
        def _proxy_detections_fetch() -> tuple[dict[str, Any], int]:
            camera_status = _resolve_camera_status("127.0.0.1", camera_manager)
            if not camera_status["running"]:
                return ({"error": "camera service not running"}, 409)
            return _fetch_json(camera_status["detections_proxy_url"])

        automated_runner = AutomatedRunner(
            calibration_store=calibration_store,
            dance_storage=dance_storage,
            detections_fetcher=_proxy_detections_fetch,
            serial_port=DEFAULT_RUNTIME_PORT,
            serial_baudrate=DEFAULT_RUNTIME_BAUDRATE,
        )

    # Encoder uses the calibration send pipeline for live preview; we build
    # a small wrapper to push a draft 18-servo frame to the bridge.
    if dance_encoder is None:
        def _encoder_send(frame: Any) -> None:
            from runtime.protocol import format_full_dir_command  # type: ignore
            from runtime.runtime import SerialBridge  # type: ignore

            command_line = format_full_dir_command("N", list(frame))
            with SerialBridge(
                port=DEFAULT_RUNTIME_PORT,
                baudrate=DEFAULT_RUNTIME_BAUDRATE,
            ) as bridge:
                bridge.write_line(command_line)

        dance_encoder = DanceEncoder(
            calibration_store=calibration_store,
            dance_storage=dance_storage,
            send_frame=_encoder_send,
        )

    @app.get("/")
    def root() -> Any:
        return redirect(url_for("dashboard"))

    @app.get("/dashboard")
    def dashboard() -> Any:
        return _render_page(
            template_name="dashboard",
            page_key="dashboard",
            title="Dashboard",
            subtitle="",
            runtime_manager=runtime_manager,
            camera_manager=camera_manager,
            xbox_manager=xbox_manager,
            calibration_store=calibration_store,
            override_store=override_store,
            dance_state=dance_state,
            recording_manager=recording_manager,
            automated_runner=automated_runner,
            dance_storage=dance_storage,
            dance_encoder=dance_encoder,
        )

    @app.get("/manual")
    def manual() -> Any:
        return _render_page(
            template_name="manual_mode",
            page_key="manual",
            title="Manual Control",
            subtitle="Drive the robot from the browser with keyboard-style controls.",
            runtime_manager=runtime_manager,
            camera_manager=camera_manager,
            xbox_manager=xbox_manager,
            calibration_store=calibration_store,
            override_store=override_store,
            dance_state=dance_state,
            recording_manager=recording_manager,
            automated_runner=automated_runner,
            dance_storage=dance_storage,
            dance_encoder=dance_encoder,
        )

    @app.get("/calibration")
    def calibration() -> Any:
        return _render_page(
            template_name="calibration",
            page_key="calibration",
            title="Calibration",
            subtitle="Calibrate each leg servo before using manual or auto modes.",
            runtime_manager=runtime_manager,
            camera_manager=camera_manager,
            xbox_manager=xbox_manager,
            calibration_store=calibration_store,
            override_store=override_store,
            dance_state=dance_state,
            recording_manager=recording_manager,
            automated_runner=automated_runner,
            dance_storage=dance_storage,
            dance_encoder=dance_encoder,
        )

    @app.get("/auto")
    def auto() -> Any:
        return _render_page(
            template_name="auto_mode",
            page_key="auto",
            title="Auto Mode",
            subtitle="Run AprilTag calibration and prepare preset dances.",
            runtime_manager=runtime_manager,
            camera_manager=camera_manager,
            xbox_manager=xbox_manager,
            calibration_store=calibration_store,
            override_store=override_store,
            dance_state=dance_state,
            recording_manager=recording_manager,
            automated_runner=automated_runner,
            dance_storage=dance_storage,
            dance_encoder=dance_encoder,
        )

    @app.get("/dance")
    def dance() -> Any:
        return _render_page(
            template_name="dance",
            page_key="dance",
            title="Manual Dance Capture",
            subtitle="Record an override timeline while driving in manual mode. For state-by-state preset authoring, use the Dance Encoder.",
            runtime_manager=runtime_manager,
            camera_manager=camera_manager,
            xbox_manager=xbox_manager,
            calibration_store=calibration_store,
            override_store=override_store,
            dance_state=dance_state,
            recording_manager=recording_manager,
            automated_runner=automated_runner,
            dance_storage=dance_storage,
            dance_encoder=dance_encoder,
        )

    @app.get("/dance-encoding")
    def dance_encoding_page() -> Any:
        _ensure_xbox_best_effort(xbox_manager)
        return _render_page(
            template_name="dance_encoding",
            page_key="dance_encoding",
            title="Manual Dance Encoding",
            subtitle="Author a dance state-by-state: pick a leg, nudge angles, save the state.",
            runtime_manager=runtime_manager,
            camera_manager=camera_manager,
            xbox_manager=xbox_manager,
            calibration_store=calibration_store,
            override_store=override_store,
            dance_state=dance_state,
            recording_manager=recording_manager,
            automated_runner=automated_runner,
            dance_storage=dance_storage,
            dance_encoder=dance_encoder,
        )

    @app.get("/camera")
    def camera() -> Any:
        return _render_page(
            template_name="camera",
            page_key="camera",
            title="Camera Console",
            subtitle="Preview the camera, switch modes, and check AprilTag detection.",
            runtime_manager=runtime_manager,
            camera_manager=camera_manager,
            xbox_manager=xbox_manager,
            calibration_store=calibration_store,
            override_store=override_store,
            dance_state=dance_state,
            recording_manager=recording_manager,
            automated_runner=automated_runner,
            dance_storage=dance_storage,
            dance_encoder=dance_encoder,
        )

    @app.get("/api/status")
    def status() -> Any:
        return jsonify(
            _status_payload(
                request.host,
                runtime_manager=runtime_manager,
                camera_manager=camera_manager,
                xbox_manager=xbox_manager,
                calibration_store=calibration_store,
                override_store=override_store,
                dance_state=dance_state,
                recording_manager=recording_manager,
                automated_runner=automated_runner,
                dance_storage=dance_storage,
                dance_encoder=dance_encoder,
            )
        )

    @app.post("/api/runtime/start")
    def start_runtime() -> Any:
        try:
            automated_runner.shutdown()
            dance_encoder.abort()
            override_store.clear()
            calibration_store.clear()
            return jsonify(_ensure_runtime(runtime_manager, request.get_json(silent=True) or {}))
        except ValueError as exc:
            return jsonify({"error": str(exc)}), 400
        except RuntimeError as exc:
            return jsonify({"error": str(exc)}), 409

    @app.post("/api/runtime/stop")
    def stop_runtime() -> Any:
        override_store.clear()
        xbox_manager.stop()
        return jsonify(runtime_manager.stop())

    @app.post("/api/camera/start")
    def start_camera() -> Any:
        try:
            return jsonify(_ensure_camera(camera_manager, request.get_json(silent=True) or {}))
        except ValueError as exc:
            return jsonify({"error": str(exc)}), 400
        except RuntimeError as exc:
            return jsonify({"error": str(exc)}), 409

    @app.post("/api/camera/stop")
    def stop_camera() -> Any:
        # Camera is the only source of tag-based stop; losing it must also
        # disarm the automated worker so presets can't keep running blind.
        automated_runner.shutdown()
        camera_status = _resolve_camera_status(request.host, camera_manager)
        if camera_status["managed"]:
            camera_manager.stop()
        elif camera_status["running"]:
            stopped = _stop_external_camera_service(camera_status["config"])
            if not stopped:
                return jsonify({"error": "camera service did not stop cleanly"}), 504
        return jsonify(
            _status_payload(
                request.host,
                runtime_manager=runtime_manager,
                camera_manager=camera_manager,
                xbox_manager=xbox_manager,
                calibration_store=calibration_store,
                override_store=override_store,
                dance_state=dance_state,
                recording_manager=recording_manager,
                automated_runner=automated_runner,
                dance_storage=dance_storage,
                dance_encoder=dance_encoder,
            )
        )

    @app.get("/api/xbox/status")
    def xbox_status() -> Any:
        payload = _xbox_status_payload(xbox_manager)
        payload["runtime_running"] = bool(runtime_manager.snapshot()["running"])
        payload["calibration_active"] = bool(calibration_store.snapshot()["active"])
        return jsonify(payload)

    @app.get("/api/camera/detections")
    def camera_detections() -> Any:
        camera_status = _resolve_camera_status(request.host, camera_manager)
        if not camera_status["running"]:
            return jsonify({"error": "camera service is not running"}), 409
        payload, status_code = _fetch_json(camera_status["detections_proxy_url"])
        if status_code >= 500:
            time.sleep(0.2)
            payload, status_code = _fetch_json(camera_status["detections_proxy_url"])
        return jsonify(payload), status_code

    @app.post("/api/deploy/manual")
    def deploy_manual() -> Any:
        try:
            automated_runner.shutdown()
            dance_encoder.abort()
            body = request.get_json(silent=True) or {}
            _ensure_runtime(runtime_manager, body.get("runtime"))
            calibration_store.clear()
            # Auto-start Xbox controller alongside runtime
            xbox_snap = xbox_manager.snapshot()
            if not xbox_snap["running"]:
                try:
                    xbox_config = XboxControllerLaunchConfig.from_mapping(body.get("xbox"))
                    xbox_manager.start(xbox_config)
                except (RuntimeError, ValueError):
                    pass  # non-fatal if xbox fails to start
            return jsonify(
                _status_payload(
                    request.host,
                    runtime_manager=runtime_manager,
                    camera_manager=camera_manager,
                    xbox_manager=xbox_manager,
                    calibration_store=calibration_store,
                    override_store=override_store,
                    dance_state=dance_state,
                    recording_manager=recording_manager,
                    automated_runner=automated_runner,
                    dance_storage=dance_storage,
                    dance_encoder=dance_encoder,
                )
            )
        except ValueError as exc:
            return jsonify({"error": str(exc)}), 400

    @app.post("/api/deploy/automated")
    def deploy_automated() -> Any:
        try:
            body = request.get_json(silent=True) or {}
            # Automated worker owns the serial port directly, so the runtime
            # subprocess must not be running (it would contend for /dev/serial0).
            override_store.clear()
            if runtime_manager.snapshot()["running"]:
                runtime_manager.stop()
            xbox_manager.stop()
            calibration_store.clear()
            camera_payload = body.get("camera") or {}
            camera_payload = dict(camera_payload)
            camera_payload["mode"] = "detect"
            _ensure_camera(camera_manager, camera_payload)
            try:
                automated_runner.prepare()
            except SerialUnavailable as exc:
                return jsonify({"error": f"serial unavailable: {exc}"}), 409
            return jsonify(
                _status_payload(
                    request.host,
                    runtime_manager=runtime_manager,
                    camera_manager=camera_manager,
                    xbox_manager=xbox_manager,
                    calibration_store=calibration_store,
                    override_store=override_store,
                    dance_state=dance_state,
                    recording_manager=recording_manager,
                    automated_runner=automated_runner,
                    dance_storage=dance_storage,
                    dance_encoder=dance_encoder,
                )
            )
        except ValueError as exc:
            return jsonify({"error": str(exc)}), 400
        except RuntimeError as exc:
            return jsonify({"error": str(exc)}), 409

    @app.post("/api/deploy/stop-all")
    def deploy_stop_all() -> Any:
        automated_runner.shutdown()
        dance_encoder.abort()
        calibration_store.clear()
        _stop_all(
            runtime_manager=runtime_manager,
            camera_manager=camera_manager,
            xbox_manager=xbox_manager,
            override_store=override_store,
        )
        # Brief pause so SIGINT is delivered before we snapshot status.
        time.sleep(0.25)
        return jsonify(
            _status_payload(
                request.host,
                runtime_manager=runtime_manager,
                camera_manager=camera_manager,
                xbox_manager=xbox_manager,
                calibration_store=calibration_store,
                override_store=override_store,
                dance_state=dance_state,
                recording_manager=recording_manager,
                automated_runner=automated_runner,
                dance_storage=dance_storage,
                dance_encoder=dance_encoder,
                skip_external_probe=True,
            )
        )

    def _prepare_calibration_channel() -> tuple[bool, bool]:
        automated_runner.shutdown()
        dance_encoder.abort()
        override_store.clear()
        runtime_running = bool(runtime_manager.snapshot()["running"])
        if runtime_running:
            runtime_manager.stop()
        return (runtime_running, False)

    @app.post("/api/calibration/send")
    def calibration_send() -> Any:
        payload = request.get_json(silent=True) or {}
        try:
            config = CalibrationPoseConfig.from_mapping(payload)
            runtime_running, xbox_running = _prepare_calibration_channel()
            send_result = send_calibration_pose(
                config,
                legs=calibration_store.preview_legs(config),
            )
            calibration_store.record_send(
                config,
                send_result,
                runtime_stopped=runtime_running,
                xbox_stopped=xbox_running,
            )
            return jsonify(
                _status_payload(
                    request.host,
                    runtime_manager=runtime_manager,
                    camera_manager=camera_manager,
                    xbox_manager=xbox_manager,
                    calibration_store=calibration_store,
                    override_store=override_store,
                    dance_state=dance_state,
                    recording_manager=recording_manager,
                    automated_runner=automated_runner,
                    dance_storage=dance_storage,
                    dance_encoder=dance_encoder,
                )
            )
        except ValueError as exc:
            return jsonify({"error": str(exc)}), 400
        except RuntimeError as exc:
            return jsonify({"error": str(exc)}), 409

    @app.post("/api/calibration/start")
    def calibration_start() -> Any:
        payload = request.get_json(silent=True) or {}
        try:
            config = CalibrationPoseConfig.from_mapping(payload)
            runtime_running, xbox_running = _prepare_calibration_channel()
            if not xbox_manager.snapshot()["running"]:
                try:
                    _ensure_xbox(xbox_manager, payload.get("xbox"))
                except (RuntimeError, ValueError):
                    pass
            send_result = send_calibration_pose(
                config,
                legs=calibration_store.preview_legs(config),
            )
            calibration_store.record_send(
                config,
                send_result,
                runtime_stopped=runtime_running,
                xbox_stopped=xbox_running,
            )
            return jsonify(
                _status_payload(
                    request.host,
                    runtime_manager=runtime_manager,
                    camera_manager=camera_manager,
                    xbox_manager=xbox_manager,
                    calibration_store=calibration_store,
                    override_store=override_store,
                    dance_state=dance_state,
                    recording_manager=recording_manager,
                    automated_runner=automated_runner,
                    dance_storage=dance_storage,
                    dance_encoder=dance_encoder,
                )
            )
        except ValueError as exc:
            return jsonify({"error": str(exc)}), 400
        except RuntimeError as exc:
            return jsonify({"error": str(exc)}), 409

    @app.post("/api/calibration/stop")
    def calibration_stop() -> Any:
        automated_runner.shutdown()
        dance_encoder.abort()
        override_store.clear()
        calibration_store.clear()
        xbox_manager.stop()
        return jsonify(
            _status_payload(
                request.host,
                runtime_manager=runtime_manager,
                camera_manager=camera_manager,
                xbox_manager=xbox_manager,
                calibration_store=calibration_store,
                override_store=override_store,
                dance_state=dance_state,
                recording_manager=recording_manager,
                automated_runner=automated_runner,
                dance_storage=dance_storage,
                dance_encoder=dance_encoder,
            )
        )

    @app.post("/api/calibration/reset")
    def calibration_reset() -> Any:
        calibration_store.reset_profile()
        return jsonify(
            _status_payload(
                request.host,
                runtime_manager=runtime_manager,
                camera_manager=camera_manager,
                xbox_manager=xbox_manager,
                calibration_store=calibration_store,
                override_store=override_store,
                dance_state=dance_state,
                recording_manager=recording_manager,
                automated_runner=automated_runner,
                dance_storage=dance_storage,
                dance_encoder=dance_encoder,
            )
        )

    @app.get("/api/calibration/check")
    def calibration_check() -> Any:
        snap = calibration_store.snapshot()
        profile_path = Path(snap.get("profile_path", ""))
        file_exists = profile_path.is_file()
        saved_count = snap.get("saved_count", 0)
        complete = snap.get("complete", False)
        return jsonify({
            "file_exists": file_exists,
            "saved_count": saved_count,
            "complete": complete,
            "profile_path": str(profile_path),
        })

    @app.post("/api/calibration/save")
    def calibration_save() -> Any:
        payload = request.get_json(silent=True) or {}
        try:
            config = CalibrationPoseConfig.from_mapping(payload)
            runtime_running, xbox_running = _prepare_calibration_channel()
            send_result = send_calibration_pose(
                config,
                legs=calibration_store.preview_legs(config),
            )
            applied_angles = dict(send_result.get("applied_angles") or {})
            calibration_store.save_leg(config.leg_index, applied_angles)
            calibration_store.record_send(
                CalibrationPoseConfig(
                    port=config.port,
                    baudrate=config.baudrate,
                    leg_index=config.leg_index,
                ),
                send_result,
                runtime_stopped=runtime_running,
                xbox_stopped=xbox_running,
            )
            return jsonify(
                _status_payload(
                    request.host,
                    runtime_manager=runtime_manager,
                    camera_manager=camera_manager,
                    xbox_manager=xbox_manager,
                    calibration_store=calibration_store,
                    override_store=override_store,
                    dance_state=dance_state,
                    recording_manager=recording_manager,
                    automated_runner=automated_runner,
                    dance_storage=dance_storage,
                    dance_encoder=dance_encoder,
                )
            )
        except ValueError as exc:
            return jsonify({"error": str(exc)}), 400
        except RuntimeError as exc:
            return jsonify({"error": str(exc)}), 409

    @app.post("/api/override/set")
    def set_override() -> Any:
        payload = request.get_json(silent=True) or {}
        try:
            magnitude = int(payload.get("magnitude", 700))
            expires_after = float(payload.get("expires_after", 0.7))
            snapshot = override_store.set_command(
                str(payload.get("mode") or "N"),
                magnitude=magnitude,
                expires_after=expires_after,
                source="web",
            )
            recording_manager.record_override(snapshot)
            return jsonify(snapshot)
        except ValueError as exc:
            return jsonify({"error": str(exc)}), 400

    @app.post("/api/override/clear")
    def clear_override() -> Any:
        snapshot = override_store.clear()
        recording_manager.record_clear(snapshot)
        return jsonify(snapshot)

    @app.post("/api/dance/recording/start")
    def start_dance_recording() -> Any:
        payload = request.get_json(silent=True) or {}
        try:
            snapshot = recording_manager.start(
                mode=str(payload.get("mode") or "manual"),
                name=str(payload.get("name") or ""),
            )
            return jsonify(snapshot)
        except ValueError as exc:
            return jsonify({"error": str(exc)}), 400
        except RuntimeError as exc:
            return jsonify({"error": str(exc)}), 409

    @app.post("/api/dance/recording/stop")
    def stop_dance_recording() -> Any:
        try:
            snapshot = recording_manager.stop()
            return jsonify(snapshot)
        except RuntimeError as exc:
            return jsonify({"error": str(exc)}), 409

    def _status() -> Any:
        return _status_payload(
            request.host,
            runtime_manager=runtime_manager,
            camera_manager=camera_manager,
            xbox_manager=xbox_manager,
            calibration_store=calibration_store,
            override_store=override_store,
            dance_state=dance_state,
            recording_manager=recording_manager,
            automated_runner=automated_runner,
            dance_storage=dance_storage,
            dance_encoder=dance_encoder,
        )

    @app.post("/api/dance/start")
    def start_dance() -> Any:
        payload = request.get_json(silent=True) or {}
        preset = str(payload.get("preset") or "").strip()
        if not preset:
            return jsonify({"error": "preset is required"}), 400
        if runtime_manager.snapshot()["running"]:
            return jsonify({
                "error": "runtime is running; deploy automated mode first",
                "reason": "runtime_busy",
            }), 409
        camera_status = _resolve_camera_status(request.host, camera_manager)
        if not camera_status["running"]:
            return jsonify({
                "error": "camera service is not running; deploy automated mode first",
                "reason": "camera_missing",
            }), 409
        try:
            dance = dance_storage.get_dance(preset)
        except DancePresetNotFound as exc:
            return jsonify({"error": str(exc), "reason": "preset_not_found"}), 404
        dance_state["last_request"] = {
            "preset": preset,
            "requested_at": time.time(),
        }
        try:
            automated_runner.load_dance(dance)
            automated_runner.start(source="manual_button")
        except CalibrationIncomplete as exc:
            return jsonify({"error": str(exc), "reason": "calibration_incomplete"}), 409
        except SerialUnavailable as exc:
            return jsonify({"error": f"serial unavailable: {exc}"}), 409
        except PresetNotLoaded as exc:
            return jsonify({"error": str(exc), "reason": "preset_not_loaded"}), 409
        return jsonify(_status())

    @app.post("/api/dance/stop")
    def stop_dance() -> Any:
        automated_runner.stop(reason="manual")
        return jsonify(_status())

    # ----- preset CRUD -----------------------------------------------------

    @app.get("/api/dance/presets")
    def list_dance_presets() -> Any:
        return jsonify({
            "presets": dance_storage.list_presets(),
            "stop_tag_id": TAG_STOP,
        })

    @app.get("/api/dance/presets/<preset_id>")
    def get_dance_preset(preset_id: str) -> Any:
        try:
            dance = dance_storage.get_dance(preset_id)
        except DancePresetNotFound as exc:
            return jsonify({"error": str(exc)}), 404
        return jsonify(dance)

    @app.delete("/api/dance/presets/<preset_id>")
    def delete_dance_preset(preset_id: str) -> Any:
        # Stop any active playback so a deleted preset doesn't keep running.
        try:
            snap = automated_runner.snapshot()
            if snap.get("preset_id") == preset_id:
                automated_runner.stop(reason="preset_deleted")
        except Exception:
            pass
        try:
            dance_storage.delete_preset(preset_id)
        except DancePresetNotFound as exc:
            return jsonify({"error": str(exc)}), 404
        automated_runner.refresh_tag_map()
        return jsonify(_status())

    @app.post("/api/dance/presets/<preset_id>/tag")
    def rebind_dance_preset_tag(preset_id: str) -> Any:
        payload = request.get_json(silent=True) or {}
        raw = payload.get("tag_id", None)
        tag_id: int | None
        if raw is None or raw == "":
            tag_id = None
        else:
            try:
                tag_id = int(raw)
            except (TypeError, ValueError):
                return jsonify({"error": "tag_id must be an integer or null"}), 400
        try:
            dance_storage.rebind_tag(preset_id, tag_id)
        except DancePresetNotFound as exc:
            return jsonify({"error": str(exc)}), 404
        except TagConflict as exc:
            return jsonify({"error": str(exc), "reason": "tag_conflict"}), 409
        automated_runner.refresh_tag_map()
        return jsonify(_status())

    # ----- dance encoder (authoring session) -------------------------------

    def _encoder_channel(*, reset_session: bool = False) -> tuple[bool, bool]:
        """Make sure nothing else owns the serial before the encoder writes."""
        automated_runner.shutdown()
        if reset_session:
            dance_encoder.abort()
        override_store.clear()
        runtime_running = bool(runtime_manager.snapshot()["running"])
        if runtime_running:
            runtime_manager.stop()
        # The Xbox status worker does not own the serial port; keep it running
        # so the encoder page can continue reading button/axis state.
        return (runtime_running, bool(xbox_manager.snapshot()["running"]))

    @app.post("/api/dance/encoding/start")
    def encoding_start() -> Any:
        payload = request.get_json(silent=True) or {}
        name = str(payload.get("name") or "").strip()
        try:
            default_segment_ms = int(payload.get("default_segment_ms") or 440)
        except (TypeError, ValueError):
            return jsonify({"error": "default_segment_ms must be an integer"}), 400
        try:
            _encoder_channel(reset_session=True)
            dance_encoder.start(name=name, default_segment_ms=default_segment_ms)
        except (EncoderError, EncoderBusy) as exc:
            return jsonify({"error": str(exc)}), 409
        _ensure_xbox_best_effort(xbox_manager, payload.get("xbox"))
        return jsonify(_status())

    @app.post("/api/dance/encoding/abort")
    def encoding_abort() -> Any:
        dance_encoder.abort()
        return jsonify(_status())

    @app.post("/api/dance/encoding/select-leg")
    def encoding_select_leg() -> Any:
        payload = request.get_json(silent=True) or {}
        try:
            leg_index = int(payload.get("leg_index"))
        except (TypeError, ValueError):
            return jsonify({"error": "leg_index is required"}), 400
        try:
            dance_encoder.select_leg(leg_index)
        except EncoderNotActive as exc:
            return jsonify({"error": str(exc)}), 409
        except EncoderError as exc:
            return jsonify({"error": str(exc)}), 400
        return jsonify(_status())

    @app.post("/api/dance/encoding/duration")
    def encoding_duration() -> Any:
        payload = request.get_json(silent=True) or {}
        try:
            duration_ms = int(payload.get("duration_ms"))
        except (TypeError, ValueError):
            return jsonify({"error": "duration_ms is required"}), 400
        try:
            dance_encoder.set_leg_duration_ms(duration_ms)
        except EncoderNotActive as exc:
            return jsonify({"error": str(exc)}), 409
        except EncoderError as exc:
            return jsonify({"error": str(exc)}), 400
        return jsonify(_status())

    @app.post("/api/dance/encoding/draft")
    def encoding_draft() -> Any:
        payload = request.get_json(silent=True) or {}
        try:
            leg_index = int(payload.get("leg_index"))
        except (TypeError, ValueError):
            return jsonify({"error": "leg_index is required"}), 400
        angles = payload.get("angles") or {}
        send = bool(payload.get("send", True))
        try:
            dance_encoder.draft_leg(leg_index, angles)
            if send:
                # Re-acquire serial in case any other path grabbed it (should be
                # idempotent since _encoder_channel ran at session start. Do not
                # reset the session here; preview must preserve the current draft.
                _encoder_channel(reset_session=False)
                dance_encoder.send_preview()
        except EncoderNotActive as exc:
            return jsonify({"error": str(exc)}), 409
        except EncoderError as exc:
            return jsonify({"error": str(exc)}), 400
        except (RuntimeError, OSError) as exc:
            return jsonify({"error": f"serial send failed: {exc}"}), 409
        return jsonify(_status())

    @app.post("/api/dance/encoding/save-state")
    def encoding_save_state() -> Any:
        try:
            dance_encoder.save_state()
        except EncoderNotActive as exc:
            return jsonify({"error": str(exc)}), 409
        except EncoderError as exc:
            return jsonify({"error": str(exc)}), 400
        return jsonify(_status())

    @app.post("/api/dance/encoding/save-dance")
    def encoding_save_dance() -> Any:
        payload = request.get_json(silent=True) or {}
        raw_tag = payload.get("tag_id")
        if raw_tag is None or raw_tag == "":
            tag_id: int | None = None
        else:
            try:
                tag_id = int(raw_tag)
            except (TypeError, ValueError):
                return jsonify({"error": "tag_id must be an integer"}), 400
        try:
            entry = dance_encoder.save_dance(tag_id=tag_id)
        except EncoderNotActive as exc:
            return jsonify({"error": str(exc)}), 409
        except EncoderError as exc:
            return jsonify({"error": str(exc)}), 400
        except TagConflict as exc:
            return jsonify({"error": str(exc), "reason": "tag_conflict"}), 409
        except DanceValidationError as exc:
            return jsonify({"error": str(exc)}), 400
        except DanceError as exc:
            return jsonify({"error": str(exc)}), 500
        automated_runner.refresh_tag_map()
        response = _status()
        response["preset"] = entry
        return jsonify(response)

    @app.get("/health")
    def health() -> Any:
        dance_snapshot = automated_runner.snapshot()
        return jsonify(
            {
                "status": "ok",
                "runtime_running": runtime_manager.snapshot()["running"],
                "camera_running": camera_manager.snapshot()["running"],
                "override_active": override_store.snapshot()["active"],
                "dance_implemented": bool(dance_snapshot.get("implemented")),
                "dance_active": bool(dance_snapshot.get("active")),
            }
        )

    return app


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Robot dashboard web host")
    parser.add_argument("--host", default=DEFAULT_DASHBOARD_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_DASHBOARD_PORT)
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    app = create_app()
    app.run(host=args.host, port=args.port, threaded=True)


if __name__ == "__main__":
    main()
