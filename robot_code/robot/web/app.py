#!/usr/bin/env python3
"""Robot dashboard for runtime control and camera workflows."""

from __future__ import annotations

import argparse
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
DANCE_PRESETS = [
    {"id": "groove-alpha", "label": "Groove Alpha", "detail": "Wide stance + sway placeholder"},
    {"id": "shuffle-beta", "label": "Shuffle Beta", "detail": "Quick side-step placeholder"},
    {"id": "spin-gamma", "label": "Spin Gamma", "detail": "Turn-heavy placeholder"},
]


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
    if configured_host in {"0.0.0.0", "::", ""}:
        return request_host.split(":", 1)[0]
    return configured_host


def _proxy_host(configured_host: str) -> str:
    if configured_host in {"0.0.0.0", "::", ""}:
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
) -> dict[str, Any]:
    return {
        "runtime": runtime_manager.snapshot()["default_config"],
        "camera": camera_manager.snapshot()["default_config"],
        "xbox": xbox_manager.snapshot()["default_config"],
        "calibration": calibration_store.snapshot()["default_config"],
        "dance": {"presets": DANCE_PRESETS},
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
            **dict(dance_state),
            "recording": recording_manager.snapshot(),
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
) -> str:
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
        ),
        ui_defaults=_build_ui_defaults(runtime_manager, camera_manager, xbox_manager, calibration_store),
        dance_presets=DANCE_PRESETS,
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
) -> Flask:
    if runtime_manager is None or camera_manager is None or xbox_manager is None:
        runtime_manager, camera_manager, xbox_manager = _make_manager_bundle()
    else:
        runtime_manager, camera_manager, xbox_manager = runtime_manager, camera_manager, xbox_manager
    calibration_store = calibration_store or CalibrationStore()
    override_store = override_store or OverrideStore()
    recording_manager = recording_manager or DanceRecordingManager()
    override_store.clear()
    app = Flask(
        __name__,
        template_folder=str(APP_ROOT / "templates"),
        static_folder=str(APP_ROOT / "static"),
    )
    app.config["SEND_FILE_MAX_AGE_DEFAULT"] = 0
    dance_state: dict[str, Any] = {
        "implemented": False,
        "presets": list(DANCE_PRESETS),
        "last_request": None,
    }

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
        )

    @app.get("/dance")
    def dance() -> Any:
        return _render_page(
            template_name="dance",
            page_key="dance",
            title="Create Dance",
            subtitle="Record a manual session and save it into dance storage.",
            runtime_manager=runtime_manager,
            camera_manager=camera_manager,
            xbox_manager=xbox_manager,
            calibration_store=calibration_store,
            override_store=override_store,
            dance_state=dance_state,
            recording_manager=recording_manager,
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
            )
        )

    @app.post("/api/runtime/start")
    def start_runtime() -> Any:
        try:
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
                )
            )
        except ValueError as exc:
            return jsonify({"error": str(exc)}), 400

    @app.post("/api/deploy/automated")
    def deploy_automated() -> Any:
        try:
            body = request.get_json(silent=True) or {}
            _ensure_runtime(runtime_manager, body.get("runtime"))
            camera_payload = body.get("camera") or {}
            camera_payload = dict(camera_payload)
            camera_payload["mode"] = "detect"
            _ensure_camera(camera_manager, camera_payload)
            calibration_store.clear()
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
                )
            )
        except ValueError as exc:
            return jsonify({"error": str(exc)}), 400
        except RuntimeError as exc:
            return jsonify({"error": str(exc)}), 409

    @app.post("/api/deploy/stop-all")
    def deploy_stop_all() -> Any:
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
                skip_external_probe=True,
            )
        )

    def _prepare_calibration_channel() -> tuple[bool, bool]:
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
                )
            )
        except ValueError as exc:
            return jsonify({"error": str(exc)}), 400
        except RuntimeError as exc:
            return jsonify({"error": str(exc)}), 409

    @app.post("/api/calibration/stop")
    def calibration_stop() -> Any:
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

    @app.post("/api/dance/start")
    def start_dance() -> Any:
        payload = request.get_json(silent=True) or {}
        preset = str(payload.get("preset") or "").strip()
        if not preset:
            return jsonify({"error": "preset is required"}), 400
        dance_state["last_request"] = {
            "preset": preset,
            "requested_at": time.time(),
        }
        return (
            jsonify(
                {
                    "status": "not_implemented",
                    "implemented": False,
                    "preset": preset,
                    "message": "Dance engine placeholder only. Wire this endpoint to the real choreography runner later.",
                }
            ),
            501,
        )

    @app.get("/health")
    def health() -> Any:
        return jsonify(
            {
                "status": "ok",
                "runtime_running": runtime_manager.snapshot()["running"],
                "camera_running": camera_manager.snapshot()["running"],
                "override_active": override_store.snapshot()["active"],
                "dance_implemented": False,
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
