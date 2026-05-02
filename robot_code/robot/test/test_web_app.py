from __future__ import annotations

import copy
import json
import sys
import tempfile
import unittest
from dataclasses import asdict, is_dataclass
from pathlib import Path
from unittest.mock import patch


WEB_ROOT = Path(__file__).resolve().parents[1] / "web"
if str(WEB_ROOT) not in sys.path:
    sys.path.insert(0, str(WEB_ROOT))

try:
    import app as web_app  # noqa: E402
    from dance_recording import DanceRecordingManager  # noqa: E402
    from process_control import OverrideStore  # noqa: E402
    from automated_runner import AutomatedRunner  # noqa: E402
    from dance_storage import DanceStorage  # noqa: E402
    from dance_encoder import DanceEncoder  # noqa: E402
except Exception:  # pragma: no cover - local dev env may not have Flask
    web_app = None
    DanceRecordingManager = None
    OverrideStore = None
    AutomatedRunner = None
    DanceStorage = None
    DanceEncoder = None


class _FakeSerialBridge:
    def __init__(self, port: str = "", baudrate: int = 0) -> None:
        self.port = port
        self.baudrate = baudrate
        self.lines: list[str] = []
        self.closed = False

    def write_line(self, line: str) -> None:
        self.lines.append(line)

    def close(self) -> None:
        self.closed = True


class _FakeCalibrationStore:
    """Minimal CalibrationStore stand-in for web tests."""

    def __init__(self, *, complete: bool = True) -> None:
        self._complete = complete
        self._active = False
        self._legs = [
            {
                "leg_index": i,
                "angles_deg": {
                    "coxa_deg": 90.0 + i,
                    "femur_deg": 80.0 + i,
                    "tibia_deg": 100.0 + i,
                },
                "saved": complete,
            }
            for i in range(6)
        ]

    def snapshot(self):
        return {
            "name": "manual-calibration",
            "active": self._active,
            "config": {},
            "default_config": {},
            "profile_path": "/tmp/cal.json",
            "legs": [dict(leg, angles_deg=dict(leg["angles_deg"])) for leg in self._legs],
            "default_frame": [0.0] * 18,
            "saved_count": 6 if self._complete else 0,
            "complete": self._complete,
            "applied_frame": [0.0] * 18,
            "command_text": "",
            "updated_at": None,
            "profile_updated_at": None,
            "runtime_stopped": False,
            "xbox_stopped": False,
        }

    def preview_legs(self, config):  # pragma: no cover - not exercised here
        return [dict(leg) for leg in self._legs]

    def save_leg(self, leg_index, angles):  # pragma: no cover
        return self._legs[leg_index]

    def record_send(self, *args, **kwargs):  # pragma: no cover
        return self.snapshot()

    def clear(self):
        self._active = False
        return self.snapshot()

    def reset_profile(self):  # pragma: no cover
        return self.snapshot()


class _DummyManager:
    def __init__(self, snapshot):
        self._snapshot = copy.deepcopy(snapshot)

    def snapshot(self):
        return copy.deepcopy(self._snapshot)

    def start(self, config):
        if is_dataclass(config):
            self._snapshot["config"] = copy.deepcopy(asdict(config))
        self._snapshot["running"] = True
        self._snapshot["pid"] = 4321
        return self.snapshot()

    def stop(self):
        self._snapshot["running"] = False
        self._snapshot["pid"] = None
        return self.snapshot()


@unittest.skipUnless(web_app is not None, "Flask is unavailable in this environment")
class WebAppTest(unittest.TestCase):
    def _make_client(
        self,
        *,
        runtime_running=False,
        camera_running=True,
        xbox_running=False,
        camera_host="0.0.0.0",
    ):
        runtime_snapshot = {
            "name": "motion-runtime",
            "running": runtime_running,
            "pid": None,
            "cwd": "/tmp",
            "command": [],
            "command_text": "",
            "config": {
                "port": "/dev/serial0",
                "baudrate": 115200,
                "verbose": True,
                "duration": 0.0,
                "cmd_scale": 1000.0,
                "timeout": 0.25,
                "deadzone": 100,
                "override_file": "/tmp/runtime_override.json",
            },
            "default_config": {
                "port": "/dev/serial0",
                "baudrate": 115200,
                "verbose": True,
                "duration": 0.0,
                "cmd_scale": 1000.0,
                "timeout": 0.25,
                "deadzone": 100,
                "override_file": "/tmp/runtime_override.json",
            },
            "last_exit_code": None,
            "started_at": None,
            "stopped_at": None,
            "logs": [],
        }
        camera_snapshot = {
            "name": "camera-service",
            "running": camera_running,
            "pid": 5566 if camera_running else None,
            "cwd": "/tmp",
            "command": [],
            "command_text": "",
            "config": {
                "mode": "detect",
                "host": camera_host,
                "port": 8080,
                "detector": "apriltag",
            },
            "default_config": {
                "mode": "detect",
                "host": camera_host,
                "port": 8080,
                "detector": "apriltag",
            },
            "last_exit_code": None,
            "started_at": None,
            "stopped_at": None,
            "logs": [],
        }
        temp_dir = tempfile.TemporaryDirectory()
        self.addCleanup(temp_dir.cleanup)
        recording_root = Path(temp_dir.name) / "dance_storage"
        calibration_store = _FakeCalibrationStore(complete=True)

        def _no_detections():
            return ({"detections": []}, 200)

        dance_root = Path(temp_dir.name) / "dance_auto"
        dance_storage = DanceStorage(dance_root)
        automated_runner = AutomatedRunner(
            calibration_store=calibration_store,
            dance_storage=dance_storage,
            serial_factory=lambda port, baud: _FakeSerialBridge(port, baud),
            detections_fetcher=_no_detections,
            frame_serializer=lambda frame: "N:" + ",".join(f"{v:.1f}" for v in frame) + "\n",
            interp_step_seconds=0.0,
            tick_seconds=0.01,
        )
        self.addCleanup(automated_runner.shutdown)
        self._encoder_sent: list[list[float]] = []
        dance_encoder = DanceEncoder(
            calibration_store=calibration_store,
            dance_storage=dance_storage,
            send_frame=lambda frame: self._encoder_sent.append(list(frame)),
        )
        xbox_snapshot = {
            "name": "xbox-controller",
            "running": xbox_running,
            "pid": 7788 if xbox_running else None,
            "cwd": "/tmp",
            "command": [],
            "command_text": "",
            "config": {"override_file": "/tmp/override.json", "status_file": "/tmp/xbox.json", "device": ""},
            "default_config": {"override_file": "/tmp/override.json", "status_file": "/tmp/xbox.json", "device": ""},
            "last_exit_code": None,
            "started_at": None,
            "stopped_at": None,
            "logs": [],
        }
        runtime_manager = _DummyManager(runtime_snapshot)
        camera_manager = _DummyManager(camera_snapshot)
        xbox_manager = _DummyManager(xbox_snapshot)
        app = web_app.create_app(
            runtime_manager=runtime_manager,
            camera_manager=camera_manager,
            xbox_manager=xbox_manager,
            calibration_store=calibration_store,
            override_store=OverrideStore(Path(temp_dir.name) / "override.json"),
            recording_manager=DanceRecordingManager(recording_root),
            automated_runner=automated_runner,
            dance_storage=dance_storage,
            dance_encoder=dance_encoder,
        )
        self._automated_runner = automated_runner
        self._dance_storage = dance_storage
        self._dance_encoder = dance_encoder
        self._calibration_store = calibration_store
        self._runtime_manager = runtime_manager
        self._camera_manager = camera_manager
        self._xbox_manager = xbox_manager
        return app.test_client()

    def test_status_uses_request_host_for_public_camera_links(self) -> None:
        client = self._make_client()
        response = client.get("/api/status", headers={"Host": "team6.local:8091"})
        self.assertEqual(response.status_code, 200)
        payload = response.get_json()
        self.assertEqual(payload["camera"]["page_url"], "http://team6.local:8080/")
        self.assertEqual(payload["camera"]["stream_url"], "http://team6.local:8080/stream.mjpg")
        self.assertEqual(payload["camera"]["active_stream_url"], "http://team6.local:8080/stream.mjpg")

    def test_status_ignores_stale_camera_ip_for_public_links(self) -> None:
        client = self._make_client(camera_host="128.237.1.23")
        response = client.get("/api/status", headers={"Host": "homebot.local:8091"})
        self.assertEqual(response.status_code, 200)
        payload = response.get_json()
        self.assertEqual(payload["camera"]["page_url"], "http://homebot.local:8080/")
        self.assertEqual(payload["camera"]["stream_url"], "http://homebot.local:8080/stream.mjpg")
        self.assertEqual(payload["camera"]["detections_proxy_url"], "http://127.0.0.1:8080/detections")

    def test_status_detects_external_camera_service(self) -> None:
        client = self._make_client(camera_running=False)
        seen_urls = []

        def fake_fetch(url, *, timeout=1.5):
            seen_urls.append(url)
            return (
                {
                    "status": "ok",
                    "mode": "camera",
                    "detector": {"name": "none"},
                },
                200,
            )

        with patch.object(
            web_app,
            "_fetch_json",
            side_effect=fake_fetch,
        ):
            response = client.get("/api/status", headers={"Host": "team6.local:8091"})
        self.assertEqual(response.status_code, 200)
        payload = response.get_json()
        self.assertEqual(seen_urls[0], "http://127.0.0.1:8080/health")
        self.assertTrue(payload["camera"]["running"])
        self.assertFalse(payload["camera"]["managed"])
        self.assertEqual(payload["camera"]["config"]["mode"], "camera")
        self.assertEqual(payload["camera"]["active_stream_url"], "http://team6.local:8080/raw.mjpg")

    def test_dashboard_routes_are_available(self) -> None:
        client = self._make_client()
        redirect_response = client.get("/")
        self.assertEqual(redirect_response.status_code, 302)
        self.assertTrue(redirect_response.location.endswith("/dashboard"))
        self.assertEqual(client.get("/dashboard").status_code, 200)
        self.assertEqual(client.get("/manual").status_code, 200)
        self.assertEqual(client.get("/auto").status_code, 200)
        self.assertEqual(client.get("/dance").status_code, 200)
        self.assertEqual(client.get("/camera").status_code, 200)

    def test_dance_encoding_page_autostarts_xbox(self) -> None:
        client = self._make_client(xbox_running=False)
        response = client.get("/dance-encoding")
        self.assertEqual(response.status_code, 200)
        self.assertTrue(self._xbox_manager.snapshot()["running"])

    def test_camera_detections_proxies_from_local_camera_service(self) -> None:
        client = self._make_client()
        with patch.object(
            web_app,
            "_fetch_json",
            return_value=({"count": 1, "detections": [{"tag_id": 4}]}, 200),
        ):
            response = client.get("/api/camera/detections", headers={"Host": "team6.local:8091"})
        self.assertEqual(response.status_code, 200)
        payload = response.get_json()
        self.assertEqual(payload["count"], 1)
        self.assertEqual(payload["detections"][0]["tag_id"], 4)

    def test_camera_detections_accept_external_camera_service(self) -> None:
        client = self._make_client(camera_running=False)

        def fake_fetch(url, *, timeout=1.5):
            if url.endswith("/health"):
                return (
                    {
                        "status": "ok",
                        "mode": "detect",
                        "detector": {"name": "apriltag"},
                    },
                    200,
                )
            if url.endswith("/detections"):
                return ({"count": 1, "detections": [{"tag_id": 7}]}, 200)
            raise AssertionError(f"unexpected url: {url}")

        with patch.object(web_app, "_fetch_json", side_effect=fake_fetch):
            response = client.get("/api/camera/detections", headers={"Host": "team6.local:8091"})
        self.assertEqual(response.status_code, 200)
        payload = response.get_json()
        self.assertEqual(payload["count"], 1)
        self.assertEqual(payload["detections"][0]["tag_id"], 7)

    def test_camera_stop_stops_detected_external_service(self) -> None:
        client = self._make_client(camera_running=False)
        state = {"running": True}

        def fake_fetch(url, *, timeout=1.5):
            if url.endswith("/health"):
                if state["running"]:
                    return (
                        {
                            "status": "ok",
                            "mode": "detect",
                            "detector": {"name": "apriltag"},
                        },
                        200,
                    )
                return ({"error": "offline"}, 503)
            raise AssertionError(f"unexpected url: {url}")

        def fake_run(*args, **kwargs):
            state["running"] = False
            return None

        with patch.object(web_app, "_fetch_json", side_effect=fake_fetch):
            with patch.object(web_app.subprocess, "run", side_effect=fake_run):
                response = client.post("/api/camera/stop", headers={"Host": "team6.local:8091"})
        self.assertEqual(response.status_code, 200)
        payload = response.get_json()
        self.assertFalse(payload["camera"]["running"])

    def test_automated_deploy_starts_camera_but_not_runtime(self) -> None:
        # Per V1 plan, the automated worker owns the serial link, so runtime
        # must NOT be started by /api/deploy/automated.
        client = self._make_client(runtime_running=False, camera_running=False)
        response = client.post("/api/deploy/automated", json={})
        self.assertEqual(response.status_code, 200)
        payload = response.get_json()
        self.assertFalse(payload["runtime"]["running"])
        self.assertTrue(payload["camera"]["running"])
        self.assertEqual(payload["camera"]["config"]["mode"], "detect")

    def _seed_preset(
        self, name: str = "Wave Alpha", *, tag_id: int | None = None,
    ) -> str:
        """Write a single-state dance directly into storage for tests."""
        def triplet(c=90.0, f=90.0, t=90.0):
            return {"coxa_deg": c, "femur_deg": f, "tibia_deg": t}

        neutral = triplet()
        edited = triplet(c=95.0)
        dance = {
            "name": name,
            "tag_id": tag_id,
            "default_segment_ms": 440,
            "calibration_snapshot": {"angles": [90.0] * 18},
            "states": [
                {
                    "index": 0,
                    "duration_ms": 400,
                    "legs": [
                        {
                            "leg_index": i,
                            "edited": (i == 0),
                            "from": neutral,
                            "to": edited if i == 0 else neutral,
                        }
                        for i in range(6)
                    ],
                }
            ],
        }
        return self._dance_storage.save_dance(dance)["preset_id"]

    def test_dance_start_rejects_unknown_preset(self) -> None:
        client = self._make_client()
        client.post("/api/deploy/automated", json={})
        response = client.post("/api/dance/start", json={"preset": "does-not-exist"})
        self.assertEqual(response.status_code, 404)
        body = response.get_json()
        self.assertEqual(body.get("reason"), "preset_not_found")

    def test_dance_start_with_full_calibration_goes_active(self) -> None:
        client = self._make_client()
        preset_id = self._seed_preset("My Dance")
        self.assertEqual(client.post("/api/deploy/automated", json={}).status_code, 200)
        response = client.post("/api/dance/start", json={"preset": preset_id})
        self.assertEqual(response.status_code, 200, response.get_data(as_text=True))
        status = client.get("/api/status").get_json()
        self.assertTrue(status["dance"]["active"])
        self.assertEqual(status["dance"]["preset_id"], preset_id)
        self.assertEqual(status["dance"]["source"], "manual_button")

    def test_dance_stop_returns_to_idle_with_reason(self) -> None:
        client = self._make_client()
        preset_id = self._seed_preset("Dance Two")
        client.post("/api/deploy/automated", json={})
        client.post("/api/dance/start", json={"preset": preset_id})
        response = client.post("/api/dance/stop", json={})
        self.assertEqual(response.status_code, 200)
        payload = response.get_json()
        self.assertFalse(payload["dance"]["active"])
        self.assertEqual(payload["dance"]["last_stop_reason"], "manual")

    def test_dance_start_rejected_while_runtime_running(self) -> None:
        client = self._make_client(runtime_running=True, camera_running=True)
        preset_id = self._seed_preset("Blocked")
        response = client.post("/api/dance/start", json={"preset": preset_id})
        self.assertEqual(response.status_code, 409)
        body = response.get_json()
        self.assertEqual(body.get("reason"), "runtime_busy")

    def test_dance_start_rejected_when_camera_missing(self) -> None:
        client = self._make_client(runtime_running=False, camera_running=False)
        preset_id = self._seed_preset("NoCam")
        response = client.post("/api/dance/start", json={"preset": preset_id})
        self.assertEqual(response.status_code, 409)
        body = response.get_json()
        self.assertEqual(body.get("reason"), "camera_missing")

    def test_dance_start_allows_xbox_process_when_runtime_stopped(self) -> None:
        client = self._make_client(
            runtime_running=False, camera_running=True, xbox_running=True,
        )
        preset_id = self._seed_preset("XboxOK")
        response = client.post("/api/dance/start", json={"preset": preset_id})
        self.assertEqual(response.status_code, 200, response.get_data(as_text=True))
        body = response.get_json()
        self.assertTrue(body["dance"]["active"])

    def test_camera_stop_shuts_down_automated_runner(self) -> None:
        client = self._make_client()
        preset_id = self._seed_preset("Cam")
        client.post("/api/deploy/automated", json={})
        client.post("/api/dance/start", json={"preset": preset_id})
        self.assertTrue(self._automated_runner.snapshot()["active"])
        client.post("/api/camera/stop", json={})
        snap = self._automated_runner.snapshot()
        self.assertFalse(snap["active"])
        self.assertFalse(snap["ready"])

    def test_deploy_manual_stops_automated_worker(self) -> None:
        client = self._make_client()
        preset_id = self._seed_preset("Manual")
        client.post("/api/deploy/automated", json={})
        client.post("/api/dance/start", json={"preset": preset_id})
        self.assertTrue(self._automated_runner.snapshot()["active"])
        client.post("/api/deploy/manual", json={})
        snap = self._automated_runner.snapshot()
        self.assertFalse(snap["active"])
        self.assertFalse(snap["ready"])

    # ----- preset CRUD endpoints ------------------------------------------

    def test_list_presets_returns_seeded(self) -> None:
        client = self._make_client()
        self._seed_preset("Alpha", tag_id=2)
        self._seed_preset("Beta", tag_id=3)
        response = client.get("/api/dance/presets")
        self.assertEqual(response.status_code, 200)
        body = response.get_json()
        ids = {p["preset_id"] for p in body["presets"]}
        self.assertEqual(ids, {"alpha", "beta"})
        self.assertEqual(body["stop_tag_id"], 1)

    def test_delete_preset_removes_it(self) -> None:
        client = self._make_client()
        preset_id = self._seed_preset("Removable")
        response = client.delete(f"/api/dance/presets/{preset_id}")
        self.assertEqual(response.status_code, 200)
        self.assertEqual(self._dance_storage.list_presets(), [])

    def test_rebind_tag_conflict(self) -> None:
        client = self._make_client()
        self._seed_preset("Red", tag_id=5)
        self._seed_preset("Blue", tag_id=6)
        response = client.post("/api/dance/presets/blue/tag", json={"tag_id": 5})
        self.assertEqual(response.status_code, 409)
        self.assertEqual(response.get_json().get("reason"), "tag_conflict")

    def test_rebind_tag_rejects_reserved_stop(self) -> None:
        client = self._make_client()
        self._seed_preset("Lonely", tag_id=7)
        response = client.post("/api/dance/presets/lonely/tag", json={"tag_id": 1})
        self.assertEqual(response.status_code, 409)

    # ----- encoder endpoints ----------------------------------------------

    def test_encoder_flow_save_dance_registers_preset(self) -> None:
        client = self._make_client()
        r = client.post("/api/dance/encoding/start",
                        json={"name": "EncoderFlow", "default_segment_ms": 500})
        self.assertEqual(r.status_code, 200, r.get_data(as_text=True))
        r = client.post("/api/dance/encoding/select-leg", json={"leg_index": 0})
        self.assertEqual(r.status_code, 200)
        r = client.post("/api/dance/encoding/draft",
                        json={"leg_index": 0,
                              "angles": {"coxa_deg": 92.0, "femur_deg": 80.0, "tibia_deg": 100.0},
                              "send": False})
        self.assertEqual(r.status_code, 200)
        r = client.post("/api/dance/encoding/save-state", json={})
        self.assertEqual(r.status_code, 200)
        r = client.post("/api/dance/encoding/save-dance", json={"tag_id": 6})
        self.assertEqual(r.status_code, 200)
        body = r.get_json()
        self.assertIn("preset", body)
        self.assertEqual(body["preset"]["preset_id"], "encoderflow")
        self.assertEqual(body["preset"]["tag_id"], 6)
        # Now status reflects the new preset list
        status = client.get("/api/status").get_json()
        preset_ids = [p["preset_id"] for p in status["dance"]["presets"]]
        self.assertIn("encoderflow", preset_ids)

    def test_encoder_save_state_requires_edits(self) -> None:
        client = self._make_client()
        client.post("/api/dance/encoding/start", json={"name": "Empty"})
        r = client.post("/api/dance/encoding/save-state", json={})
        self.assertEqual(r.status_code, 400)
        client.post("/api/dance/encoding/abort", json={})

    def test_encoder_reject_start_when_calibration_incomplete(self) -> None:
        client = self._make_client()
        self._calibration_store._complete = False
        for leg in self._calibration_store._legs:
            leg["saved"] = False
        r = client.post("/api/dance/encoding/start", json={"name": "Incomplete"})
        self.assertEqual(r.status_code, 409)

    def test_encoder_preview_send_keeps_session_active(self) -> None:
        client = self._make_client()
        r = client.post("/api/dance/encoding/start", json={"name": "Preview"})
        self.assertEqual(r.status_code, 200, r.get_data(as_text=True))
        self._runtime_manager.start({})
        self._xbox_manager.start({})
        r = client.post(
            "/api/dance/encoding/draft",
            json={
                "leg_index": 0,
                "angles": {"coxa_deg": 92.0, "femur_deg": 81.0, "tibia_deg": 101.0},
                "send": True,
            },
        )
        self.assertEqual(r.status_code, 200, r.get_data(as_text=True))
        body = r.get_json()
        self.assertTrue(body["dance"]["encoder"]["active"])
        self.assertEqual(len(self._encoder_sent), 1)
        current = body["dance"]["encoder"]["current_state_to"][0]
        self.assertEqual(current["coxa_deg"], 92.0)
        self.assertEqual(current["femur_deg"], 81.0)
        self.assertEqual(current["tibia_deg"], 101.0)
        self.assertFalse(self._runtime_manager.snapshot()["running"])
        self.assertTrue(self._xbox_manager.snapshot()["running"])

    def test_manual_recording_saves_timeline_under_manual_storage(self) -> None:
        client = self._make_client(runtime_running=True, camera_running=False)
        start_response = client.post(
            "/api/dance/recording/start",
            json={"mode": "manual", "name": "Side Step Combo"},
        )
        self.assertEqual(start_response.status_code, 200)
        start_payload = start_response.get_json()
        self.assertTrue(start_payload["active"])
        self.assertEqual(start_payload["mode"], "manual")

        override_response = client.post(
            "/api/override/set",
            json={"mode": "F", "magnitude": 650, "expires_after": 1.0},
        )
        self.assertEqual(override_response.status_code, 200)
        clear_response = client.post("/api/override/clear", json={})
        self.assertEqual(clear_response.status_code, 200)

        stop_response = client.post("/api/dance/recording/stop", json={})
        self.assertEqual(stop_response.status_code, 200)
        stop_payload = stop_response.get_json()
        self.assertFalse(stop_payload["active"])
        self.assertIsNotNone(stop_payload["last_saved"])

        saved = stop_payload["last_saved"]
        self.assertEqual(saved["mode"], "manual")
        self.assertIn("/manual/", saved["path"])

        recording_file = Path(saved["path"]) / "recording.json"
        self.assertTrue(recording_file.exists())
        payload = json.loads(recording_file.read_text(encoding="utf-8"))
        self.assertEqual(payload["mode"], "manual")
        self.assertEqual(payload["name"], "Side Step Combo")
        self.assertGreaterEqual(payload["event_count"], 2)


if __name__ == "__main__":
    unittest.main()
