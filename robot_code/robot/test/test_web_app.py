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
except Exception:  # pragma: no cover - local dev env may not have Flask
    web_app = None
    DanceRecordingManager = None
    OverrideStore = None


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
    def _make_client(self, *, runtime_running=False, camera_running=True):
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
                "host": "0.0.0.0",
                "port": 8080,
                "detector": "apriltag",
            },
            "default_config": {
                "mode": "detect",
                "host": "0.0.0.0",
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
        app = web_app.create_app(
            runtime_manager=_DummyManager(runtime_snapshot),
            camera_manager=_DummyManager(camera_snapshot),
            override_store=OverrideStore(Path(temp_dir.name) / "override.json"),
            recording_manager=DanceRecordingManager(recording_root),
        )
        return app.test_client()

    def test_status_uses_request_host_for_public_camera_links(self) -> None:
        client = self._make_client()
        response = client.get("/api/status", headers={"Host": "team6.local:8091"})
        self.assertEqual(response.status_code, 200)
        payload = response.get_json()
        self.assertEqual(payload["camera"]["page_url"], "http://team6.local:8080/")
        self.assertEqual(payload["camera"]["stream_url"], "http://team6.local:8080/stream.mjpg")
        self.assertEqual(payload["camera"]["active_stream_url"], "http://team6.local:8080/stream.mjpg")

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
        self.assertEqual(client.get("/dashboard/manual").status_code, 200)
        self.assertEqual(client.get("/dashboard/automated").status_code, 200)
        self.assertEqual(client.get("/dashboard/create-dance").status_code, 200)
        self.assertEqual(client.get("/camera").status_code, 200)

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

    def test_automated_deploy_starts_runtime_and_camera(self) -> None:
        client = self._make_client(runtime_running=False, camera_running=False)
        response = client.post("/api/deploy/automated", json={})
        self.assertEqual(response.status_code, 200)
        payload = response.get_json()
        self.assertTrue(payload["runtime"]["running"])
        self.assertTrue(payload["camera"]["running"])
        self.assertEqual(payload["camera"]["config"]["mode"], "detect")

    def test_dance_start_returns_placeholder_status(self) -> None:
        client = self._make_client()
        response = client.post("/api/dance/start", json={"preset": "groove-alpha"})
        self.assertEqual(response.status_code, 501)
        payload = response.get_json()
        self.assertEqual(payload["status"], "not_implemented")
        self.assertEqual(payload["preset"], "groove-alpha")

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
