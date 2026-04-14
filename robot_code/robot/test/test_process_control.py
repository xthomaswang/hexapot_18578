from __future__ import annotations

import signal
import sys
import tempfile
import unittest
from pathlib import Path


WEB_ROOT = Path(__file__).resolve().parents[1] / "web"
if str(WEB_ROOT) not in sys.path:
    sys.path.insert(0, str(WEB_ROOT))

from process_control import (  # noqa: E402
    CalibrationPoseConfig,
    CalibrationStore,
    CameraLaunchConfig,
    ManagedProcessManager,
    OverrideStore,
    RuntimeLaunchConfig,
    build_camera_command,
    build_runtime_command,
)


class _FakeStdout:
    def __iter__(self):
        return iter(())

    def close(self) -> None:
        return None


class _FakePopen:
    next_pid = 3000

    def __init__(self, command, **kwargs):
        self.command = command
        self.kwargs = kwargs
        self.stdout = _FakeStdout()
        self.pid = _FakePopen.next_pid
        _FakePopen.next_pid += 1
        self._returncode = None
        self.signals = []

    def poll(self):
        return self._returncode

    def send_signal(self, sig):
        self.signals.append(sig)
        self._returncode = 0

    def wait(self, timeout=None):
        del timeout
        if self._returncode is None:
            self._returncode = 0
        return self._returncode

    def terminate(self):
        self._returncode = 0

    def kill(self):
        self._returncode = -9


class _FakePopenFactory:
    def __init__(self):
        self.instances = []

    def __call__(self, *args, **kwargs):
        proc = _FakePopen(*args, **kwargs)
        self.instances.append(proc)
        return proc


class ProcessControlTest(unittest.TestCase):
    def test_runtime_command_places_verbose_before_subcommand(self) -> None:
        command = build_runtime_command(RuntimeLaunchConfig(), python_executable="python3")
        # Command shape: [python3, -m, runtime.main, <global flags...>, gait-leg-bridge, <subcommand flags...>]
        self.assertEqual(command[:3], ["python3", "-m", "runtime.main"])
        self.assertIn("gait-leg-bridge", command)
        self.assertIn("--verbose", command)
        self.assertLess(command.index("--verbose"), command.index("gait-leg-bridge"))
        self.assertNotIn("--leg-count", command)

    def test_camera_command_uses_detect_mode_and_detector(self) -> None:
        command = build_camera_command(
            CameraLaunchConfig(mode="detect", host="0.0.0.0", port=8082, detector="apriltag"),
            python_executable="python3",
        )
        self.assertEqual(
            command,
            [
                "python3",
                "live.py",
                "detect",
                "--host",
                "0.0.0.0",
                "--port",
                "8082",
                "--detector",
                "apriltag",
            ],
        )

    def test_manager_tracks_start_and_stop_status(self) -> None:
        popen_factory = _FakePopenFactory()
        manager = ManagedProcessManager(
            "motion-runtime",
            cwd=Path("/tmp"),
            default_config=RuntimeLaunchConfig(),
            command_builder=build_runtime_command,
            python_executable="python3",
            popen_factory=popen_factory,
            register_atexit=False,
        )

        started = manager.start(RuntimeLaunchConfig(verbose=False))
        self.assertTrue(started["running"])
        self.assertEqual(started["pid"], 3000)
        self.assertIn("gait-leg-bridge", started["command_text"])
        self.assertNotIn("--leg-count", started["command_text"])

        stopped = manager.stop()
        self.assertFalse(stopped["running"])
        self.assertEqual(stopped["last_exit_code"], 0)
        self.assertEqual(popen_factory.instances[0].signals, [signal.SIGINT])

    def test_override_store_sets_and_clears_web_priority_command(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            store = OverrideStore(Path(temp_dir) / "override.json")
            active = store.set_command("F", magnitude=650, expires_after=1.0)
            self.assertTrue(active["active"])
            self.assertEqual(active["mode"], "F")
            self.assertEqual(active["forward_cmd"], 650)
            self.assertEqual(active["strafe_cmd"], 0)
            self.assertEqual(active["turn_cmd"], 0)

            cleared = store.clear()
            self.assertFalse(cleared["active"])
            self.assertEqual(cleared["mode"], "N")

    def test_calibration_store_clear_resets_draft_state(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            store = CalibrationStore(Path(temp_dir) / "calibration.json")
            store.record_send(
                CalibrationPoseConfig(leg_index=2, coxa_deg=12.0, femur_deg=-3.0, tibia_deg=7.0),
                {
                    "applied_angles": {"coxa_deg": 100.0, "femur_deg": 70.0, "tibia_deg": 120.0},
                    "applied_frame": [0.0] * 18,
                    "command_text": "N:test",
                },
                runtime_stopped=True,
                xbox_stopped=False,
            )

            cleared = store.clear()
            self.assertFalse(cleared["active"])
            self.assertEqual(
                cleared["config"],
                {
                    "port": "/dev/serial0",
                    "baudrate": 115200,
                    "leg_index": 0,
                    "coxa_deg": 0.0,
                    "femur_deg": 0.0,
                    "tibia_deg": 0.0,
                },
            )
            self.assertEqual(cleared["command_text"], "")
            self.assertIsNone(cleared["updated_at"])


if __name__ == "__main__":
    unittest.main()
