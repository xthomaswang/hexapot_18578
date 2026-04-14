from __future__ import annotations

import importlib
import json
import sys
import tempfile
import unittest
from pathlib import Path


MOTION_ENGINE_ROOT = Path(__file__).resolve().parents[1] / "motion_engine"
if str(MOTION_ENGINE_ROOT) not in sys.path:
    sys.path.insert(0, str(MOTION_ENGINE_ROOT))

from ik_engine.calibration import (
    CalibrationLegProfile,
    CalibrationProfile,
    apply_to_servo_config,
    load_profile,
    save_profile,
)
from ik_engine.config import DEFAULT_CONFIG
from runtime.protocol import (
    format_full_dir_command,
    format_move_dir_command,
    front_leg_angles,
)
from runtime.runtime import ControlCommand, FullGaitLegBridgeMotion, describe_motion


def sample_motion_cases() -> tuple[tuple[str, str], ...]:
    """Representative ESP input lines for the directional bridge."""
    return (
        ("forward", "CMD 500 0 0 1"),
        ("backward", "CMD -500 0 0 2"),
        ("leftward", "CMD 0 500 0 3"),
        ("rightward", "CMD 0 -500 0 4"),
        ("left rotation", "CMD 0 0 500 5"),
        ("right rotation", "CMD 0 0 -500 6"),
    )


def _full_angles(prefix: list[float]) -> list[float]:
    """Pad a 2-leg prefix to a full 18-servo pose."""
    return prefix + [90.0] * (18 - len(prefix))


class FakeMotionController:
    """Deterministic controller so bridge tests can assert exact UART output."""

    STAND = _full_angles([90.0, 114.4, 115.2, 90.0, 114.4, 115.2])
    FORWARD = _full_angles([94.0, 110.0, 120.0, 95.0, 109.0, 121.0])
    BACKWARD = _full_angles([86.0, 118.0, 110.0, 85.0, 119.0, 109.0])
    LEFT = _full_angles([96.0, 112.0, 118.0, 97.0, 111.0, 119.0])
    RIGHT = _full_angles([84.0, 112.0, 118.0, 83.0, 111.0, 119.0])
    LEFT_ROTATE = _full_angles([92.0, 113.0, 117.0, 93.0, 112.0, 118.0])
    RIGHT_ROTATE = _full_angles([88.0, 113.0, 117.0, 87.0, 112.0, 118.0])

    # Bridges (``FullGaitLegBridgeMotion`` / ``ManualBridgeMotion``) read
    # ``self.controller.config.servo.is_saved(leg_index)`` to decide which
    # legs can bypass the bench safety clamp. The default ServoConfig has
    # all legs unsaved, which matches the bench-test baseline.
    config = DEFAULT_CONFIG

    def __init__(self) -> None:
        self._angles = list(self.STAND)

    def stand(self):
        self._angles = list(self.STAND)
        return []

    def update(
        self,
        *,
        forward: float = 0.0,
        strafe: float = 0.0,
        turn: float = 0.0,
        dt: float = 0.02,
    ):
        del dt
        if abs(turn) > max(abs(forward), abs(strafe)):
            self._angles = list(self.LEFT_ROTATE if turn > 0 else self.RIGHT_ROTATE)
        elif abs(forward) >= abs(strafe) and abs(forward) > 0:
            self._angles = list(self.FORWARD if forward > 0 else self.BACKWARD)
        elif abs(strafe) > 0:
            self._angles = list(self.LEFT if strafe > 0 else self.RIGHT)
        else:
            self._angles = list(self.STAND)
        return []

    def get_angles(self) -> list[float]:
        return list(self._angles)

    def get_front_leg_angles(self, leg_count: int = 1) -> list[float]:
        return self.get_angles()[: leg_count * 3]


class FakeBridge:
    """In-memory UART stub for end-to-end runtime tests.

    Matches the surface that ``FullGaitLegBridgeMotion`` and
    ``ManualBridgeMotion`` actually use at runtime: they call
    ``write_line(str)`` with a fully-formatted line and ``poll_lines()``
    for incoming ESP frames.
    """

    def __init__(self, rx_lines: list[str] | None = None) -> None:
        self._rx_lines = list(rx_lines or [])
        self.sent_lines: list[str] = []

    def poll_lines(self) -> list[str]:
        lines = list(self._rx_lines)
        self._rx_lines.clear()
        return lines

    def write_line(self, line: str) -> None:
        self.sent_lines.append(line)

    def send_angle_triplet(
        self,
        angles,
        *,
        seq: int = 0,
        wait_for_reply: bool = False,
    ):
        """Shim for ``ManualBridgeMotion`` which sends a single-leg triplet."""
        del seq, wait_for_reply
        self.sent_lines.append(
            ",".join(f"{float(a):.1f}" for a in angles) + "\n"
        )
        return None

    def send_leg_angles(
        self,
        angles,
        *,
        move_dir: str,
        leg_count: int,
        wait_for_reply: bool = False,
    ):
        """Legacy path kept so older tests that call it explicitly still work."""
        del wait_for_reply
        line = format_move_dir_command(move_dir, angles, leg_count=leg_count)
        self.sent_lines.append(line)
        return None


def simulate_sent_output(rx_line: str) -> str:
    """Run one synthetic ESP command through the runtime bridge."""
    bridge = FakeBridge([rx_line])
    motion = FullGaitLegBridgeMotion(
        bridge,
        controller=FakeMotionController(),
        deadzone=100,
        timeout_s=0.25,
        cmd_scale=1000.0,
    )
    motion.tick()
    if not bridge.sent_lines:
        raise AssertionError(f"no UART output produced for {rx_line!r}")
    return bridge.sent_lines[-1]


def print_sample_motion_io() -> None:
    """Print ESP input -> UART output mappings for quick manual inspection."""
    for label, rx_line in sample_motion_cases():
        tx_line = simulate_sent_output(rx_line).strip()
        print(f"{label:14} | ESP input: {rx_line}")
        print(f"{'':14} | sent output: {tx_line}")


class MotionProtocolTest(unittest.TestCase):
    def test_ik_engine_protocol_module_is_removed(self) -> None:
        with self.assertRaises(ModuleNotFoundError):
            importlib.import_module("ik_engine.protocol")

    def test_front_leg_angles_returns_first_one_leg_from_full_gait(self) -> None:
        angles = [float(i) for i in range(18)]
        self.assertEqual(front_leg_angles(angles, leg_count=1), [0.0, 1.0, 2.0])

    def test_front_leg_angles_returns_first_two_legs_from_full_gait(self) -> None:
        angles = [float(i) for i in range(18)]
        self.assertEqual(
            front_leg_angles(angles, leg_count=2),
            [0.0, 1.0, 2.0, 3.0, 4.0, 5.0],
        )

    def test_format_move_dir_command_uses_requested_direction_prefix(self) -> None:
        angles = [
            90.0, 114.4, 115.2,
            91.0, 113.4, 116.2,
        ] + [90.0] * 12
        self.assertEqual(
            format_move_dir_command("F", angles, leg_count=2),
            "F:90.0,114.4,115.2,91.0,113.4,116.2\n",
        )

    def test_format_move_dir_command_supports_rotation_prefix(self) -> None:
        angles = [90.0, 114.4, 115.2] + [90.0] * 15
        self.assertEqual(
            format_move_dir_command("LL", angles, leg_count=1),
            "LL:90.0,114.4,115.2\n",
        )

    def test_format_full_dir_command_emits_all_18_angles(self) -> None:
        angles = [
            90.0, 64.2, 116.3,
            90.0, 64.2, 116.3,
            90.0, 64.2, 116.3,
            90.0, 64.2, 116.3,
            90.0, 64.2, 116.3,
            90.0, 64.2, 116.3,
        ]
        self.assertEqual(
            format_full_dir_command("N", angles),
            "N:90.0,64.2,116.3,90.0,64.2,116.3,90.0,64.2,116.3,"
            "90.0,64.2,116.3,90.0,64.2,116.3,90.0,64.2,116.3\n",
        )

    def test_format_full_dir_command_preserves_calibrated_pose_outside_bench_window(self) -> None:
        """A saved calibrated pose may sit outside the conservative bench window.

        ``format_move_dir_command`` would clip these values; the canonical
        ``format_full_dir_command`` must not.
        """
        angles = [
            45.0, 150.0, 30.0,  # deliberately outside (60,120)/(90,145)/(88,145)
        ] + [90.0] * 15
        result = format_full_dir_command("F", angles)
        self.assertTrue(result.startswith("F:45.0,150.0,30.0,"))
        self.assertEqual(len(result.split(",")), 18)

    def test_format_full_dir_command_rejects_wrong_angle_count(self) -> None:
        with self.assertRaises(ValueError):
            format_full_dir_command("N", [90.0] * 6)

    def test_format_full_dir_command_rejects_unknown_direction_tag(self) -> None:
        with self.assertRaises(ValueError):
            format_full_dir_command("Z", [90.0] * 18)

    def test_format_full_dir_command_clamps_to_0_180_contract(self) -> None:
        angles = [-5.0, 200.0] + [90.0] * 16
        result = format_full_dir_command("N", angles)
        self.assertTrue(result.startswith("N:0.0,180.0,"))

    def test_calibration_profile_v1_reads_as_default_direction(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "profile.json"
            path.write_text(
                json.dumps(
                    {
                        "version": 1,
                        "legs": [
                            {
                                "leg_index": i,
                                "saved": (i == 2),
                                "offsets_deg": (
                                    {"coxa_deg": 1.0, "femur_deg": 2.0, "tibia_deg": 3.0}
                                    if i == 2
                                    else {"coxa_deg": 0.0, "femur_deg": 0.0, "tibia_deg": 0.0}
                                ),
                            }
                            for i in range(6)
                        ],
                    }
                ),
                encoding="utf-8",
            )

            profile = load_profile(path)
            self.assertEqual(profile.version, 1)
            self.assertTrue(profile.is_saved(2))
            self.assertIsNone(profile.leg(2).direction)  # v1 has no direction

            servo = apply_to_servo_config(profile, DEFAULT_CONFIG.servo)
            self.assertTrue(servo.is_saved(2))
            self.assertEqual(servo.offset_deg[2], (1.0, 2.0, 3.0))
            # Direction falls back to the DEFAULT_CONFIG base
            self.assertEqual(servo.direction[2], DEFAULT_CONFIG.servo.direction[2])

    def test_calibration_profile_v2_direction_override_wins(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "profile.json"
            path.write_text(
                json.dumps(
                    {
                        "version": 2,
                        "legs": [
                            {
                                "leg_index": i,
                                "saved": (i == 0),
                                "offsets_deg": (
                                    {"coxa_deg": 0.5, "femur_deg": 0.0, "tibia_deg": 0.0}
                                    if i == 0
                                    else {"coxa_deg": 0.0, "femur_deg": 0.0, "tibia_deg": 0.0}
                                ),
                                "direction": (
                                    {"coxa_deg": -1, "femur_deg": 1, "tibia_deg": -1}
                                    if i == 0
                                    else None
                                ),
                            }
                            for i in range(6)
                        ],
                    }
                ),
                encoding="utf-8",
            )

            profile = load_profile(path)
            self.assertEqual(profile.version, 2)
            self.assertEqual(profile.leg(0).direction, (-1, 1, -1))

            servo = apply_to_servo_config(profile, DEFAULT_CONFIG.servo)
            self.assertEqual(servo.direction[0], (-1, 1, -1))
            # Other legs keep the base default
            self.assertEqual(servo.direction[1], DEFAULT_CONFIG.servo.direction[1])

    def test_calibration_profile_save_round_trips(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "profile.json"
            profile = CalibrationProfile(
                version=2,
                legs=tuple(
                    CalibrationLegProfile(
                        leg_index=i,
                        saved=(i == 3),
                        offsets_deg=((1.0, 2.0, 3.0) if i == 3 else (0.0, 0.0, 0.0)),
                        direction=((1, -1, 1) if i == 3 else None),
                    )
                    for i in range(6)
                ),
            )
            save_profile(path, profile)

            reread = load_profile(path)
            self.assertEqual(reread.version, 2)
            self.assertTrue(reread.is_saved(3))
            self.assertEqual(reread.leg(3).offsets_deg, (1.0, 2.0, 3.0))
            self.assertEqual(reread.leg(3).direction, (1, -1, 1))
            self.assertIsNone(reread.leg(0).direction)

    def test_calibration_profile_save_is_atomic(self) -> None:
        """Tmp files must land via os.replace so pollers never see a partial write."""
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "profile.json"
            save_profile(path, CalibrationProfile.default())
            # No .tmp residue should remain after a successful save
            leftovers = [p for p in Path(temp_dir).iterdir() if p.suffix == ".tmp"]
            self.assertEqual(leftovers, [])

    def test_describe_motion_matches_expected_direction_codes(self) -> None:
        cases = (
            (ControlCommand(), "N"),
            (ControlCommand(forward_cmd=500), "F"),
            (ControlCommand(forward_cmd=-500), "B"),
            (ControlCommand(strafe_cmd=500), "L"),
            (ControlCommand(strafe_cmd=-500), "R"),
            (ControlCommand(turn_cmd=500), "LL"),
            (ControlCommand(turn_cmd=-500), "RR"),
            (ControlCommand(forward_cmd=300, turn_cmd=700), "LL"),
        )

        for command, expected in cases:
            with self.subTest(command=command, expected=expected):
                self.assertEqual(describe_motion(command), expected)

    def test_full_gait_leg_bridge_maps_numeric_esp_inputs_to_sent_uart_output(self) -> None:
        # FakeMotionController fills legs 3..6 with (90, 90, 90). All unsaved
        # legs go through clamp_bench_leg_angle which is a no-op on 90/114.4/…
        # values here, so each 18-angle line stays byte-exact.
        tail = ",".join(["90.0"] * 12)
        cases = (
            ("CMD 500 0 0 1", f"F:94.0,110.0,120.0,95.0,109.0,121.0,{tail}\n"),
            ("CMD -500 0 0 2", f"B:86.0,118.0,110.0,85.0,119.0,109.0,{tail}\n"),
            ("CMD 0 500 0 3", f"L:96.0,112.0,118.0,97.0,111.0,119.0,{tail}\n"),
            ("CMD 0 -500 0 4", f"R:84.0,112.0,118.0,83.0,111.0,119.0,{tail}\n"),
            ("CMD 0 0 500 5", f"LL:92.0,113.0,117.0,93.0,112.0,118.0,{tail}\n"),
            ("CMD 0 0 -500 6", f"RR:88.0,113.0,117.0,87.0,112.0,118.0,{tail}\n"),
        )

        for rx_line, expected_tx in cases:
            with self.subTest(rx_line=rx_line, expected_tx=expected_tx.strip()):
                self.assertEqual(simulate_sent_output(rx_line), expected_tx)

    def test_full_gait_leg_bridge_times_out_to_neutral_stand_output(self) -> None:
        bridge = FakeBridge()
        motion = FullGaitLegBridgeMotion(
            bridge,
            controller=FakeMotionController(),
            deadzone=100,
            timeout_s=0.0,
            cmd_scale=1000.0,
        )

        motion.tick()

        tail = ",".join(["90.0"] * 12)
        self.assertEqual(
            bridge.sent_lines,
            [f"N:90.0,114.4,115.2,90.0,114.4,115.2,{tail}\n"],
        )

    def test_web_override_beats_conflicting_uart_input(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            override_path = Path(temp_dir) / "override.json"
            override_path.write_text(
                json.dumps(
                    {
                        "active": True,
                        "mode": "B",
                        "forward_cmd": -500,
                        "strafe_cmd": 0,
                        "turn_cmd": 0,
                        "seq": 0,
                        "source": "web",
                        "updated_at": 0.0,
                        "expires_at": 9999999999.0,
                    }
                ),
                encoding="utf-8",
            )

            bridge = FakeBridge(["CMD 500 0 0 1"])
            motion = FullGaitLegBridgeMotion(
                bridge,
                controller=FakeMotionController(),
                deadzone=100,
                timeout_s=0.25,
                cmd_scale=1000.0,
                override_file=str(override_path),
            )
            motion.tick()

            tail = ",".join(["90.0"] * 12)
            self.assertEqual(
                bridge.sent_lines,
                [f"B:86.0,118.0,110.0,85.0,119.0,109.0,{tail}\n"],
            )


if __name__ == "__main__":
    if "--demo" in sys.argv:
        sys.argv.remove("--demo")
        print_sample_motion_io()
    else:
        unittest.main()
