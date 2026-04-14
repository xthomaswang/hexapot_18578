from __future__ import annotations

import sys
import unittest
from pathlib import Path


MOTION_ROOT = Path(__file__).resolve().parents[1] / "motion_engine"
if str(MOTION_ROOT) not in sys.path:
    sys.path.insert(0, str(MOTION_ROOT))

from ik_engine.config import ServoConfig  # noqa: E402
from motion import servo_deg_to_pwm_us  # noqa: E402


class MotionServoMappingTest(unittest.TestCase):
    def test_pwm_mapping_keeps_neutral_centered_at_1520us(self) -> None:
        servo = ServoConfig()
        self.assertEqual(servo_deg_to_pwm_us(30.0, servo), 1000)
        self.assertEqual(servo_deg_to_pwm_us(90.0, servo), 1520)
        self.assertEqual(servo_deg_to_pwm_us(150.0, servo), 2000)
        self.assertEqual(servo_deg_to_pwm_us(60.0, servo), 1260)
        self.assertEqual(servo_deg_to_pwm_us(120.0, servo), 1760)

    def test_pwm_mapping_clamps_to_safe_window_before_conversion(self) -> None:
        servo = ServoConfig()
        self.assertEqual(servo_deg_to_pwm_us(-10.0, servo), 1000)
        self.assertEqual(servo_deg_to_pwm_us(999.0, servo), 2000)


if __name__ == "__main__":
    unittest.main()
