from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path


MOTION_ENGINE_ROOT = Path(__file__).resolve().parents[1] / "motion_engine"
if str(MOTION_ENGINE_ROOT) not in sys.path:
    sys.path.insert(0, str(MOTION_ENGINE_ROOT))

from motion import MotionController
from ik_engine.locomotion import rotation_foot_pos


class LocomotionRotationTest(unittest.TestCase):
    def test_rotation_foot_pos_keeps_radius_and_uses_swing_lift(self) -> None:
        bx, by, dz = rotation_foot_pos(
            phase=0.75,
            yaw_stride_rad=math.radians(10.0),
            lift_h_mm=25.0,
            default_bx=120.0,
            default_by=80.0,
        )
        self.assertAlmostEqual(bx, 120.0, places=6)
        self.assertAlmostEqual(by, 80.0, places=6)
        self.assertAlmostEqual(dz, 25.0, places=6)

    def test_motion_controller_turn_uses_in_place_rotation_arc(self) -> None:
        controller = MotionController(yaw_stride_deg=10.0)
        defaults = [controller.model.default_foot_body(i) for i in range(controller.model.num_legs)]
        default_radii = [math.hypot(bx, by) for bx, by, _ in defaults]
        ground_z = controller.config.default_foot_z

        controller.turn_left(dt=0.1)

        lifted_legs = 0
        for leg_index, (bx0, by0, _) in enumerate(defaults):
            bx, by, bz = controller.model.feet_body[leg_index]
            self.assertAlmostEqual(math.hypot(bx, by), default_radii[leg_index], places=5)
            if bz > ground_z:
                lifted_legs += 1
            if abs(bx - bx0) > 1e-6 or abs(by - by0) > 1e-6:
                self.assertNotAlmostEqual(math.atan2(by, bx), math.atan2(by0, bx0), places=6)

        self.assertGreater(lifted_legs, 0)


if __name__ == "__main__":
    unittest.main()
