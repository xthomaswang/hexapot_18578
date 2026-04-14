"""Body-frame robot model used by the deployed motion stack."""

from __future__ import annotations

import math
from typing import List, Tuple

from .config import DEFAULT_CONFIG, HexapodConfig
from .leg_ik import IKResult, forward_kinematics, ik_to_servo_angles, solve_leg_ik


class HexapodModel:
    """Full hexapod body model with body-frame foot targets."""

    def __init__(self, config: HexapodConfig | None = None):
        self.config = config or DEFAULT_CONFIG
        self.leg_mounts = self.config.leg_mounts
        self.num_legs = len(self.leg_mounts)

        self._ox = []
        self._oy = []
        self._cos_a = []
        self._sin_a = []
        self._default_body = []

        for leg_index in range(self.num_legs):
            mount = self.leg_mounts[leg_index]
            ox, oy, angle_deg = mount.x, mount.y, mount.angle_deg
            angle_rad = math.radians(angle_deg)
            ca, sa = math.cos(angle_rad), math.sin(angle_rad)
            self._ox.append(ox)
            self._oy.append(oy)
            self._cos_a.append(ca)
            self._sin_a.append(sa)

            bx = ox + self.config.default_foot_x * ca
            by = oy + self.config.default_foot_x * sa
            bz = self.config.default_foot_z
            self._default_body.append((bx, by, bz))

        self.feet_body = [pos for pos in self._default_body]

    def default_foot_body(self, leg_index: int) -> Tuple[float, float, float]:
        return self._default_body[leg_index]

    def set_foot_body(self, leg_index: int, bx: float, by: float, bz: float) -> None:
        self.feet_body[leg_index] = (bx, by, bz)

    def body_to_leg(
        self,
        leg_index: int,
        bx: float,
        by: float,
        bz: float,
    ) -> Tuple[float, float, float]:
        """Transform a point from body frame to leg-local frame."""
        dx = bx - self._ox[leg_index]
        dy = by - self._oy[leg_index]
        ca = self._cos_a[leg_index]
        sa = self._sin_a[leg_index]
        lx = dx * ca + dy * sa
        ly = -dx * sa + dy * ca
        return (lx, ly, bz)

    def stand(self) -> None:
        self.feet_body = [pos for pos in self._default_body]

    def solve_all(self) -> List[IKResult]:
        results = []
        for leg_index in range(self.num_legs):
            bx, by, bz = self.feet_body[leg_index]
            lx, ly, lz = self.body_to_leg(leg_index, bx, by, bz)
            results.append(solve_leg_ik(lx, ly, lz, geometry=self.config.geometry))
        return results

    def get_servo_angles(self) -> List[float]:
        angles = []
        servo = self.config.servo
        for leg_index, ik in enumerate(self.solve_all()):
            # Saved (user-calibrated) legs may sit outside the default
            # mechanical-travel safe-range because the user's stored neutral
            # pose was intentionally chosen there. Honor it verbatim and let
            # only the hard 0..180 contract range apply.
            bypass = servo.is_saved(leg_index)
            s1, s2, s3 = ik_to_servo_angles(
                ik,
                leg_index=leg_index,
                servo=servo,
                bypass_safe_range=bypass,
            )
            angles.extend([s1, s2, s3])
        return angles

    def get_leg_world_points(self, leg_index: int) -> List[Tuple[float, float, float]]:
        """Joint positions in body frame for visualization and debugging."""
        bx, by, bz = self.feet_body[leg_index]
        lx, ly, lz = self.body_to_leg(leg_index, bx, by, bz)
        ik = solve_leg_ik(lx, ly, lz, geometry=self.config.geometry)

        joints_local = forward_kinematics(ik.q1, ik.q2, ik.q3, geometry=self.config.geometry)
        ox = self._ox[leg_index]
        oy = self._oy[leg_index]
        ca = self._cos_a[leg_index]
        sa = self._sin_a[leg_index]

        world = []
        for lx, ly, lz in joints_local:
            wx = ox + lx * ca - ly * sa
            wy = oy + lx * sa + ly * ca
            world.append((wx, wy, lz))
        return world
