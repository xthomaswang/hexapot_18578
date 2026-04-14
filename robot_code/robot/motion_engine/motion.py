"""High-level motion control API for the deployed robot stack."""

from __future__ import annotations

import math
from typing import List, Tuple

from ik_engine.body_model import HexapodModel
from ik_engine.config import DEFAULT_CONFIG, GeometryConfig, HexapodConfig, ServoConfig
from ik_engine.leg_ik import ik_to_servo_angles, solve_leg_ik
from ik_engine.locomotion import leg_phase, rotation_foot_pos, step_pos, suggested_stride


def servo_deg_to_pwm_us(deg: float, servo: ServoConfig | None = None) -> int:
    """Convert a servo angle in degrees to a PWM pulse width in microseconds."""
    servo = servo or DEFAULT_CONFIG.servo
    lo, hi = servo.safe_range_deg
    clamped = max(lo, min(hi, deg))
    neutral = float(servo.neutral_deg)
    if clamped <= neutral:
        span_deg = max(neutral - lo, 1e-9)
        span_us = servo.pwm_neutral_us - servo.pwm_min_us
        frac = (clamped - lo) / span_deg
        return int(round(servo.pwm_min_us + frac * span_us))
    span_deg = max(hi - neutral, 1e-9)
    span_us = servo.pwm_max_us - servo.pwm_neutral_us
    frac = (clamped - neutral) / span_deg
    return int(round(servo.pwm_neutral_us + frac * span_us))


class MotionController:
    """Stateful high-level controller for full-body motion commands."""

    def __init__(
        self,
        config: HexapodConfig | None = None,
        frequency: float = 0.45,
        stride: float | None = None,
        lift_scale: float = 0.55,
        yaw_stride_deg: float = 10.0,
    ):
        self.config = config or DEFAULT_CONFIG
        self.model = HexapodModel(self.config)
        self.frequency = frequency
        self.stride = stride if stride is not None else suggested_stride(self.config)
        self.lift_scale = lift_scale
        self.yaw_stride_deg = yaw_stride_deg
        self._phase = 0.0

    @staticmethod
    def _clamp_intent(value: float) -> float:
        return max(-1.0, min(1.0, float(value)))

    def stand(self) -> List[Tuple[int, int]]:
        """Reset to the default standing pose."""
        self.model.stand()
        self._phase = 0.0
        return self.get_pwm()

    def update(
        self,
        forward: float = 0.0,
        strafe: float = 0.0,
        turn: float = 0.0,
        dt: float = 0.02,
    ) -> List[Tuple[int, int]]:
        """Advance the controller state and return PWM outputs."""
        forward = self._clamp_intent(forward)
        strafe = self._clamp_intent(strafe)
        turn = self._clamp_intent(turn)

        speed = max(abs(forward), abs(strafe), abs(turn))
        if speed > 0.05:
            self._phase = (self._phase + dt * self.frequency * speed) % 1.0

        ground_z = self.config.default_foot_z
        rotate_in_place = abs(turn) > max(abs(forward), abs(strafe))

        for leg_index in range(self.model.num_legs):
            bx0, by0, _ = self.model.default_foot_body(leg_index)

            if speed < 0.05:
                self.model.set_foot_body(leg_index, bx0, by0, ground_z)
                continue

            phase = leg_phase(self._phase, leg_index)
            if rotate_in_place:
                yaw_stride_rad = math.radians(turn * self.yaw_stride_deg)
                radius = math.hypot(bx0, by0)
                lift_h = radius * abs(yaw_stride_rad) * self.lift_scale
                bx, by, dz = rotation_foot_pos(
                    phase,
                    yaw_stride_rad,
                    lift_h,
                    bx0,
                    by0,
                )
                self.model.set_foot_body(leg_index, bx, by, ground_z + dz)
                continue

            stride_x = forward * self.stride
            stride_y = strafe * self.stride
            leg_stride = math.hypot(stride_x, stride_y)

            if leg_stride < 1e-6:
                self.model.set_foot_body(leg_index, bx0, by0, ground_z)
                continue

            dx, dz = step_pos(phase, leg_stride, lift_h=leg_stride * self.lift_scale)
            self.model.set_foot_body(
                leg_index,
                bx0 + dx * stride_x / leg_stride,
                by0 + dx * stride_y / leg_stride,
                ground_z + dz,
            )

        return self.get_pwm()

    def forward(self, amount: float = 1.0, dt: float = 0.02) -> List[Tuple[int, int]]:
        return self.update(forward=abs(self._clamp_intent(amount)), dt=dt)

    def backward(self, amount: float = 1.0, dt: float = 0.02) -> List[Tuple[int, int]]:
        return self.update(forward=-abs(self._clamp_intent(amount)), dt=dt)

    def leftward(self, amount: float = 1.0, dt: float = 0.02) -> List[Tuple[int, int]]:
        return self.update(strafe=abs(self._clamp_intent(amount)), dt=dt)

    def rightward(self, amount: float = 1.0, dt: float = 0.02) -> List[Tuple[int, int]]:
        return self.update(strafe=-abs(self._clamp_intent(amount)), dt=dt)

    def turn_left(self, amount: float = 1.0, dt: float = 0.02) -> List[Tuple[int, int]]:
        return self.update(turn=abs(self._clamp_intent(amount)), dt=dt)

    def turn_right(self, amount: float = 1.0, dt: float = 0.02) -> List[Tuple[int, int]]:
        return self.update(turn=-abs(self._clamp_intent(amount)), dt=dt)

    def get_pwm(self) -> List[Tuple[int, int]]:
        """Current servo state as ``(channel, pwm_us)`` pairs."""
        angles = self.model.get_servo_angles()
        servo = self.config.servo
        result: List[Tuple[int, int]] = []
        for leg_index in range(self.model.num_legs):
            channels = servo.servo_map[leg_index]
            for joint_index in range(3):
                deg = angles[leg_index * 3 + joint_index]
                result.append((channels[joint_index], servo_deg_to_pwm_us(deg, servo)))
        return result

    def get_angles(self) -> List[float]:
        return self.model.get_servo_angles()

    def get_leg_angles(self, leg_index: int) -> Tuple[float, float, float]:
        if not 0 <= leg_index < self.model.num_legs:
            raise ValueError(f"leg_index must be in [0, {self.model.num_legs - 1}]")
        angles = self.get_angles()
        start = leg_index * 3
        return (angles[start], angles[start + 1], angles[start + 2])

    def get_front_leg_angles(self, leg_count: int = 1) -> List[float]:
        if not 1 <= leg_count <= self.model.num_legs:
            raise ValueError(f"leg_count must be in [1, {self.model.num_legs}]")
        return self.get_angles()[: leg_count * 3]

    def get_move_command(self) -> str:
        from runtime.protocol import format_move_command

        return format_move_command(self.get_angles())

    def get_front_leg_move_command(
        self,
        leg_count: int = 1,
        move_dir: str = "N",
    ) -> str:
        from runtime.protocol import MOVE_DIR_NEUTRAL, format_leg_move_command

        return format_leg_move_command(
            self.get_angles(),
            leg_count=leg_count,
            move_dir=move_dir or MOVE_DIR_NEUTRAL,
        )

    def get_leg_move_command(self, leg_index: int) -> str:
        from runtime.protocol import MOVE_DIR_NEUTRAL, format_leg_move_command

        return format_leg_move_command(
            self.get_leg_angles(leg_index),
            move_dir=MOVE_DIR_NEUTRAL,
        )


class SingleLegMotionController:
    """High-level bench-test controller for a single 3-DOF leg."""

    def __init__(
        self,
        geometry: GeometryConfig | None = None,
        servo: ServoConfig | None = None,
        channels: Tuple[int, int, int] = (0, 1, 2),
        leg_index: int = 0,
        stride: float | None = None,
        lift_scale: float = 0.8,
        frequency: float = 0.5,
    ):
        self.geometry = geometry or DEFAULT_CONFIG.geometry
        self.servo = servo or DEFAULT_CONFIG.servo
        self.channels = channels
        self.leg_index = leg_index
        self.frequency = frequency
        self.lift_scale = lift_scale

        reach_2dof = self.geometry.femur_length + self.geometry.tibia_length
        self.stride = stride if stride is not None else reach_2dof * 0.33
        self._foot_x = self.geometry.coxa_length + reach_2dof * 0.25
        self._foot_z = -reach_2dof * 0.60
        self._phase = 0.0
        self._stepping = False
        self._angles: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._solve(self._foot_x, 0.0, self._foot_z)

    def _solve(self, x: float, y: float, z: float) -> None:
        ik = solve_leg_ik(x, y, z, geometry=self.geometry)
        self._angles = ik_to_servo_angles(ik, self.leg_index, servo=self.servo)

    def _to_pwm(self) -> List[Tuple[int, int]]:
        return [
            (self.channels[joint_index], servo_deg_to_pwm_us(self._angles[joint_index], self.servo))
            for joint_index in range(3)
        ]

    def stand(self) -> List[Tuple[int, int]]:
        self._phase = 0.0
        self._stepping = False
        self._solve(self._foot_x, 0.0, self._foot_z)
        return self._to_pwm()

    def move_to(self, x: float, y: float, z: float) -> List[Tuple[int, int]]:
        self._stepping = False
        self._solve(x, y, z)
        return self._to_pwm()

    def start_stepping(self) -> None:
        self._stepping = True
        self._phase = 0.0

    def stop_stepping(self) -> List[Tuple[int, int]]:
        return self.stand()

    def update(self, dt: float = 0.02) -> List[Tuple[int, int]]:
        if self._stepping:
            self._phase = (self._phase + dt * self.frequency) % 1.0
            dx, dz = step_pos(
                self._phase,
                self.stride,
                lift_h=self.stride * self.lift_scale,
            )
            self._solve(self._foot_x + dx, 0.0, self._foot_z + dz)
        return self._to_pwm()

    def get_angles(self) -> Tuple[float, float, float]:
        return self._angles

    def get_move_command(self) -> str:
        from runtime.protocol import MOVE_DIR_NEUTRAL, format_leg_move_command

        return format_leg_move_command(
            self.get_angles(),
            move_dir=MOVE_DIR_NEUTRAL,
        )


__all__ = [
    "MotionController",
    "SingleLegMotionController",
    "servo_deg_to_pwm_us",
]
