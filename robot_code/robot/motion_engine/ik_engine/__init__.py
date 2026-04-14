"""Pure geometry, kinematics, and locomotion primitives."""

from .body_model import HexapodModel
from .config import (
    DEFAULT_CONFIG,
    GeometryConfig,
    HexapodConfig,
    LayoutConfig,
    LegMount,
    ServoConfig,
    build_hexapod_config,
)
from .leg_ik import IKResult, forward_kinematics, ik_to_servo_angles, servo_to_ik_angles, solve_leg_ik
from .locomotion import (
    GROUP_A,
    leg_phase,
    min_adjacent_spacing,
    rotation_foot_pos,
    step_pos,
    suggested_stride,
)

__all__ = [
    "DEFAULT_CONFIG",
    "GeometryConfig",
    "HexapodConfig",
    "HexapodModel",
    "IKResult",
    "GROUP_A",
    "LayoutConfig",
    "LegMount",
    "ServoConfig",
    "build_hexapod_config",
    "forward_kinematics",
    "ik_to_servo_angles",
    "leg_phase",
    "min_adjacent_spacing",
    "rotation_foot_pos",
    "servo_to_ik_angles",
    "solve_leg_ik",
    "step_pos",
    "suggested_stride",
]
