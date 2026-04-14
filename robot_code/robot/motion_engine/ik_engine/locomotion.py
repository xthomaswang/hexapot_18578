"""Locomotion trajectory helpers for deployed hexapod motion."""

from __future__ import annotations

import math

from .config import DEFAULT_CONFIG, HexapodConfig


GROUP_A = {0, 2, 4}


def leg_phase(base_phase: float, leg_index: int) -> float:
    return (base_phase + (0.0 if leg_index in GROUP_A else 0.5)) % 1.0


def min_adjacent_spacing(config: HexapodConfig = DEFAULT_CONFIG) -> float:
    """Minimum front-mid or mid-rear spacing on either side of the body."""
    mounts = config.leg_mounts
    left_x = [mounts[i].x for i in (0, 1, 2)]
    right_x = [mounts[i].x for i in (3, 4, 5)]
    spacings = [
        abs(side[i] - side[i + 1])
        for side in (left_x, right_x)
        for i in range(len(side) - 1)
    ]
    return min(spacings)


def suggested_stride(
    config: HexapodConfig = DEFAULT_CONFIG,
    clearance: float = 40.0,
    max_stride: float = 120.0,
    min_stride: float = 20.0,
) -> float:
    """Conservative gait stride chosen from leg spacing."""
    spacing_limited = (min_adjacent_spacing(config) - clearance) / 2.0
    return max(min_stride, min(max_stride, spacing_limited))


def step_pos(
    phase: float,
    stride: float,
    lift_h: float | None = None,
    *,
    lift_scale: float = 0.8,
) -> tuple[float, float]:
    """Smooth stance/swing trajectory in body-frame X and Z."""
    lift_h = stride * lift_scale if lift_h is None else lift_h
    p = phase % 1.0

    if p < 0.5:
        u = p / 0.5
        x = stride * math.cos(math.pi * u)
        z = 0.0
    else:
        u = (p - 0.5) / 0.5
        x = -stride * math.cos(math.pi * u)
        z = lift_h * math.sin(math.pi * u)

    return (x, z)


def rotation_foot_pos(
    phase: float,
    yaw_stride_rad: float,
    lift_h_mm: float,
    default_bx: float,
    default_by: float,
) -> tuple[float, float, float]:
    """Foot position and lift during in-place yaw rotation."""
    angle_offset, dz = step_pos(phase, yaw_stride_rad, lift_h=lift_h_mm)
    ca = math.cos(angle_offset)
    sa = math.sin(angle_offset)
    bx = default_bx * ca - default_by * sa
    by = default_bx * sa + default_by * ca
    return (bx, by, dz)
