"""
Inverse kinematics solver for a single 3-DOF hexapod leg.

Coordinate frame (leg-local, origin at coxa joint):
    +X = outward from body (along leg's default direction)
    +Y = left (when facing outward)
    +Z = up

Joints:
    q1 (coxa)  — rotates leg horizontally (yaw around Z axis)
    q2 (femur) — lifts upper leg (pitch around Y axis at femur joint)
    q3 (tibia) — bends lower leg (pitch around Y axis at tibia joint)
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

from .config import DEFAULT_CONFIG, GeometryConfig, ServoConfig


@dataclass
class IKResult:
    """Result of an IK solve."""
    q1: float          # coxa angle (degrees)
    q2: float          # femur angle (degrees)
    q3: float          # tibia angle (degrees)
    reachable: bool    # whether target is within workspace


def solve_leg_ik(
    x: float, y: float, z: float,
    *,
    geometry: GeometryConfig = DEFAULT_CONFIG.geometry,
) -> IKResult:
    """
    Solve IK for a single leg.

    Args:
        x, y, z: target foot position in leg-local frame (mm).
                  x = outward, y = sideways, z = up/down.

    Returns:
        IKResult with joint angles in degrees and reachability flag.
    """
    coxa_len = geometry.coxa_length
    femur_len = geometry.femur_length
    tibia_len = geometry.tibia_length

    # --- Coxa angle (horizontal rotation) ---
    q1_rad = math.atan2(y, x)

    # --- Project into the leg's vertical plane ---
    # Horizontal distance from coxa joint to foot
    d_horiz = math.sqrt(x * x + y * y) - coxa_len
    d_vert = z  # vertical offset (negative = below coxa)

    # Distance from femur joint to foot in the leg plane
    r = math.sqrt(d_horiz * d_horiz + d_vert * d_vert)

    # Check reachability
    max_reach = femur_len + tibia_len
    min_reach = abs(femur_len - tibia_len)
    reachable = min_reach <= r <= max_reach

    if r < 1e-6:
        # Foot at femur joint — degenerate, just fold leg
        return IKResult(
            q1=math.degrees(q1_rad),
            q2=0.0,
            q3=0.0,
            reachable=False,
        )

    # Clamp r to reachable range for graceful degradation
    r_clamped = max(min_reach + 0.1, min(r, max_reach - 0.1))

    # --- Tibia angle (elbow) via law of cosines ---
    cos_q3 = (femur_len**2 + tibia_len**2 - r_clamped**2) / (2 * femur_len * tibia_len)
    cos_q3 = max(-1.0, min(1.0, cos_q3))
    q3_rad = math.acos(cos_q3)  # 0 = fully folded, pi = fully extended

    # --- Femur angle ---
    # alpha: angle from horizontal to the line from femur joint to foot
    alpha = math.atan2(d_vert, d_horiz)
    # beta: angle from femur to the line from femur joint to foot
    beta = math.atan2(tibia_len * math.sin(q3_rad),
                      femur_len - tibia_len * math.cos(q3_rad))
    q2_rad = alpha + beta

    # Convert to degrees
    q1_deg = math.degrees(q1_rad)
    q2_deg = math.degrees(q2_rad)
    q3_deg = math.degrees(q3_rad)

    return IKResult(q1=q1_deg, q2=q2_deg, q3=q3_deg, reachable=reachable)


def ik_to_servo_angles(
    ik: IKResult,
    leg_index: int = 0,
    *,
    servo: ServoConfig = DEFAULT_CONFIG.servo,
    bypass_safe_range: bool = False,
) -> Tuple[float, float, float]:
    """
    Convert IK joint angles to servo angles clamped to the servo's safe range.

    The S6510 with standard PWM (1000–2000 µs) has ~120° of mechanical travel.
    Servo output is centered at SERVO_NEUTRAL (90°) with ±EFFECTIVE_TRAVEL_DEG/2
    on each side.

    Convention:
        servo_angle = NEUTRAL + direction * (ik_angle + offset)

    When ``bypass_safe_range`` is True, the conservative safe-range clamp is
    skipped and only the hard 0..180 contract range is enforced. This is used
    by calibrated legs whose saved neutral pose intentionally sits outside the
    default (30, 150) guard — otherwise the clamp silently clips the
    user-saved calibration and the leg snaps to a different pose than what
    was calibrated.

    Returns:
        (coxa_servo, femur_servo, tibia_servo) in degrees, clamped to the
        servo safe range (or just 0..180 when ``bypass_safe_range`` is True).
    """
    dirs = servo.direction.get(leg_index, (1, 1, 1))
    offs = servo.offset_deg.get(leg_index, (0.0, 0.0, 0.0))

    raw = [
        servo.neutral_deg + dirs[0] * (ik.q1 + offs[0]),
        servo.neutral_deg + dirs[1] * (ik.q2 + offs[1]),
        servo.neutral_deg - dirs[2] * (ik.q3 - 90.0 + offs[2]),  # tibia: 90° = straight
    ]

    if bypass_safe_range:
        lo, hi = 0.0, 180.0
    else:
        # Clamp to the servo's actual mechanical travel range
        lo, hi = servo.safe_range_deg

    clamped = [max(lo, min(hi, a)) for a in raw]
    return (clamped[0], clamped[1], clamped[2])


def servo_to_ik_angles(
    coxa_servo: float, femur_servo: float, tibia_servo: float,
    leg_index: int = 0,
    *,
    servo: ServoConfig = DEFAULT_CONFIG.servo,
) -> Tuple[float, float, float]:
    """
    Inverse of ik_to_servo_angles — convert servo angles back to IK angles (q1, q2, q3).
    Used for the interactive servo-slider demo.
    """
    dirs = servo.direction.get(leg_index, (1, 1, 1))
    offs = servo.offset_deg.get(leg_index, (0.0, 0.0, 0.0))

    q1 = (coxa_servo - servo.neutral_deg) / dirs[0] - offs[0]
    q2 = (femur_servo - servo.neutral_deg) / dirs[1] - offs[1]
    q3 = (servo.neutral_deg - tibia_servo) / dirs[2] + 90.0 - offs[2]

    return (q1, q2, q3)


def forward_kinematics(
    q1_deg: float, q2_deg: float, q3_deg: float,
    *,
    geometry: GeometryConfig = DEFAULT_CONFIG.geometry,
) -> Tuple[Tuple[float, float, float], ...]:
    """
    Compute forward kinematics — returns joint positions for visualization.

    Returns:
        (coxa_joint, femur_joint, tibia_joint, foot_tip) as (x, y, z) tuples.
    """
    coxa_len = geometry.coxa_length
    femur_len = geometry.femur_length
    tibia_len = geometry.tibia_length

    q1 = math.radians(q1_deg)
    q2 = math.radians(q2_deg)
    q3 = math.radians(q3_deg)

    # Coxa joint is at origin
    coxa_pos = (0.0, 0.0, 0.0)

    # Femur joint — end of coxa segment
    fx = coxa_len * math.cos(q1)
    fy = coxa_len * math.sin(q1)
    fz = 0.0
    femur_pos = (fx, fy, fz)

    # Tibia joint — end of femur segment
    tx = fx + femur_len * math.cos(q1) * math.cos(q2)
    ty = fy + femur_len * math.sin(q1) * math.cos(q2)
    tz = fz + femur_len * math.sin(q2)
    tibia_pos = (tx, ty, tz)

    # Foot tip — end of tibia segment
    q23 = q2 - (math.pi - q3)  # combined angle for tibia
    foot_x = tx + tibia_len * math.cos(q1) * math.cos(q23)
    foot_y = ty + tibia_len * math.sin(q1) * math.cos(q23)
    foot_z = tz + tibia_len * math.sin(q23)
    foot_pos = (foot_x, foot_y, foot_z)

    return (coxa_pos, femur_pos, tibia_pos, foot_pos)
