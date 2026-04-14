"""UART command contract owned by the Raspberry Pi runtime layer."""

from __future__ import annotations

from typing import Sequence


DEFAULT_SERIAL_PORT = "/dev/serial0"
BAUD_RATE = 115200
UPDATE_INTERVAL_S = 0.02

MOVE_ALL = "M"
HOME = "H"
STOP = "S"
SPEED = "V"
QUERY = "Q"

ACK = "OK"
ERROR = "ERR"
STATUS = "ST"
IMU = "IMU"

NUM_LEGS = 6
JOINTS_PER_LEG = 3
NUM_SERVOS = NUM_LEGS * JOINTS_PER_LEG
ANGLE_MIN = 0.0
ANGLE_MAX = 180.0
BENCH_LEG_SAFE_LIMITS_DEG: tuple[tuple[float, float], ...] = (
    (60.0, 120.0),
    (90.0, 145.0),
    (88.0, 145.0),
)
MOVE_DIR_CODES = ("N", "F", "B", "L", "R", "LL", "RR")
MOVE_DIR_NEUTRAL = "N"


def clamp_angle(angle: float) -> float:
    """Clamp a servo angle to the contract's valid range."""
    return max(ANGLE_MIN, min(ANGLE_MAX, float(angle)))


def clamp_bench_leg_angle(angle: float, joint_index: int) -> float:
    """Conservative bench-test clamp for the one-leg / three-servo setup."""
    if not 0 <= joint_index < JOINTS_PER_LEG:
        raise ValueError(f"joint_index must be in [0, {JOINTS_PER_LEG - 1}]")
    lo, hi = BENCH_LEG_SAFE_LIMITS_DEG[joint_index]
    return max(lo, min(hi, clamp_angle(angle)))


def normalize_move_dir(move_dir: str) -> str:
    direction = str(move_dir).upper()
    if direction not in MOVE_DIR_CODES:
        raise ValueError(f"move_dir must be one of {MOVE_DIR_CODES}, got {move_dir!r}")
    return direction


def _validate_leg_count(leg_count: int) -> int:
    count = int(leg_count)
    if not 1 <= count <= NUM_LEGS:
        raise ValueError(f"leg_count must be in [1, {NUM_LEGS}]")
    return count


def _format_angles(angles: Sequence[float], *, expected: int) -> list[float]:
    values = [clamp_angle(angle) for angle in angles]
    if len(values) != expected:
        raise ValueError(f"expected {expected} angles, got {len(values)}")
    return values


def front_leg_angles(angles: Sequence[float], *, leg_count: int = 1) -> list[float]:
    """
    Return the first ``leg_count`` legs in configured leg order as
    ``[coxa, femur, tibia, ...]``.

    The configured order is ``L1, L2, L3, R1, R2, R3``. This means:
    - ``leg_count=1`` -> the first left leg
    - ``leg_count=2`` -> the first two left legs

    The input may already be sliced to ``leg_count * JOINTS_PER_LEG`` angles or may be the
    full 18-angle gait output; in the latter case the extra legs are dropped from the tail.
    """
    count = _validate_leg_count(leg_count)
    values = [clamp_angle(angle) for angle in angles]
    expected = count * JOINTS_PER_LEG

    if len(values) == NUM_SERVOS:
        return values[:expected]
    if len(values) == expected:
        return values

    raise ValueError(
        f"expected {NUM_SERVOS} total angles or {expected} angles for {count} leg(s), "
        f"got {len(values)}"
    )


def format_move_command(angles: Sequence[float]) -> str:
    """Format a full-body move command with 18 servo angles."""
    values = _format_angles(angles, expected=NUM_SERVOS)
    return f"{MOVE_ALL}:" + ",".join(f"{angle:.1f}" for angle in values) + "\n"


def format_full_dir_command(
    move_dir: str,
    angles: Sequence[float],
) -> str:
    """
    Format a canonical production DIR frame with 18 servo angles.

    This is the single wire-format serializer shared by both the runtime
    bridge (``FullGaitLegBridgeMotion``) and the calibration preview path
    (``web.process_control.send_calibration_pose``). Both must agree on the
    byte-for-byte output or the calibrated pose drifts between preview and
    production.

    Semantics:
        - ``move_dir`` must be one of ``N/F/B/L/R/LL/RR``.
        - ``angles`` must be exactly 18 values.
        - Only the hard 0..180 contract clamp is applied. The conservative
          bench-safety window (see ``clamp_bench_leg_angle``) is NOT applied
          here so that calibrated neutral poses sitting outside that window
          are not silently clipped. Per-leg saved-vs-unsaved policy must be
          enforced upstream before angles reach this serializer.
    """
    direction = normalize_move_dir(move_dir)
    values = _format_angles(angles, expected=NUM_SERVOS)
    return f"{direction}:" + ",".join(f"{angle:.1f}" for angle in values) + "\n"


def format_move_dir_command(
    move_dir: str,
    angles: Sequence[float],
    *,
    leg_count: int = 1,
) -> str:
    """
    Legacy bench-test serializer for ``leg_count`` legs with a bench-safety clamp.

    Kept for 1- or 2-leg bench harnesses that still want the conservative
    ``clamp_bench_leg_angle`` window. Not used on the production wire path;
    production uses ``format_full_dir_command`` instead.
    """
    direction = normalize_move_dir(move_dir)
    raw_values = front_leg_angles(angles, leg_count=leg_count)
    values = [
        clamp_bench_leg_angle(angle, joint_index % JOINTS_PER_LEG)
        for joint_index, angle in enumerate(raw_values)
    ]
    return f"{direction}:" + ",".join(f"{angle:.1f}" for angle in values) + "\n"


def format_leg_move_command(
    angles: Sequence[float],
    *,
    leg_count: int = 1,
    move_dir: str = MOVE_DIR_NEUTRAL,
) -> str:
    """Backward-compatible wrapper around ``format_move_dir_command``."""
    return format_move_dir_command(move_dir, angles, leg_count=leg_count)


def format_home_command() -> str:
    return f"{HOME}:\n"


def format_stop_command() -> str:
    return f"{STOP}:\n"


def format_speed_command(percent: int) -> str:
    percent = max(0, min(100, int(percent)))
    return f"{SPEED}:{percent}\n"


def format_query_command() -> str:
    return f"{QUERY}:\n"


def parse_message(line: str) -> tuple[str, str]:
    """Split a raw UART line into ``(command, payload)``."""
    message = line.strip()
    if not message:
        return ("", "")
    if ":" not in message:
        return (message, "")
    command, payload = message.split(":", 1)
    return (command, payload)
