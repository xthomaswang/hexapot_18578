"""Process control helpers for the robot web dashboard."""

from __future__ import annotations

import atexit
import json
import os
import shlex
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence


ROBOT_ROOT = Path(__file__).resolve().parents[1]
MOTION_ENGINE_ROOT = ROBOT_ROOT / "motion_engine"
VISION_ROOT = ROBOT_ROOT / "vision"

DEFAULT_RUNTIME_PORT = "/dev/serial0"
DEFAULT_RUNTIME_BAUDRATE = 115200
DEFAULT_RUNTIME_TIMEOUT = 0.5
DEFAULT_RUNTIME_STRIDE_MM = 160.0
DEFAULT_RUNTIME_LIFT_SCALE = 0.55
DEFAULT_RUNTIME_FREQUENCY_HZ = 0.45
DEFAULT_RUNTIME_YAW_STRIDE_DEG = 10.0
DEFAULT_RUNTIME_STANCE_Z_MM = -495.0
DEFAULT_CALIBRATION_COXA_DEG = 90.0
DEFAULT_CALIBRATION_FEMUR_DEG = 64.2
DEFAULT_CALIBRATION_TIBIA_DEG = 116.3
CALIBRATION_DRAFT_LIMIT_DEG = 360.0
DEFAULT_CAMERA_HOST = "0.0.0.0"
DEFAULT_CAMERA_PORT = 8080
DEFAULT_DASHBOARD_HOST = "0.0.0.0"
DEFAULT_DASHBOARD_PORT = 8091
DEFAULT_OVERRIDE_FILE = str(ROBOT_ROOT / "web" / "runtime_override.json")
DEFAULT_XBOX_STATUS_FILE = str(ROBOT_ROOT / "web" / "xbox_status.json")
DEFAULT_CALIBRATION_FILE = str(MOTION_ENGINE_ROOT / "_cache" / "calibration_profile.json")
MAX_LOG_LINES = 400
MAX_UART_LINES = 600
UART_TX_PREFIX = "[UART-TX]"
UART_RX_PREFIX = "[UART-RX]"
OVERRIDE_MODES = ("N", "F", "B", "L", "R", "LL", "RR")
CALIBRATION_LEG_LABELS = ("L1", "L2", "L3", "R1", "R2", "R3")
CALIBRATION_JOINT_KEYS = ("coxa_deg", "femur_deg", "tibia_deg")

_DEFAULT_CALIBRATION_FRAME_CACHE: list[float] | None = None


def _coerce_str(value: Any, *, default: str) -> str:
    if value is None:
        return default
    text = str(value).strip()
    return text if text else default


def _coerce_int(value: Any, *, default: int) -> int:
    if value is None or value == "":
        return default
    return int(value)


def _coerce_float(value: Any, *, default: float) -> float:
    if value is None or value == "":
        return default
    return float(value)


def _coerce_bool(value: Any, *, default: bool) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in {"1", "true", "yes", "on", "y"}:
        return True
    if text in {"0", "false", "no", "off", "n"}:
        return False
    return default


def _motion_engine_import_ready() -> None:
    motion_engine_root = str(MOTION_ENGINE_ROOT)
    if motion_engine_root not in sys.path:
        sys.path.insert(0, motion_engine_root)


def _fallback_calibration_frame() -> list[float]:
    return [
        DEFAULT_CALIBRATION_COXA_DEG,
        DEFAULT_CALIBRATION_FEMUR_DEG,
        DEFAULT_CALIBRATION_TIBIA_DEG,
    ] * len(CALIBRATION_LEG_LABELS)


def _default_calibration_frame() -> list[float]:
    global _DEFAULT_CALIBRATION_FRAME_CACHE
    if _DEFAULT_CALIBRATION_FRAME_CACHE is not None:
        return list(_DEFAULT_CALIBRATION_FRAME_CACHE)

    _motion_engine_import_ready()
    try:
        from runtime.runtime import build_default_stand_frame

        frame = [float(angle) for angle in build_default_stand_frame(stance_z=DEFAULT_RUNTIME_STANCE_Z_MM)]
        if len(frame) == len(CALIBRATION_LEG_LABELS) * len(CALIBRATION_JOINT_KEYS):
            _DEFAULT_CALIBRATION_FRAME_CACHE = frame
            return list(_DEFAULT_CALIBRATION_FRAME_CACHE)
    except Exception:
        pass

    _DEFAULT_CALIBRATION_FRAME_CACHE = _fallback_calibration_frame()
    return list(_DEFAULT_CALIBRATION_FRAME_CACHE)


def _frame_leg_angles(frame: Sequence[float], leg_index: int) -> dict[str, float]:
    start = leg_index * len(CALIBRATION_JOINT_KEYS)
    return {
        joint_key: float(frame[start + joint_offset])
        for joint_offset, joint_key in enumerate(CALIBRATION_JOINT_KEYS)
    }


def _angles_triplet(angles: Mapping[str, Any]) -> tuple[float, float, float]:
    return tuple(float(angles.get(joint_key, 0.0)) for joint_key in CALIBRATION_JOINT_KEYS)  # type: ignore[return-value]


def _angles_dict_from_values(values: Sequence[float]) -> dict[str, float]:
    return {
        joint_key: float(values[joint_offset])
        for joint_offset, joint_key in enumerate(CALIBRATION_JOINT_KEYS)
    }


def _servo_offsets_from_angles(leg_index: int, angles: Mapping[str, Any]) -> dict[str, float]:
    baseline = _frame_leg_angles(_default_calibration_frame(), leg_index)
    return {
        joint_key: float(_coerce_float(angles.get(joint_key), default=baseline[joint_key]) - baseline[joint_key])
        for joint_key in CALIBRATION_JOINT_KEYS
    }


def _default_leg_profile(leg_index: int) -> dict[str, Any]:
    angles = _frame_leg_angles(_default_calibration_frame(), leg_index)
    return {
        "leg_index": leg_index,
        "label": CALIBRATION_LEG_LABELS[leg_index],
        "angles_deg": dict(angles),
        "offsets_deg": {joint_key: 0.0 for joint_key in CALIBRATION_JOINT_KEYS},
        "saved": False,
        "updated_at": None,
    }


def _compute_leg_offsets(leg_index: int, angles: Mapping[str, Any]) -> dict[str, float]:
    _motion_engine_import_ready()
    from ik_engine.config import DEFAULT_CONFIG

    baseline = _frame_leg_angles(_default_calibration_frame(), leg_index)
    requested = _angles_triplet(angles)
    baseline_values = _angles_triplet(baseline)
    directions = DEFAULT_CONFIG.servo.direction.get(leg_index, (1, 1, 1))

    return {
        "coxa_deg": float(directions[0] * (requested[0] - baseline_values[0])),
        "femur_deg": float(directions[1] * (requested[1] - baseline_values[1])),
        "tibia_deg": float(-directions[2] * (requested[2] - baseline_values[2])),
    }


def _servo_safe_range_snapshot() -> dict[str, float]:
    _motion_engine_import_ready()
    from ik_engine.config import DEFAULT_CONFIG

    lo, hi = DEFAULT_CONFIG.servo.safe_range_deg
    return {
        "min_deg": float(lo),
        "max_deg": float(hi),
        "neutral_deg": float(DEFAULT_CONFIG.servo.neutral_deg),
        "pwm_min_us": int(DEFAULT_CONFIG.servo.pwm_min_us),
        "pwm_neutral_us": int(DEFAULT_CONFIG.servo.pwm_neutral_us),
        "pwm_max_us": int(DEFAULT_CONFIG.servo.pwm_max_us),
    }


def _normalize_leg_profile(raw_leg: Mapping[str, Any] | None, leg_index: int) -> dict[str, Any]:
    default_leg = _default_leg_profile(leg_index)
    raw_leg = raw_leg or {}
    raw_angles = raw_leg.get("angles_deg")
    if not isinstance(raw_angles, Mapping):
        raw_angles = {}

    requested_values = [
        _coerce_float(raw_angles.get(joint_key), default=default_leg["angles_deg"][joint_key])
        for joint_key in CALIBRATION_JOINT_KEYS
    ]

    _motion_engine_import_ready()
    from runtime.protocol import clamp_angle

    # Calibration defines the neutral offset; only the hard 0..180 servo range
    # applies here. The bench safety clamp is applied later, after offsets are
    # added, in ik_engine.leg_ik.ik_to_servo_angles.
    applied_values = [clamp_angle(value) for value in requested_values]
    applied_angles = _angles_dict_from_values(applied_values)
    saved = _coerce_bool(raw_leg.get("saved"), default=default_leg["saved"])

    return {
        "leg_index": leg_index,
        "label": CALIBRATION_LEG_LABELS[leg_index],
        "angles_deg": applied_angles,
        "offsets_deg": _compute_leg_offsets(leg_index, applied_angles) if saved else dict(default_leg["offsets_deg"]),
        "saved": saved,
        "updated_at": raw_leg.get("updated_at"),
    }


def _default_calibration_profile() -> dict[str, Any]:
    return {
        "version": 1,
        "name": "manual-calibration",
        "reference_stance_z_mm": DEFAULT_RUNTIME_STANCE_Z_MM,
        "legs": [_default_leg_profile(leg_index) for leg_index in range(len(CALIBRATION_LEG_LABELS))],
        "updated_at": None,
    }


def _normalize_calibration_profile(payload: Mapping[str, Any] | None) -> dict[str, Any]:
    default_profile = _default_calibration_profile()
    raw_legs = payload.get("legs") if isinstance(payload, Mapping) else None
    legs: list[dict[str, Any]] = []

    for leg_index in range(len(CALIBRATION_LEG_LABELS)):
        leg_payload: Mapping[str, Any] | None = None
        if isinstance(raw_legs, list) and leg_index < len(raw_legs) and isinstance(raw_legs[leg_index], Mapping):
            leg_payload = raw_legs[leg_index]
        elif isinstance(raw_legs, Mapping):
            maybe_leg = raw_legs.get(str(leg_index))
            if isinstance(maybe_leg, Mapping):
                leg_payload = maybe_leg
        legs.append(_normalize_leg_profile(leg_payload, leg_index))

    return {
        "version": int(payload.get("version", 1)) if isinstance(payload, Mapping) else 1,
        "name": str(payload.get("name") or default_profile["name"]) if isinstance(payload, Mapping) else default_profile["name"],
        "reference_stance_z_mm": DEFAULT_RUNTIME_STANCE_Z_MM,
        "legs": legs,
        "updated_at": payload.get("updated_at") if isinstance(payload, Mapping) else None,
    }


def _profile_full_frame(legs: Sequence[Mapping[str, Any]]) -> list[float]:
    frame: list[float] = []
    for leg_index in range(len(CALIBRATION_LEG_LABELS)):
        leg = legs[leg_index] if leg_index < len(legs) else _default_leg_profile(leg_index)
        frame.extend(_angles_triplet(leg.get("angles_deg") if isinstance(leg, Mapping) else {}))
    return frame


@dataclass(frozen=True)
class RuntimeLaunchConfig:
    port: str = DEFAULT_RUNTIME_PORT
    baudrate: int = DEFAULT_RUNTIME_BAUDRATE
    verbose: bool = True
    duration: float = 0.0
    cmd_scale: float = 1000.0
    timeout: float = DEFAULT_RUNTIME_TIMEOUT
    deadzone: int = 100
    stride: float = DEFAULT_RUNTIME_STRIDE_MM
    lift_scale: float = DEFAULT_RUNTIME_LIFT_SCALE
    frequency: float = DEFAULT_RUNTIME_FREQUENCY_HZ
    yaw_stride_deg: float = DEFAULT_RUNTIME_YAW_STRIDE_DEG
    stance_z: float = DEFAULT_RUNTIME_STANCE_Z_MM
    override_file: str = DEFAULT_OVERRIDE_FILE
    calibration_file: str = DEFAULT_CALIBRATION_FILE

    @classmethod
    def from_mapping(cls, payload: Mapping[str, Any] | None) -> "RuntimeLaunchConfig":
        data = payload or {}
        config = cls(
            port=_coerce_str(data.get("port"), default=DEFAULT_RUNTIME_PORT),
            baudrate=_coerce_int(data.get("baudrate"), default=DEFAULT_RUNTIME_BAUDRATE),
            verbose=_coerce_bool(data.get("verbose"), default=True),
            duration=_coerce_float(data.get("duration"), default=0.0),
            cmd_scale=_coerce_float(data.get("cmd_scale"), default=1000.0),
            timeout=_coerce_float(data.get("timeout"), default=DEFAULT_RUNTIME_TIMEOUT),
            deadzone=_coerce_int(data.get("deadzone"), default=100),
            stride=_coerce_float(data.get("stride"), default=DEFAULT_RUNTIME_STRIDE_MM),
            lift_scale=_coerce_float(data.get("lift_scale"), default=DEFAULT_RUNTIME_LIFT_SCALE),
            frequency=_coerce_float(data.get("frequency"), default=DEFAULT_RUNTIME_FREQUENCY_HZ),
            yaw_stride_deg=_coerce_float(data.get("yaw_stride_deg"), default=DEFAULT_RUNTIME_YAW_STRIDE_DEG),
            stance_z=_coerce_float(data.get("stance_z"), default=DEFAULT_RUNTIME_STANCE_Z_MM),
            override_file=_coerce_str(data.get("override_file"), default=DEFAULT_OVERRIDE_FILE),
            calibration_file=_coerce_str(data.get("calibration_file"), default=DEFAULT_CALIBRATION_FILE),
        )
        if not config.port:
            raise ValueError("runtime serial port must not be empty")
        if config.baudrate <= 0:
            raise ValueError("runtime baudrate must be positive")
        if config.duration < 0:
            raise ValueError("runtime duration must be non-negative")
        if config.cmd_scale <= 0:
            raise ValueError("runtime cmd_scale must be positive")
        if config.timeout < 0:
            raise ValueError("runtime timeout must be non-negative")
        if config.deadzone < 0:
            raise ValueError("runtime deadzone must be non-negative")
        if config.stride <= 0:
            raise ValueError("runtime stride must be positive")
        if config.lift_scale <= 0:
            raise ValueError("runtime lift_scale must be positive")
        if config.frequency <= 0:
            raise ValueError("runtime frequency must be positive")
        if config.yaw_stride_deg <= 0:
            raise ValueError("runtime yaw_stride_deg must be positive")
        if config.stance_z >= 0:
            raise ValueError("runtime stance_z must be negative")
        if not config.override_file:
            raise ValueError("runtime override_file must not be empty")
        if not config.calibration_file:
            raise ValueError("runtime calibration_file must not be empty")
        return config


@dataclass(frozen=True)
class CalibrationPoseConfig:
    port: str = DEFAULT_RUNTIME_PORT
    baudrate: int = DEFAULT_RUNTIME_BAUDRATE
    leg_index: int = 0
    coxa_deg: float = 0.0
    femur_deg: float = 0.0
    tibia_deg: float = 0.0

    @classmethod
    def from_mapping(cls, payload: Mapping[str, Any] | None) -> "CalibrationPoseConfig":
        data = payload or {}
        config = cls(
            port=_coerce_str(data.get("port"), default=DEFAULT_RUNTIME_PORT),
            baudrate=_coerce_int(data.get("baudrate"), default=DEFAULT_RUNTIME_BAUDRATE),
            leg_index=_coerce_int(data.get("leg_index"), default=0),
            coxa_deg=_coerce_float(data.get("coxa_deg"), default=0.0),
            femur_deg=_coerce_float(data.get("femur_deg"), default=0.0),
            tibia_deg=_coerce_float(data.get("tibia_deg"), default=0.0),
        )
        if not config.port:
            raise ValueError("calibration serial port must not be empty")
        if config.baudrate <= 0:
            raise ValueError("calibration baudrate must be positive")
        if not 0 <= config.leg_index < len(CALIBRATION_LEG_LABELS):
            raise ValueError(f"leg_index must be between 0 and {len(CALIBRATION_LEG_LABELS) - 1}")
        for joint_name, angle in (
            ("coxa_deg", config.coxa_deg),
            ("femur_deg", config.femur_deg),
            ("tibia_deg", config.tibia_deg),
        ):
            if not -CALIBRATION_DRAFT_LIMIT_DEG <= angle <= CALIBRATION_DRAFT_LIMIT_DEG:
                raise ValueError(
                    f"{joint_name} must be between {-CALIBRATION_DRAFT_LIMIT_DEG:.0f} "
                    f"and {CALIBRATION_DRAFT_LIMIT_DEG:.0f}"
                )
        return config


def _draft_offsets_from_config(config: CalibrationPoseConfig) -> dict[str, float]:
    return {
        "coxa_deg": float(config.coxa_deg),
        "femur_deg": float(config.femur_deg),
        "tibia_deg": float(config.tibia_deg),
    }


def _compose_calibration_preview(
    config: CalibrationPoseConfig,
    *,
    legs: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    _motion_engine_import_ready()
    from runtime.protocol import clamp_angle

    saved_leg = legs[config.leg_index] if config.leg_index < len(legs) else _default_leg_profile(config.leg_index)
    baseline_angles = _frame_leg_angles(_default_calibration_frame(), config.leg_index)
    saved_angles = _angles_dict_from_values(_angles_triplet(saved_leg.get("angles_deg") if isinstance(saved_leg, Mapping) else {}))
    draft_offsets = _draft_offsets_from_config(config)
    requested_angles = {
        joint_key: saved_angles[joint_key] + draft_offsets[joint_key]
        for joint_key in CALIBRATION_JOINT_KEYS
    }
    applied_angles = {
        joint_key: clamp_angle(requested_angles[joint_key])
        for joint_key in CALIBRATION_JOINT_KEYS
    }
    preview_legs = [dict(leg) for leg in legs]
    preview_legs[config.leg_index] = {
        **saved_leg,
        "angles_deg": dict(applied_angles),
    }
    saved_servo_offsets = _servo_offsets_from_angles(config.leg_index, saved_angles)
    applied_servo_offsets = _servo_offsets_from_angles(config.leg_index, applied_angles)
    return {
        "active_leg_index": config.leg_index,
        "baseline_angles_deg": dict(baseline_angles),
        "saved_angles_deg": dict(saved_angles),
        "saved_servo_offsets_deg": saved_servo_offsets,
        "saved_runtime_offsets_deg": _compute_leg_offsets(config.leg_index, saved_angles),
        "draft_offsets_deg": dict(draft_offsets),
        "requested_angles_deg": requested_angles,
        "applied_angles": applied_angles,
        "applied_servo_offsets_deg": applied_servo_offsets,
        "applied_runtime_offsets_deg": _compute_leg_offsets(config.leg_index, applied_angles),
        "applied_frame": _profile_full_frame(preview_legs),
    }


def send_calibration_pose(
    config: CalibrationPoseConfig,
    *,
    legs: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    """Send a six-leg manual command while only one leg is actively edited.

    Uses the canonical production serializer so calibration preview lands
    on the wire byte-for-byte identical to what the runtime bridge sends.
    """
    _motion_engine_import_ready()
    try:
        from runtime.protocol import format_full_dir_command
        from runtime.runtime import SerialBridge
    except Exception as exc:  # pragma: no cover - depends on target Pi env
        raise RuntimeError(f"unable to import motion runtime serial bridge: {exc}") from exc

    preview = _compose_calibration_preview(config, legs=legs)
    full_frame = list(preview["applied_frame"])
    command_line = format_full_dir_command("N", full_frame)
    command_text = command_line.rstrip("\n")

    with SerialBridge(port=config.port, baudrate=config.baudrate) as bridge:
        bridge.write_line(command_line)

    return {
        **preview,
        "command_text": command_text,
    }


@dataclass(frozen=True)
class CameraLaunchConfig:
    mode: str = "detect"
    host: str = DEFAULT_CAMERA_HOST
    port: int = DEFAULT_CAMERA_PORT
    detector: str = "apriltag"

    @classmethod
    def from_mapping(cls, payload: Mapping[str, Any] | None) -> "CameraLaunchConfig":
        data = payload or {}
        config = cls(
            mode=_coerce_str(data.get("mode"), default="detect").lower(),
            host=_coerce_str(data.get("host"), default=DEFAULT_CAMERA_HOST),
            port=_coerce_int(data.get("port"), default=DEFAULT_CAMERA_PORT),
            detector=_coerce_str(data.get("detector"), default="apriltag").lower(),
        )
        if config.mode not in {"camera", "detect"}:
            raise ValueError("camera mode must be 'camera' or 'detect'")
        if not config.host:
            raise ValueError("camera host must not be empty")
        if config.port <= 0:
            raise ValueError("camera port must be positive")
        if not config.detector:
            raise ValueError("camera detector must not be empty")
        return config


def build_runtime_command(
    config: RuntimeLaunchConfig,
    python_executable: str | None = None,
) -> list[str]:
    python_exec = python_executable or sys.executable or "python3"
    command = [
        python_exec,
        "-m",
        "runtime.main",
        "--port",
        config.port,
        "--baudrate",
        str(config.baudrate),
        "--calibration-file",
        config.calibration_file,
    ]
    if config.verbose:
        command.append("--verbose")
    command.extend(
        [
            "gait-leg-bridge",
            "--duration",
            str(config.duration),
            "--cmd-scale",
            str(config.cmd_scale),
            "--timeout",
            str(config.timeout),
            "--deadzone",
            str(config.deadzone),
            "--stride",
            str(config.stride),
            "--lift-scale",
            str(config.lift_scale),
            "--frequency",
            str(config.frequency),
            "--yaw-stride-deg",
            str(config.yaw_stride_deg),
            "--stance-z",
            str(config.stance_z),
            "--override-file",
            config.override_file,
        ]
    )
    return command


@dataclass(frozen=True)
class XboxControllerLaunchConfig:
    override_file: str = DEFAULT_OVERRIDE_FILE
    status_file: str = DEFAULT_XBOX_STATUS_FILE
    device: str = ""

    @classmethod
    def from_mapping(cls, payload: Mapping[str, Any] | None) -> "XboxControllerLaunchConfig":
        data = payload or {}
        return cls(
            override_file=_coerce_str(data.get("override_file"), default=DEFAULT_OVERRIDE_FILE),
            status_file=_coerce_str(data.get("status_file"), default=DEFAULT_XBOX_STATUS_FILE),
            device=_coerce_str(data.get("device"), default=""),
        )


def build_xbox_command(
    config: XboxControllerLaunchConfig,
    python_executable: str | None = None,
) -> list[str]:
    python_exec = python_executable or sys.executable or "python3"
    command = [
        python_exec,
        "-m",
        "runtime.xbox_controller",
        "--override-file",
        config.override_file,
        "--status-file",
        config.status_file,
    ]
    if config.device:
        command.extend(["--device", config.device])
    return command


def read_xbox_status(status_file: str = DEFAULT_XBOX_STATUS_FILE) -> dict[str, Any]:
    """Read the Xbox controller status JSON file."""
    try:
        data = json.loads(Path(status_file).read_text(encoding="utf-8"))
        updated_at = data.get("updated_at")
        if updated_at is not None and time.time() - float(updated_at) > 5.0:
            data["connected"] = False
        data.setdefault("calibration_adjust", 0)
        data.setdefault("buttons", {
            "a": False,
            "b": False,
            "x": False,
            "y": False,
            "lb": False,
            "rb": False,
        })
        data.setdefault("button_seq", {
            "a": 0,
            "b": 0,
            "x": 0,
            "y": 0,
            "lb": 0,
            "rb": 0,
        })
        return data
    except (FileNotFoundError, OSError, json.JSONDecodeError):
        return {
            "connected": False,
            "device_name": "",
            "connection_type": "none",
            "battery": None,
            "forward_cmd": 0,
            "strafe_cmd": 0,
            "turn_cmd": 0,
            "calibration_adjust": 0,
            "buttons": {
                "a": False,
                "b": False,
                "x": False,
                "y": False,
                "lb": False,
                "rb": False,
            },
            "button_seq": {
                "a": 0,
                "b": 0,
                "x": 0,
                "y": 0,
                "lb": 0,
                "rb": 0,
            },
            "updated_at": None,
        }


def build_camera_command(
    config: CameraLaunchConfig,
    python_executable: str | None = None,
) -> list[str]:
    python_exec = python_executable or sys.executable or "python3"
    command = [
        python_exec,
        "live.py",
        config.mode,
        "--host",
        config.host,
        "--port",
        str(config.port),
    ]
    if config.mode == "detect":
        command.extend(["--detector", config.detector])
    return command


def _command_for_mode(mode: str, magnitude: int) -> dict[str, int]:
    amount = max(0, min(1000, int(magnitude)))
    direction = str(mode).upper()
    if direction not in OVERRIDE_MODES:
        raise ValueError(f"override mode must be one of {OVERRIDE_MODES}")
    if direction == "F":
        return {"forward_cmd": amount, "strafe_cmd": 0, "turn_cmd": 0}
    if direction == "B":
        return {"forward_cmd": -amount, "strafe_cmd": 0, "turn_cmd": 0}
    if direction == "L":
        return {"forward_cmd": 0, "strafe_cmd": amount, "turn_cmd": 0}
    if direction == "R":
        return {"forward_cmd": 0, "strafe_cmd": -amount, "turn_cmd": 0}
    if direction == "LL":
        return {"forward_cmd": 0, "strafe_cmd": 0, "turn_cmd": amount}
    if direction == "RR":
        return {"forward_cmd": 0, "strafe_cmd": 0, "turn_cmd": -amount}
    return {"forward_cmd": 0, "strafe_cmd": 0, "turn_cmd": 0}


class OverrideStore:
    """Atomic JSON file used by the dashboard to override UART commands."""

    def __init__(self, path: str | Path = DEFAULT_OVERRIDE_FILE):
        self.path = Path(path)
        self._lock = threading.Lock()

    def _default_payload(self) -> dict[str, Any]:
        return {
            "active": False,
            "mode": "N",
            "forward_cmd": 0,
            "strafe_cmd": 0,
            "turn_cmd": 0,
            "seq": 0,
            "magnitude": 0,
            "source": "web",
            "updated_at": time.time(),
            "expires_at": None,
        }

    def _write_locked(self, payload: dict[str, Any]) -> dict[str, Any]:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temp_path = self.path.with_suffix(f"{self.path.suffix}.tmp")
        temp_path.write_text(json.dumps(payload), encoding="utf-8")
        temp_path.replace(self.path)
        return payload

    def _snapshot_from_raw(self, data: dict[str, Any]) -> dict[str, Any]:
        return {
            "path": str(self.path),
            "active": bool(data.get("active")),
            "mode": str(data.get("mode") or "N"),
            "magnitude": int(data.get("magnitude") or 0),
            "forward_cmd": int(data.get("forward_cmd") or 0),
            "strafe_cmd": int(data.get("strafe_cmd") or 0),
            "turn_cmd": int(data.get("turn_cmd") or 0),
            "source": str(data.get("source") or "web"),
            "updated_at": data.get("updated_at"),
            "expires_at": data.get("expires_at"),
        }

    def read_raw(self) -> dict[str, Any]:
        try:
            data = json.loads(self.path.read_text(encoding="utf-8"))
        except (FileNotFoundError, IsADirectoryError, OSError, json.JSONDecodeError):
            data = self._default_payload()
        expires_at = data.get("expires_at")
        if data.get("active") and expires_at is not None:
            try:
                if float(expires_at) <= time.time():
                    data = self._default_payload()
            except (TypeError, ValueError):
                data = self._default_payload()
        return data

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            data = self.read_raw()
            return self._snapshot_from_raw(data)

    def set_command(
        self,
        mode: str,
        *,
        magnitude: int = 700,
        expires_after: float = 0.7,
        source: str = "web",
    ) -> dict[str, Any]:
        commands = _command_for_mode(mode, magnitude)
        now = time.time()
        payload = {
            "active": True,
            "mode": str(mode).upper(),
            "magnitude": max(0, min(1000, int(magnitude))),
            "source": source,
            "updated_at": now,
            "expires_at": now + max(0.1, float(expires_after)),
            "seq": 0,
            **commands,
        }
        with self._lock:
            self._write_locked(payload)
            return self._snapshot_from_raw(payload)

    def clear(self) -> dict[str, Any]:
        with self._lock:
            payload = self._default_payload()
            self._write_locked(payload)
            return self._snapshot_from_raw(payload)


class CalibrationStore:
    """Persistent six-leg calibration profile plus last-send status."""

    def __init__(self, path: str | Path = DEFAULT_CALIBRATION_FILE) -> None:
        self.path = Path(path)
        self._lock = threading.Lock()
        self._profile = self._load_profile_locked()
        self._config = CalibrationPoseConfig(
            leg_index=0,
        )
        self._applied_angles = dict(self._profile["legs"][0]["angles_deg"])
        self._applied_frame = _profile_full_frame(self._profile["legs"])
        self._command_text = ""
        self._updated_at: float | None = None
        self._active = False
        self._runtime_stopped = False
        self._xbox_stopped = False

    def _read_profile_from_disk_locked(self) -> dict[str, Any]:
        try:
            payload = json.loads(self.path.read_text(encoding="utf-8"))
        except (FileNotFoundError, IsADirectoryError, OSError, json.JSONDecodeError):
            payload = None
        return _normalize_calibration_profile(payload)

    def _write_profile_locked(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temp_path = self.path.with_suffix(f"{self.path.suffix}.tmp")
        temp_path.write_text(json.dumps(self._profile, indent=2), encoding="utf-8")
        temp_path.replace(self.path)

    def _load_profile_locked(self) -> dict[str, Any]:
        profile = self._read_profile_from_disk_locked()
        self._profile = profile
        self._write_profile_locked()
        return profile

    def _default_config_locked(self) -> dict[str, Any]:
        return asdict(
            CalibrationPoseConfig(
                leg_index=0,
            )
        )

    def _selected_leg_snapshot_locked(self) -> dict[str, Any]:
        leg_index = int(self._config.leg_index)
        leg_profile = self._profile["legs"][leg_index]
        saved_angles = _angles_dict_from_values(_angles_triplet(leg_profile.get("angles_deg")))
        return {
            "baseline_angles_deg": _frame_leg_angles(_default_calibration_frame(), leg_index),
            "saved_angles_deg": saved_angles,
            "saved_servo_offsets_deg": _servo_offsets_from_angles(leg_index, saved_angles),
            "saved_runtime_offsets_deg": _compute_leg_offsets(leg_index, saved_angles),
            "draft_offsets_deg": _draft_offsets_from_config(self._config),
            "applied_angles": dict(self._applied_angles),
            "applied_servo_offsets_deg": _servo_offsets_from_angles(leg_index, self._applied_angles),
            "applied_runtime_offsets_deg": _compute_leg_offsets(leg_index, self._applied_angles),
            "servo_safe_range_deg": _servo_safe_range_snapshot(),
        }

    def _snapshot_locked(self) -> dict[str, Any]:
        saved_count = sum(1 for leg in self._profile["legs"] if leg.get("saved"))
        leg_snapshot = self._selected_leg_snapshot_locked()
        return {
            "name": "manual-calibration",
            "active": self._active,
            "config": asdict(self._config),
            "default_config": self._default_config_locked(),
            "profile_path": str(self.path),
            "legs": [dict(leg) for leg in self._profile["legs"]],
            "default_frame": _default_calibration_frame(),
            "saved_count": saved_count,
            "complete": saved_count == len(CALIBRATION_LEG_LABELS),
            **leg_snapshot,
            "applied_frame": list(self._applied_frame),
            "command_text": self._command_text,
            "updated_at": self._updated_at,
            "profile_updated_at": self._profile.get("updated_at"),
            "runtime_stopped": self._runtime_stopped,
            "xbox_stopped": self._xbox_stopped,
        }

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return self._snapshot_locked()

    def preview_legs(self, config: CalibrationPoseConfig) -> list[dict[str, Any]]:
        with self._lock:
            return [dict(leg) for leg in self._profile["legs"]]

    def save_leg(self, leg_index: int, angles: Mapping[str, Any]) -> dict[str, Any]:
        with self._lock:
            leg_profile = _normalize_leg_profile(
                {
                    "angles_deg": dict(angles),
                    "saved": True,
                    "updated_at": time.time(),
                },
                leg_index,
            )
            self._profile["legs"][leg_index] = leg_profile
            self._profile["updated_at"] = time.time()
            self._write_profile_locked()
            return dict(leg_profile)

    def record_send(
        self,
        config: CalibrationPoseConfig,
        send_result: Mapping[str, Any],
        *,
        runtime_stopped: bool,
        xbox_stopped: bool,
    ) -> dict[str, Any]:
        with self._lock:
            self._config = config
            self._applied_angles = dict(send_result.get("applied_angles") or self._applied_angles)
            applied_frame = send_result.get("applied_frame")
            if isinstance(applied_frame, list):
                self._applied_frame = [float(value) for value in applied_frame]
            self._command_text = str(send_result.get("command_text") or "")
            self._updated_at = time.time()
            self._active = True
            self._runtime_stopped = runtime_stopped
            self._xbox_stopped = xbox_stopped
            return self._snapshot_locked()

    def clear(self) -> dict[str, Any]:
        with self._lock:
            self._config = CalibrationPoseConfig(leg_index=0)
            self._applied_angles = dict(self._profile["legs"][0]["angles_deg"])
            self._applied_frame = _profile_full_frame(self._profile["legs"])
            self._command_text = ""
            self._updated_at = None
            self._active = False
            self._runtime_stopped = False
            self._xbox_stopped = False
            return self._snapshot_locked()

    def reset_profile(self) -> dict[str, Any]:
        with self._lock:
            try:
                self.path.unlink()
            except FileNotFoundError:
                pass
            except OSError:
                pass
            self._profile = _normalize_calibration_profile(None)
            self._applied_angles = dict(self._profile["legs"][0]["angles_deg"])
            self._applied_frame = _profile_full_frame(self._profile["legs"])
            self._command_text = ""
            self._updated_at = None
            return self._snapshot_locked()


class ManagedProcessManager:
    """Launch and supervise one named subprocess with recent log retention."""

    def __init__(
        self,
        name: str,
        *,
        cwd: Path,
        default_config: Any,
        command_builder: Callable[[Any, str | None], list[str]],
        python_executable: str | None = None,
        popen_factory: Callable[..., subprocess.Popen[str]] = subprocess.Popen,
        max_log_lines: int = MAX_LOG_LINES,
        register_atexit: bool = True,
    ):
        self.name = name
        self.cwd = Path(cwd)
        self.default_config = default_config
        self.python_executable = python_executable or sys.executable or "python3"
        self._command_builder = command_builder
        self._popen_factory = popen_factory
        self._logs: deque[str] = deque(maxlen=max_log_lines)
        self._uart_tx: deque[tuple[float, str]] = deque(maxlen=MAX_UART_LINES)
        self._uart_rx: deque[tuple[float, str]] = deque(maxlen=MAX_UART_LINES)
        self._lock = threading.Lock()
        self._proc: subprocess.Popen[str] | None = None
        self._last_command: list[str] = []
        self._last_config = default_config
        self._last_exit_code: int | None = None
        self._started_at: float | None = None
        self._stopped_at: float | None = None
        if register_atexit:
            atexit.register(self.shutdown)

    def _append_log_locked(self, line: str) -> None:
        text = line.rstrip("\r\n")
        if not text:
            return
        if text.startswith(UART_TX_PREFIX):
            payload = text[len(UART_TX_PREFIX):].lstrip()
            self._uart_tx.append((time.time(), payload))
            return
        if text.startswith(UART_RX_PREFIX):
            payload = text[len(UART_RX_PREFIX):].lstrip()
            self._uart_rx.append((time.time(), payload))
            return
        self._logs.append(text)

    def _refresh_locked(self) -> None:
        if self._proc is None:
            return
        exit_code = self._proc.poll()
        if exit_code is None:
            return
        self._last_exit_code = exit_code
        self._stopped_at = time.time()
        self._append_log_locked(f"[web] {self.name} exited with code {exit_code}")
        self._proc = None

    def _collect_output(self, proc: subprocess.Popen[str]) -> None:
        stdout = proc.stdout
        if stdout is None:
            return
        try:
            for raw_line in stdout:
                with self._lock:
                    self._append_log_locked(raw_line)
        finally:
            try:
                stdout.close()
            except Exception:
                pass
            with self._lock:
                if self._proc is proc:
                    self._refresh_locked()

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            self._refresh_locked()
            running = self._proc is not None
            pid = None if self._proc is None else self._proc.pid
            return {
                "name": self.name,
                "running": running,
                "pid": pid,
                "cwd": str(self.cwd),
                "command": list(self._last_command),
                "command_text": shlex.join(self._last_command) if self._last_command else "",
                "config": asdict(self._last_config),
                "default_config": asdict(self.default_config),
                "last_exit_code": self._last_exit_code,
                "started_at": self._started_at,
                "stopped_at": self._stopped_at,
                "logs": list(self._logs),
                "uart_tx": [{"t": ts, "line": line} for ts, line in self._uart_tx],
                "uart_rx": [{"t": ts, "line": line} for ts, line in self._uart_rx],
            }

    def uart_snapshot(self) -> dict[str, Any]:
        """Lightweight snapshot of just the UART TX/RX ring buffers."""
        with self._lock:
            return {
                "running": self._proc is not None,
                "uart_tx": [{"t": ts, "line": line} for ts, line in self._uart_tx],
                "uart_rx": [{"t": ts, "line": line} for ts, line in self._uart_rx],
            }

    def start(self, config: Any | None = None) -> dict[str, Any]:
        launch_config = config or self.default_config
        command = self._command_builder(launch_config, self.python_executable)
        env = os.environ.copy()
        env["PYTHONUNBUFFERED"] = "1"

        with self._lock:
            self._refresh_locked()
            if self._proc is not None:
                raise RuntimeError(f"{self.name} is already running")
            self._append_log_locked(f"[web] starting {self.name}: {shlex.join(command)}")
            proc = self._popen_factory(
                command,
                cwd=str(self.cwd),
                env=env,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
            self._proc = proc
            self._last_command = list(command)
            self._last_config = launch_config
            self._last_exit_code = None
            self._started_at = time.time()
            self._stopped_at = None
            reader = threading.Thread(
                target=self._collect_output,
                args=(proc,),
                name=f"{self.name}-log-reader",
                daemon=True,
            )
            reader.start()
        return self.snapshot()

    def stop(self) -> dict[str, Any]:
        with self._lock:
            self._refresh_locked()
            proc = self._proc
            if proc is not None:
                self._append_log_locked(f"[web] stopping {self.name} (pid={proc.pid})")

        if proc is None:
            return self.snapshot()

        try:
            proc.send_signal(signal.SIGINT)
            proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            proc.terminate()
            try:
                proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                proc.kill()
                try:
                    proc.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    pass
        except ProcessLookupError:
            pass

        return self.snapshot()

    def shutdown(self) -> None:
        try:
            self.stop()
        except Exception:
            pass
