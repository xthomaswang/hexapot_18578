"""Thin Raspberry Pi runtime layer for serial motion control."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

from ik_engine.calibration import apply_to_servo_config, load_profile
from ik_engine.config import DEFAULT_CONFIG, build_hexapod_config
from motion import MotionController, SingleLegMotionController
from .protocol import (
    BAUD_RATE,
    DEFAULT_SERIAL_PORT,
    JOINTS_PER_LEG,
    MOVE_DIR_NEUTRAL,
    NUM_LEGS,
    NUM_SERVOS,
    UPDATE_INTERVAL_S,
    clamp_angle,
    clamp_bench_leg_angle,
    format_full_dir_command,
    format_home_command,
    format_move_command,
    format_move_dir_command,
    format_query_command,
    format_speed_command,
    format_stop_command,
    normalize_move_dir,
)

try:
    import serial
except ImportError:  # pragma: no cover - depends on target Pi environment
    serial = None


DEFAULT_COMMAND_TIMEOUT_S = 0.5
DEFAULT_GAIT_STRIDE_MM = 160.0
DEFAULT_GAIT_LIFT_SCALE = 0.55
DEFAULT_GAIT_FREQUENCY_HZ = 0.45
DEFAULT_GAIT_YAW_STRIDE_DEG = 10.0
DEFAULT_GAIT_STANCE_Z_MM = -495.0
DEFAULT_CALIBRATION_FILE = str(Path(__file__).resolve().parents[2] / "motion_engine" / "_cache" / "calibration_profile.json")


@dataclass(frozen=True)
class ControlCommand:
    """High-level manual command forwarded by the robot ESP."""

    forward_cmd: int = 0
    strafe_cmd: int = 0
    turn_cmd: int = 0
    seq: int = 0


@dataclass(frozen=True)
class ControlOverride:
    """Higher-priority control command injected outside the UART uplink path."""

    command: ControlCommand
    source: str = "web"
    mode: str = MOVE_DIR_NEUTRAL


def apply_deadzone(value: int, threshold: int) -> int:
    """Zero out small joystick noise around center."""
    if threshold < 0:
        raise ValueError("threshold must be non-negative")
    return 0 if abs(value) < threshold else value


def apply_command_deadzone(command: ControlCommand, threshold: int) -> ControlCommand:
    """Apply the same deadzone to forward/strafe/turn command channels."""
    return ControlCommand(
        forward_cmd=apply_deadzone(command.forward_cmd, threshold),
        strafe_cmd=apply_deadzone(command.strafe_cmd, threshold),
        turn_cmd=apply_deadzone(command.turn_cmd, threshold),
        seq=command.seq,
    )


def describe_motion(command: ControlCommand) -> str:
    """Human-readable dominant motion label for debug logs."""
    forward_mag = abs(command.forward_cmd)
    strafe_mag = abs(command.strafe_cmd)
    turn_mag = abs(command.turn_cmd)

    if forward_mag == 0 and strafe_mag == 0 and turn_mag == 0:
        return "N"

    if turn_mag > max(forward_mag, strafe_mag):
        return "LL" if command.turn_cmd > 0 else "RR"

    if forward_mag >= strafe_mag and forward_mag > 0:
        return "F" if command.forward_cmd > 0 else "B"

    if strafe_mag > 0:
        return "L" if command.strafe_cmd > 0 else "R"

    return "N"


def parse_control_command(line: str) -> ControlCommand | None:
    """Parse canonical numeric or legacy symbolic ``CMD`` lines from the robot ESP."""
    parts = line.strip().split()
    if not parts or parts[0] != "CMD":
        return None

    if len(parts) == 5:
        try:
            return ControlCommand(
                forward_cmd=int(parts[1]),
                strafe_cmd=int(parts[2]),
                turn_cmd=int(parts[3]),
                seq=int(parts[4]),
            )
        except ValueError:
            return None

    if len(parts) != 6:
        return None

    move_dir = parts[1].upper()
    turn_dir = parts[3].upper()

    try:
        move_mag = max(0, min(1000, int(parts[2])))
        turn_mag = max(0, min(1000, int(parts[4])))
        seq = int(parts[5])
    except ValueError:
        return None

    forward_cmd = 0
    strafe_cmd = 0
    turn_cmd = 0

    if move_dir == "F":
        forward_cmd = move_mag
    elif move_dir == "B":
        forward_cmd = -move_mag
    elif move_dir == "L":
        strafe_cmd = move_mag
    elif move_dir == "R":
        strafe_cmd = -move_mag
    elif move_dir != "N":
        return None

    if turn_dir == "LL":
        turn_cmd = turn_mag
    elif turn_dir == "RR":
        turn_cmd = -turn_mag
    elif turn_dir != "N":
        return None

    return ControlCommand(
        forward_cmd=forward_cmd,
        strafe_cmd=strafe_cmd,
        turn_cmd=turn_cmd,
        seq=seq,
    )


def read_control_override(override_file: str | None) -> ControlOverride | None:
    """Read an optional higher-priority control override from disk."""
    if not override_file:
        return None

    try:
        payload = json.loads(Path(override_file).read_text(encoding="utf-8"))
    except (FileNotFoundError, IsADirectoryError, OSError, json.JSONDecodeError):
        return None

    if not payload.get("active"):
        return None

    try:
        expires_at = payload.get("expires_at")
        if expires_at is not None and float(expires_at) <= time.time():
            return None

        command = ControlCommand(
            forward_cmd=int(payload.get("forward_cmd", 0)),
            strafe_cmd=int(payload.get("strafe_cmd", 0)),
            turn_cmd=int(payload.get("turn_cmd", 0)),
            seq=int(payload.get("seq", 0)),
        )
    except (TypeError, ValueError):
        return None

    source = str(payload.get("source") or "web")
    mode = str(payload.get("mode") or describe_motion(command)).upper()
    return ControlOverride(command=command, source=source, mode=mode)


def format_angle_triplet_command(angles: Sequence[float], seq: int = 0) -> str:
    """Temporary compatibility command for the current robot ESP firmware."""
    if len(angles) != 3:
        raise ValueError(f"expected 3 angles, got {len(angles)}")
    a0, a1, a2 = (int(round(angle)) for angle in angles)
    return f"ANG {a0} {a1} {a2} {int(seq)}\n"


def normalize_control(value: int, scale: float) -> float:
    """Map signed joystick command values to ``[-1.0, 1.0]``."""
    if scale <= 0:
        raise ValueError("scale must be positive")
    return max(-1.0, min(1.0, float(value) / float(scale)))


def build_motion_controller(
    *,
    stride: float = DEFAULT_GAIT_STRIDE_MM,
    lift_scale: float = DEFAULT_GAIT_LIFT_SCALE,
    frequency: float = DEFAULT_GAIT_FREQUENCY_HZ,
    yaw_stride_deg: float = DEFAULT_GAIT_YAW_STRIDE_DEG,
    stance_z: float = DEFAULT_GAIT_STANCE_Z_MM,
    calibration_file: str | None = None,
) -> MotionController:
    profile = load_profile(calibration_file)
    servo = apply_to_servo_config(profile, DEFAULT_CONFIG.servo)

    return MotionController(
        config=build_hexapod_config(default_foot_z=stance_z, servo=servo),
        stride=stride,
        lift_scale=lift_scale,
        frequency=frequency,
        yaw_stride_deg=yaw_stride_deg,
    )


def build_default_stand_frame(*, stance_z: float = DEFAULT_GAIT_STANCE_Z_MM) -> list[float]:
    """Nominal uncalibrated standing pose used as the calibration reference."""
    controller = MotionController(config=build_hexapod_config(default_foot_z=stance_z))
    controller.stand()
    return [float(angle) for angle in controller.get_angles()]


class SerialBridge:
    """Small wrapper around the Pi <-> ESP32 UART link."""

    def __init__(
        self,
        port: str = DEFAULT_SERIAL_PORT,
        baudrate: int = BAUD_RATE,
        timeout: float = 0.1,
    ):
        if serial is None:
            raise RuntimeError("pyserial is required on the Raspberry Pi")
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._serial = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        self._rx_buffer = bytearray()

    def close(self) -> None:
        self._serial.close()

    def __enter__(self) -> "SerialBridge":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def write_line(self, line: str) -> None:
        self._serial.write(line.encode("ascii"))

    def read_line(self) -> str:
        return self._serial.readline().decode("ascii", errors="replace").strip()

    def poll_lines(self) -> list[str]:
        """Read all complete UART lines currently available without blocking."""
        waiting = self._serial.in_waiting
        if waiting <= 0:
            return []

        self._rx_buffer.extend(self._serial.read(waiting))
        chunks = self._rx_buffer.split(b"\n")
        self._rx_buffer = bytearray(chunks.pop() if chunks else b"")
        return [
            chunk.rstrip(b"\r").decode("ascii", errors="replace")
            for chunk in chunks
            if chunk.strip()
        ]

    def transact(self, line: str, *, wait_for_reply: bool = False) -> str | None:
        self.write_line(line)
        if not wait_for_reply:
            return None
        return self.read_line()

    def send_angles(
        self,
        angles: list[float] | tuple[float, ...],
        *,
        wait_for_reply: bool = False,
    ) -> str | None:
        return self.transact(
            format_move_command(angles),
            wait_for_reply=wait_for_reply,
        )

    def send_leg_angles(
        self,
        angles: list[float] | tuple[float, ...],
        *,
        move_dir: str = MOVE_DIR_NEUTRAL,
        leg_count: int = 1,
        wait_for_reply: bool = False,
    ) -> str | None:
        return self.transact(
            format_move_dir_command(move_dir, angles, leg_count=leg_count),
            wait_for_reply=wait_for_reply,
        )

    def home(self, *, wait_for_reply: bool = False) -> str | None:
        return self.transact(format_home_command(), wait_for_reply=wait_for_reply)

    def stop(self, *, wait_for_reply: bool = False) -> str | None:
        return self.transact(format_stop_command(), wait_for_reply=wait_for_reply)

    def set_speed(self, percent: int, *, wait_for_reply: bool = False) -> str | None:
        return self.transact(
            format_speed_command(percent),
            wait_for_reply=wait_for_reply,
        )

    def query(self) -> str:
        response = self.transact(format_query_command(), wait_for_reply=True)
        return "" if response is None else response

    def send_angle_triplet(
        self,
        angles: Sequence[float],
        *,
        seq: int = 0,
        wait_for_reply: bool = False,
    ) -> str | None:
        return self.transact(
            format_angle_triplet_command(angles, seq=seq),
            wait_for_reply=wait_for_reply,
        )


class RuntimeMotionCore:
    """Pure-state hexapod motion core.

    Owns the ``MotionController`` and exposes ``stand()`` / ``tick()`` /
    ``current_angles()`` as the narrow interface every runtime bridge
    (``FullGaitLegBridgeMotion``, ``ManualBridgeMotion``) and CLI subcommand
    (``runtime.main stand``/``walk``) builds on top of.

    The core does NOT own a serial link. All UART I/O happens in the
    containing bridge/CLI layer; the core only mutates internal state and
    returns the resulting 18-angle frame. This decoupling makes the core
    unit-testable without a fake bridge and lets the same core drive both
    the manual/automated production path and the ``stand``/``walk`` CLIs.
    """

    def __init__(
        self,
        controller: MotionController | None = None,
        dt: float = UPDATE_INTERVAL_S,
    ):
        self.controller = controller or MotionController()
        self.dt = dt

    def stand(self) -> list[float]:
        self.controller.stand()
        return self.current_angles()

    def tick(
        self,
        *,
        forward: float = 0.0,
        strafe: float = 0.0,
        turn: float = 0.0,
        dt: float | None = None,
    ) -> list[float]:
        self.controller.update(
            forward=forward,
            strafe=strafe,
            turn=turn,
            dt=dt or self.dt,
        )
        return self.current_angles()

    def current_angles(self) -> list[float]:
        """Return the latest 18-angle servo frame without advancing state."""
        return list(self.controller.get_angles())


class SingleLegMotion:
    """Drive a single-leg bench-test controller over UART."""

    def __init__(
        self,
        bridge: SerialBridge,
        controller: SingleLegMotionController | None = None,
        dt: float = UPDATE_INTERVAL_S,
    ):
        self.bridge = bridge
        self.controller = controller or SingleLegMotionController()
        self.dt = dt

    def stand(self, *, wait_for_reply: bool = False) -> str | None:
        self.controller.stand()
        return self.bridge.send_leg_angles(
            self.controller.get_angles(),
            wait_for_reply=wait_for_reply,
        )

    def move_to(
        self,
        *,
        x: float,
        y: float,
        z: float,
        wait_for_reply: bool = False,
    ) -> str | None:
        self.controller.move_to(x=x, y=y, z=z)
        return self.bridge.send_leg_angles(
            self.controller.get_angles(),
            wait_for_reply=wait_for_reply,
        )

    def tick(self, *, wait_for_reply: bool = False) -> str | None:
        self.controller.update(dt=self.dt)
        return self.bridge.send_leg_angles(
            self.controller.get_angles(),
            wait_for_reply=wait_for_reply,
        )

    def run(self, *, duration: float) -> None:
        self.controller.start_stepping()
        end_time = time.monotonic() + duration
        try:
            while time.monotonic() < end_time:
                self.tick()
                time.sleep(self.dt)
        finally:
            try:
                self.stand()
            except Exception:
                pass


class ManualBridgeMotion:
    """
    Temporary manual-mode bridge for the current robot ESP firmware.

    Input:  ``CMD forward_cmd strafe_cmd turn_cmd seq``
    Output: ``ANG servo0 servo1 servo2 seq``

    The outgoing angle triplet is currently taken from one leg of the
    full-body gait controller so this path can reuse the same motion math
    that the future 18-servo pipeline will use.
    """

    def __init__(
        self,
        bridge: SerialBridge,
        controller: MotionController | None = None,
        *,
        leg_index: int = 0,
        dt: float = UPDATE_INTERVAL_S,
        cmd_scale: float = 1000.0,
        timeout_s: float = DEFAULT_COMMAND_TIMEOUT_S,
        deadzone: int = 100,
        verbose: bool = False,
        stride: float = DEFAULT_GAIT_STRIDE_MM,
        lift_scale: float = DEFAULT_GAIT_LIFT_SCALE,
        frequency: float = DEFAULT_GAIT_FREQUENCY_HZ,
        yaw_stride_deg: float = DEFAULT_GAIT_YAW_STRIDE_DEG,
        stance_z: float = DEFAULT_GAIT_STANCE_Z_MM,
        calibration_file: str | None = None,
    ):
        if not 0 <= leg_index < NUM_LEGS:
            raise ValueError(f"leg_index must be in [0, {NUM_LEGS - 1}]")
        self.bridge = bridge
        _mc = controller or build_motion_controller(
            stride=stride,
            lift_scale=lift_scale,
            frequency=frequency,
            yaw_stride_deg=yaw_stride_deg,
            stance_z=stance_z,
            calibration_file=calibration_file,
        )
        self.core = RuntimeMotionCore(controller=_mc, dt=dt)
        self.controller = _mc  # alias kept for introspection / legacy call sites
        self.leg_index = leg_index
        self.dt = dt
        self.cmd_scale = cmd_scale
        self.timeout_s = timeout_s
        self.deadzone = deadzone
        self.verbose = verbose
        self._latest_command = ControlCommand()
        self._last_command_time: float | None = None
        self._was_stale = True

    def _log(self, message: str) -> None:
        if self.verbose:
            print(message, flush=True)

    def _log_exchange(self, rx: str, tx: str) -> None:
        self._log(f"RX {rx:<18} | TX {tx}")

    def _current_leg_angles(self) -> list[float]:
        start = self.leg_index * 3
        return self.core.current_angles()[start:start + 3]

    def _current_tx_line(self, *, seq: int = 0) -> str:
        return format_angle_triplet_command(self._current_leg_angles(), seq=seq).strip()

    def _current_debug_tx(self, command: ControlCommand) -> str:
        angles = self._current_leg_angles()
        label = describe_motion(command)
        return f"{label}:{angles[0]:.1f},{angles[1]:.1f},{angles[2]:.1f}"

    def _send_current_leg(self, *, seq: int = 0) -> str | None:
        return self.bridge.send_angle_triplet(self._current_leg_angles(), seq=seq)

    def stand(self, *, seq: int = 0) -> str | None:
        self.core.stand()
        return self._send_current_leg(seq=seq)

    def _apply_command(self, command: ControlCommand) -> str | None:
        command = apply_command_deadzone(command, self.deadzone)
        forward = normalize_control(command.forward_cmd, self.cmd_scale)
        strafe = normalize_control(command.strafe_cmd, self.cmd_scale)
        turn = normalize_control(command.turn_cmd, self.cmd_scale)
        self.core.tick(
            forward=forward,
            strafe=strafe,
            turn=turn,
            dt=self.dt,
        )
        return self._send_current_leg(seq=command.seq)

    def tick(self) -> None:
        now = time.monotonic()
        rx_display: str | None = None
        for line in self.bridge.poll_lines():
            command = parse_control_command(line)
            if command is not None:
                rx_display = line
                self._latest_command = command
                self._last_command_time = now

        stale = (
            self._last_command_time is None
            or (now - self._last_command_time) > self.timeout_s
        )

        if stale:
            self.stand(seq=self._latest_command.seq)
            tx_line = self._current_debug_tx(ControlCommand())
            if not self._was_stale:
                self._log_exchange("timeout", tx_line)
            self._was_stale = True
            return

        filtered_command = apply_command_deadzone(self._latest_command, self.deadzone)
        self._apply_command(filtered_command)
        tx_line = self._current_debug_tx(filtered_command)
        if rx_display is not None:
            self._log_exchange(rx_display, tx_line)
        self._was_stale = False

    def run(self, *, duration: float | None = None) -> None:
        self.stand()
        deadline = None if duration is None or duration <= 0 else time.monotonic() + duration

        try:
            while deadline is None or time.monotonic() < deadline:
                self.tick()
                time.sleep(self.dt)
        finally:
            try:
                self.stand()
            except Exception:
                pass


class FullGaitLegBridgeMotion:
    """
    Production runtime bridge for the full 6-leg 18-servo hexapod.

    Input:  ``CMD forward_cmd strafe_cmd turn_cmd seq`` from the ESP, plus
            the web override file (``runtime_override.json``).
    Output: ``DIR:<18 angles>\\n`` where ``DIR`` is one of ``N/F/B/L/R/LL/RR``
            and the 18 angles span all 6 legs in configured leg order.

    This is the single production wire path. All 18 servo angles are sent
    every frame; there is no ``leg_count`` slicing. Bench-mode 1-/2-leg
    harnesses should use a dedicated CLI under ``test/`` instead.

    Per-leg saved-vs-unsaved calibration policy is still honored at the
    clamp stage: saved legs only get the hard 0..180 contract clamp so the
    user's stored neutral pose is preserved verbatim; un-saved legs go
    through the conservative bench-safety window.
    """

    def __init__(
        self,
        bridge: SerialBridge,
        controller: MotionController | None = None,
        *,
        dt: float = UPDATE_INTERVAL_S,
        cmd_scale: float = 1000.0,
        timeout_s: float = DEFAULT_COMMAND_TIMEOUT_S,
        deadzone: int = 100,
        verbose: bool = False,
        override_file: str | None = None,
        stride: float = DEFAULT_GAIT_STRIDE_MM,
        lift_scale: float = DEFAULT_GAIT_LIFT_SCALE,
        frequency: float = DEFAULT_GAIT_FREQUENCY_HZ,
        yaw_stride_deg: float = DEFAULT_GAIT_YAW_STRIDE_DEG,
        stance_z: float = DEFAULT_GAIT_STANCE_Z_MM,
        calibration_file: str | None = None,
    ):
        self.bridge = bridge
        _mc = controller or build_motion_controller(
            stride=stride,
            lift_scale=lift_scale,
            frequency=frequency,
            yaw_stride_deg=yaw_stride_deg,
            stance_z=stance_z,
            calibration_file=calibration_file,
        )
        self.core = RuntimeMotionCore(controller=_mc, dt=dt)
        self.controller = _mc  # alias kept for introspection / legacy call sites
        self.dt = dt
        self.cmd_scale = cmd_scale
        self.timeout_s = timeout_s
        self.deadzone = deadzone
        self.verbose = verbose
        self.override_file = override_file
        self._latest_command = ControlCommand()
        self._last_command_time: float | None = None
        self._was_stale = True
        self._last_override_signature: tuple[int, int, int, int] | None = None

    def _log(self, message: str) -> None:
        if self.verbose:
            print(message, flush=True)

    def _log_exchange(self, rx: str, tx: str) -> None:
        self._log(f"RX {rx:<18} | TX {tx}")

    def _clamp_full_angles(
        self,
        angles: Sequence[float],
    ) -> list[float]:
        """
        Clamp all 18 servo angles with per-leg saved-aware policy.

        Saved (calibrated) legs only get the hard 0..180 contract clamp so the
        user's stored neutral pose can live outside the conservative bench
        safety window. Un-saved legs still go through ``clamp_bench_leg_angle``
        for joint-by-joint protection.
        """
        if len(angles) != NUM_SERVOS:
            raise ValueError(f"expected {NUM_SERVOS} angles, got {len(angles)}")
        result: list[float] = []
        servo = self.core.controller.config.servo
        for leg_index in range(NUM_LEGS):
            saved = servo.is_saved(leg_index)
            for joint_index in range(JOINTS_PER_LEG):
                value = float(angles[leg_index * JOINTS_PER_LEG + joint_index])
                if saved:
                    result.append(clamp_angle(value))
                else:
                    result.append(clamp_bench_leg_angle(value, joint_index))
        return result

    def _format_full_tx(self, move_dir: str) -> str:
        """Build the outbound ``DIR:<18 angles>\\n`` line honoring calibration.

        The saved-aware clamp is applied upstream of the canonical serializer
        so the final wire format stays identical between runtime bridge and
        calibration preview (both go through ``format_full_dir_command``).
        """
        values = self._clamp_full_angles(self.core.current_angles())
        return format_full_dir_command(move_dir, values)

    def _current_full_angles(self) -> list[float]:
        return self._clamp_full_angles(self.core.current_angles())

    def _current_tx_line(self, move_dir: str) -> str:
        return self._format_full_tx(move_dir).strip()

    def _current_debug_tx(self, command: ControlCommand) -> str:
        angles = self._current_full_angles()
        label = describe_motion(command)
        return f"{label}:{','.join(f'{angle:.1f}' for angle in angles)}"

    def _send_current_frame(self, *, move_dir: str) -> str | None:
        self.bridge.write_line(self._format_full_tx(move_dir))
        return None

    def stand(self) -> str | None:
        self.core.stand()
        return self._send_current_frame(move_dir=MOVE_DIR_NEUTRAL)

    def _apply_command(self, command: ControlCommand) -> str | None:
        command = apply_command_deadzone(command, self.deadzone)
        forward = normalize_control(command.forward_cmd, self.cmd_scale)
        strafe = normalize_control(command.strafe_cmd, self.cmd_scale)
        turn = normalize_control(command.turn_cmd, self.cmd_scale)
        self.core.tick(
            forward=forward,
            strafe=strafe,
            turn=turn,
            dt=self.dt,
        )
        return self._send_current_frame(move_dir=describe_motion(command))

    def tick(self) -> None:
        now = time.monotonic()
        rx_display: str | None = None
        for line in self.bridge.poll_lines():
            command = parse_control_command(line)
            if command is not None:
                rx_display = line
                self._latest_command = command
                self._last_command_time = now

        override = read_control_override(self.override_file)
        if override is not None:
            filtered_command = apply_command_deadzone(override.command, self.deadzone)
            self._apply_command(filtered_command)
            tx_line = self._current_debug_tx(filtered_command)
            signature = (
                filtered_command.forward_cmd,
                filtered_command.strafe_cmd,
                filtered_command.turn_cmd,
                filtered_command.seq,
            )
            if signature != self._last_override_signature or rx_display is not None:
                self._log_exchange(f"{override.source}:{override.mode}", tx_line)
            self._last_override_signature = signature
            self._was_stale = False
            return

        self._last_override_signature = None

        stale = (
            self._last_command_time is None
            or (now - self._last_command_time) > self.timeout_s
        )

        if stale:
            self.stand()
            tx_line = self._current_debug_tx(ControlCommand())
            if not self._was_stale:
                self._log_exchange("timeout", tx_line)
            self._was_stale = True
            return

        filtered_command = apply_command_deadzone(self._latest_command, self.deadzone)
        self._apply_command(filtered_command)
        tx_line = self._current_debug_tx(filtered_command)
        if rx_display is not None:
            self._log_exchange(rx_display, tx_line)
        self._was_stale = False

    def run(self, *, duration: float | None = None) -> None:
        self.stand()
        deadline = None if duration is None or duration <= 0 else time.monotonic() + duration

        try:
            while deadline is None or time.monotonic() < deadline:
                self.tick()
                time.sleep(self.dt)
        finally:
            try:
                self.stand()
            except Exception:
                pass
