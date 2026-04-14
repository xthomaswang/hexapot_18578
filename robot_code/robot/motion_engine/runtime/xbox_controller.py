"""Xbox controller input → runtime override bridge.

Reads an Xbox controller via evdev and writes runtime_override.json
so that FullGaitLegBridgeMotion picks up the commands each tick.

Also writes a status JSON file for the web dashboard to display
controller state (connection, battery, axes).

Mapping:
  Left stick Y  → forward/backward  (up = forward)
  Left stick X  → strafe left/right (left = positive)
  Right stick X → turn/rotation     (left = positive, counterclockwise)
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import time
from pathlib import Path

import evdev
from evdev import ecodes

# Xbox Series S|X axis codes
AXIS_LEFT_X = ecodes.ABS_X       # 0
AXIS_LEFT_Y = ecodes.ABS_Y       # 1
AXIS_RIGHT_X = ecodes.ABS_RX     # 3
AXIS_RIGHT_Y = ecodes.ABS_RY     # 4
AXIS_LT = ecodes.ABS_Z           # 2  (left trigger)
AXIS_RT = ecodes.ABS_RZ          # 5  (right trigger)

STICK_MAX = 32767
STICK_DEADZONE = 2000  # ~6% of full range
CMD_SCALE = 1000       # output range ±1000 to match runtime cmd_scale
OVERRIDE_EXPIRE_S = 0.25
UPDATE_HZ = 100        # 10ms control loop
STATUS_UPDATE_HZ = 10  # status file for web display
BUTTON_CODES = {
    ecodes.BTN_SOUTH: "a",
    ecodes.BTN_EAST: "b",
    # This controller/driver pair reports the physical X/Y buttons swapped
    # relative to the default evdev BTN_WEST/BTN_NORTH expectation.
    ecodes.BTN_WEST: "y",
    ecodes.BTN_NORTH: "x",
    ecodes.BTN_TL: "lb",
    ecodes.BTN_TR: "rb",
}


def find_xbox_device() -> evdev.InputDevice | None:
    """Auto-detect the Xbox controller event device."""
    for path in sorted(evdev.list_devices()):
        dev = evdev.InputDevice(path)
        if "xbox" in dev.name.lower() or "microsoft" in dev.name.lower():
            return dev
    return None


def get_bluetooth_battery(mac: str) -> int | None:
    """Try to read battery percentage via bluetoothctl."""
    try:
        result = subprocess.run(
            ["bluetoothctl", "info", mac],
            capture_output=True, text=True, timeout=2,
        )
        for line in result.stdout.splitlines():
            if "Battery Percentage" in line:
                # Format: "Battery Percentage: 0x32 (50)"
                start = line.find("(")
                end = line.find(")")
                if start != -1 and end != -1:
                    return int(line[start + 1:end])
    except (subprocess.TimeoutExpired, OSError, ValueError):
        pass
    return None


def apply_deadzone(value: int, deadzone: int) -> int:
    if abs(value) < deadzone:
        return 0
    sign = 1 if value > 0 else -1
    return sign * (abs(value) - deadzone)


def scale_axis(value: int, deadzone: int = STICK_DEADZONE) -> int:
    """Map raw stick value to ±CMD_SCALE with deadzone."""
    cleaned = apply_deadzone(value, deadzone)
    effective_max = STICK_MAX - deadzone
    if effective_max <= 0:
        return 0
    return int(round(cleaned / effective_max * CMD_SCALE))


def clamp(value: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, value))


def write_json_atomic(path: Path, payload: dict) -> None:
    """Atomically write JSON to a file."""
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(payload), encoding="utf-8")
    os.replace(tmp, path)


def write_override(
    path: Path,
    forward_cmd: int,
    strafe_cmd: int,
    turn_cmd: int,
    seq: int,
) -> None:
    payload = {
        "active": True,
        "forward_cmd": clamp(forward_cmd, -CMD_SCALE, CMD_SCALE),
        "strafe_cmd": clamp(strafe_cmd, -CMD_SCALE, CMD_SCALE),
        "turn_cmd": clamp(turn_cmd, -CMD_SCALE, CMD_SCALE),
        "seq": seq,
        "source": "xbox",
        "expires_at": time.time() + OVERRIDE_EXPIRE_S,
    }
    write_json_atomic(path, payload)


def clear_override(path: Path) -> None:
    payload = {"active": False, "source": "xbox"}
    write_json_atomic(path, payload)


def write_status(
    path: Path,
    *,
    connected: bool,
    device_name: str = "",
    connection_type: str = "none",
    battery: int | None = None,
    forward_cmd: int = 0,
    strafe_cmd: int = 0,
    turn_cmd: int = 0,
    calibration_adjust: int = 0,
    buttons: dict[str, bool] | None = None,
    button_seq: dict[str, int] | None = None,
) -> None:
    payload = {
        "connected": connected,
        "device_name": device_name,
        "connection_type": connection_type,
        "battery": battery,
        "forward_cmd": forward_cmd,
        "strafe_cmd": strafe_cmd,
        "turn_cmd": turn_cmd,
        "calibration_adjust": calibration_adjust,
        "buttons": dict(buttons or {}),
        "button_seq": dict(button_seq or {}),
        "updated_at": time.time(),
    }
    write_json_atomic(path, payload)


def clear_status(path: Path) -> None:
    payload = {
        "connected": False,
        "device_name": "",
        "connection_type": "none",
        "battery": None,
        "forward_cmd": 0,
        "strafe_cmd": 0,
        "turn_cmd": 0,
        "calibration_adjust": 0,
        "buttons": {name: False for name in BUTTON_CODES.values()},
        "button_seq": {name: 0 for name in BUTTON_CODES.values()},
        "updated_at": time.time(),
    }
    write_json_atomic(path, payload)


def detect_connection_type(dev: evdev.InputDevice) -> str:
    """Detect whether the controller is USB or Bluetooth."""
    try:
        phys = dev.phys or ""
        # USB devices have "usb-" in phys (e.g. "usb-xhci-hcd.1-1/input0")
        if "usb" in phys.lower():
            return "usb"
        # Bluetooth via xpadneo: phys is the adapter MAC, bus type 0x0005
        # Also check sysfs MODALIAS for bus type b0005 (BT)
        uevent_path = Path(f"/sys/class/input/{Path(dev.path).name}/device/uevent")
        if uevent_path.exists():
            uevent = uevent_path.read_text()
            if "b0005" in uevent:
                return "bluetooth"
        # Fallback: phys looks like a MAC address = Bluetooth
        if len(phys.split(":")) == 6:
            return "bluetooth"
    except Exception:
        pass
    return "usb"


def run(
    override_file: str,
    status_file: str,
    device_path: str | None = None,
) -> None:
    override_path = Path(override_file)
    status_path = Path(status_file)
    interval = 1.0 / UPDATE_HZ
    status_interval = 1.0 / STATUS_UPDATE_HZ
    seq = 0

    # Write initial "searching" status
    write_status(status_path, connected=False, device_name="Searching...")
    print("Searching for Xbox controller...")

    # Wait for controller with retry
    dev: evdev.InputDevice | None = None
    if device_path:
        dev = evdev.InputDevice(device_path)
    else:
        while dev is None:
            dev = find_xbox_device()
            if dev is None:
                write_status(status_path, connected=False, device_name="No controller found")
                time.sleep(2)

    conn_type = detect_connection_type(dev)
    battery: int | None = None
    print(f"Using controller: {dev.name} ({dev.path}) [{conn_type}]")

    # Current axis state
    axes = {
        AXIS_LEFT_X: 0,
        AXIS_LEFT_Y: 0,
        AXIS_RIGHT_X: 0,
        AXIS_RIGHT_Y: 0,
    }
    buttons = {name: False for name in BUTTON_CODES.values()}
    button_seq = {name: 0 for name in BUTTON_CODES.values()}

    running = True

    def shutdown(signum, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    print("Xbox controller active. Ctrl+C to stop.")
    print("  Left stick  -> move (forward/back/strafe)")
    print("  Right stick -> rotate")

    last_status_time = 0.0
    last_battery_time = -999.0  # force immediate battery check

    try:
        while running:
            # Drain all pending events (non-blocking)
            try:
                while True:
                    event = dev.read_one()
                    if event is None:
                        break
                    if event.type == ecodes.EV_ABS and event.code in axes:
                        axes[event.code] = event.value
                    if event.type == ecodes.EV_KEY and event.code in BUTTON_CODES:
                        button_name = BUTTON_CODES[event.code]
                        pressed = bool(event.value)
                        buttons[button_name] = pressed
                        if event.value == 1:
                            button_seq[button_name] += 1
            except OSError:
                # Controller disconnected
                print("\nController disconnected! Waiting for reconnect...")
                clear_override(override_path)
                write_status(status_path, connected=False, device_name="Disconnected")
                dev = None
                while dev is None and running:
                    dev = find_xbox_device()
                    if dev is None:
                        time.sleep(2)
                if dev is None:
                    break
                conn_type = detect_connection_type(dev)
                print(f"Reconnected: {dev.name} [{conn_type}]")
                for k in axes:
                    axes[k] = 0
                for button_name in buttons:
                    buttons[button_name] = False
                continue

            # Left stick Y: up is negative in evdev, but forward is positive
            forward_cmd = scale_axis(-axes[AXIS_LEFT_Y])
            # Left stick X: right is positive in evdev, strafe left is positive
            strafe_cmd = scale_axis(-axes[AXIS_LEFT_X])
            # Right stick X: right is positive in evdev, turn left (CCW) is positive
            turn_cmd = scale_axis(-axes[AXIS_RIGHT_X])
            # Right stick Y: up is negative in evdev, positive means "increase angle"
            calibration_adjust = scale_axis(-axes[AXIS_RIGHT_Y])

            seq += 1
            write_override(override_path, forward_cmd, strafe_cmd, turn_cmd, seq)

            # Update status file at lower rate
            now = time.time()
            if now - last_status_time >= status_interval:
                # Check battery every 30 seconds
                if now - last_battery_time >= 30:
                    if conn_type == "bluetooth":
                        battery = get_bluetooth_battery("F4:6A:D7:2D:D8:AD")
                    last_battery_time = now
                write_status(
                    status_path,
                    connected=True,
                    device_name=dev.name,
                    connection_type=conn_type,
                    battery=battery,
                    forward_cmd=forward_cmd,
                    strafe_cmd=strafe_cmd,
                    turn_cmd=turn_cmd,
                    calibration_adjust=calibration_adjust,
                    buttons=buttons,
                    button_seq=button_seq,
                )
                last_status_time = now

            # Debug output
            if seq % UPDATE_HZ == 0:
                print(
                    f"  fwd={forward_cmd:+5d}  str={strafe_cmd:+5d}  "
                    f"turn={turn_cmd:+5d}",
                    end="\r",
                )

            time.sleep(interval)
    finally:
        clear_override(override_path)
        clear_status(status_path)
        print("\nController stopped, override cleared.")


def main() -> None:
    parser = argparse.ArgumentParser(description="Xbox controller -> runtime override")
    parser.add_argument(
        "--override-file",
        default="runtime_override.json",
        help="Path to the override JSON file (default: runtime_override.json)",
    )
    parser.add_argument(
        "--status-file",
        default="xbox_status.json",
        help="Path to the status JSON file (default: xbox_status.json)",
    )
    parser.add_argument(
        "--device",
        default=None,
        help="Event device path (e.g. /dev/input/event5). Auto-detects if omitted.",
    )
    args = parser.parse_args()
    run(args.override_file, args.status_file, args.device)


if __name__ == "__main__":
    main()
