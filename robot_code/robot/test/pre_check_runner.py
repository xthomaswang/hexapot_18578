#!/usr/bin/env python3
"""Interactive pre-flight check runner for the hexapod robot.

Sends servo commands directly over UART to ESP32.
Must stop runtime first before running this script.

Usage:
    cd /home/team6/Desktop/robot
    python3 test/pre_check_runner.py
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "motion_engine"))

try:
    import serial
except ImportError:
    print("ERROR: pyserial not installed. Run: pip install pyserial")
    sys.exit(1)

SERIAL_PORT = "/dev/serial0"
BAUDRATE = 115200
NUM_LEGS = 6
JOINTS_PER_LEG = 3
LEG_LABELS = ["L1", "L2", "L3", "R1", "R2", "R3"]

# Default standing pose (uncalibrated)
DEFAULT_STAND = [90.0, 64.2, 116.3] * 6

# Calibration file
CALIBRATION_FILE = Path(__file__).resolve().parents[1] / "motion_engine" / "_cache" / "calibration_profile.json"


def send_frame(ser: serial.Serial, angles: list[float], prefix: str = "N") -> str:
    """Send a full 18-angle frame to ESP32."""
    cmd = prefix + ":" + ",".join(f"{a:.1f}" for a in angles) + "\n"
    ser.write(cmd.encode("ascii"))
    return cmd.strip()


def make_frame(base: list[float], leg_index: int = -1,
               coxa: float | None = None, femur: float | None = None,
               tibia: float | None = None) -> list[float]:
    """Create a frame modifying one leg's angles from a base frame."""
    frame = list(base)
    if leg_index >= 0:
        s = leg_index * 3
        if coxa is not None:
            frame[s] = coxa
        if femur is not None:
            frame[s + 1] = femur
        if tibia is not None:
            frame[s + 2] = tibia
    return frame


def pause(msg: str = "Press Enter to continue..."):
    input(f"\n  >> {msg}")


def ask_yn(question: str) -> bool:
    while True:
        ans = input(f"  >> {question} (y/n): ").strip().lower()
        if ans in ("y", "yes"):
            return True
        if ans in ("n", "no"):
            return False


def print_frame(angles: list[float]):
    """Print a frame in readable format."""
    for i in range(NUM_LEGS):
        s = i * 3
        print(f"    {LEG_LABELS[i]}: coxa={angles[s]:.1f}  femur={angles[s+1]:.1f}  tibia={angles[s+2]:.1f}")


def load_calibrated_stand() -> list[float] | None:
    """Load calibrated standing pose from profile."""
    try:
        data = json.loads(CALIBRATION_FILE.read_text())
        frame = []
        for leg in data["legs"]:
            a = leg["angles_deg"]
            frame.extend([a["coxa_deg"], a["femur_deg"], a["tibia_deg"]])
        return frame
    except Exception as e:
        print(f"  WARNING: Could not load calibration: {e}")
        return None


def sweep(ser: serial.Serial, base: list[float], leg_index: int,
          joint: str, start: float, end: float, steps: int = 10,
          delay: float = 0.3):
    """Smoothly sweep one joint from start to end."""
    for i in range(steps + 1):
        val = start + (end - start) * i / steps
        kwargs = {joint: val}
        frame = make_frame(base, leg_index, **kwargs)
        cmd = send_frame(ser, frame)
        print(f"    {LEG_LABELS[leg_index]} {joint}={val:.1f}  ->  {cmd}")
        time.sleep(delay)


# ─── TESTS ───────────────────────────────────────────────────────────

def step1_neutral(ser: serial.Serial) -> bool:
    """Step 1: All servos to 90 deg."""
    print("\n" + "=" * 60)
    print("STEP 1: All Servos to 90 deg (Neutral)")
    print("=" * 60)
    print("  Sending all 18 servos to 90.0 degrees...")

    frame = [90.0] * 18
    cmd = send_frame(ser, frame)
    print(f"  Sent: {cmd}")
    print_frame(frame)

    pause("Look at the robot. All servos should be at mechanical center.")
    return ask_yn("Do all 18 servos respond and look roughly centered?")


def step2_l1_coxa(ser: serial.Serial) -> bool:
    """Step 2: L1 coxa sweep."""
    print("\n" + "=" * 60)
    print("STEP 2: L1 Coxa Sweep (70 -> 110 deg)")
    print("=" * 60)
    print("  All other servos at 90. Sweeping L1 coxa...")

    base = [90.0] * 18
    sweep(ser, base, 0, "coxa", 70, 110, steps=8, delay=0.4)

    print("\n  Returning to 90...")
    send_frame(ser, base)

    pause("Observe L1 coxa movement.")
    ok = ask_yn("Did L1 coxa rotate horizontally (yaw) smoothly?")
    if ok:
        fwd = ask_yn("When coxa INCREASED (70->110), did the leg swing FORWARD?")
        if not fwd:
            print("  *** WARNING: Coxa direction may be wrong for L1! ***")
            print("  *** Expected: coxa increase = forward (dir=+1) ***")
            return False
    return ok


def step3_l1_femur(ser: serial.Serial) -> bool:
    """Step 3: L1 femur sweep."""
    print("\n" + "=" * 60)
    print("STEP 3: L1 Femur Sweep (90 -> 50 deg)")
    print("=" * 60)
    print("  All other servos at 90. Sweeping L1 femur down from 90 to 50...")

    base = [90.0] * 18
    sweep(ser, base, 0, "femur", 90, 50, steps=8, delay=0.4)

    print("\n  Returning to 90...")
    send_frame(ser, base)

    pause("Observe L1 femur movement.")
    ok = ask_yn("Did L1 femur rotate vertically (pitch upper leg)?")
    if ok:
        up = ask_yn("When femur DECREASED (90->50), did the leg LIFT UP?")
        if not up:
            print("  *** WARNING: Femur direction may still be wrong! ***")
            print("  *** Expected: femur decrease = lift up (dir=-1) ***")
            return False
    return ok


def step4_l1_tibia(ser: serial.Serial) -> bool:
    """Step 4: L1 tibia sweep."""
    print("\n" + "=" * 60)
    print("STEP 4: L1 Tibia Sweep (90 -> 140 deg)")
    print("=" * 60)
    print("  All other servos at 90. Sweeping L1 tibia from 90 to 140...")

    base = [90.0] * 18
    sweep(ser, base, 0, "tibia", 90, 140, steps=10, delay=0.4)

    print("\n  Returning to 90...")
    send_frame(ser, base)

    pause("Observe L1 tibia movement.")
    ok = ask_yn("Did L1 tibia bend/extend the lower leg?")
    if ok:
        ext = ask_yn("When tibia INCREASED (90->140), did the lower leg EXTEND (straighten)?")
        if not ext:
            print("  *** WARNING: Tibia direction may be wrong! ***")
            print("  *** Expected: tibia increase = extend (dir=+1) ***")
            return False
    return ok


def step5_r1_check(ser: serial.Serial) -> bool:
    """Step 5: R1 direction check (mirror of L1)."""
    print("\n" + "=" * 60)
    print("STEP 5: R1 Direction Check (mirror of L1)")
    print("=" * 60)

    base = [90.0] * 18

    print("\n  --- R1 Coxa Sweep (70 -> 110) ---")
    sweep(ser, base, 3, "coxa", 70, 110, steps=8, delay=0.4)
    send_frame(ser, base)
    pause("Observe R1 coxa.")
    coxa_ok = ask_yn("R1 coxa moved same as L1 but mirrored (forward when increasing)?")

    print("\n  --- R1 Femur Sweep (90 -> 50) ---")
    sweep(ser, base, 3, "femur", 90, 50, steps=8, delay=0.4)
    send_frame(ser, base)
    pause("Observe R1 femur.")
    femur_ok = ask_yn("R1 femur lifted up when decreasing (same as L1)?")

    print("\n  --- R1 Tibia Sweep (90 -> 140) ---")
    sweep(ser, base, 3, "tibia", 90, 140, steps=10, delay=0.4)
    send_frame(ser, base)
    pause("Observe R1 tibia.")
    tibia_ok = ask_yn("R1 tibia extended when increasing (same as L1)?")

    if not (coxa_ok and femur_ok and tibia_ok):
        print("  *** WARNING: R1 direction mismatch with L1! ***")
    return coxa_ok and femur_ok and tibia_ok


def step6_default_stand(ser: serial.Serial) -> bool:
    """Step 6: Default standing pose."""
    print("\n" + "=" * 60)
    print("STEP 6: Default Standing Pose (uncalibrated)")
    print("=" * 60)

    frame = list(DEFAULT_STAND)
    cmd = send_frame(ser, frame)
    print(f"  Sent: {cmd}")
    print_frame(frame)

    pause("Observe the standing pose.")
    return ask_yn("Does the robot look like a reasonable standing pose? (symmetric, level-ish)")


def step7_calibrated_stand(ser: serial.Serial) -> bool:
    """Step 7: Calibrated standing pose."""
    print("\n" + "=" * 60)
    print("STEP 7: Calibrated Standing Pose")
    print("=" * 60)

    cal_frame = load_calibrated_stand()
    if cal_frame is None:
        print("  SKIP: No calibration file found.")
        return True

    cmd = send_frame(ser, cal_frame)
    print(f"  Sent: {cmd}")
    print_frame(cal_frame)

    pause("Compare with Step 6. Is this more level?")
    return ask_yn("Does the calibrated pose look better than the uncalibrated one?")


def step8_leg_lift(ser: serial.Serial) -> bool:
    """Step 8: Single-leg lift test for each leg."""
    print("\n" + "=" * 60)
    print("STEP 8: Single-Leg Lift Test")
    print("=" * 60)

    base = list(DEFAULT_STAND)
    all_ok = True

    for leg_idx in range(NUM_LEGS):
        label = LEG_LABELS[leg_idx]
        print(f"\n  --- Lifting {label} ---")

        # First send standing pose
        send_frame(ser, base)
        time.sleep(0.5)

        # Then lift this leg (reduce femur)
        frame = make_frame(base, leg_idx, femur=45.0)
        cmd = send_frame(ser, frame)
        print(f"  Sent: {cmd}")

        pause(f"Check if {label} lifted off the ground.")
        ok = ask_yn(f"Did {label} lift clearly off the ground?")
        if not ok:
            print(f"  *** WARNING: {label} did not lift properly! ***")
            all_ok = False

        # Return to standing
        send_frame(ser, base)
        time.sleep(0.3)

    return all_ok


def step9_pwm_check(ser: serial.Serial) -> bool:
    """Step 9: PWM neutral verification."""
    print("\n" + "=" * 60)
    print("STEP 9: PWM Neutral Verification")
    print("=" * 60)
    print("  Config says pwm_neutral = 1520 us")
    print("  But code computes 90 deg = 1500 us (linear 1000-2000 over 30-150 deg)")
    print("  This means there might be a ~1.5 deg systematic offset.")
    print()
    print("  Currently at 90 deg (from Step 1 or any prior step).")

    frame = [90.0] * 18
    send_frame(ser, frame)

    pause("Check if servo output shafts are at their physical center marks.")
    offset = ask_yn("Are the servos noticeably off from physical center (~1-2 deg)?")
    if offset:
        print("  NOTE: Consider adjusting pwm_neutral_us in config.py to match actual servo behavior.")
        print("  Or add a global offset to all servo angles.")
        return False
    return True


# ─── MAIN ────────────────────────────────────────────────────────────

def main():
    print("=" * 60)
    print("  HEXAPOD PRE-FLIGHT CHECK")
    print("=" * 60)
    print(f"  Serial port: {SERIAL_PORT} @ {BAUDRATE}")
    print(f"  Calibration: {CALIBRATION_FILE}")
    print()
    print("  Make sure runtime is STOPPED before proceeding.")
    print("  The robot should be on its chair support.")

    pause("Ready to start? (robot powered, servos connected)")

    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
    except Exception as e:
        print(f"  ERROR: Cannot open serial port: {e}")
        print("  Is runtime still running? Stop it first:")
        print("    curl -X POST http://localhost:8091/api/runtime/stop")
        sys.exit(1)

    results = {}
    steps = [
        ("Step 1: Neutral (all 90)", step1_neutral),
        ("Step 2: L1 Coxa Sweep", step2_l1_coxa),
        ("Step 3: L1 Femur Sweep", step3_l1_femur),
        ("Step 4: L1 Tibia Sweep", step4_l1_tibia),
        ("Step 5: R1 Direction Check", step5_r1_check),
        ("Step 6: Default Stand", step6_default_stand),
        ("Step 7: Calibrated Stand", step7_calibrated_stand),
        ("Step 8: Leg Lift Test", step8_leg_lift),
        ("Step 9: PWM Neutral Check", step9_pwm_check),
    ]

    for name, func in steps:
        try:
            result = func(ser)
            results[name] = "PASS" if result else "FAIL"
        except KeyboardInterrupt:
            print("\n  Interrupted. Sending neutral pose for safety...")
            send_frame(ser, [90.0] * 18)
            results[name] = "SKIPPED"
            break
        except Exception as e:
            print(f"  ERROR in {name}: {e}")
            results[name] = "ERROR"

        if results[name] == "FAIL":
            cont = ask_yn("This step FAILED. Continue to next step anyway?")
            if not cont:
                print("  Sending neutral pose for safety...")
                send_frame(ser, [90.0] * 18)
                break

    # Return to safe position
    print("\n  Returning to neutral (90 deg all)...")
    send_frame(ser, [90.0] * 18)
    ser.close()

    # Summary
    print("\n" + "=" * 60)
    print("  PRE-FLIGHT CHECK SUMMARY")
    print("=" * 60)
    for name, result in results.items():
        icon = "OK" if result == "PASS" else "!!" if result == "FAIL" else "--"
        print(f"  [{icon}] {name}: {result}")

    fail_count = sum(1 for r in results.values() if r == "FAIL")
    if fail_count == 0:
        print("\n  All checks passed! Ready for gait testing.")
    else:
        print(f"\n  {fail_count} check(s) FAILED. Fix issues before gait testing.")


if __name__ == "__main__":
    main()
