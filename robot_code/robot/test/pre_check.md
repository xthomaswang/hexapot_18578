# Hexapod Pre-Flight Check

> Run these checks **before** attempting any gait/walk testing.
> Each step sends commands directly to ESP32 via UART (`/dev/serial0` @ 115200).
> **Prerequisite**: Stop runtime first.

## Quick Reference: UART Protocol

```
Format:  N:coxa1,femur1,tibia1,coxa2,...,coxa6,femur6,tibia6\n
Example: N:90.0,64.2,116.3,90.0,64.2,116.3,90.0,64.2,116.3,90.0,64.2,116.3,90.0,64.2,116.3,90.0,64.2,116.3
Legs:    L1(0-2), L2(3-5), L3(6-8), R1(9-11), R2(12-14), R3(15-17)
```

## Configuration Summary (verify with ME)

```
Coxa length    254.0 mm   Coxa rotation axis to Femur rotation axis
Femur length   228.6 mm   Femur rotation axis to Tibia rotation axis
Tibia length   594.4 mm   Tibia rotation axis to Foot tip
Body length    812.8 mm   Front edge to Rear edge
Body width     381.0 mm   Left edge to Right edge
Corner inset   50.0 mm    Corner leg mount inset from body corner
Servo neutral  90 deg     Mechanical center position
Servo travel   120 deg    Total range (30-150 deg)
PWM range      1000-2000 us
```

### Servo Direction Table

```
Leg   Coxa   Femur   Tibia
L1     +1     -1      +1
L2     +1     -1      +1
L3     +1     -1      +1
R1     -1     -1      +1
R2     -1     -1      +1
R3     -1     -1      +1
```

### Servo Channel Map

```
Leg   Coxa   Femur   Tibia
L1    ch0    ch1     ch2
L2    ch3    ch4     ch5
L3    ch6    ch7     ch8
R1    ch9    ch10    ch11
R2    ch12   ch13    ch14
R3    ch15   ch16    ch17
```

---

## Test Steps

### Step 0: Stop Runtime and Prepare

Stop any running runtime so we can directly control the serial port.

```bash
curl -X POST http://localhost:8091/api/runtime/stop
curl -X POST http://localhost:8091/api/calibration/stop
```

Then run the interactive test script:
```bash
cd /home/team6/Desktop/robot
python3 test/pre_check_runner.py
```

---

### Step 1: All Servos to 90 deg (Neutral)

**Purpose**: Verify all 18 servos respond and go to their mechanical center.

**Command**: All angles = 90.0

**What to check**:
- All 18 servos move
- No servos are stuck or making grinding noise
- Left and right sides look roughly mirror-symmetric
- Note any servo that seems offset from center

**If FAIL**: Check servo wiring, channel mapping, power supply.

---

### Step 2: L1 Coxa Sweep (70 to 110 deg)

**Purpose**: Verify coxa direction and range.

Sweep L1 coxa from 70 to 110 deg while keeping femur=90, tibia=90.
All other legs stay at 90/90/90.

**What to check**:
- L1 coxa rotates horizontally (yaw)
- 70 deg: leg swings one direction
- 110 deg: leg swings opposite direction
- Movement is smooth, no skipping
- ONLY L1 moves, other legs stay still
- Record: which direction is "forward" when coxa increases?

**Expected (dir=+1)**: Coxa increase should swing leg forward.

---

### Step 3: L1 Femur Sweep (50 to 90 deg)

**Purpose**: Verify femur direction (previously found reversed, now corrected to -1).

Sweep L1 femur from 50 to 90 deg while keeping coxa=90, tibia=90.

**What to check**:
- L1 femur rotates vertically (pitch of upper leg)
- 50 deg: leg should be lifted UP
- 90 deg: leg roughly horizontal or at neutral
- Movement is smooth
- ONLY L1 moves

**Expected (dir=-1)**: Femur decrease from 90 means leg lifts up.

---

### Step 4: L1 Tibia Sweep (90 to 140 deg)

**Purpose**: Verify tibia direction. Tibia uses a different formula:
`servo = 90 - dir * (q3 - 90 + offset)` (note the minus and the -90 shift).

Sweep L1 tibia from 90 to 140 deg while keeping coxa=90, femur=90.

**What to check**:
- L1 tibia bends/extends the lower leg
- 90 deg: lower leg roughly at right angle to upper leg
- 140 deg: lower leg extends downward/outward
- Movement is smooth
- ONLY L1 moves

**Expected (dir=+1)**: Tibia increase means lower leg extends (straightens).

---

### Step 5: R1 Direction Check (mirror of L1)

**Purpose**: Verify right-side directions are mirrored correctly.
R1 has coxa=-1, femur=-1, tibia=+1.

**What to check**:
- R1 coxa sweep 70 to 110: same forward/backward sense as L1 (mirrored)
- R1 femur sweep 50 to 90: same up/down sense as L1
- R1 tibia sweep 90 to 140: same bend/extend sense as L1

**If any direction is opposite to L1**: The direction multiplier is wrong for that joint.

---

### Step 6: Default Standing Pose (uncalibrated)

**Purpose**: Verify the IK-computed standing pose looks correct.

**Command**: coxa=90.0, femur=64.2, tibia=116.3 for all legs.

**What to check**:
- All legs form a symmetric stance
- Robot body is roughly level (not tilting)
- Legs are not crossing or hitting each other
- Feet are touching ground with reasonable spread

---

### Step 7: Calibrated Standing Pose

**Purpose**: Verify calibration offsets produce a better stance.

Sends the calibrated angles from `motion_engine/_cache/calibration_profile.json`.

**What to check**:
- Stance is more level than Step 6
- No leg is visibly higher/lower than others
- Compare with Step 6 to see improvement

---

### Step 8: Single-Leg Lift Test (each leg)

**Purpose**: Verify femur can lift a leg off the ground from standing pose.

Lift each leg one at a time by decreasing its femur from 64.2 to 45.0.
Other legs stay at standing pose.

**What to check**:
- Target leg lifts clearly off the ground
- Robot remains stable on other 5 legs (+ chair support)
- Repeat for each leg: L1, L2, L3, R1, R2, R3
- Any leg that does not lift: check femur direction, wiring, or binding

---

### Step 9: PWM Neutral Verification

**Purpose**: Check if servo physical center matches code assumption.

Config says `pwm_neutral = 1520 us` and the PWM mapping should now preserve
`90 deg = 1520 us`.

**What to check**:
- At 90 deg command, is the servo output shaft at its physical center mark?
- If center is still off, the horn indexing or per-servo calibration is wrong, not the
  software neutral mapping.

---

## Known Issues Found in Code Review

```
#   Issue                                              Severity   Status
1   PWM neutral mismatch (1520 vs 1500)                Medium     Fixed in motion.py
2   Femur direction was reversed                       Fixed      dir = -1 for all legs
3   Bench clamp narrower than IK range                 Low        By design for safety
4   Default stride 160mm > suggested 120mm             Low        Can tune in web UI
5   Coxa forward direction needs physical verification High       Step 2
6   Tibia formula has different sign convention         Medium     Step 4
```

---

## After All Checks Pass

1. Deploy manual mode from web UI
2. Use Xbox controller or keyboard controls
3. Start with very low stride (80mm) and frequency (0.2Hz)
4. Test forward only first, then add other directions
5. Gradually increase stride and frequency
