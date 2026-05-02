# Hexapod robot_code — Claude operating notes

CMU 18-578 hexapod project. Raspberry Pi runs the high-level logic in this `robot/` tree and talks UART to an ESP32 (`../esp32/hexapod_main/`) which drives 18 servos through 2× PCA9685.

## Top-level layout

```
robot/
  motion_engine/         IK + runtime, the only side that owns the UART link
    ik_engine/           pure math: geometry, leg IK, locomotion, calibration parsing
    runtime/             SerialBridge, FullGaitLegBridgeMotion, CLI main, xbox controller
    motion.py            servo_map / direct PWM helpers (NOT on the UART path)
  web/                   Flask UI (calibration page, dance encoder, runner controls)
  test/                  pytest suite + pre_check_runner.py (interactive bench tool)
  dance_engine/          dance encoding/playback engine, called from web
  vision/                vision pipeline (separate)
```

Pi target: `ssh raspi-cmu`, project root `/home/team6/Desktop/robot/`. Use `scp <local> raspi-cmu:/home/team6/Desktop/robot/<same-relative-path>` to deploy.

## Leg model

- 6 legs, 3 joints each (coxa, femur, tibia) → 18 servos.
- Logical order everywhere in Python: `("L1", "L2", "L3", "R1", "R2", "R3")` → indices `0..5`.
- Frame layout on the wire: 18 floats `[L1.coxa, L1.femur, L1.tibia, L2.coxa, …]`, leg `i` at slice `[i*3 : i*3+3]`.
- ESP32 PCA9685 mapping (firmware-side, see `esp32/hexapod_main/main/control_system.c:126-131`):
  | Leg | PCA9685 | coxa | femur | tibia |
  |-----|---------|------|-------|-------|
  | L1  | 0x40    | 0    | 1     | 2     |
  | L2  | 0x40    | 4    | 5     | 6     |
  | L3  | 0x40    | 8    | 9     | 10    |
  | R1  | 0x41    | 0    | 1     | 2     |
  | R2  | 0x41    | 4    | 5     | 6     |
  | R3  | 0x41    | 8    | 9     | 10    |

  Note: `motion_engine/ik_engine/config.py` `ServoConfig.servo_map` uses contiguous channels `(0,1,2),(3,4,5),…(15,16,17)` — that map only feeds `motion.py`'s direct-PWM helpers and is **not** used on the UART path. The ESP32 has its own table.

## UART contract (Pi → ESP32)

Single source of truth: `motion_engine/runtime/protocol.py`.

- Port `/dev/serial0`, 115200 baud, ASCII, `\n`-terminated.
- Commands:
  - `M:<a0>,<a1>,…,<a17>\n` — full body move (legacy / CLI).
  - `<DIR>:<a0>,…,<a17>\n` where `DIR ∈ N|F|B|L|R|LL|RR` — production frames carrying the current move intent. Built by `format_full_dir_command`.
  - `H:` home, `S:` stop, `V:<percent>` speed, `Q:` query.
  - `ANG <c> <f> <t> <seq>\n` — legacy 3-servo bench format used by `ManualBridgeMotion` only.
- Response: `OK` / `ERR` / `ST` / `IMU`.
- Hard angle clamp: `0..180`. Bench-only tighter window in `BENCH_LEG_SAFE_LIMITS_DEG`.

### Wire vs. logical order — important

`logical_to_wire_angles(angles)` reorders the 18-angle frame by `WIRE_LEG_LOGICAL_INDICES` before serialization. **Currently identity** (`(0,1,2,3,4,5)`) — Pi sends in logical order and the ESP32 firmware does its own mapping.

History: an earlier ESP32 firmware had R1/R3 swapped at the firmware level, so `WIRE_LEG_LOGICAL_INDICES` was set to `(0,1,2,5,4,3)` and `WIRE_LEG_ORDER` to `("L1","L2","L3","R3","R2","R1")` to compensate. The firmware was later updated to handle the mapping internally; with both swaps active R1/R3 appeared reversed in calibration. The Python-side swap was removed on 2026-04-27.

If the firmware is ever re-flashed with a different mapping, fix it in `WIRE_LEG_LOGICAL_INDICES` only — every production path goes through it.

## Every UART write path (audit)

All production writes go through `SerialBridge.write_line` at `motion_engine/runtime/runtime.py:268` (the only `serial.Serial.write` call). Callers:

| Path | Caller | Serializer | Wire reorder applied? |
|------|--------|------------|-----------------------|
| Web calibration nudge | `web/process_control.py:427` `send_calibration_pose` | `format_full_dir_command` | yes |
| Web dance live preview | `web/app.py:484` dance encoder cb | `format_full_dir_command` | yes |
| Web dance playback | `web/automated_runner.py:622` `_write_frame` | `format_full_dir_command` | yes |
| Production runtime loop | `runtime.py:701` `FullGaitLegBridgeMotion._send_current_frame` | `format_full_dir_command` | yes |
| CLI `gait-leg-bridge` | `runtime/main.py:166` | `format_full_dir_command` | yes |
| CLI `stand` / `walk` | `runtime/main.py:88,108` | `format_move_command` | yes |
| CLI `single-leg-*` (bench) | `runtime/main.py:128-143` | `format_move_dir_command` (1-leg) | no — bench, leg_count=1 |
| CLI `manual-bridge` | `runtime/main.py:147` | `format_angle_triplet_command` (`ANG`) | no — 3-servo legacy |
| Xbox controller | writes `runtime_override.json`; runtime loop picks it up | (via runtime loop) | yes |
| **`test/pre_check_runner.py`** | `send_frame` at line 40-44 | **hand-rolled `N:..` string** | **NO — bypasses protocol entirely** |
| `test/test_*.py` mocks | various | hand-rolled `N:..` lambdas | n/a (no real UART) |

The pre_check_runner being out-of-band is the trap: if you change `WIRE_LEG_LOGICAL_INDICES`, web/runtime/dance change but pre_check does not. Pre_check sends raw logical order to the ESP32 directly.

## Calibration data flow (web)

1. UI buttons in `web/templates/calibration.html` carry `data-calibration-leg=0..5`. `web/static/robot_ui.js` (`calibrationLegLabels`, line 31) labels them L1..R3.
2. Slider nudge → POST `/api/calibration/send` (`web/app.py:839`) with a `CalibrationPoseConfig`.
3. `web/process_control.py:427 send_calibration_pose` builds the 18-angle frame (edited leg overrides saved-neutral pose for the others), then `format_full_dir_command("N", frame)` → `SerialBridge.write_line`.
4. Save → writes `motion_engine/_cache/calibration_profile.json` (v2 schema, see `ik_engine/calibration.py`). Per-leg fields: `saved`, `offsets_deg`, optional `direction` override.
5. Runtime startup re-merges the profile via `apply_to_servo_config` into the live `ServoConfig`.

`ik_engine/config.py` defines `ServoConfig` defaults (neutral 90°, 120° effective travel, PWM 1000–2000µs). Direction tuples for R-side legs are `(-1,-1,1)`, L-side `(1,-1,1)`. Saved legs bypass the conservative bench safe-range clamp in `ik_to_servo_angles`.

## Geometry / IK quick reference

`ik_engine/config.py:GeometryConfig` (mm): coxa 254, femur 228.6, tibia 594.4, body 812.8 × 381. `LayoutConfig.build_leg_mounts` plants L1/R1 at the front corners (`+body_length/2 - corner_x_inset`), L3/R3 at the rear corners, L2/R2 mid-side. Mount angles: L = `+90°`, R = `-90°`. `default_foot_outward_offset = 0.25 * (femur+tibia)`, `default_foot_z = -0.60 * (femur+tibia)` when geometry is overridden.

## Deploying to the Pi

- Single file: `scp <local-path> raspi-cmu:/home/team6/Desktop/robot/<same-relative-path>`.
- Multiple files: prefer `rsync -av --exclude __pycache__ --exclude _cache <local> raspi-cmu:/home/team6/Desktop/robot/`.
- Always verify with a remote `grep` after pushing protocol/config changes — Google Drive sync vs. SSH copies have bitten us before.
- Don't push `_cache/` (calibration JSON is Pi-local) or `__pycache__/`.

## Gotchas

- **Two leg-ordering layers.** `WIRE_LEG_LOGICAL_INDICES` (Pi-side reorder) and the ESP32's `control_system.c` PCA9685 table. Change one without the other and R1/R3 (or R2) flips. When in doubt: keep the Pi side identity and own the mapping in firmware.
- **`pre_check_runner.py` does not use the protocol module.** Any wire-order fix must be replicated in its `send_frame` if you rely on it for hardware checks. Today it sends logical order verbatim.
- **`servo_map` in `ServoConfig` is a red herring for UART debugging.** It only feeds the on-board PCA9685 driver path in `motion.py`, which the production runtime does not use.
- `dance_engine` and `web/automated_runner` both share the same serializer as production — a wire-order regression there will show up everywhere at once.
- Calibration neutral poses can sit outside the bench safe-range; `format_full_dir_command` deliberately skips that clamp. Per-leg saved/unsaved policy is enforced upstream.
