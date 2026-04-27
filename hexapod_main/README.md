# hexapod_main

ESP32 firmware for the hexapod robot. This is the **production target** for the
Pi-driven 6-leg 18-servo setup; all other variants under `project/archive/esp32/`
are legacy and should not be flashed.

## What it does

- UART bridge between the Raspberry Pi and the two PCA9685 PWM drivers
- Parses Pi's wire protocol (see below) and drives 18 servos across 2× PCA9685
- Reads BNO055 IMU (NDOF mode); reports Euler angles on `Q:` query
- Boot-safe: servos stay inert at power-on until the first valid Pi frame
- Responds to `S:` (stop) and `H:` (home) with a conservative midpoint pose

Target: **ESP32** (classic, not S3/C3). Tested with ESP-IDF v5.5.3.

## Hardware

| Peripheral | Bus | Address / Pin |
|---|---|---|
| PCA9685 #0 (legs 1,2,3) | I2C @ 100 kHz | `0x40` |
| PCA9685 #1 (legs 4,5,6) | I2C @ 100 kHz | `0x41` (A0 soldered) |
| BNO055 IMU | I2C @ 100 kHz | `0x28` |
| I2C SDA / SCL | GPIO | 21 / 22 |
| Pi UART | `UART_NUM_2` @ 115200 8N1 | TX=17, RX=16 |

Per-leg PCA9685 channel map (see `g_leg_map` in `control_system.c`):

| Leg | PCA | Coxa | Femur | Tibia |
|---|---|---|---|---|
| L1 | 0x40 | 0 | 1 | 2 |
| L2 | 0x40 | 4 | 5 | 6 |
| L3 | 0x40 | 8 | 9 | 10 |
| R1 | 0x41 | 0 | 1 | 2 |
| R2 | 0x41 | 4 | 5 | 6 |
| R3 | 0x41 | 8 | 9 | 10 |

Servo PWM window: 1000–2000 µs linearly mapped to 30°–150° (servo hard clamp).
Pi-side degree inputs are clamped to `[0, 180]` in float domain first, then rounded.

## Wire protocol (aligned with `robot/motion_engine/runtime/protocol.py`)

### Pi → ESP (downlink, newline-terminated)

| Line | Meaning | ESP action |
|---|---|---|
| `M:f0,f1,...,f17\n` | full-body move (18 servo degrees) | drive all 18 servos |
| `N:f0,...,f17\n` | neutral direction tag, same payload | drive all 18 servos |
| `F:` / `B:` / `L:` / `R:` / `LL:` / `RR:` | direction-tagged full frame | drive all 18 servos; tag is logged |
| `H:\n` | home | move all legs to `(90, 90, 90)` |
| `S:\n` | emergency stop | move all legs to `(90, 90, 90)`, **keep PWM active** |
| `V:<percent>\n` | speed (0–100) | record advisory value, no effect on driving |
| `Q:\n` | query status | reply with `ST:...` (see below) |

Unknown prefixes or wrong-float-count payloads are dropped with a warning log
and do **not** halt the RX task.

### ESP → Pi (uplink)

| Line | When |
|---|---|
| `ST:<heading>,<roll>,<pitch>,<uptime_ms>,<last_cmd_age_ms>\n` | in reply to `Q:` |

`last_cmd_age_ms` is `-1` until the first Pi frame is accepted. Heading/roll/pitch
are in degrees; if the BNO055 is absent or fails, all three report `0.00`.

No `CMD forward strafe turn seq` uplink. Input commands come from the Pi side
(Xbox controller / web override / future auto-drive), not from ESP peripherals.

## Boot policy (important)

At power-on the firmware:

1. Initializes I2C, both PCA9685 chips, and tries to init the BNO055 (best-effort)
2. **Does NOT drive any servo.** All PCA9685 channels remain at zero pulse width —
   servos are un-powered and will free-run (droop under load)
3. Starts the UART RX task and waits for the first Pi frame

Servos only start receiving PWM once a valid frame arrives from the Pi
(`M:`, any direction tag, `H:`, or `S:`). This prevents any involuntary snap
during boot.

**Implication:** if the Pi never sends anything after ESP boots, the servos
stay limp. This is intentional — the robot's "waking up" is the Pi's decision,
not the firmware's.

## Build / flash

IDF setup (one time):

```bash
# ESP-IDF v5.5.3 expected at ~/esp/esp-idf
# Install was run from inside conda robot env (Python 3.10) to create the right venv
```

Every build session:

```bash
conda activate robot
source ~/esp/esp-idf/export.sh
cd /path/to/project/robot_code/esp32/hexapod_main
idf.py set-target esp32        # only needed once
idf.py build
idf.py -p /dev/cu.SLAB_USBtoUART flash monitor
```

Replace `/dev/cu.SLAB_USBtoUART` with the actual USB serial device for your
ESP32 board (`ls /dev/cu.*` will show what's available).

`idf.py monitor` opens a serial monitor at 115200 showing `ESP_LOG*` output.
Exit with `Ctrl+]`.

## File layout

```
hexapod_main/
├── CMakeLists.txt          # root IDF project
├── sdkconfig.defaults      # log level, main task stack, FreeRTOS tick
├── .gitignore              # ignores build/, managed_components/, sdkconfig
├── README.md               # this file
└── main/
    ├── CMakeLists.txt      # idf_component_register
    └── control_system.c    # everything (~500 lines)
```

## Safety notes

- `S:` does NOT disable PWM output. Servos stay locked at `(90, 90, 90)` so the
  robot doesn't collapse. If you need true motor-off, power-cycle the servo rail.
- `H:` and `S:` go to the same pose in this build. Difference is semantic
  (home = normal ack, stop = warning log + intended as emergency).
- No watchdog / stale-command fallback inside the firmware. The Pi side's
  `FullGaitLegBridgeMotion.timeout_s` handles stale commands by sending a
  stand frame; the ESP just applies whatever arrives.
- Servo angles are hard-clamped to `[30, 150]` degrees at the PWM-conversion
  step. Anything Pi sends outside that range is silently clipped.
