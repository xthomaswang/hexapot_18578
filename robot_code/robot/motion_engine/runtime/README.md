# Runtime

Minimal runtime layout:

- `motion.py`: high-level motion control API
- `ik_engine/`: pure IK, body model, geometry, and locomotion math
- `runtime/protocol.py`: UART contract and command formatting
- `runtime/runtime.py`: serial bridge plus thin motion wrappers
- `runtime/main.py`: one CLI entry point for bring-up and bench testing

The standalone simulation workspace lives outside this deployed tree in
`/Users/tuomasier/Library/CloudStorage/GoogleDrive-thomasw3@andrew.cmu.edu/My Drive/18578/ik_engine`.
Only Pi-facing code belongs here.

## Contract

The UART contract now stays consistent across the codebase:

- Full robot move: `M:a0,a1,...,a17\n`
- Directional front-leg test: `DIR:a0,a1,a2,...\n`
- Home: `H:\n`
- Stop: `S:\n`
- Speed: `V:percent\n`
- Query: `Q:\n`

This keeps the ESP parser simple:

- `M` always means 18 angles
- `DIR` is computed by the motion engine and is one of `N`, `F`, `B`, `L`, `R`, `LL`, `RR`
- the directional command means the first `N` legs from the configured leg order
- current leg order is `L1, L2, L3, R1, R2, R3`
- `N=1` sends the first 3 angles, `N=2` sends the first 6 angles (`L1` and `L2`)

Current temporary robot-ESP bridge compatibility:

- recommended uplink from robot ESP to Pi: `CMD forward_cmd strafe_cmd turn_cmd seq\n`
- downlink from Pi to robot ESP: `ANG servo0 servo1 servo2 seq\n`

This compatibility path exists only for the current
`esp/control/main/control_system.c` bench-test firmware.

Front-leg bridge using the full gait engine:

- uplink from robot ESP to Pi: `CMD forward_cmd strafe_cmd turn_cmd seq\n`
- downlink from Pi to robot ESP: `DIR:a0,a1,a2,...\n`
- direction codes: `F`, `B`, `L`, `R`, `LL`, `RR`, `N`
- protective clamp on each transmitted leg: `coxa 60..120`, `femur 90..145`, `tibia 88..145`
- this clamp applies only to the directional bench command path; calibration poses use the raw `0..180` servo range so offsets can be discovered
- optional web override file can inject a higher-priority command when `--override-file` is set

This path is intended for the one-leg / three-servo bench test while still
using the full hexapod gait and IK pipeline on the Pi.

## Pi Setup

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r runtime/requirements.txt
```

## Run

```bash
python3 -m runtime.main stand
python3 -m runtime.main walk --forward 0.8 --strafe 0.2 --duration 5
python3 -m runtime.main single-leg-step --leg-index 0 --duration 5
python3 -m runtime.main single-leg-move --leg-index 0 --x 300 --z -500
python3 -m runtime.main manual-bridge --leg-index 0
python3 -m runtime.main gait-leg-bridge --leg-count 1
python3 -m runtime.main gait-leg-bridge --leg-count 2
python3 -m runtime.main home
python3 -m runtime.main stop
```

## Minimal Files To Copy To Pi

Copy only these files and directories:

- `motion.py`
- `ik_engine/__init__.py`
- `ik_engine/config.py`
- `ik_engine/body_model.py`
- `ik_engine/leg_ik.py`
- `ik_engine/locomotion.py`
- `runtime/__init__.py`
- `runtime/protocol.py`
- `runtime/runtime.py`
- `runtime/main.py`
- `runtime/requirements.txt`
