# IK Engine

`ik_engine/` is the pure motion-math layer for the deployed robot stack.
It should not know about:

- `/dev/serial0`
- `pyserial`
- Flask / web dashboard
- Pi process management
- ESP32 UART parsing

If a file needs those things, it belongs in `runtime/`, not here.

## File Roles

- `config.py`
  Static robot description. Geometry, body layout, servo map, direction flips,
  angle offsets, and the default standing pose all live here.
- `leg_ik.py`
  One-leg math only. It solves a single 3-DOF leg in leg-local coordinates and
  converts between IK joint angles and servo angles.
- `body_model.py`
  Body model. It stores all foot targets in body coordinates, transforms them
  into each leg's local frame, then runs single-leg IK across all legs.
- `locomotion.py`
  Reusable trajectory helpers. It contains tripod phase grouping plus the
  translation and in-place rotation foot trajectories.
- `__init__.py`
  Convenience exports for low-level math callers.

## Internal Data Flow

`config.py`
-> `locomotion.py` chooses phase offsets and body-frame foot motion
-> `motion.py` advances gait state and updates desired foot targets
-> `body_model.py` converts body-frame targets into leg-local targets
-> `leg_ik.py` solves each leg and returns joint / servo angles

Nothing in that flow should write serial bytes directly.

## Practical Mental Model

- `config.py` answers: what robot are we solving for?
- `locomotion.py` answers: where should each foot go over time?
- `body_model.py` answers: how do body targets map into each leg frame?
- `leg_ik.py` answers: what joint angles reach that target?
- `motion.py` answers: how do joystick-like commands drive the gait state?

## Keep / Move Guidance

Keep in `ik_engine/`:

- geometry constants
- mount/layout definitions
- body-frame foot target logic
- gait trajectories
- IK and FK
- servo-angle conversion math
- controller state that only depends on normalized motion commands

Move out of `ik_engine/` if added later:

- UART message strings
- serial port defaults
- ESP command parsing
- override-file I/O
- process lifecycle / logging
- anything Pi-device-specific

## Current Cleanup Notes

- `__pycache__/` is runtime artifact, not source.
