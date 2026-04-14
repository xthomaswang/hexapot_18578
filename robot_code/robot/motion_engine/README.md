# Motion Engine

This directory is the deployed motion stack for the Raspberry Pi.

## Boundaries

- `motion.py`
  High-level motion control API. This layer turns normalized movement intent
  such as forward, strafe, and turn into foot-target updates and servo-angle
  snapshots.
- `ik_engine/`
  Pure geometry, inverse kinematics, body model transforms, and locomotion
  primitives. Detailed file roles live in `ik_engine/README.md`.
- `runtime/`
  Raspberry Pi runtime glue: UART protocol, serial bridge, CLI entry point,
  control-command parsing, and web override support.

## What stays out of this folder

- MuJoCo and matplotlib simulation tooling live in the standalone desktop
  [ik_engine](/Users/tuomasier/Library/CloudStorage/GoogleDrive-thomasw3@andrew.cmu.edu/My%20Drive/18578/ik_engine)
  workspace, not in the Pi deployment tree.

## Practical rule

If code needs `/dev/serial0`, UART message formatting, or Pi process bring-up,
it belongs in `runtime/`.
If code turns high-level movement intent into body-frame foot motion, it
belongs in `motion.py`.
If code only needs robot geometry, body-frame transforms, locomotion
trajectories, or servo-angle math, it belongs in `ik_engine/`.
