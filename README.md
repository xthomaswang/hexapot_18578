# HexaPod — CMU Mechatronics 2026, Team 6

Project repository for **Peas in a HexaPod**, a six-legged walking robot built for
18-578 / 16-778 / 24-778 Mechatronic Design (Spring 2026).

Team: Thomas Wang, Dillon Ma, Leena Aljurashi, Mithun Roy.

## Repository layout

```
hexapot_18578/
├── docs/              # Project website (deployed via GitHub Pages)
├── hexapod_main/      # ESP-IDF firmware project for the on-board ESP32
├── robot_code/        # Application code that runs on the Raspberry Pi + ESP32
└── README.md
```

### `docs/` — project website
Static site documenting the project per the 18-578 website rubric.
- `index.html` — landing page
- `pages/` — system-design, implementation, performance, management, media,
  team, and documents pages
- `assets/` — CSS, JS, images, and PDFs (ILRs, design proposal, presentations)

### `hexapod_main/` — ESP-IDF firmware
CMake-based ESP-IDF project for the ESP32 that drives the servos and handles
low-level motor control. Build with `idf.py build` from this directory.

### `robot_code/` — robot application code
Code that runs on the Raspberry Pi (high-level control) and a single-file
ESP32 controller used during bring-up.

```
robot_code/
├── esp32/
│   └── control_system.c          # Standalone ESP32 servo controller
└── robot/                        # Raspberry Pi application
    ├── motion_engine/            # Inverse kinematics, gait, and runtime
    │   ├── ik_engine/            # Per-leg IK, body model, calibration, locomotion
    │   └── runtime/              # Main loop, Xbox controller, ESP32 protocol
    ├── dance_engine/             # Dance recording and playback storage
    ├── vision/                   # Raspberry Pi camera + AprilTag detection
    ├── web/                      # Flask web UI (dashboard, manual/auto modes,
    │                             #   calibration, camera stream, dance editor)
    └── test/                     # pytest suite for the above modules
```

## Quick start

**Website (local preview):**
```
cd docs && python3 -m http.server 8000
```

**Raspberry Pi web UI:**
```
cd robot_code/robot/web
pip install -r requirements.txt
python app.py
```

**ESP32 firmware:**
```
cd hexapod_main
idf.py build flash monitor
```

## Documentation
The full project website (system design, implementation progress, performance
results, ILRs, presentations) lives in `docs/` and is the canonical reference
for course deliverables.
