"""CLI entry point for the minimal Raspberry Pi motion runtime."""

from __future__ import annotations

import argparse

from .protocol import DEFAULT_SERIAL_PORT
from .runtime import (
    DEFAULT_CALIBRATION_FILE,
    DEFAULT_COMMAND_TIMEOUT_S,
    DEFAULT_GAIT_FREQUENCY_HZ,
    DEFAULT_GAIT_LIFT_SCALE,
    DEFAULT_GAIT_STANCE_Z_MM,
    DEFAULT_GAIT_STRIDE_MM,
    DEFAULT_GAIT_YAW_STRIDE_DEG,
    UPDATE_INTERVAL_S,
    FullGaitLegBridgeMotion,
    ManualBridgeMotion,
    RuntimeMotionCore,
    SerialBridge,
    SingleLegMotion,
    build_motion_controller,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Hexapod Raspberry Pi runtime")
    parser.add_argument("--port", default=DEFAULT_SERIAL_PORT)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--calibration-file", default=DEFAULT_CALIBRATION_FILE)

    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("home")
    subparsers.add_parser("stop")
    subparsers.add_parser("stand")

    walk = subparsers.add_parser("walk")
    walk.add_argument("--forward", type=float, default=0.8)
    walk.add_argument("--strafe", type=float, default=0.0)
    walk.add_argument("--turn", type=float, default=0.0)
    walk.add_argument("--duration", type=float, default=5.0)

    single_leg_stand = subparsers.add_parser("single-leg-stand")
    single_leg_stand.add_argument("--leg-index", type=int, default=0)

    single_leg_step = subparsers.add_parser("single-leg-step")
    single_leg_step.add_argument("--leg-index", type=int, default=0)
    single_leg_step.add_argument("--duration", type=float, default=5.0)

    single_leg_move = subparsers.add_parser("single-leg-move")
    single_leg_move.add_argument("--leg-index", type=int, default=0)
    single_leg_move.add_argument("--x", type=float, required=True)
    single_leg_move.add_argument("--y", type=float, default=0.0)
    single_leg_move.add_argument("--z", type=float, required=True)

    manual_bridge = subparsers.add_parser("manual-bridge")
    manual_bridge.add_argument("--leg-index", type=int, default=0)
    manual_bridge.add_argument("--duration", type=float, default=0.0)
    manual_bridge.add_argument("--cmd-scale", type=float, default=1000.0)
    manual_bridge.add_argument("--timeout", type=float, default=DEFAULT_COMMAND_TIMEOUT_S)
    manual_bridge.add_argument("--deadzone", type=int, default=100)
    manual_bridge.add_argument("--stride", type=float, default=DEFAULT_GAIT_STRIDE_MM)
    manual_bridge.add_argument("--lift-scale", type=float, default=DEFAULT_GAIT_LIFT_SCALE)
    manual_bridge.add_argument("--frequency", type=float, default=DEFAULT_GAIT_FREQUENCY_HZ)
    manual_bridge.add_argument("--yaw-stride-deg", type=float, default=DEFAULT_GAIT_YAW_STRIDE_DEG)
    manual_bridge.add_argument("--stance-z", type=float, default=DEFAULT_GAIT_STANCE_Z_MM)

    gait_leg_bridge = subparsers.add_parser("gait-leg-bridge")
    gait_leg_bridge.add_argument("--duration", type=float, default=0.0)
    gait_leg_bridge.add_argument("--cmd-scale", type=float, default=1000.0)
    gait_leg_bridge.add_argument("--timeout", type=float, default=DEFAULT_COMMAND_TIMEOUT_S)
    gait_leg_bridge.add_argument("--deadzone", type=int, default=100)
    gait_leg_bridge.add_argument("--stride", type=float, default=DEFAULT_GAIT_STRIDE_MM)
    gait_leg_bridge.add_argument("--lift-scale", type=float, default=DEFAULT_GAIT_LIFT_SCALE)
    gait_leg_bridge.add_argument("--frequency", type=float, default=DEFAULT_GAIT_FREQUENCY_HZ)
    gait_leg_bridge.add_argument("--yaw-stride-deg", type=float, default=DEFAULT_GAIT_YAW_STRIDE_DEG)
    gait_leg_bridge.add_argument("--stance-z", type=float, default=DEFAULT_GAIT_STANCE_Z_MM)
    gait_leg_bridge.add_argument("--override-file", default="runtime_override.json")

    return parser


def main() -> None:
    args = build_parser().parse_args()

    with SerialBridge(port=args.port, baudrate=args.baudrate) as bridge:
        if args.command == "home":
            bridge.home()
            return

        if args.command == "stop":
            bridge.stop()
            return

        if args.command == "stand":
            controller = build_motion_controller(calibration_file=args.calibration_file)
            core = RuntimeMotionCore(controller=controller)
            bridge.send_angles(core.stand())
            return

        if args.command == "walk":
            import time as _time

            controller = build_motion_controller(calibration_file=args.calibration_file)
            core = RuntimeMotionCore(controller=controller)
            bridge.send_angles(core.stand())

            end_time = _time.monotonic() + args.duration
            try:
                while _time.monotonic() < end_time:
                    frame = core.tick(
                        forward=args.forward,
                        strafe=args.strafe,
                        turn=args.turn,
                    )
                    bridge.send_angles(frame)
                    _time.sleep(UPDATE_INTERVAL_S)
            finally:
                try:
                    bridge.send_angles(core.stand())
                except Exception:
                    pass
            return

        if args.command == "single-leg-stand":
            motion = SingleLegMotion(bridge)
            motion.controller.leg_index = args.leg_index
            motion.stand()
            return

        if args.command == "single-leg-step":
            motion = SingleLegMotion(bridge)
            motion.controller.leg_index = args.leg_index
            motion.stand()
            motion.run(duration=args.duration)
            return

        if args.command == "single-leg-move":
            motion = SingleLegMotion(bridge)
            motion.controller.leg_index = args.leg_index
            motion.move_to(x=args.x, y=args.y, z=args.z)
            return

        if args.command == "manual-bridge":
            motion = ManualBridgeMotion(
                bridge,
                leg_index=args.leg_index,
                cmd_scale=args.cmd_scale,
                timeout_s=args.timeout,
                deadzone=args.deadzone,
                verbose=args.verbose,
                stride=args.stride,
                lift_scale=args.lift_scale,
                frequency=args.frequency,
                yaw_stride_deg=args.yaw_stride_deg,
                stance_z=args.stance_z,
                calibration_file=args.calibration_file,
            )
            duration = None if args.duration <= 0 else args.duration
            motion.run(duration=duration)
            return

        if args.command == "gait-leg-bridge":
            motion = FullGaitLegBridgeMotion(
                bridge,
                cmd_scale=args.cmd_scale,
                timeout_s=args.timeout,
                deadzone=args.deadzone,
                verbose=args.verbose,
                override_file=args.override_file,
                stride=args.stride,
                lift_scale=args.lift_scale,
                frequency=args.frequency,
                yaw_stride_deg=args.yaw_stride_deg,
                stance_z=args.stance_z,
                calibration_file=args.calibration_file,
            )
            duration = None if args.duration <= 0 else args.duration
            motion.run(duration=duration)


if __name__ == "__main__":
    main()
