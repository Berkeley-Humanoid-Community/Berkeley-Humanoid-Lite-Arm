import argparse
from math import radians

from lerobot_robot_bhl_arm import BHLArm, BHLArmConfig
from lerobot_teleoperator_bhl_arm import BHLArmKeyboardTeleop, BHLArmKeyboardTeleopConfig


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Keyboard teleop sanity test for BHLArm wrist yaw joints."
    )
    parser.add_argument(
        "--calibrate",
        action="store_true",
        help="Run calibration if no saved calibration is available.",
    )
    parser.add_argument(
        "--step-deg",
        type=float,
        default=2.0,
        help="Increment size in degrees for each keypress.",
    )
    parser.add_argument(
        "--stiffness",
        type=float,
        default=8.0,
        help="Joint stiffness to apply during teleop.",
    )
    parser.add_argument(
        "--damping",
        type=float,
        default=1.0,
        help="Joint damping to apply during teleop.",
    )
    parser.add_argument(
        "--frequency",
        type=float,
        default=20.0,
        help="Control loop frequency in Hz.",
    )
    parser.add_argument(
        "--soft-start-duration",
        "--ramp-duration",
        dest="soft_start_duration",
        type=float,
        default=2.0,
        help="Seconds to softly apply the configured stiffness and damping.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    robot = BHLArm(BHLArmConfig())
    teleop = BHLArmKeyboardTeleop(
        BHLArmKeyboardTeleopConfig(
            control_frequency=args.frequency,
            position_step=radians(args.step_deg),
            stiffness=args.stiffness,
            damping=args.damping,
            soft_start_duration=args.soft_start_duration,
        )
    )

    try:
        robot.connect(calibrate=args.calibrate)
        print(f"Connected to {robot.name} with {robot.num_joints} joints.")
        teleop.run(robot)
    finally:
        robot.disconnect()
        print("\nDisconnected.")


if __name__ == "__main__":
    main()
