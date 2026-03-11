import argparse
import time

from lerobot_robot_bhl_arm import BHLArm, BHLArmConfig


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Connect to the Berkeley Humanoid Lite arm and print measured joint positions."
    )
    parser.add_argument(
        "--calibrate",
        action="store_true",
        help="Run calibration if no saved calibration is available.",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=1,
        help="Number of observations to print before disconnecting.",
    )
    parser.add_argument(
        "--period",
        type=float,
        default=0.5,
        help="Seconds to wait between observations.",
    )
    parser.add_argument(
        "--full-observation",
        action="store_true",
        help="Print the full observation dict instead of positions only.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    robot = BHLArm(BHLArmConfig())

    try:
        robot.connect(calibrate=args.calibrate)
        robot.configure_stiffness_and_damping(0.0, 0.0)
        print(f"Connected to {robot.name} with {robot.num_joints} joints.")

        safe_action = {"positions": [0.0] * robot.num_joints}

        for observation_index in range(args.count):
            robot.send_action(safe_action)
            observation = robot.get_observation()
            print(f"\nObservation {observation_index + 1}/{args.count}")

            if args.full_observation:
                for key, value in observation.items():
                    print(f"{key}: {value}")
            else:
                for joint_name, measured_position in zip(robot.joint_names, robot.measured_positions, strict=True):
                    print(f"{joint_name}: {measured_position:.6f}")

            if observation_index + 1 < args.count:
                time.sleep(args.period)
    finally:
        robot.disconnect()
        print("\nDisconnected.")


if __name__ == "__main__":
    main()
