import argparse

from lerobot_robot_bhl_arm import BHLArm, BHLArmConfig
from lerobot_teleoperator_bhl_arm import BHLArmKeyboardIKTeleop, BHLArmKeyboardIKTeleopConfig


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Keyboard teleop with MuJoCo/Mink IK for BHLArm end-effector targets."
    )
    parser.add_argument(
        "--calibrate",
        action="store_true",
        help="Run calibration if no saved calibration is available.",
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
    parser.add_argument(
        "--ee-step",
        type=float,
        default=0.01,
        help="Cartesian target step size in meters per keypress.",
    )
    parser.add_argument(
        "--xml-path",
        type=str,
        default="",
        help="Optional MuJoCo XML path. Defaults to the sibling BHL workspace path if available.",
    )
    parser.add_argument(
        "--viewer",
        action="store_true",
        help="Launch MuJoCo passive viewer with current/target frame markers.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    robot = BHLArm(BHLArmConfig())
    config = BHLArmKeyboardIKTeleopConfig(
        control_frequency=args.frequency,
        ee_position_step=args.ee_step,
        stiffness=args.stiffness,
        damping=args.damping,
        soft_start_duration=args.soft_start_duration,
        launch_viewer=args.viewer,
    )
    if args.xml_path:
        config.robot_xml_path = args.xml_path

    teleop = BHLArmKeyboardIKTeleop(config)

    try:
        robot.connect(calibrate=args.calibrate)
        print(f"Connected to {robot.name} with {robot.num_joints} joints.")
        teleop.run(robot)
    finally:
        robot.disconnect()
        print("\nDisconnected.")


if __name__ == "__main__":
    main()
