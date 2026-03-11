import argparse

from lerobot_robot_bhl_arm import BHLArm, BHLArmConfig
from lerobot_teleoperator_bhl_arm import BHLArmSpaceMouseIKTeleop, BHLArmSpaceMouseIKTeleopConfig


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Dual SpaceMouse teleop with MuJoCo/Mink IK for BHLArm end-effector targets."
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
        default=30.0,
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
        "--translation-speed",
        type=float,
        default=0.12,
        help="Meters per second at full SpaceMouse deflection.",
    )
    parser.add_argument(
        "--rotation-speed",
        type=float,
        default=1.2,
        help="Radians per second at full SpaceMouse deflection.",
    )
    parser.add_argument(
        "--input-max-value",
        type=float,
        default=350.0,
        help="Raw SpaceMouse magnitude treated as full-scale input.",
    )
    parser.add_argument(
        "--input-deadzone",
        type=float,
        default=0.05,
        help="Normalized deadzone applied to translation and rotation inputs.",
    )
    parser.add_argument(
        "--left-device",
        type=str,
        default="",
        help="Optional hidraw path for the left-arm SpaceMouse, e.g. /dev/hidraw1.",
    )
    parser.add_argument(
        "--right-device",
        type=str,
        default="",
        help="Optional hidraw path for the right-arm SpaceMouse, e.g. /dev/hidraw2.",
    )
    parser.add_argument(
        "--xml-path",
        type=str,
        default="",
        help="Optional MuJoCo XML path. Defaults to the repo-local BHL MJCF if available.",
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
    config = BHLArmSpaceMouseIKTeleopConfig(
        control_frequency=args.frequency,
        stiffness=args.stiffness,
        damping=args.damping,
        soft_start_duration=args.soft_start_duration,
        launch_viewer=args.viewer,
        translation_speed=args.translation_speed,
        rotation_speed=args.rotation_speed,
        input_max_value=args.input_max_value,
        input_deadzone=args.input_deadzone,
        left_device_path=args.left_device or None,
        right_device_path=args.right_device or None,
    )
    if args.xml_path:
        config.robot_xml_path = args.xml_path

    teleop = BHLArmSpaceMouseIKTeleop(config)

    try:
        robot.connect(calibrate=args.calibrate)
        print(f"Connected to {robot.name} with {robot.num_joints} joints.")
        teleop.run(robot)
    finally:
        robot.disconnect()
        print("\nDisconnected.")


if __name__ == "__main__":
    main()
