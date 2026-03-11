import argparse

from lerobot_robot_bhl_arm import BHLArm, BHLArmConfig
from lerobot_teleoperator_bhl_arm import BHLArmSteamVRIKTeleop, BHLArmSteamVRIKTeleopConfig


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="SteamVR teleop with MuJoCo/Mink IK for BHLArm end-effector targets."
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
        "--steamvr-host",
        type=str,
        default="0.0.0.0",
        help="UDP host to bind for incoming SteamVR packets.",
    )
    parser.add_argument(
        "--steamvr-port",
        type=int,
        default=11005,
        help="UDP port to bind for incoming SteamVR packets.",
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
    config = BHLArmSteamVRIKTeleopConfig(
        control_frequency=args.frequency,
        stiffness=args.stiffness,
        damping=args.damping,
        soft_start_duration=args.soft_start_duration,
        launch_viewer=args.viewer,
        steamvr_host=args.steamvr_host,
        steamvr_port=args.steamvr_port,
    )
    if args.xml_path:
        config.robot_xml_path = args.xml_path

    teleop = BHLArmSteamVRIKTeleop(config)

    try:
        robot.connect(calibrate=args.calibrate)
        print(f"Connected to {robot.name} with {robot.num_joints} joints.")
        teleop.run(robot)
    finally:
        robot.disconnect()
        print("\nDisconnected.")


if __name__ == "__main__":
    main()
