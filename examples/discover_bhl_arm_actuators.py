import argparse

from actuator_control import RobstrideBus

from lerobot_robot_bhl_arm import BHLArmConfig


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Scan the default Berkeley Humanoid Lite arm CAN buses and list connected actuators."
    )
    parser.add_argument(
        "--start-id",
        type=int,
        default=1,
        help="First actuator ID to scan.",
    )
    parser.add_argument(
        "--end-id",
        type=int,
        default=50,
        help="Last actuator ID to scan.",
    )
    return parser.parse_args()


def expected_actuators_for_bus(config: BHLArmConfig, bus_name: str) -> dict[int, str]:
    return {
        actuator_config.id: actuator_name
        for actuator_name, actuator_config in config.actuator_configs.items()
        if actuator_name.startswith(f"{bus_name}_")
    }


def main() -> None:
    args = parse_args()
    config = BHLArmConfig()

    for bus_name, channel in config.bus_configs.items():
        expected = expected_actuators_for_bus(config, bus_name)
        detected = RobstrideBus.scan_channel(channel, start_id=args.start_id, end_id=args.end_id)
        detected_ids = sorted(detected)

        print(f"\n[{bus_name}] channel={channel}")
        print(f"Detected actuator IDs: {detected_ids}")

        if not detected_ids:
            print("No actuators detected.")
            continue

        for actuator_id in detected_ids:
            actuator_name = expected.get(actuator_id)
            if actuator_name is None:
                print(f"- id={actuator_id}: not in default BHLArmConfig")
            else:
                print(f"- id={actuator_id}: {actuator_name}")


if __name__ == "__main__":
    main()
