from lerobot.teleoperators.config import TeleoperatorConfig

import lerobot_teleoperator_bhl_arm  # noqa: F401


def list_teleoperators() -> None:
    print("Registered LeRobot teleoperator types:\n")
    for teleop_type in TeleoperatorConfig.get_known_choices():
        print(f"- {teleop_type}")


if __name__ == "__main__":
    list_teleoperators()
