from lerobot.robots import RobotConfig

import lerobot_robot_bhl_arm  # noqa: F401


def list_robots() -> None:
    print("Registered LeRobot robot types:\n")
    for robot_type in RobotConfig.get_known_choices():
        print(f"- {robot_type}")


if __name__ == "__main__":
    list_robots()
