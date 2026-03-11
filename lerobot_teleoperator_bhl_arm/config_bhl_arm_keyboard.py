from dataclasses import dataclass
from math import radians

from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("bhl_arm_keyboard")
@dataclass(kw_only=True)
class BHLArmKeyboardTeleopConfig(TeleoperatorConfig):
    control_frequency: float = 20.0
    position_step: float = radians(2.0)
    stiffness: float = 1.0
    damping: float = 1.0
    soft_start_duration: float = 2.0
    shutdown_stiffness: float = 0.0
    shutdown_damping: float = 8.0
    left_positive_key: str = "q"
    left_negative_key: str = "a"
    right_positive_key: str = "w"
    right_negative_key: str = "s"
    quit_key: str = "x"

    @property
    def controlled_joints(self) -> tuple[str, str]:
        return ("left_wrist_yaw", "right_wrist_yaw")

    def describe_controls(self) -> list[str]:
        return [
            f"{self.left_positive_key}: increase left_wrist_yaw",
            f"{self.left_negative_key}: decrease left_wrist_yaw",
            f"{self.right_positive_key}: increase right_wrist_yaw",
            f"{self.right_negative_key}: decrease right_wrist_yaw",
            f"{self.quit_key}: enter damping mode",
        ]
