from dataclasses import dataclass, field
from math import radians
from pathlib import Path

from lerobot.teleoperators.config import TeleoperatorConfig

from lerobot_robot_bhl_arm.config_bhl_arm import BHLArmConfig

from .config_bhl_arm_ik import BHLArmIKConfig
from .motion_retargeting import default_bhl_mjcf_path


def _default_joint_names() -> tuple[str, ...]:
    return tuple(BHLArmConfig().actuator_configs)


def _default_initial_joint_positions() -> tuple[float, ...]:
    return (
        0.0,
        0.02,
        0.0,
        -1.68,
        0.0,
        0.0,
        0.0,
        0.0,
        0.02,
        0.0,
        -1.68,
        0.0,
        0.0,
        0.0,
    )


@TeleoperatorConfig.register_subclass("bhl_arm_keyboard_ik")
@dataclass(kw_only=True)
class BHLArmKeyboardIKTeleopConfig(TeleoperatorConfig):
    control_frequency: float = 20.0
    ee_position_step: float = 0.01
    ee_rotation_step: float = radians(5.0)
    stiffness: float = 8.0
    damping: float = 1.0
    soft_start_duration: float = 2.0
    shutdown_stiffness: float = 0.0
    shutdown_damping: float = 8.0
    robot_xml_path: str = field(default_factory=lambda: str(default_bhl_mjcf_path()))
    joint_names: tuple[str, ...] = field(default_factory=_default_joint_names)
    initial_joint_positions: tuple[float, ...] = field(default_factory=_default_initial_joint_positions)
    left_body_name: str = "left_fingertip"
    right_body_name: str = "right_fingertip"
    launch_viewer: bool = False
    left_forward_key: str = "w"
    left_backward_key: str = "s"
    left_left_key: str = "a"
    left_right_key: str = "d"
    left_up_key: str = "r"
    left_down_key: str = "f"
    left_roll_positive_key: str = "q"
    left_roll_negative_key: str = "e"
    left_yaw_positive_key: str = "z"
    left_yaw_negative_key: str = "x"
    right_forward_key: str = "i"
    right_backward_key: str = "k"
    right_left_key: str = "j"
    right_right_key: str = "l"
    right_up_key: str = "y"
    right_down_key: str = "h"
    right_roll_positive_key: str = "u"
    right_roll_negative_key: str = "o"
    right_yaw_positive_key: str = "n"
    right_yaw_negative_key: str = "m"
    reset_target_key: str = "c"
    quit_key: str = "p"
    ik: BHLArmIKConfig = field(default_factory=BHLArmIKConfig)

    @property
    def controlled_joints(self) -> tuple[str, ...]:
        return self.joint_names

    @property
    def body_names(self) -> tuple[str, str]:
        return (self.left_body_name, self.right_body_name)

    def robot_xml(self) -> Path:
        return Path(self.robot_xml_path).expanduser().resolve()

    def describe_controls(self) -> list[str]:
        return [
            "Left arm:",
            (
                f"  {self.left_forward_key}/{self.left_backward_key}: +x / -x translation, "
                f"{self.left_left_key}/{self.left_right_key}: +y / -y translation, "
                f"{self.left_up_key}/{self.left_down_key}: +z / -z translation"
            ),
            (
                f"  {self.left_roll_positive_key}/{self.left_roll_negative_key}: +x roll / -x roll, "
                f"{self.left_yaw_positive_key}/{self.left_yaw_negative_key}: +z yaw / -z yaw"
            ),
            "Right arm:",
            (
                f"  {self.right_forward_key}/{self.right_backward_key}: +x / -x translation, "
                f"{self.right_left_key}/{self.right_right_key}: +y / -y translation, "
                f"{self.right_up_key}/{self.right_down_key}: +z / -z translation"
            ),
            (
                f"  {self.right_roll_positive_key}/{self.right_roll_negative_key}: +x roll / -x roll, "
                f"{self.right_yaw_positive_key}/{self.right_yaw_negative_key}: +z yaw / -z yaw"
            ),
            f"{self.reset_target_key}: reset both targets to current poses",
            f"{self.quit_key}: enter damping mode",
        ]
