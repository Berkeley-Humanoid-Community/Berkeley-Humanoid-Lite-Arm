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


@TeleoperatorConfig.register_subclass("bhl_arm_spacemouse_ik")
@dataclass(kw_only=True)
class BHLArmSpaceMouseIKTeleopConfig(TeleoperatorConfig):
    control_frequency: float = 30.0
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
    left_device_path: str | None = None
    right_device_path: str | None = None
    launch_viewer: bool = False
    input_max_value: float = 350.0
    input_deadzone: float = 0.05
    translation_speed: float = 0.12
    rotation_speed: float = 1.2
    left_translation_signs: tuple[float, float, float] = (1.0, 1.0, 1.0)
    right_translation_signs: tuple[float, float, float] = (1.0, 1.0, 1.0)
    left_rotation_signs: tuple[float, float, float] = (1.0, 1.0, 1.0)
    right_rotation_signs: tuple[float, float, float] = (1.0, 1.0, 1.0)
    log_interval_s: float = 0.1
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
            "Left SpaceMouse: control left end effector pose",
            "Right SpaceMouse: control right end effector pose",
            "Left button on either SpaceMouse: reset that arm target to the current pose",
            "Ctrl+C once: enter damping mode",
            "Ctrl+C again: exit",
        ]
