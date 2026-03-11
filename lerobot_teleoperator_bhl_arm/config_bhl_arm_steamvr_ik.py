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


@TeleoperatorConfig.register_subclass("bhl_arm_steamvr_ik")
@dataclass(kw_only=True)
class BHLArmSteamVRIKTeleopConfig(TeleoperatorConfig):
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
    launch_viewer: bool = False
    steamvr_host: str = "0.0.0.0"
    steamvr_port: int = 11005
    log_interval_s: float = 0.1
    max_packet_age_s: float = 1.0
    ik: BHLArmIKConfig = field(default_factory=BHLArmIKConfig)

    @property
    def controlled_joints(self) -> tuple[str, ...]:
        return self.joint_names

    @property
    def body_names(self) -> tuple[str, str]:
        return (self.left_body_name, self.right_body_name)

    def robot_xml(self) -> Path:
        return Path(self.robot_xml_path).expanduser().resolve()
