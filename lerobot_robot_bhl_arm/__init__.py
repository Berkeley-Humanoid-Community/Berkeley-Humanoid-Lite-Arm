from .bhl_arm import BHLArm
from .config_bhl_arm import BHLArmActuatorConfig, BHLArmConfig

BerkeleyHumanoidLiteArmRobot = BHLArm
BerkeleyHumanoidLiteArmRobotConfig = BHLArmConfig

__all__ = [
    "BHLArm",
    "BHLArmActuatorConfig",
    "BHLArmConfig",
    "BerkeleyHumanoidLiteArmRobot",
    "BerkeleyHumanoidLiteArmRobotConfig",
]
