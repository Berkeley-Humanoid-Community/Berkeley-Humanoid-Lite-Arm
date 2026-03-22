from dataclasses import dataclass, field
from math import radians

from lerobot.cameras import CameraConfig
from lerobot.robots import RobotConfig


@dataclass(kw_only=True)
class BHLArmActuatorConfig:
    id: int
    """ CAN ID of the actuator. """

    model: str
    """ Robstride model of the actuator. """

    direction: int
    """ Direction of the actuator. +1 for clockwise, -1 for counterclockwise.

    This converts the actuator's rotation direction to the robot's urdf joint direction.
    """

    lower_limit: float
    """ Lower limit of the actuator's joint position, in radians. """

    upper_limit: float
    """ Upper limit of the actuator's joint position, in radians. """


def _default_bus_configs() -> dict[str, str]:
    return {
        "left": "can0",
        "right": "can1",
    }


def _default_actuator_configs() -> dict[str, BHLArmActuatorConfig]:
    return {
        "left_shoulder_pitch": BHLArmActuatorConfig(
            id=11,
            model="rs-02",
            direction=-1,
            lower_limit=radians(-180),
            upper_limit=radians(45),
        ),
        "left_shoulder_roll": BHLArmActuatorConfig(
            id=12,
            model="rs-00",
            direction=-1,
            lower_limit=radians(-90),
            upper_limit=radians(110),
        ),
        "left_shoulder_yaw": BHLArmActuatorConfig(
            id=13,
            model="rs-00",
            direction=1,
            lower_limit=radians(-90),
            upper_limit=radians(90),
        ),
        "left_elbow_pitch": BHLArmActuatorConfig(
            id=14,
            model="rs-00",
            direction=-1,
            lower_limit=radians(-135),
            upper_limit=radians(0),
        ),
        "left_wrist_yaw": BHLArmActuatorConfig(
            id=15,
            model="rs-05",
            direction=1,
            lower_limit=radians(-90),
            upper_limit=radians(90),
        ),
        "left_wrist_roll": BHLArmActuatorConfig(
            id=16,
            model="rs-05",
            direction=-1,
            lower_limit=radians(-40),
            upper_limit=radians(40),
        ),
        "left_wrist_pitch": BHLArmActuatorConfig(
            id=17,
            model="rs-05",
            direction=-1,
            lower_limit=radians(-45),
            upper_limit=radians(45),
        ),
        "right_shoulder_pitch": BHLArmActuatorConfig(
            id=21,
            model="rs-02",
            direction=1,
            lower_limit=radians(-180),
            upper_limit=radians(45),
        ),
        "right_shoulder_roll": BHLArmActuatorConfig(
            id=22,
            model="rs-00",
            direction=-1,
            lower_limit=radians(-110),
            upper_limit=radians(90),
        ),
        "right_shoulder_yaw": BHLArmActuatorConfig(
            id=23,
            model="rs-00",
            direction=1,
            lower_limit=radians(-90),
            upper_limit=radians(90),
        ),
        "right_elbow_pitch": BHLArmActuatorConfig(
            id=24,
            model="rs-00",
            direction=1,
            lower_limit=radians(-135),
            upper_limit=radians(0),
        ),
        "right_wrist_yaw": BHLArmActuatorConfig(
            id=25,
            model="rs-05",
            direction=1,
            lower_limit=radians(-90),
            upper_limit=radians(90),
        ),
        "right_wrist_roll": BHLArmActuatorConfig(
            id=26,
            model="rs-05",
            direction=1,
            lower_limit=radians(-40),
            upper_limit=radians(40),
        ),
        "right_wrist_pitch": BHLArmActuatorConfig(
            id=27,
            model="rs-05",
            direction=-1,
            lower_limit=radians(-45),
            upper_limit=radians(45),
        ),
    }


@RobotConfig.register_subclass("berkeley_humanoid_lite_arm")
@dataclass(kw_only=True)
class BHLArmConfig(RobotConfig):
    """Configuration class for Berkeley Humanoid Lite Arm, following the LeRobot API."""

    control_frequency: float = 50.0
    """ Frequency of the communication with actuators, in Hz. """

    bus_configs: dict[str, str] = field(default_factory=_default_bus_configs)
    """ Configuration for the bus channels for each side of the arm. """

    actuator_configs: dict[str, BHLArmActuatorConfig] = field(
        default_factory=_default_actuator_configs
    )
    """ Configuration for the actuators on each side of the arm. """

    stiffness: float = 0.0
    """ Stiffness (kP) of the actuators, in Nm/rad. """

    damping: float = 0.0
    """ Damping (kD) of the actuators, in Nm/rad/s. """

    soft_start_enabled: bool = False
    """ When True, the actuators will gradually increase the kP kD gains to the target values on startup. """

    soft_start_duration: float = 0.0
    """ Duration of the soft start, in seconds. """

    soft_start_show_progress: bool = False
    """ Whether to show progress bar of the soft start. """

    disable_on_disconnect: bool = False
    """ Whether to disable the actuators when the robot is disconnected. """

    cameras: dict[str, CameraConfig] = field(default_factory=dict)
    """ Configuration for the cameras to capture images from. """
