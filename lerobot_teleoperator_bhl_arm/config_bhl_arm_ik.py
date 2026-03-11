from dataclasses import dataclass
from math import radians


@dataclass(kw_only=True)
class BHLArmIKConfig:
    """Shared IK tuning parameters for BHL teleoperators."""

    solver: str = "daqp"
    damping: float = 1e-3
    step_size: float = 1.0
    max_joint_delta: float = radians(7.0)
    position_weight: float = 1.0
    orientation_weight: float = 0.25
    max_iterations: int = 30
    error_tolerance: float = 1e-4
