# ur5lib/types/common_types.py

from typing import List, NamedTuple


class Pose(NamedTuple):
    x: float
    y: float
    z: float
    rx: float
    ry: float
    rz: float


class JointAngles(NamedTuple):
    joints: List[float]  # Should be 6 for UR5
