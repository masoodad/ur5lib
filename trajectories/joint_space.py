# ur5lib/trajectories/joint_space.py

import numpy as np
from typing import List
from ur5lib.ur5_types.common_types import JointAngles


class JointSpaceTrajectory:
    """
    Linear (LERP) interpolation between joint-space waypoints.

    Each segment is independently interpolated; waypoints are stitched
    together without velocity continuity, which produces sharp corners in
    Cartesian space but is computationally cheap and easy to plan.

    Properties
    ----------
    NAME       : "Joint Space"
    BEST_FOR   : Fast motion between sparse waypoints
    COMPLEXITY : Low
    SMOOTHNESS : Medium
    """

    NAME = "Joint Space"
    BEST_FOR = "Fast motion"
    COMPLEXITY = "Low"
    SMOOTHNESS = "Medium"

    def __init__(self, waypoints: List[JointAngles], steps_per_segment: int = 40):
        if len(waypoints) < 2:
            raise ValueError("JointSpaceTrajectory requires at least 2 waypoints.")
        self.waypoints = waypoints
        self.steps_per_segment = steps_per_segment

    def generate(self) -> List[List[float]]:
        """Return list of joint-angle lists via linear interpolation."""
        trajectory: List[List[float]] = []
        n = len(self.waypoints)
        for i in range(n - 1):
            s = np.array(self.waypoints[i].joints)
            g = np.array(self.waypoints[i + 1].joints)
            # Include endpoint only on the last segment
            endpoint = (i == n - 2)
            for alpha in np.linspace(0.0, 1.0, self.steps_per_segment,
                                     endpoint=endpoint):
                trajectory.append((s + alpha * (g - s)).tolist())
        trajectory.append(self.waypoints[-1].joints)
        return trajectory
