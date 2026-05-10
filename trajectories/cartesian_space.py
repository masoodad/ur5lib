# ur5lib/trajectories/cartesian_space.py

import numpy as np
from typing import List
from ur5lib.types.common_types import JointAngles


class CartesianLinearTrajectory:
    """
    Piecewise-linear Cartesian path via dense joint-space interpolation.

    By choosing closely-spaced waypoints (small Cartesian steps) and using
    a high step density, the TCP traces an approximately straight line between
    consecutive waypoints.  This mirrors how industrial controllers implement
    MOVEL (linear Cartesian move) commands.

    Properties
    ----------
    NAME       : "Cartesian Linear"
    BEST_FOR   : Precise straight-line tool paths (welding, dispensing)
    COMPLEXITY : Medium
    SMOOTHNESS : High
    """

    NAME = "Cartesian Linear"
    BEST_FOR = "Precise tool path"
    COMPLEXITY = "Medium"
    SMOOTHNESS = "High"

    def __init__(self, waypoints: List[JointAngles], steps_per_segment: int = 60):
        if len(waypoints) < 2:
            raise ValueError("CartesianLinearTrajectory requires at least 2 waypoints.")
        self.waypoints = waypoints
        self.steps_per_segment = steps_per_segment

    def generate(self) -> List[List[float]]:
        """Return dense joint-angle lists that approximate Cartesian linearity."""
        trajectory: List[List[float]] = []
        n = len(self.waypoints)
        for i in range(n - 1):
            s = np.array(self.waypoints[i].joints)
            g = np.array(self.waypoints[i + 1].joints)
            endpoint = (i == n - 2)
            for alpha in np.linspace(0.0, 1.0, self.steps_per_segment,
                                     endpoint=endpoint):
                trajectory.append((s + alpha * (g - s)).tolist())
        trajectory.append(self.waypoints[-1].joints)
        return trajectory
