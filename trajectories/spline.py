# ur5lib/trajectories/spline.py

import numpy as np
from typing import List
from ur5lib.types.common_types import JointAngles


class SplineTrajectory:
    """
    Catmull-Rom spline interpolation through joint-space waypoints.

    Produces C¹-continuous trajectories (position *and* velocity are
    continuous at waypoints), which eliminates the sharp corners present
    in joint-space LERP and reduces mechanical jerk.  This is well-suited
    for painting, coating, or any operation that requires a smooth,
    uninterrupted TCP path.

    Algorithm
    ---------
    For each interior segment (p1 → p2) the Catmull-Rom formula uses the
    neighbouring control points p0 and p3:

        q(t) = ½ · [ 2p1
                     + (−p0 + p2) t
                     + (2p0 − 5p1 + 4p2 − p3) t²
                     + (−p0 + 3p1 − 3p2 + p3) t³ ]

    Endpoint tangents are mirrored (p_ghost = 2·p_end − p_next).

    Properties
    ----------
    NAME       : "Circular / Spline"
    BEST_FOR   : Smooth continuous motion (painting, coating, arc-welding)
    COMPLEXITY : High
    SMOOTHNESS : Very High
    """

    NAME = "Circular / Spline"
    BEST_FOR = "Smooth continuous motion"
    COMPLEXITY = "High"
    SMOOTHNESS = "Very High"

    def __init__(self, waypoints: List[JointAngles], steps_per_segment: int = 50):
        if len(waypoints) < 3:
            raise ValueError("SplineTrajectory requires at least 3 waypoints.")
        self.waypoints = waypoints
        self.steps_per_segment = steps_per_segment

    @staticmethod
    def _catmull_rom(p0: np.ndarray, p1: np.ndarray,
                     p2: np.ndarray, p3: np.ndarray,
                     t: float) -> np.ndarray:
        """Evaluate Catmull-Rom spline at parameter t ∈ [0, 1]."""
        return 0.5 * (
            2.0 * p1
            + (-p0 + p2) * t
            + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * (t ** 2)
            + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * (t ** 3)
        )

    def generate(self) -> List[List[float]]:
        """Return spline-interpolated joint-angle lists."""
        pts = [np.array(w.joints) for w in self.waypoints]

        # Ghost control points at both ends for natural boundary behaviour
        p_start_ghost = 2.0 * pts[0] - pts[1]
        p_end_ghost   = 2.0 * pts[-1] - pts[-2]
        ctrl = [p_start_ghost] + pts + [p_end_ghost]

        trajectory: List[List[float]] = []
        n_seg = len(pts) - 1
        for i in range(n_seg):
            p0, p1, p2, p3 = ctrl[i], ctrl[i + 1], ctrl[i + 2], ctrl[i + 3]
            endpoint = (i == n_seg - 1)
            for t in np.linspace(0.0, 1.0, self.steps_per_segment,
                                 endpoint=endpoint):
                q = self._catmull_rom(p0, p1, p2, p3, t)
                trajectory.append(q.tolist())

        trajectory.append(pts[-1].tolist())
        return trajectory
