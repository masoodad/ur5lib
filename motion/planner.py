# ur5lib/motion/planner.py

from typing import List
from ur5lib.types.common_types import JointAngles, Pose
import numpy as np


class MotionPlanner:
    def __init__(self, num_points: int = 10):
        self.num_points = num_points

    def plan_joint_motion(self, start: JointAngles, goal: JointAngles) -> List[List[float]]:
        """Linearly interpolate joint space"""
        motion_plan = []
        start_array = np.array(start.joints)
        goal_array = np.array(goal.joints)

        for alpha in np.linspace(0, 1, self.num_points):
            point = (1 - alpha) * start_array + alpha * goal_array
            motion_plan.append(point.tolist())

        return motion_plan

    def plan_cartesian_motion(self, start: Pose, goal: Pose) -> List[Pose]:
        """Linearly interpolate Cartesian space"""
        motion_plan = []
        start_array = np.array(start)
        goal_array = np.array(goal)

        for alpha in np.linspace(0, 1, self.num_points):
            point = (1 - alpha) * start_array + alpha * goal_array
            motion_plan.append(Pose(*point))

        return motion_plan
