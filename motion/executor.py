# ur5lib/motion/executor.py

from ur5lib.motion.planner import MotionPlanner
from ur5lib.core import UR5Base
from ur5lib.types.common_types import Pose, JointAngles


class MotionExecutor:
    def __init__(self, robot: UR5Base, planner: MotionPlanner = None):
        self.robot = robot
        self.planner = planner or MotionPlanner()

    def move_to_joint_position(self, target: JointAngles):
        current = self.robot.get_joint_angles()
        path = self.planner.plan_joint_motion(current, target)
        self.robot.run_motion(path)

    def move_to_pose(self, target: Pose):
        current = self.robot.get_current_pose()
        path = self.planner.plan_cartesian_motion(current, target)
        self.robot.run_motion(path)
