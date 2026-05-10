# ur5lib/io/simulator.py

from ur5lib.core import UR5Base
from ur5lib.ur5_types.common_types import Pose, JointAngles
import time
import random


class UR5Sim(UR5Base):
    def __init__(self, config=None):
        super().__init__(config)
        self.fake_pose = Pose(0.1, 0.2, 0.3, 0.0, 0.0, 0.0)
        self.fake_joints = JointAngles(joints=[0, 0, 0, 0, 0, 0])

    def connect_rtde(self):
        self.log("Simulator connected (no real robot).")

    def get_joint_angles(self) -> JointAngles:
        # Simulate some movement
        jitter = [j + random.uniform(-0.01, 0.01) for j in self.fake_joints.joints]
        return JointAngles(joints=jitter)

    def get_current_pose(self) -> Pose:
        # Simulate pose drift
        drift = [self.fake_pose[i] + random.uniform(-0.001, 0.001) for i in range(6)]
        return Pose(*drift)

    def run_motion(self, motion_plan):
        self.log(f"Simulating motion for {len(motion_plan)} points...")
        for point in motion_plan:
            self.fake_joints = JointAngles(joints=list(point))
            time.sleep(0.1)
        self.log("Simulation done.")

    def servoJ(self, joint_angles: JointAngles,
               speed=1.0, acceleration=1.0,
               time=0.008, lookahead_time=0.1, gain=300):
        """Simulate a single servo step by updating the internal joint state."""
        self.fake_joints = JointAngles(joints=list(joint_angles.joints))
