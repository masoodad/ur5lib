from ur5lib.io.simulator import UR5Sim
from ur5lib.motion.executor import MotionExecutor
from ur5lib.ur5_types.common_types import JointAngles

robot = UR5Sim()
robot.connect_()

executor = MotionExecutor(robot)
target = JointAngles(joints=[0.1, -0.5, 0.3, -1.2, 1.5, 0.0])
executor.move_to_joint_position(target)
