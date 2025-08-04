from ur5lib.io.simulator import UR5Sim
from ur5lib.motion.executor import MotionExecutor
from ur5lib.types.common_types import JointAngles

# 1. Make the robot
robot = UR5Sim()

# 2. Wake it up (connect)
robot.connect_()

# 3. Tell it where to move (in radians)
joints = JointAngles(joints=[0.1, -0.5, 0.3, -1.2, 1.5, 0.0])

# 4. Make a helper that sends motion commands
executor = MotionExecutor(robot)

# 5. Move the robot!
executor.move_to_joint_position(joints)
