from ur5lib.core import UR5Base
from ur5lib.types.common_types import Pose, JointAngles
from ur5lib.exceptions import NotConnectedError

try:
    import rtde_control
    import rtde_receive
except ImportError:
    rtde_control = None
    rtde_receive = None


class UR5RTDE(UR5Base):
    def __init__(self, config=None):
        super().__init__(config)
        self.rtde_c = None
        self.rtde_r = None
        self.robot_ip = self.config.get("robot_ip", "172.22.22.2")

    def connect_rtde(self):
        """
        Connects to the robot's RTDE interfaces.

        Raises:
            ImportError: If the rtde_control or rtde_receive modules are not installed.
        """
        if rtde_control is None or rtde_receive is None:
            raise ImportError("Please install rtde_control_interface and rtde_receive_interface")
        self.rtde_c = rtde_control.RTDEControlInterface(self.robot_ip)
        self.rtde_r = rtde_receive.RTDEReceiveInterface(self.robot_ip)
        self.log("RTDE interfaces initialized.")

    def get_joint_angles(self) -> JointAngles:
        """
        Retrieves the current joint angles of the robot.

        Returns:
            JointAngles: The current joint angles as a JointAngles object.

        Raises:
            NotConnectedError: If RTDE interface is not connected.
        """
        self.validate_connection()
        joints = self.rtde_r.getActualQ()
        return JointAngles(joints=joints)

    def get_current_pose(self) -> Pose:
        """
        Retrieves the current TCP (tool center point) pose of the robot.

        Returns:
            Pose: The current TCP pose as a Pose object.

        Raises:
            NotConnectedError: If RTDE interface is not connected.
        """
        self.validate_connection()
        tcp = self.rtde_r.getActualTCPPose()
        return Pose(*tcp)

    def run_motion(self, motion_plan):
        """
        Executes a motion plan given as a sequence of joint configurations.

        Args:
            motion_plan (list): A list of joint configurations (list or tuple of floats).

        Raises:
            NotConnectedError: If RTDE interface is not connected.
        """
        self.validate_connection()
        self.log("Executing joint path...")
        for point in motion_plan:
            self.rtde_c.moveJ(point, speed=0.5)
        self.log("Motion complete.")

    def moveL(self, pose: Pose, speed=0.25, acceleration=0.5, blend_radius=0.0, async=False):
        """
        Moves the robot linearly in Cartesian space to the specified pose.

        Args:
            pose (Pose): The target TCP pose to move to.
            speed (float): The speed of the movement (m/s).
            acceleration (float): The acceleration for the movement (m/s^2).
            blend_radius (float): Radius for blending with subsequent moves (meters). Default is 0 (no blending).
            async (bool): If True, move command is asynchronous; otherwise, it blocks until move completes.

        Raises:
            NotConnectedError: If RTDE interface is not connected.
        """
        self.validate_connection()
        self.log(f"Moving linearly to pose {pose} with speed={speed}, acceleration={acceleration}, blend_radius={blend_radius}")
        self.rtde_c.moveL(pose.to_list(), speed, acceleration, async, blend_radius)

    def servoJ(self, joint_angles: JointAngles, speed=1.0, acceleration=1.0, time=0.008, lookahead_time=0.1, gain=300):
        """
        Performs a servo move in joint space to the specified joint angles.

        This command allows for real-time, low-latency streaming of joint targets, ideal for fine control and coordination.

        Args:
            joint_angles (JointAngles): The target joint angles.
            speed (float): Speed of the movement (rad/s).
            acceleration (float): Acceleration of the movement (rad/s^2).
            time (float): Time over which to perform the move (seconds).
            lookahead_time (float): Lookahead time for smoothing the path (seconds).
            gain (int): Gain for servo control (higher means stiffer control).

        Raises:
            NotConnectedError: If RTDE interface is not connected.
        """
        self.validate_connection()
        self.log(f"Servo joint to angles {joint_angles.joints} with speed={speed}, acceleration={acceleration}, time={time}")
        self.rtde_c.servoJ(joint_angles.joints, speed, acceleration, time, lookahead_time, gain)

    def servoL(self, pose: Pose, speed=0.25, acceleration=0.5, time=0.008, lookahead_time=0.1, gain=300):
        """
        Performs a servo move in Cartesian space to the specified TCP pose.

        This command enables real-time, low-latency streaming of TCP positions for smooth, precise control.

        Args:
            pose (Pose): The target TCP pose.
            speed (float): Speed of the movement (m/s).
            acceleration (float): Acceleration of the movement (m/s^2).
            time (float): Time over which to perform the move (seconds).
            lookahead_time (float): Lookahead time for smoothing the path (seconds).
            gain (int): Gain for servo control (higher means stiffer control).

        Raises:
            NotConnectedError: If RTDE interface is not connected.
        """
        self.validate_connection()
        self.log(f"Servo linear to pose {pose} with speed={speed}, acceleration={acceleration}, time={time}")
        self.rtde_c.servoL(pose.to_list(), speed, acceleration, time, lookahead_time, gain)
