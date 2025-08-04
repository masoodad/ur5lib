# ur5lib/io/ur_rtde.py

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
        if rtde_control is None or rtde_receive is None:
            raise ImportError("Please install rtde_control_interface and rtde_receive_interface")
        self.rtde_c = rtde_control.RTDEControlInterface(self.robot_ip)
        self.rtde_r = rtde_receive.RTDEReceiveInterface(self.robot_ip)
        self.log("RTDE interfaces initialized.")

    def get_joint_angles(self) -> JointAngles:
        self.validate_connection()
        joints = self.rtde_r.getActualQ()
        return JointAngles(joints=joints)

    def get_current_pose(self) -> Pose:
        self.validate_connection()
        tcp = self.rtde_r.getActualTCPPose()
        return Pose(*tcp)

    def run_motion(self, motion_plan):
        self.validate_connection()
        self.log("Executing joint path...")
        for point in motion_plan:
            self.rtde_c.moveJ(point, speed=0.5)
        self.log("Motion complete.")
