# ur5lib/cli.py

import argparse
from ur5lib.io.simulator import UR5Sim
from ur5lib.motion.executor import MotionExecutor
from ur5lib.ur5_types.common_types import JointAngles


def main():
    parser = argparse.ArgumentParser(description="UR5Lib CLI Interface")
    parser.add_argument('--move-joints', nargs=6, type=float, metavar=('J1', 'J2', 'J3', 'J4', 'J5', 'J6'),
                        help='Move to joint positions in radians')
    parser.add_argument('--sim', action='store_true', help='Use simulator instead of RTDE')
    parser.add_argument('--ip', type=str, help='UR5 IP for RTDE control')
    args = parser.parse_args()

    if args.sim:
        robot = UR5Sim()
    else:
        from ur5lib.io.ur_rtde import UR5RTDE
        robot = UR5RTDE(config={"robot_ip": args.ip or "192.168.0.100"})

    robot.connect_()
    executor = MotionExecutor(robot)

    if args.move_joints:
        joints = JointAngles(joints=args.move_joints)
        executor.move_to_joint_position(joints)


if __name__ == '__main__':
    main()
