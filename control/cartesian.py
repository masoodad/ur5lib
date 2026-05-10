# ur5lib/control/cartesian.py
"""
Jacobian-based Cartesian-space velocity controller.

``CartesianController`` maps a desired TCP pose to joint-velocity commands
using the damped least-squares pseudoinverse of the geometric Jacobian.
It is suitable for real-time Cartesian tracking at a fixed servo rate.

The controller does **not** require an IK solution — it works directly
in velocity space and can therefore follow curved Cartesian paths that
would be expensive to solve analytically.

Usage
-----
>>> from ur5lib.control.cartesian import CartesianController
>>> from ur5lib.types.common_types import Pose
>>>
>>> ctrl = CartesianController(robot, kp=1.5)
>>> q_cmd = ctrl.step(target_pose=Pose(0.4, 0.0, 0.5, 0.0, 3.14, 0.0), dt=0.008)
"""

import numpy as np

from ur5lib.types.common_types import Pose
from ur5lib.control.kinematics import tcp_pose, geometric_jacobian


class CartesianController:
    """
    One-step Cartesian velocity controller using the Jacobian pseudoinverse.

    At each step the controller:
    1. Reads the current joint state from the robot.
    2. Computes FK to get the current TCP pose.
    3. Builds the geometric Jacobian J(q).
    4. Computes the pose error Δx = x_target − x_current.
    5. Solves  q̇ = J⁺ · (Kp · Δx)  using damped least squares.
    6. Returns the updated joint position  q + q̇ · dt.

    Parameters
    ----------
    robot     : UR5Base subclass — must implement ``get_joint_angles()``
    kp        : float — Cartesian proportional gain (default 1.0)
    lam       : float — DLS damping factor (default 0.05); increase to
                        avoid singularity amplification
    dh_params : (n, 3) ndarray or None — robot DH table; None → UR5
    """

    def __init__(self, robot, kp: float = 1.0,
                 lam: float = 0.05, dh_params=None):
        self.robot     = robot
        self.kp        = kp
        self.lam       = lam
        self.dh_params = dh_params

    def step(self, target_pose: Pose, dt: float = 0.008) -> list:
        """
        Compute the next joint position command toward *target_pose*.

        Parameters
        ----------
        target_pose : Pose  — desired TCP pose
        dt          : float — servo time step (seconds)

        Returns
        -------
        list[float] — joint position command for the next servo tick
        """
        q = list(self.robot.get_joint_angles().joints)
        current = tcp_pose(q, self.dh_params)

        dx = np.array([
            target_pose.x  - current.x,
            target_pose.y  - current.y,
            target_pose.z  - current.z,
            target_pose.rx - current.rx,
            target_pose.ry - current.ry,
            target_pose.rz - current.rz,
        ])

        J     = geometric_jacobian(q, self.dh_params)
        J_dls = J.T @ np.linalg.inv(J @ J.T + self.lam ** 2 * np.eye(6))
        q_dot = J_dls @ (self.kp * dx)

        return (np.array(q) + q_dot * dt).tolist()

    def move_to_pose(self, target_pose: Pose,
                     tol: float = 1e-3,
                     max_steps: int = 500,
                     dt: float = 0.008) -> list:
        """
        Iterate :meth:`step` until the TCP is within *tol* of *target_pose*
        or *max_steps* is reached.

        Suitable for simulation; on real hardware use :class:`ControlExecutor`
        for proper servo timing.

        Parameters
        ----------
        target_pose : Pose
        tol         : float — convergence threshold on ‖Δx‖ (metres + rad)
        max_steps   : int
        dt          : float

        Returns
        -------
        list[float] — final joint configuration reached
        """
        from ur5lib.types.common_types import JointAngles

        q = list(self.robot.get_joint_angles().joints)

        for _ in range(max_steps):
            current = tcp_pose(q, self.dh_params)
            dx = np.array([
                target_pose.x  - current.x,
                target_pose.y  - current.y,
                target_pose.z  - current.z,
                target_pose.rx - current.rx,
                target_pose.ry - current.ry,
                target_pose.rz - current.rz,
            ])
            if np.linalg.norm(dx) < tol:
                break

            J     = geometric_jacobian(q, self.dh_params)
            J_dls = J.T @ np.linalg.inv(J @ J.T + self.lam ** 2 * np.eye(6))
            q_dot = J_dls @ (self.kp * dx)
            q     = (np.array(q) + q_dot * dt).tolist()

            if hasattr(self.robot, 'servoJ'):
                self.robot.servoJ(JointAngles(joints=q),
                                  speed=1.0, acceleration=0.5,
                                  time=dt, lookahead_time=0.1, gain=300)
            else:
                self.robot.run_motion([q])

        return q
