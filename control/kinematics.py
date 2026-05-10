# ur5lib/control/kinematics.py
"""
Kinematic utilities for any serial manipulator.

DH parameters come from ``ur5lib.robots`` and are never hardcoded here.
All functions accept an optional *dh_params* argument so they work with
robots other than the UR5.

Public API
----------
  forward_kinematics_transforms(joints, dh_params)  → list of (n+1) 4×4 ndarrays
  tcp_pose(joints, dh_params)                        → Pose
  geometric_jacobian(joints, dh_params)              → (6, n) ndarray
  ik_numerical(target_pose, q_init, ...)             → list[float]
"""

import numpy as np

from ur5lib.robots import UR5_DH
from ur5lib.types.common_types import Pose
from ur5lib.exceptions import KinematicsError


# ---------------------------------------------------------------------------
# Private helpers
# ---------------------------------------------------------------------------

def _dh_matrix(theta: float, a: float, d: float, alpha: float) -> np.ndarray:
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.,  sa,       ca,      d     ],
        [0.,  0.,       0.,      1.    ],
    ])


def _rot_to_rotvec(R: np.ndarray) -> np.ndarray:
    """Convert a 3×3 rotation matrix to an axis-angle rotation vector."""
    angle = np.arccos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))

    if abs(angle) < 1e-10:
        return np.zeros(3)

    if np.pi - abs(angle) < 1e-7:
        # angle ≈ π: extract axis from (R + I) / 2 = n ⊗ n
        M = (R + np.eye(3)) / 2.0
        axis = np.sqrt(np.maximum(np.diag(M), 0.0))
        if M[0, 1] < 0.0:
            axis[1] = -axis[1]
        if M[0, 2] < 0.0:
            axis[2] = -axis[2]
        n = np.linalg.norm(axis)
        return angle * axis / n if n > 1e-12 else np.zeros(3)

    axis = np.array([R[2, 1] - R[1, 2],
                     R[0, 2] - R[2, 0],
                     R[1, 0] - R[0, 1]]) / (2.0 * np.sin(angle))
    return angle * axis


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def forward_kinematics_transforms(joints: list,
                                  dh_params: np.ndarray = None) -> list:
    """
    Compute the homogeneous transforms T_0_i for every joint frame.

    Parameters
    ----------
    joints    : sequence of float (radians), length == DOF
    dh_params : (n, 3) array [a, d, alpha].  None → UR5_DH.

    Returns
    -------
    list of (n+1) 4×4 ndarrays
        Index 0 is the base frame (identity).
        Index i is T from base to frame i.
    """
    if dh_params is None:
        dh_params = UR5_DH
    transforms = [np.eye(4)]
    T = np.eye(4)
    for theta, (a, d, alpha) in zip(joints, dh_params):
        T = T @ _dh_matrix(theta, a, d, alpha)
        transforms.append(T.copy())
    return transforms


def tcp_pose(joints: list, dh_params: np.ndarray = None) -> Pose:
    """
    Compute the TCP pose from joint angles (forward kinematics).

    Returns a :class:`Pose` with position (x, y, z) in metres and
    orientation as an axis-angle rotation vector (rx, ry, rz) in radians,
    matching the UR robot convention.

    Parameters
    ----------
    joints    : sequence of float (radians)
    dh_params : (n, 3) array.  None → UR5_DH.
    """
    transforms = forward_kinematics_transforms(joints, dh_params)
    T = transforms[-1]
    pos = T[:3, 3]
    rot = _rot_to_rotvec(T[:3, :3])
    return Pose(pos[0], pos[1], pos[2], rot[0], rot[1], rot[2])


def geometric_jacobian(joints: list, dh_params: np.ndarray = None) -> np.ndarray:
    """
    Compute the geometric Jacobian J(q) relating joint velocities to TCP velocity.

    ẋ = J(q) · q̇   where  ẋ = [vx, vy, vz, ωx, ωy, ωz]ᵀ

    For each revolute joint i:
      - Linear part:  z_{i-1} × (p_e − p_{i-1})
      - Angular part: z_{i-1}

    Parameters
    ----------
    joints    : sequence of float (radians)
    dh_params : (n, 3) array.  None → UR5_DH.

    Returns
    -------
    np.ndarray, shape (6, n)
    """
    if dh_params is None:
        dh_params = UR5_DH
    transforms = forward_kinematics_transforms(joints, dh_params)
    p_e = transforms[-1][:3, 3]
    n = len(joints)
    J = np.zeros((6, n))
    for i in range(n):
        T_prev = transforms[i]
        z = T_prev[:3, 2]      # z-axis of frame i-1 in base frame
        p = T_prev[:3, 3]      # origin of frame i-1 in base frame
        J[:3, i] = np.cross(z, p_e - p)
        J[3:, i] = z
    return J


def ik_numerical(target_pose: Pose,
                 q_init: list,
                 dh_params: np.ndarray = None,
                 tol: float = 1e-4,
                 max_iter: int = 200,
                 lam: float = 0.05) -> list:
    """
    Solve inverse kinematics numerically via damped least-squares iteration.

    Iterates:  q ← q + J⁺(q) · Δx
    where J⁺ = Jᵀ(JJᵀ + λ²I)⁻¹  (damped pseudoinverse).

    Parameters
    ----------
    target_pose : Pose  — desired TCP pose (position + axis-angle orientation)
    q_init      : list  — initial joint guess (radians)
    dh_params   : (n, 3) array or None
    tol         : float — convergence tolerance on ‖Δx‖ (default 1e-4)
    max_iter    : int   — maximum iterations (default 200)
    lam         : float — damping factor (default 0.05)

    Returns
    -------
    list[float] — joint angles that achieve *target_pose* within *tol*

    Raises
    ------
    KinematicsError  — if the solver does not converge within *max_iter*
    """
    if dh_params is None:
        dh_params = UR5_DH
    q = np.array(q_init, dtype=float)
    err = np.inf

    for _ in range(max_iter):
        current = tcp_pose(q, dh_params)
        dx = np.array([
            target_pose.x  - current.x,
            target_pose.y  - current.y,
            target_pose.z  - current.z,
            target_pose.rx - current.rx,
            target_pose.ry - current.ry,
            target_pose.rz - current.rz,
        ])
        err = float(np.linalg.norm(dx))
        if err < tol:
            return q.tolist()

        J = geometric_jacobian(q, dh_params)
        J_dls = J.T @ np.linalg.inv(J @ J.T + lam ** 2 * np.eye(6))
        q = q + J_dls @ dx

    raise KinematicsError(
        f"IK did not converge after {max_iter} iterations "
        f"(final error = {err:.6f}, tolerance = {tol})."
    )
