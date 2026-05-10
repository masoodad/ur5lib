# ur5lib/control/safety.py
"""
Pre-execution safety checks for UR5 trajectories.

``SafetyChecker`` validates joint configurations and full trajectories against
hardware limits before any command is sent to the robot.  This prevents
protective stops, joint damage, and out-of-range commands.

UR5 hardware limits (from Universal Robots specification)
---------------------------------------------------------
Position  : ±360° (±2π rad) on all six joints
Velocity  : ≤ 180°/s (π rad/s) per joint
Acceleration : ≤ 400°/s² (≈ 6.98 rad/s²) per joint
"""

import numpy as np

from ur5lib.exceptions import JointLimitError, SafetyViolationError

# ---------------------------------------------------------------------------
# UR5 hardware limits
# ---------------------------------------------------------------------------
UR5_JOINT_LIMITS = [(-2 * np.pi, 2 * np.pi)] * 6   # (lower, upper) per joint

UR5_MAX_JOINT_VEL  = np.pi               # rad/s  (180°/s)
UR5_MAX_JOINT_ACC  = 400.0 * np.pi / 180.0  # rad/s² (400°/s²)


class SafetyChecker:
    """
    Validates joint configurations and trajectories against position,
    velocity, and acceleration limits.

    Parameters
    ----------
    joint_limits : list of (lower, upper) tuples, one per joint.
                   Defaults to ``UR5_JOINT_LIMITS``.
    v_max        : float — per-joint velocity limit (rad/s).
                   Defaults to ``UR5_MAX_JOINT_VEL``.
    a_max        : float — per-joint acceleration limit (rad/s²).
                   Defaults to ``UR5_MAX_JOINT_ACC``.
    """

    def __init__(self, joint_limits=None, v_max=None, a_max=None):
        self.joint_limits = joint_limits or UR5_JOINT_LIMITS
        self.v_max = v_max if v_max is not None else UR5_MAX_JOINT_VEL
        self.a_max = a_max if a_max is not None else UR5_MAX_JOINT_ACC

    # ------------------------------------------------------------------
    # Single-configuration check
    # ------------------------------------------------------------------

    def check_configuration(self, joints: list) -> list:
        """
        Check whether *joints* violates any position limit.

        Returns
        -------
        list[str]  — human-readable violation messages; empty means safe.
        """
        violations = []
        for i, (q, (lo, hi)) in enumerate(zip(joints, self.joint_limits)):
            if q < lo or q > hi:
                violations.append(
                    f"Joint {i + 1}: {np.degrees(q):.2f}° outside "
                    f"[{np.degrees(lo):.0f}°, {np.degrees(hi):.0f}°]"
                )
        return violations

    def validate_configuration(self, joints: list) -> None:
        """
        Raise :class:`~ur5lib.exceptions.JointLimitError` if *joints*
        violates any position limit.
        """
        violations = self.check_configuration(joints)
        if violations:
            raise JointLimitError(
                "Joint limit violation(s):\n  " + "\n  ".join(violations)
            )

    # ------------------------------------------------------------------
    # Trajectory check
    # ------------------------------------------------------------------

    def check_trajectory(self, trajectory: list, dt: float) -> list:
        """
        Check an entire trajectory for position, velocity, and acceleration
        violations.

        Parameters
        ----------
        trajectory : list[list[float]] — sequence of joint configurations
        dt         : float             — time step between configurations (s)

        Returns
        -------
        list[str] — all violation messages; empty means safe.
        """
        violations = []

        pts = [np.array(q) for q in trajectory]

        for k, q in enumerate(pts):
            for msg in self.check_configuration(q.tolist()):
                violations.append(f"Step {k}: {msg}")

        if dt <= 0:
            return violations

        # Velocity check  (finite differences)
        for k in range(1, len(pts)):
            vel = np.abs(pts[k] - pts[k - 1]) / dt
            for j, v in enumerate(vel):
                if v > self.v_max:
                    violations.append(
                        f"Step {k}, Joint {j + 1}: velocity "
                        f"{np.degrees(v):.1f}°/s > limit "
                        f"{np.degrees(self.v_max):.0f}°/s"
                    )

        # Acceleration check  (second finite differences)
        for k in range(2, len(pts)):
            acc = np.abs(pts[k] - 2 * pts[k - 1] + pts[k - 2]) / dt ** 2
            for j, a in enumerate(acc):
                if a > self.a_max:
                    violations.append(
                        f"Step {k}, Joint {j + 1}: acceleration "
                        f"{np.degrees(a):.1f}°/s² > limit "
                        f"{np.degrees(self.a_max):.0f}°/s²"
                    )

        return violations

    def validate_trajectory(self, trajectory: list, dt: float) -> None:
        """
        Raise :class:`~ur5lib.exceptions.SafetyViolationError` if the
        trajectory contains any position, velocity, or acceleration violation.
        """
        violations = self.check_trajectory(trajectory, dt)
        if violations:
            # Show first five violations to keep the message readable
            shown = violations[:5]
            tail  = f"\n  … and {len(violations) - 5} more." \
                    if len(violations) > 5 else ""
            raise SafetyViolationError(
                f"Trajectory has {len(violations)} safety violation(s):\n  "
                + "\n  ".join(shown) + tail
            )
