# ur5lib/control/joint_pid.py
"""
Discrete-time PID controllers for joint-space closed-loop control.

Classes
-------
PIDController
    Single-axis PID with anti-windup integral clamping.

JointPID
    Convenience wrapper: one PIDController per joint, driven by a
    (q_ref, q_actual) pair to produce per-joint command corrections.

Usage
-----
>>> from ur5lib.control.joint_pid import JointPID
>>> pid = JointPID(kp=2.0, ki=0.05, kd=0.2, dt=0.008)
>>> q_cmd = pid.step(q_ref=[0.5, -1.0, 0.3, -1.5, 0.0, 0.0],
...                  q_actual=[0.48, -0.98, 0.31, -1.49, 0.01, 0.0])
"""

from typing import List, Tuple, Union


class PIDController:
    """
    Single-axis discrete-time PID with integral anti-windup.

    The control law is:

        u[k] = Kp·e[k]  +  Ki·Σe·dt  +  Kd·(e[k] − e[k-1])/dt

    The integral term is clamped to *output_limits* to prevent wind-up.

    Parameters
    ----------
    kp            : float — proportional gain
    ki            : float — integral gain
    kd            : float — derivative gain
    dt            : float — sample period (seconds)
    output_limits : (min, max) — clamp on the total output (default ±π)
    """

    def __init__(self,
                 kp: float,
                 ki: float,
                 kd: float,
                 dt: float,
                 output_limits: Tuple[float, float] = (-3.14159, 3.14159)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self._out_min, self._out_max = output_limits
        self._integral = 0.0
        self._prev_error = 0.0

    def reset(self) -> None:
        """Reset integrator and derivative memory."""
        self._integral   = 0.0
        self._prev_error = 0.0

    def step(self, error: float) -> float:
        """
        Advance one time step.

        Parameters
        ----------
        error : float — reference − actual

        Returns
        -------
        float — control output u[k]
        """
        self._integral += error * self.dt
        # Anti-windup: clamp integral contribution
        i_term = self.ki * self._integral
        i_term = max(self._out_min, min(self._out_max, i_term))
        self._integral = i_term / self.ki if self.ki != 0 else self._integral

        derivative = (error - self._prev_error) / self.dt
        self._prev_error = error

        u = self.kp * error + i_term + self.kd * derivative
        return max(self._out_min, min(self._out_max, u))


class JointPID:
    """
    One :class:`PIDController` per robot joint.

    Gains can be supplied as a single scalar (applied to all joints) or as a
    list with one value per joint.

    Parameters
    ----------
    kp  : float or list[float] — proportional gain(s)
    ki  : float or list[float] — integral gain(s)
    kd  : float or list[float] — derivative gain(s)
    dt  : float                — sample period (seconds)
    n_joints : int             — number of joints (default 6)
    """

    def __init__(self,
                 kp: Union[float, List[float]] = 2.0,
                 ki: Union[float, List[float]] = 0.05,
                 kd: Union[float, List[float]] = 0.1,
                 dt: float = 0.008,
                 n_joints: int = 6):
        self.n_joints = n_joints

        def _expand(v):
            return v if isinstance(v, (list, tuple)) else [v] * n_joints

        kp_list = _expand(kp)
        ki_list = _expand(ki)
        kd_list = _expand(kd)

        self._pids = [
            PIDController(kp_list[i], ki_list[i], kd_list[i], dt)
            for i in range(n_joints)
        ]

    def reset(self) -> None:
        """Reset all per-joint controllers."""
        for pid in self._pids:
            pid.reset()

    def step(self, q_ref: list, q_actual: list) -> list:
        """
        Compute corrected joint commands for one servo tick.

        The correction is added to the reference position so that the
        commanded value steers the robot toward the reference while
        compensating for tracking error:

            q_cmd[i] = q_ref[i] + pid_i.step(q_ref[i] - q_actual[i])

        Parameters
        ----------
        q_ref    : list[float] — reference joint configuration (radians)
        q_actual : list[float] — measured joint configuration (radians)

        Returns
        -------
        list[float] — corrected joint command
        """
        return [
            q_ref[i] + self._pids[i].step(q_ref[i] - q_actual[i])
            for i in range(self.n_joints)
        ]
