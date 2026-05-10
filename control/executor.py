# ur5lib/control/executor.py
"""
Closed-loop trajectory executor using the robot's servoJ interface.

``ControlExecutor`` is the top-level control entry point.  It replaces the
open-loop ``MotionExecutor`` for applications that require accurate tracking:

  Open-loop (MotionExecutor)  →  reads state once → sends all waypoints
  Closed-loop (ControlExecutor) →  reads state at every tick → PID corrects → servoJ

At each servo tick the executor:
  1. Reads the actual joint state from the robot.
  2. Computes the per-joint error between the reference waypoint and the measurement.
  3. Applies :class:`~ur5lib.control.joint_pid.JointPID` to produce a corrected command.
  4. Optionally validates the command through :class:`~ur5lib.control.safety.SafetyChecker`.
  5. Sends the command via ``robot.servoJ`` (or falls back to ``run_motion``
     for backends that do not implement servoJ).

Usage — plain trajectory
------------------------
>>> from ur5lib.io.simulator import UR5Sim
>>> from ur5lib.control.executor import ControlExecutor
>>> from ur5lib.trajectories.spline import SplineTrajectory
>>>
>>> robot = UR5Sim(); robot.connect_()
>>> traj  = SplineTrajectory(waypoints).generate()
>>> ControlExecutor(robot).execute(traj)

Usage — with safety + custom PID
---------------------------------
>>> from ur5lib.control.joint_pid import JointPID
>>> from ur5lib.control.safety   import SafetyChecker
>>>
>>> pid     = JointPID(kp=3.0, ki=0.1, kd=0.2, dt=0.008)
>>> safety  = SafetyChecker()
>>> executor = ControlExecutor(robot, pid=pid, safety=safety)
>>> executor.execute(traj, dt=0.008)

Usage — time-parameterised trajectory (from a profile)
-------------------------------------------------------
>>> from ur5lib.control.profiles import TrapezoidalProfile
>>>
>>> timed = TrapezoidalProfile(v_max=1.0, a_max=2.0).parameterize(traj)
>>> executor.execute_timed(timed)
"""

import time as _time

from ur5lib.types.common_types import JointAngles
from ur5lib.control.joint_pid  import JointPID
from ur5lib.control.safety     import SafetyChecker


class ControlExecutor:
    """
    Closed-loop trajectory executor.

    Parameters
    ----------
    robot   : UR5Base subclass — connected robot (sim or real)
    pid     : JointPID or None — defaults to JointPID(kp=2.0, ki=0.05, kd=0.1, dt=0.008)
    safety  : SafetyChecker or None — when provided, the trajectory is
              validated before execution and each command is checked before
              being sent. Pass ``None`` to skip all safety checks.
    """

    def __init__(self, robot, pid: JointPID = None,
                 safety: SafetyChecker = None):
        self.robot  = robot
        self.pid    = pid or JointPID()
        self.safety = safety

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def execute(self, trajectory: list, dt: float = 0.008) -> None:
        """
        Execute *trajectory* in closed-loop at a fixed servo rate *dt*.

        Parameters
        ----------
        trajectory : list[list[float]] — joint configurations from any
                     trajectory generator's ``generate()`` method.
        dt         : float — servo period in seconds (default 8 ms / 125 Hz).
                     Used both as the PID time step and the servoJ time
                     parameter.
        """
        if self.safety:
            self.safety.validate_trajectory(trajectory, dt)

        self.pid.reset()

        for q_ref in trajectory:
            self._servo_step(list(q_ref), dt)

    def execute_timed(self, timed_trajectory: list) -> None:
        """
        Execute a time-stamped trajectory produced by a profile class.

        The executor sleeps between steps to honour the requested timestamps,
        making this suitable for hardware-accurate playback.

        Parameters
        ----------
        timed_trajectory : list of (float, list[float])
            Output of ``TrapezoidalProfile.parameterize()`` or
            ``SCurveProfile.parameterize()`` — pairs of (t_seconds, joints).
        """
        if not timed_trajectory:
            return

        trajectory  = [q for _, q in timed_trajectory]
        timestamps  = [t for t, _ in timed_trajectory]

        if self.safety and len(timestamps) > 1:
            dt_avg = (timestamps[-1] - timestamps[0]) / max(len(timestamps) - 1, 1)
            self.safety.validate_trajectory(trajectory, dt_avg)

        self.pid.reset()
        t_start = _time.monotonic()

        for (t_target, q_ref) in timed_trajectory:
            # Sleep until the scheduled time
            t_elapsed = _time.monotonic() - t_start
            wait = t_target - t_elapsed
            if wait > 0:
                _time.sleep(wait)

            dt = t_target - (timestamps[timed_trajectory.index((t_target, q_ref)) - 1]
                              if timed_trajectory.index((t_target, q_ref)) > 0 else t_target)
            dt = max(dt, 1e-4)
            self._servo_step(list(q_ref), dt)

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _servo_step(self, q_ref: list, dt: float) -> None:
        """Read actual state, apply PID correction, send one servo command."""
        q_actual = list(self.robot.get_joint_angles().joints)
        q_cmd    = self.pid.step(q_ref, q_actual)

        if self.safety:
            self.safety.validate_configuration(q_cmd)

        if hasattr(self.robot, 'servoJ'):
            self.robot.servoJ(
                JointAngles(joints=q_cmd),
                speed=1.0, acceleration=0.5,
                time=dt, lookahead_time=0.1, gain=300,
            )
        else:
            self.robot.run_motion([q_cmd])
