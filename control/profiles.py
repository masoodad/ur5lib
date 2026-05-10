# ur5lib/control/profiles.py
"""
Time parameterisation for joint-space trajectories.

A trajectory generator (JointSpaceTrajectory, SplineTrajectory, …) produces
a list of joint configurations with no time information.  The profile classes
here assign a timestamp to every configuration so that a control loop can
track both *position* and *velocity*.

Classes
-------
TrapezoidalProfile
    Accelerate at constant rate → cruise → decelerate.
    Maximum velocity and acceleration are respected exactly.

SCurveProfile
    Quintic-smoothstep position profile.
    Velocity and acceleration ramp in smoothly — zero jerk at start/end.
    Mechanically gentler than trapezoidal; suitable for precision tasks.

Both classes expose the same interface::

    profile = TrapezoidalProfile(v_max=1.0, a_max=2.0)
    timed   = profile.parameterize(trajectory)   # list of (t, joints)
"""

import numpy as np


class TrapezoidalProfile:
    """
    Assigns timestamps using a trapezoidal joint-space velocity profile.

    The arc length in joint space (Euclidean norm of configuration-space
    increments) is computed and then mapped to time via the trapezoidal law.

    Parameters
    ----------
    v_max : float  — peak joint-space speed (rad/s equivalent, default 1.0)
    a_max : float  — peak joint-space acceleration (default 2.0)
    """

    def __init__(self, v_max: float = 1.0, a_max: float = 2.0):
        if v_max <= 0 or a_max <= 0:
            raise ValueError("v_max and a_max must be positive.")
        self.v_max = v_max
        self.a_max = a_max

    def parameterize(self, trajectory: list) -> list:
        """
        Parameters
        ----------
        trajectory : list[list[float]]  — joint configurations

        Returns
        -------
        list of (float, list[float]) — (timestamp_seconds, joints) pairs
        """
        pts = [np.array(q) for q in trajectory]
        dists = [float(np.linalg.norm(pts[i + 1] - pts[i]))
                 for i in range(len(pts) - 1)]
        S = sum(dists)

        if S < 1e-12:
            return [(0.0, list(trajectory[0]))]

        # Cumulative arc lengths
        cumul = np.zeros(len(pts))
        for i, d in enumerate(dists):
            cumul[i + 1] = cumul[i] + d

        # Determine profile shape
        t_acc  = self.v_max / self.a_max
        s_acc  = self.v_max ** 2 / (2.0 * self.a_max)

        if 2.0 * s_acc >= S:
            # Triangular: no cruise phase
            v_peak = np.sqrt(self.a_max * S)
            t_acc  = v_peak / self.a_max
            s_acc  = S / 2.0
            v_c    = v_peak
            T      = 2.0 * t_acc
        else:
            v_c = self.v_max
            T   = 2.0 * t_acc + (S - 2.0 * s_acc) / v_c

        def _s_to_t(s: float) -> float:
            if s <= s_acc:
                return np.sqrt(2.0 * s / self.a_max)
            if s <= S - s_acc:
                return t_acc + (s - s_acc) / v_c
            remaining = max(S - s, 0.0)
            return T - np.sqrt(2.0 * remaining / self.a_max)

        return [(_s_to_t(s), list(q))
                for s, q in zip(cumul, trajectory)]


class SCurveProfile:
    """
    Assigns timestamps using a quintic-smoothstep (S-curve) position profile.

    The position follows  p(t) = S · f(t/T)  where
    f(x) = 10x³ − 15x⁴ + 6x⁵  (quintic Hermite, zero velocity &
    acceleration at both endpoints).

    This gives a jerk profile that starts and ends at zero, making it
    mechanically gentler than a trapezoidal profile.

    Parameters
    ----------
    v_max : float  — peak joint-space speed constraint (default 1.0).
                     T is chosen so that the peak of f'(x)·S/T = v_max.
                     (Peak of quintic f' occurs at x=0.5, value = 15/8.)
    """

    # Peak value of the quintic smoothstep derivative at x = 0.5
    _PEAK_DERIV = 1.875   # = 15/8

    def __init__(self, v_max: float = 1.0):
        if v_max <= 0:
            raise ValueError("v_max must be positive.")
        self.v_max = v_max

    @staticmethod
    def _f(x: float) -> float:
        """Quintic smoothstep: f(x) = 10x³ − 15x⁴ + 6x⁵."""
        return x ** 3 * (10.0 - x * (15.0 - 6.0 * x))

    def _s_to_t(self, s: float, S: float, T: float) -> float:
        """Invert S·f(t/T) = s via binary search."""
        target = s / S
        lo, hi = 0.0, 1.0
        for _ in range(64):
            mid = (lo + hi) / 2.0
            if self._f(mid) < target:
                lo = mid
            else:
                hi = mid
        return ((lo + hi) / 2.0) * T

    def parameterize(self, trajectory: list) -> list:
        """
        Parameters
        ----------
        trajectory : list[list[float]]  — joint configurations

        Returns
        -------
        list of (float, list[float]) — (timestamp_seconds, joints) pairs
        """
        pts = [np.array(q) for q in trajectory]
        dists = [float(np.linalg.norm(pts[i + 1] - pts[i]))
                 for i in range(len(pts) - 1)]
        S = sum(dists)

        if S < 1e-12:
            return [(0.0, list(trajectory[0]))]

        # T so that peak velocity equals v_max
        T = S * self._PEAK_DERIV / self.v_max

        cumul = np.zeros(len(pts))
        for i, d in enumerate(dists):
            cumul[i + 1] = cumul[i] + d

        return [(self._s_to_t(s, S, T), list(q))
                for s, q in zip(cumul, trajectory)]
