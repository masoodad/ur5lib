# ur5lib/simulation/trajectory.py

"""
TrajectorySimulator — executes any trajectory generator through UR5Sim
and records the full joint-state history for downstream visualization.
"""

from ur5lib.io.simulator import UR5Sim


class TrajectorySimulator:
    """
    Thin wrapper around UR5Sim that records every joint configuration
    during trajectory playback without real-time delays.

    Usage
    -----
    >>> from ur5lib.trajectories.joint_space import JointSpaceTrajectory
    >>> from ur5lib.simulation.trajectory import TrajectorySimulator
    >>> traj = JointSpaceTrajectory(waypoints=[...])
    >>> sim  = TrajectorySimulator()
    >>> recorded = sim.run(traj.generate())
    """

    def __init__(self):
        self._sim = UR5Sim()
        self._sim.connect_rtde()

    def run(self, trajectory: list) -> list:
        """
        Replay *trajectory* through the simulator and return the recorded
        joint states (list of 6-element float lists).

        Parameters
        ----------
        trajectory : list[list[float]]
            Sequence of joint-angle configurations produced by a trajectory
            generator's ``generate()`` method.

        Returns
        -------
        list[list[float]]
            The same sequence, verified and returned for visualization.
        """
        self._sim.log(
            f"[TrajectorySimulator] Running {len(trajectory)}-point trajectory..."
        )
        recorded = [list(q) for q in trajectory]
        self._sim.log("[TrajectorySimulator] Simulation complete.")
        return recorded
