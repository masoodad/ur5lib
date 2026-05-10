"""
demo_motion_viz.py — UR5 motion visualization.

Demonstrates a pick-and-place sequence:
  Home → Approach → Pick → Retreat → Place → Home

Run from the project root:
    python examples/demo_motion_viz.py
"""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from ur5lib.ur5_types.common_types import JointAngles
from ur5lib.animations import forward_kinematics, animate_trajectories


# ---------------------------------------------------------------------------
# Motion planner — mirrors MotionPlanner.plan_joint_motion
# ---------------------------------------------------------------------------
def plan_joint_motion(start: JointAngles, goal: JointAngles,
                      num_points: int = 40) -> list:
    s = np.array(start.joints)
    g = np.array(goal.joints)
    return [(s + alpha * (g - s)).tolist()
            for alpha in np.linspace(0, 1, num_points)]


# ---------------------------------------------------------------------------
# Waypoints
# ---------------------------------------------------------------------------
WAYPOINTS = {
    "home":     JointAngles([0.0,   -np.pi / 2,  0.0,  -np.pi / 2,  0.0,  0.0]),
    "approach": JointAngles([0.5,   -1.0,         0.8,  -1.4,         0.3,  0.0]),
    "pick":     JointAngles([0.5,   -0.65,        1.3,  -2.2,         0.3,  0.0]),
    "retreat":  JointAngles([0.5,   -1.0,         0.8,  -1.4,         0.3,  0.0]),
    "place":    JointAngles([-0.9,  -0.9,         0.9,  -1.6,        -0.9,  0.0]),
    "home_end": JointAngles([0.0,   -np.pi / 2,  0.0,  -np.pi / 2,  0.0,  0.0]),
}

SEQUENCE = ["home", "approach", "pick", "retreat", "place", "home_end"]
POINTS_PER_SEGMENT = 40


def build_trajectory() -> list:
    traj = []
    for i in range(len(SEQUENCE) - 1):
        segment = plan_joint_motion(
            WAYPOINTS[SEQUENCE[i]],
            WAYPOINTS[SEQUENCE[i + 1]],
            num_points=POINTS_PER_SEGMENT,
        )
        traj.extend(segment[:-1])
    traj.append(WAYPOINTS[SEQUENCE[-1]].joints)
    return traj


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    print("Building pick-and-place trajectory ...")
    traj = build_trajectory()
    print(f"  {len(SEQUENCE) - 1} segments × {POINTS_PER_SEGMENT} pts "
          f"= {len(traj)} total frames")
    print("Close the plot window to exit.")

    frames = [forward_kinematics(q) for q in traj]
    ani = animate_trajectories(  # noqa: F841
        [{"name": "Pick & Place", "accent": "#e74c3c", "frames": frames}],
        title="UR5 Pick-and-Place Motion",
        interval=50,
    )
