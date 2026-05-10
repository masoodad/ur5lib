"""
demo_trajectory_compare.py — UR5 trajectory type comparison.

Demonstrates all three trajectory generators from ur5lib/trajectories/:

  ┌──────────────────┬─────────────────────┬────────────┬────────────┐
  │ Type             │ Best For            │ Complexity │ Smoothness │
  ├──────────────────┼─────────────────────┼────────────┼────────────┤
  │ Joint Space      │ Fast motion         │ Low        │ Medium     │
  │ Cartesian Linear │ Precise tool path   │ Medium     │ High       │
  │ Circular/Spline  │ Smooth cont. motion │ High       │ Very High  │
  └──────────────────┴─────────────────────┴────────────┴────────────┘

Each trajectory is built by its class, simulated through
``simulation.TrajectorySimulator`` (wrapping UR5Sim), and rendered as a
live 3-D Matplotlib animation via ``ur5lib.io.animate_trajectories``.

Usage
-----
    python examples/demo_trajectory_compare.py

    # or select a single type interactively:
    python examples/demo_trajectory_compare.py --select
"""

import sys
import os
import argparse

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from ur5lib.ur5_types.common_types import JointAngles
from ur5lib.trajectories.joint_space     import JointSpaceTrajectory
from ur5lib.trajectories.cartesian_space import CartesianLinearTrajectory
from ur5lib.trajectories.spline          import SplineTrajectory
from ur5lib.simulation.trajectory        import TrajectorySimulator
from ur5lib.animations import forward_kinematics, animate_trajectories


# ---------------------------------------------------------------------------
# Waypoint definitions
# ---------------------------------------------------------------------------
_HOME = JointAngles([0.0, -np.pi / 2, 0.0, -np.pi / 2, 0.0, 0.0])

JS_WAYPOINTS = [
    _HOME,
    JointAngles([ 0.5,  -0.65,  1.3,  -2.2,   0.3,  0.0]),
    JointAngles([-0.9,  -0.9,   0.9,  -1.6,  -0.9,  0.0]),
    _HOME,
]

CL_WAYPOINTS = [
    JointAngles([0.00, -np.pi / 2, 0.3, -np.pi / 2 + 0.3, 0.0, 0.0]),
    JointAngles([0.25, -np.pi / 2, 0.3, -np.pi / 2 + 0.3, 0.0, 0.0]),
    JointAngles([0.50, -np.pi / 2, 0.3, -np.pi / 2 + 0.3, 0.0, 0.0]),
    JointAngles([0.75, -np.pi / 2, 0.3, -np.pi / 2 + 0.3, 0.0, 0.0]),
    JointAngles([1.00, -np.pi / 2, 0.3, -np.pi / 2 + 0.3, 0.0, 0.0]),
    JointAngles([1.25, -np.pi / 2, 0.3, -np.pi / 2 + 0.3, 0.0, 0.0]),
]

SP_WAYPOINTS = [
    _HOME,
    JointAngles([ 0.3,  -1.2,  0.6,  -1.2,   0.2,  0.0]),
    JointAngles([ 0.6,  -0.8,  1.0,  -1.8,   0.4,  0.0]),
    JointAngles([ 0.8,  -1.0,  1.2,  -1.6,   0.6,  0.0]),
    JointAngles([ 0.5,  -0.7,  0.9,  -2.0,   0.3,  0.0]),
    JointAngles([ 0.2,  -1.1,  0.5,  -1.3,   0.1,  0.0]),
    _HOME,
]

# ---------------------------------------------------------------------------
# Trajectory registry
# ---------------------------------------------------------------------------
TRAJECTORY_DEFS = [
    dict(
        key        = "joint",
        cls        = JointSpaceTrajectory,
        waypoints  = JS_WAYPOINTS,
        accent     = "#e74c3c",
        name       = "Joint Space",
        best_for   = "Fast motion",
        complexity = "Low",
        smoothness = "Medium",
        desc       = "Linear joint interpolation (LERP).\n"
                     "Sharp Cartesian corners at each waypoint.\n"
                     "Minimal planning cost.",
    ),
    dict(
        key        = "cartesian",
        cls        = CartesianLinearTrajectory,
        waypoints  = CL_WAYPOINTS,
        accent     = "#3498db",
        name       = "Cartesian Linear",
        best_for   = "Precise tool path",
        complexity = "Medium",
        smoothness = "High",
        desc       = "Dense piecewise-linear Cartesian segments.\n"
                     "TCP follows a near-straight line per segment.\n"
                     "Used for welding, dispensing, cutting.",
    ),
    dict(
        key        = "spline",
        cls        = SplineTrajectory,
        waypoints  = SP_WAYPOINTS,
        accent     = "#2ecc71",
        name       = "Circular / Spline",
        best_for   = "Smooth continuous motion",
        complexity = "High",
        smoothness = "Very High",
        desc       = "Catmull-Rom spline — C¹-continuous.\n"
                     "No velocity discontinuities at waypoints.\n"
                     "Ideal for painting, coating, arc-welding.",
    ),
]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _print_table(defs: list):
    print()
    header = f"  {'#':<3} {'Type':<20} {'Best For':<24} {'Complexity':<12} Smoothness"
    print(header)
    print("  " + "-" * (len(header) - 2))
    for i, d in enumerate(defs):
        print(f"  {i+1:<3} {d['name']:<20} {d['best_for']:<24} "
              f"{d['complexity']:<12} {d['smoothness']}")
    print()


def _build_trajectories(selected: list) -> list:
    sim = TrajectorySimulator()
    results = []
    for defn in selected:
        traj_obj = defn["cls"](defn["waypoints"])
        recorded = sim.run(traj_obj.generate())
        frames   = [forward_kinematics(q) for q in recorded]
        results.append({**defn, "frames": frames})
        print(f"  [{defn['name']}]  {len(recorded)} frames")
    return results


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="UR5 trajectory type comparison demo."
    )
    parser.add_argument(
        "--select", action="store_true",
        help="Interactively select one trajectory type instead of showing all three.",
    )
    args = parser.parse_args()

    print("=" * 62)
    print("  UR5 Trajectory Comparison Demo")
    print("=" * 62)
    _print_table(TRAJECTORY_DEFS)

    if args.select:
        keys = {str(i + 1): d for i, d in enumerate(TRAJECTORY_DEFS)}
        keys.update({d["key"]: d for d in TRAJECTORY_DEFS})
        choice = input("Select trajectory [1/2/3 or joint/cartesian/spline]: ").strip().lower()
        selected = [keys[choice]] if choice in keys else TRAJECTORY_DEFS
        if choice not in keys:
            print(f"Unknown selection '{choice}'. Running all three.")
    else:
        selected = TRAJECTORY_DEFS

    print("Generating trajectories and running simulation...\n")
    results = _build_trajectories(selected)

    print(f"\nLaunching animation ({len(selected)} panel(s)). "
          "Close the window to exit.\n")
    ani = animate_trajectories(  # noqa: F841
        results,
        title="UR5  —  Trajectory Type Comparison",
    )


if __name__ == "__main__":
    main()
