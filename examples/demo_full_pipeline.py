"""
demo_full_pipeline.py — End-to-end UR5 plan → animate → control → analyse.

Demonstrates the complete ur5lib workflow in one script:

  Stage 1  PLAN
           Define a pick-and-place mission as named waypoints.
           Generate a smooth Catmull-Rom spline trajectory through them.
           Time-parameterise with a Trapezoidal or S-curve velocity profile.

  Stage 2  SAFETY CHECK
           Validate every configuration and the full trajectory against
           UR5 hardware limits (position, velocity, acceleration) before
           any motion command is issued.

  Stage 3  3-D TRAJECTORY ANIMATION   [Window 1]
           Replay the planned trajectory as a live 3-D arm animation so
           the operator can visually inspect the path before execution.

  Stage 4  CLOSED-LOOP CONTROL
           Execute the same trajectory through a JointPID controller
           against UR5Sim (which adds ±0.01 rad measurement noise).
           Record reference positions, actual positions, tracking error,
           velocity profile, and acceleration profile at every servo tick.

  Stage 5  CONTROL-EFFECTS ANIMATION  [Window 2]
           Animate the four control panels:
             • Position tracking  (reference vs actual)
             • Tracking error     (q_ref − q_actual)
             • Velocity profile   (shows chosen profile shape)
             • Acceleration profile

Usage
-----
    python examples/demo_full_pipeline.py
    python examples/demo_full_pipeline.py --profile scurve
    python examples/demo_full_pipeline.py --skip-3d
    python examples/demo_full_pipeline.py --skip-control
    python examples/demo_full_pipeline.py --kp 4.0 --ki 0.1 --kd 0.3
    python examples/demo_full_pipeline.py --joints 0 1 2
"""

import sys
import os
import argparse

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

# --- Types & robot backends ---
from ur5lib.ur5_types.common_types import JointAngles, Pose
from ur5lib.io.simulator import UR5Sim

# --- Trajectory generators ---
from ur5lib.trajectories.spline import SplineTrajectory

# --- Control ---
from ur5lib.control.profiles  import TrapezoidalProfile, SCurveProfile
from ur5lib.control.safety    import SafetyChecker
from ur5lib.control.joint_pid import JointPID
from ur5lib.control.kinematics import tcp_pose

# --- Animations ---
from ur5lib.animations import (forward_kinematics, animate_trajectories,
                                record_control_run, animate_control)

# --- Exceptions ---
from ur5lib.exceptions import SafetyViolationError, JointLimitError


# ===========================================================================
# Mission definition
# ===========================================================================
#
#  Pick-and-place sequence:
#
#   HOME ──► APPROACH ──► PICK ──► RETREAT ──► TRANSFER ──► PLACE ──► HOME
#
#  All angles in radians, UR5 joint order: base, shoulder, elbow, wrist1-3.

_HOME = JointAngles([0.00, -1.5708,  0.00, -1.5708,  0.00,  0.00])

WAYPOINTS = {
    "home":     _HOME,
    "approach": JointAngles([ 0.45,  -1.05,   0.75,  -1.35,   0.30,  0.00]),
    "pick":     JointAngles([ 0.45,  -0.65,   1.30,  -2.20,   0.30,  0.00]),
    "retreat":  JointAngles([ 0.45,  -1.05,   0.75,  -1.35,   0.30,  0.00]),
    "transfer": JointAngles([-0.30,  -0.90,   0.80,  -1.55,  -0.30,  0.00]),
    "place":    JointAngles([-0.80,  -0.90,   0.90,  -1.60,  -0.90,  0.00]),
    "home_end": _HOME,
}

SEQUENCE = ["home", "approach", "pick", "retreat", "transfer", "place", "home_end"]


# ===========================================================================
# Helpers
# ===========================================================================

def _print_header(text: str):
    bar = "─" * 60
    print(f"\n{bar}")
    print(f"  {text}")
    print(bar)


def _print_waypoint_table():
    print(f"\n  {'Step':<10} {'Waypoint':<12}", end="")
    print("  J1(°)  J2(°)  J3(°)  J4(°)  J5(°)  J6(°)")
    print("  " + "-" * 74)
    for i, key in enumerate(SEQUENCE):
        j = WAYPOINTS[key].joints
        deg = [round(np.degrees(v), 1) for v in j]
        print(f"  {i+1:<10} {key:<12}"
              f"  {deg[0]:>6} {deg[1]:>6} {deg[2]:>6}"
              f"  {deg[3]:>6} {deg[4]:>6} {deg[5]:>6}")
    print()


def _print_tcp_table(sequence, waypoints):
    print(f"\n  {'Waypoint':<12}  {'x (m)':>8} {'y (m)':>8} {'z (m)':>8}")
    print("  " + "-" * 42)
    for key in sequence:
        p = tcp_pose(waypoints[key].joints)
        print(f"  {key:<12}  {p.x:>8.4f} {p.y:>8.4f} {p.z:>8.4f}")
    print()


# ===========================================================================
# Stage 1 — Trajectory planning
# ===========================================================================

def plan_trajectory(profile: str, steps: int = 55) -> tuple:
    """
    Build the spline trajectory and apply a velocity profile.

    Returns
    -------
    (raw_traj, timed_traj, profile_name)
        raw_traj   : list[list[float]] — dense joint configurations
        timed_traj : list[(float, list[float])] — time-stamped configs
        profile_name : str
    """
    waypoint_list = [WAYPOINTS[k] for k in SEQUENCE]
    raw_traj = SplineTrajectory(waypoint_list,
                                steps_per_segment=steps).generate()

    if profile == "scurve":
        timed = SCurveProfile(v_max=0.8).parameterize(raw_traj)
        name  = "S-Curve  (v_max=0.8 rad/s)"
    else:
        timed = TrapezoidalProfile(v_max=0.8, a_max=1.5).parameterize(raw_traj)
        name  = "Trapezoidal  (v_max=0.8 rad/s, a_max=1.5 rad/s²)"

    return raw_traj, timed, name


def resample_uniform(timed_traj: list, dt: float) -> list:
    """Interpolate a non-uniformly-timed trajectory onto a uniform dt grid."""
    t_arr = np.array([t for t, _ in timed_traj])
    q_arr = np.array([q for _, q in timed_traj])
    t_uni = np.arange(t_arr[0], t_arr[-1], dt)
    n_j   = q_arr.shape[1]
    return [
        [float(np.interp(t, t_arr, q_arr[:, j])) for j in range(n_j)]
        for t in t_uni
    ]


# ===========================================================================
# Stage 2 — Safety validation
# ===========================================================================

def validate(trajectory: list, dt: float, verbose: bool = True) -> bool:
    """
    Run SafetyChecker over every configuration and the full trajectory.
    Prints a report and returns True if safe, False if violations found.
    """
    checker = SafetyChecker()

    # Configuration check (position limits only, no dt needed)
    cfg_violations = []
    for k, q in enumerate(trajectory):
        for msg in checker.check_configuration(q):
            cfg_violations.append(f"  step {k:4d}: {msg}")

    # Full trajectory check (position + velocity + acceleration)
    traj_violations = checker.check_trajectory(trajectory, dt)

    if verbose:
        if not cfg_violations and not traj_violations:
            print("  ✓  All configurations within UR5 hardware limits.")
            print("  ✓  Velocity and acceleration within limits.")
        else:
            if cfg_violations:
                print(f"  ✗  {len(cfg_violations)} position violation(s):")
                for v in cfg_violations[:5]:
                    print(v)
            if traj_violations:
                print(f"  ✗  {len(traj_violations)} kinematic violation(s):")
                for v in traj_violations[:5]:
                    print(v)

    return len(cfg_violations) == 0 and len(traj_violations) == 0


# ===========================================================================
# Stage 3 — 3-D trajectory animation
# ===========================================================================

def show_trajectory_animation(raw_traj: list, profile_name: str):
    """
    Render the spline path as a live 3-D UR5 arm animation.
    One panel — close the window to continue to Stage 4.
    """
    frames = [forward_kinematics(q) for q in raw_traj]
    ani = animate_trajectories(  # noqa: F841
        [dict(
            name       = "Pick & Place",
            accent     = "#3498db",
            best_for   = "Assembly / welding demo",
            complexity = "High",
            smoothness = "Very High",
            desc       = f"Trajectory: Catmull-Rom Spline\n"
                         f"Profile:    {profile_name}\n"
                         f"Waypoints:  {len(SEQUENCE)} stages\n"
                         f"Frames:     {len(raw_traj)}",
            frames     = frames,
        )],
        title="UR5 — Pick-and-Place  |  Stage 3: Trajectory Preview",
    )


# ===========================================================================
# Stage 4 — Closed-loop control
# ===========================================================================

def run_control(traj_uniform: list,
                kp: float, ki: float, kd: float,
                dt: float) -> dict:
    """
    Execute the uniform-dt trajectory through JointPID + UR5Sim and return
    the recorded data dict (positions, errors, velocities, accelerations).
    """
    robot = UR5Sim()
    robot.connect_()
    pid  = JointPID(kp=kp, ki=ki, kd=kd, dt=dt)
    data = record_control_run(traj_uniform, robot=robot, pid=pid, dt=dt)
    return data


def _print_control_report(data: dict):
    """Print a per-joint tracking-error summary table."""
    max_err = np.abs(data["error"]).max(axis=0)
    rms_err = np.sqrt(np.mean(data["error"] ** 2, axis=0))
    labels  = ["J1 Base", "J2 Shoulder", "J3 Elbow",
               "J4 Wrist1", "J5 Wrist2", "J6 Wrist3"]

    print(f"\n  {'Joint':<14} {'Max err (rad)':>14} {'Max err (°)':>12}"
          f" {'RMS err (rad)':>14}")
    print("  " + "─" * 58)
    for j, lbl in enumerate(labels):
        print(f"  {lbl:<14} {max_err[j]:>14.5f} "
              f"{np.degrees(max_err[j]):>12.3f} {rms_err[j]:>14.5f}")

    overall_rms = float(np.sqrt(np.mean(data["error"] ** 2)))
    print(f"\n  Overall RMS tracking error: {overall_rms:.5f} rad "
          f"({np.degrees(overall_rms):.3f}°)")


# ===========================================================================
# Stage 5 — Control-effects animation
# ===========================================================================

def show_control_animation(data: dict, profile_name: str,
                           kp: float, ki: float, kd: float,
                           joints: list):
    """
    Render the four-panel control animation:
      position tracking | tracking error | velocity | acceleration
    """
    title = (f"UR5 — Pick-and-Place  |  Stage 5: Control Effects  |  "
             f"{profile_name}  |  PID kp={kp} ki={ki} kd={kd}")
    ani = animate_control(data, title=title, joints=joints)  # noqa: F841


# ===========================================================================
# Main
# ===========================================================================

def main():
    parser = argparse.ArgumentParser(
        description="UR5 full pipeline: plan → animate → control → analyse.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--profile", choices=["trap", "scurve"], default="trap",
                        help="Velocity profile: trap (default) or scurve.")
    parser.add_argument("--skip-3d",      action="store_true",
                        help="Skip Stage 3 (3-D trajectory animation).")
    parser.add_argument("--skip-control", action="store_true",
                        help="Skip Stages 4–5 (control simulation + animation).")
    parser.add_argument("--kp",  type=float, default=2.0,
                        help="PID proportional gain (default 2.0).")
    parser.add_argument("--ki",  type=float, default=0.05,
                        help="PID integral gain (default 0.05).")
    parser.add_argument("--kd",  type=float, default=0.10,
                        help="PID derivative gain (default 0.10).")
    parser.add_argument("--dt",  type=float, default=0.008,
                        help="Servo time step in seconds (default 0.008).")
    parser.add_argument("--joints", nargs="+", type=int, default=None,
                        metavar="J",
                        help="Joint indices for control animation (default: all 6).")
    args = parser.parse_args()

    joints = args.joints or list(range(6))

    # ------------------------------------------------------------------
    print("\n" + "═" * 62)
    print("  UR5  Full Pipeline Demo")
    print("  plan → animate → control → analyse")
    print("═" * 62)

    # ------------------------------------------------------------------
    _print_header("Stage 1 — Trajectory Planning")

    print("\n  Mission: Pick-and-Place  (7 waypoints)")
    _print_waypoint_table()

    print("  TCP positions (forward kinematics from robots/ur5_dh.py):")
    _print_tcp_table(SEQUENCE, WAYPOINTS)

    raw_traj, timed_traj, profile_name = plan_trajectory(args.profile)
    traj_uniform = resample_uniform(timed_traj, args.dt)
    total_time   = len(traj_uniform) * args.dt

    print(f"  Trajectory type : Catmull-Rom Spline")
    print(f"  Velocity profile: {profile_name}")
    print(f"  Raw waypoints   : {len(raw_traj)}")
    print(f"  Servo ticks     : {len(traj_uniform)}  "
          f"({args.dt*1000:.1f} ms/tick)")
    print(f"  Total duration  : {total_time:.2f} s")

    # ------------------------------------------------------------------
    _print_header("Stage 2 — Safety Validation")
    print()
    safe = validate(traj_uniform, args.dt, verbose=True)
    if not safe:
        print("\n  WARNING: trajectory has limit violations — "
              "proceeding anyway (simulation only).")

    # ------------------------------------------------------------------
    if not args.skip_3d:
        _print_header("Stage 3 — 3-D Trajectory Animation  [Window 1]")
        print()
        print(f"  Rendering {len(raw_traj)} frames.")
        print("  Close the window to continue to Stage 4.\n")
        show_trajectory_animation(raw_traj, profile_name)
    else:
        print("\n  Stage 3 skipped (--skip-3d).")

    # ------------------------------------------------------------------
    if not args.skip_control:
        _print_header("Stage 4 — Closed-Loop Control Simulation")
        print()
        print(f"  Controller : JointPID  kp={args.kp}  "
              f"ki={args.ki}  kd={args.kd}")
        print(f"  Servo rate : {1/args.dt:.0f} Hz  (dt={args.dt*1000:.1f} ms)")
        print(f"  Robot      : UR5Sim  (±0.01 rad measurement noise)")
        print(f"  Ticks      : {len(traj_uniform)}\n")

        data = run_control(traj_uniform,
                           kp=args.kp, ki=args.ki, kd=args.kd,
                           dt=args.dt)
        _print_control_report(data)

        # ------------------------------------------------------------------
        _print_header("Stage 5 — Control-Effects Animation  [Window 2]")
        print()
        print("  Four panels:  position tracking │ error │ velocity │ accel")
        print("  Dashed lines = reference (planned profile).")
        print("  Solid lines  = actual (measured from sim, includes noise).")
        print(f"  Showing joints: {[f'J{j+1}' for j in joints]}")
        print("  Close the window to exit.\n")

        show_control_animation(data, profile_name,
                               kp=args.kp, ki=args.ki, kd=args.kd,
                               joints=joints)
    else:
        print("\n  Stages 4–5 skipped (--skip-control).")

    print("\n  Done.\n")


if __name__ == "__main__":
    main()
