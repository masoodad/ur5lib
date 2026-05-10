# ur5lib/animations/animation_control.py
"""
Control-effects animation: position tracking, error, velocity & acceleration.

Runs a trajectory through a closed-loop PID controller against ``UR5Sim``
(which adds ±0.01 rad measurement noise per step), records the result, and
renders four live-updating panels:

  ┌────────────────────────┬────────────────────────┐
  │  Position tracking     │   Tracking error       │
  │  q_ref (--) vs actual  │   q_ref − q_actual     │
  ├────────────────────────┼────────────────────────┤
  │  Velocity profile      │  Acceleration profile  │
  │  ref (--) vs actual    │  ref (--) vs actual    │
  └────────────────────────┴────────────────────────┘

The *reference* curves (dashed) come from the time-parameterised trajectory
and show the chosen profile shape.  The *actual* curves (solid) show what the
controller achieved — the gap is the tracking error that PID must correct.

Public API
----------
  record_control_run(trajectory, robot, pid, dt)  → data dict
  animate_control(data, title, interval, joints)  → FuncAnimation

Standalone usage
----------------
    python animations/animation_control.py
    python animations/animation_control.py --profile scurve
    python animations/animation_control.py --joints 0 1 2
    python animations/animation_control.py --kp 4.0 --ki 0.1 --kd 0.2
"""

import sys
import os
import argparse
import itertools

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.gridspec as gridspec

from ur5lib.ur5_types.common_types import JointAngles
from ur5lib.io.simulator import UR5Sim
from ur5lib.trajectories.spline import SplineTrajectory
from ur5lib.control.joint_pid import JointPID
from ur5lib.control.profiles import TrapezoidalProfile, SCurveProfile
from ur5lib.animations.animation_trajectory import LINK_COLORS

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

JOINT_LABELS = [
    "J1  Base", "J2  Shoulder", "J3  Elbow",
    "J4  Wrist 1", "J5  Wrist 2", "J6  Wrist 3",
]

_BG       = "#0a0a15"
_PANEL_BG = "#0d0d1a"
_GRID_COL = "#1e1e3a"
_TEXT_COL = "#aaaaaa"
_ACCENT   = "#ffffff"

_HOME = JointAngles([0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0])
_DEMO_WAYPOINTS = [
    _HOME,
    JointAngles([ 0.4,  -1.1,  0.7, -1.3,  0.3,  0.0]),
    JointAngles([ 0.7,  -0.8,  1.0, -1.8,  0.5,  0.0]),
    JointAngles([ 0.5,  -1.0,  0.8, -1.5,  0.2,  0.0]),
    JointAngles([ 0.2,  -1.3,  0.5, -1.2,  0.1,  0.0]),
    _HOME,
]


# ---------------------------------------------------------------------------
# Data recording
# ---------------------------------------------------------------------------

def record_control_run(trajectory: list,
                       robot=None,
                       pid: JointPID = None,
                       dt: float = 0.008) -> dict:
    """
    Execute *trajectory* step-by-step with a JointPID controller and record
    per-tick kinematics.

    Reference velocity/acceleration are computed from ``q_ref`` using central
    finite differences — they reflect the profile shape (trapezoidal, S-curve)
    without sensor noise.  Actual velocity/acceleration are derived from
    ``q_actual``, which carries the simulator's measurement jitter.

    Parameters
    ----------
    trajectory : list[list[float]]
    robot      : UR5Base or None  — defaults to a fresh UR5Sim
    pid        : JointPID or None — defaults to JointPID(kp=2.0, ki=0.05, kd=0.1)
    dt         : float            — servo period in seconds

    Returns
    -------
    dict
        ``times``       (N,)    timestamps
        ``q_ref``       (N, 6)  reference positions
        ``q_actual``    (N, 6)  measured positions (sim noise included)
        ``q_cmd``       (N, 6)  PID-corrected command
        ``error``       (N, 6)  q_ref − q_actual
        ``vel_ref``     (N, 6)  reference velocity
        ``vel_actual``  (N, 6)  actual velocity
        ``acc_ref``     (N, 6)  reference acceleration
        ``acc_actual``  (N, 6)  actual acceleration
    """
    if robot is None:
        robot = UR5Sim()
        robot.connect_()
    if pid is None:
        pid = JointPID(kp=2.0, ki=0.05, kd=0.1, dt=dt)

    pid.reset()

    times, q_refs, q_actuals, q_cmds = [], [], [], []

    t = 0.0
    for q_ref in trajectory:
        q_ref    = list(q_ref)
        q_actual = list(robot.get_joint_angles().joints)
        q_cmd    = pid.step(q_ref, q_actual)

        robot.servoJ(JointAngles(joints=q_cmd), time=dt)

        times.append(t)
        q_refs.append(q_ref)
        q_actuals.append(q_actual)
        q_cmds.append(q_cmd)
        t += dt

    q_refs    = np.array(q_refs)
    q_actuals = np.array(q_actuals)
    q_cmds    = np.array(q_cmds)
    times     = np.array(times)

    # Reference kinematics: clean, reflects profile shape
    vel_ref = np.gradient(q_refs,    dt, axis=0)
    acc_ref = np.gradient(vel_ref,   dt, axis=0)

    # Actual kinematics: derived from noisy measurements
    vel_actual = np.gradient(q_actuals,  dt, axis=0)
    acc_actual = np.gradient(vel_actual, dt, axis=0)

    return dict(
        times      = times,
        q_ref      = q_refs,
        q_actual   = q_actuals,
        q_cmd      = q_cmds,
        error      = q_refs - q_actuals,
        vel_ref    = vel_ref,
        vel_actual = vel_actual,
        acc_ref    = acc_ref,
        acc_actual = acc_actual,
    )


# ---------------------------------------------------------------------------
# Animation
# ---------------------------------------------------------------------------

def _style_ax(ax, ylabel: str, xlabel: str = ""):
    ax.set_facecolor(_PANEL_BG)
    ax.set_ylabel(ylabel, color=_TEXT_COL, fontsize=8, labelpad=4)
    if xlabel:
        ax.set_xlabel(xlabel, color=_TEXT_COL, fontsize=8, labelpad=4)
    ax.tick_params(colors="#555555", labelsize=6)
    for spine in ax.spines.values():
        spine.set_edgecolor(_GRID_COL)
    ax.grid(True, color=_GRID_COL, linewidth=0.5, linestyle="--")


def animate_control(data: dict,
                    title: str = "UR5 — Control Effects",
                    interval: int = 30,
                    joints: list = None) -> animation.FuncAnimation:
    """
    Render four live-updating panels for a recorded control run.

    Parameters
    ----------
    data     : dict   — output of :func:`record_control_run`
    title    : str    — figure suptitle
    interval : int    — milliseconds between animation frames
    joints   : list[int] or None — indices to show (default: all 6)

    Returns
    -------
    matplotlib.animation.FuncAnimation  (keep the reference alive)
    """
    times      = data["times"]
    q_ref      = data["q_ref"]
    q_actual   = data["q_actual"]
    error      = data["error"]
    vel_ref    = data["vel_ref"]
    vel_actual = data["vel_actual"]
    acc_ref    = data["acc_ref"]
    acc_actual = data["acc_actual"]

    joints = joints if joints is not None else list(range(q_ref.shape[1]))
    N      = len(times)

    # ------------------------------------------------------------------
    # Figure / axes
    # ------------------------------------------------------------------
    fig = plt.figure(figsize=(14, 9))
    fig.patch.set_facecolor(_BG)
    fig.suptitle(title, color=_ACCENT, fontsize=12,
                 fontweight="bold", y=0.985)

    gs = gridspec.GridSpec(2, 2, figure=fig,
                           hspace=0.40, wspace=0.30,
                           left=0.07, right=0.97,
                           top=0.93,  bottom=0.07)

    ax_pos = fig.add_subplot(gs[0, 0])
    ax_err = fig.add_subplot(gs[0, 1])
    ax_vel = fig.add_subplot(gs[1, 0])
    ax_acc = fig.add_subplot(gs[1, 1])

    _style_ax(ax_pos, "Position (rad)")
    _style_ax(ax_err, "Error (rad)")
    _style_ax(ax_vel, "Velocity (rad/s)",  "Time (s)")
    _style_ax(ax_acc, "Accel  (rad/s²)",   "Time (s)")

    ax_pos.set_title("Position Tracking  (-- ref · — actual)",
                     color=_TEXT_COL, fontsize=8, pad=5)
    ax_err.set_title("Tracking Error  q_ref − q_actual",
                     color=_TEXT_COL, fontsize=8, pad=5)
    ax_vel.set_title("Velocity Profile  (-- ref · — actual)",
                     color=_TEXT_COL, fontsize=8, pad=5)
    ax_acc.set_title("Acceleration Profile  (-- ref · — actual)",
                     color=_TEXT_COL, fontsize=8, pad=5)

    # Axis limits with padding
    def _lim(arr, pad=0.12):
        lo = arr[:, joints].min()
        hi = arr[:, joints].max()
        span = max(hi - lo, 0.02)
        return lo - span * pad, hi + span * pad

    t0, t1 = times[0], times[-1]
    for ax in (ax_pos, ax_err, ax_vel, ax_acc):
        ax.set_xlim(t0, t1)

    ax_pos.set_ylim(*_lim(q_ref))
    ax_err.set_ylim(*_lim(error))
    ax_vel.set_ylim(*_lim(vel_ref))
    ax_acc.set_ylim(*_lim(acc_ref))

    # Zero reference on error panel
    ax_err.axhline(0, color="#444466", lw=0.9, linestyle=":")

    # ------------------------------------------------------------------
    # Per-joint artists
    # ------------------------------------------------------------------
    color_cycle = itertools.cycle(LINK_COLORS)
    colors = [next(color_cycle) for _ in joints]

    pos_ref_lines, pos_act_lines = [], []
    err_lines                    = []
    vel_ref_lines, vel_act_lines = [], []
    acc_ref_lines, acc_act_lines = [], []

    for k, j in enumerate(joints):
        c   = colors[k]
        lbl = JOINT_LABELS[j] if j < len(JOINT_LABELS) else f"J{j+1}"

        # Position: dashed ref + solid actual
        lr, = ax_pos.plot([], [], "--", color=c, lw=1.1, alpha=0.50)
        la, = ax_pos.plot([], [], "-",  color=c, lw=1.7, alpha=0.90, label=lbl)
        pos_ref_lines.append(lr); pos_act_lines.append(la)

        # Error: solid line
        le, = ax_err.plot([], [], "-", color=c, lw=1.5, alpha=0.85, label=lbl)
        err_lines.append(le)

        # Velocity
        lvr, = ax_vel.plot([], [], "--", color=c, lw=1.1, alpha=0.50)
        lva, = ax_vel.plot([], [], "-",  color=c, lw=1.7, alpha=0.80)
        vel_ref_lines.append(lvr); vel_act_lines.append(lva)

        # Acceleration
        lar, = ax_acc.plot([], [], "--", color=c, lw=1.1, alpha=0.50)
        laa, = ax_acc.plot([], [], "-",  color=c, lw=1.7, alpha=0.80)
        acc_ref_lines.append(lar); acc_act_lines.append(laa)

    # Vertical playback cursor on every panel
    cursor_kw  = dict(color="#ffffff", lw=0.7, alpha=0.35, linestyle=":")
    cursor_lines = [ax.plot([], [], **cursor_kw)[0]
                    for ax in (ax_pos, ax_err, ax_vel, ax_acc)]

    # Shaded error region on position panel for first displayed joint
    err_fill = ax_pos.fill_between([], [], [], color=colors[0],
                                   alpha=0.08, label="_nolegend_")

    # RMS error text overlay on error panel
    rms_txt = ax_err.text(0.98, 0.95, "", transform=ax_err.transAxes,
                          fontsize=6.5, color=_TEXT_COL,
                          ha="right", va="top",
                          bbox=dict(boxstyle="round,pad=0.3",
                                    fc=_PANEL_BG, alpha=0.8, ec=_GRID_COL))

    # Frame counter
    ftxt = ax_pos.text(0.98, 0.95, "", transform=ax_pos.transAxes,
                       fontsize=6, color="#555566", ha="right", va="top")

    # Legends
    for ax in (ax_pos, ax_err):
        ax.legend(loc="upper left", fontsize=6, framealpha=0.25,
                  facecolor=_PANEL_BG, edgecolor=_GRID_COL,
                  labelcolor=_TEXT_COL)

    # Collect all mutable artists for blit
    all_lines = (pos_ref_lines + pos_act_lines +
                 err_lines +
                 vel_ref_lines + vel_act_lines +
                 acc_ref_lines + acc_act_lines +
                 cursor_lines)

    # ------------------------------------------------------------------
    # Animation callbacks
    # ------------------------------------------------------------------
    def init():
        for ln in all_lines:
            ln.set_data([], [])
        rms_txt.set_text("")
        ftxt.set_text("")
        return all_lines + [rms_txt, ftxt]

    def update(frame):
        s = slice(0, frame + 1)
        t_win = times[s]
        t_now = times[frame]

        for k, j in enumerate(joints):
            pos_ref_lines[k].set_data(t_win, q_ref   [s, j])
            pos_act_lines[k].set_data(t_win, q_actual [s, j])
            err_lines[k]    .set_data(t_win, error    [s, j])
            vel_ref_lines[k].set_data(t_win, vel_ref  [s, j])
            vel_act_lines[k].set_data(t_win, vel_actual[s, j])
            acc_ref_lines[k].set_data(t_win, acc_ref  [s, j])
            acc_act_lines[k].set_data(t_win, acc_actual[s, j])

        # Update shaded error region for the first joint
        j0 = joints[0]
        err_fill.remove()
        # Redraw fill_between by patching the axes collection
        new_fill = ax_pos.fill_between(
            t_win,
            q_ref[s, j0],
            q_actual[s, j0],
            color=colors[0], alpha=0.08,
        )
        all_lines.append(new_fill)   # prevent GC
        update.__dict__["_fill"] = new_fill  # keep ref

        # Cursors
        for ln, ax in zip(cursor_lines,
                          (ax_pos, ax_err, ax_vel, ax_acc)):
            lo, hi = ax.get_ylim()
            ln.set_data([t_now, t_now], [lo, hi])

        # RMS error (over accumulated window)
        rms = np.sqrt(np.mean(error[s, :][:, joints] ** 2))
        rms_txt.set_text(f"RMS error: {rms:.5f} rad")

        ftxt.set_text(f"{frame + 1}/{N}  t={t_now:.2f}s")

        return all_lines + [rms_txt, ftxt]

    ani = animation.FuncAnimation(
        fig, update,
        frames=N,
        init_func=init,
        interval=interval,
        blit=False,
        repeat=True,
    )

    plt.show()
    return ani


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="UR5 control-effects animation."
    )
    parser.add_argument("--profile", choices=["trap", "scurve"], default="trap",
                        help="Velocity profile: trap (default) or scurve.")
    parser.add_argument("--joints", nargs="+", type=int, default=None,
                        metavar="J",
                        help="Joint indices to display (default: all 6).")
    parser.add_argument("--kp",  type=float, default=2.0)
    parser.add_argument("--ki",  type=float, default=0.05)
    parser.add_argument("--kd",  type=float, default=0.1)
    parser.add_argument("--dt",  type=float, default=0.008,
                        help="Servo time step in seconds (default: 0.008).")
    args = parser.parse_args()

    print("=" * 60)
    print("  UR5 Control-Effects Animation")
    print("=" * 60)

    # 1. Build trajectory
    print("\nGenerating SplineTrajectory...")
    raw_traj = SplineTrajectory(_DEMO_WAYPOINTS, steps_per_segment=60).generate()

    # 2. Apply velocity profile
    if args.profile == "scurve":
        print("Applying S-curve profile (v_max=1.0)...")
        timed = SCurveProfile(v_max=1.0).parameterize(raw_traj)
        profile_name = "S-Curve"
    else:
        print("Applying Trapezoidal profile (v_max=1.0, a_max=2.0)...")
        timed = TrapezoidalProfile(v_max=1.0, a_max=2.0).parameterize(raw_traj)
        profile_name = "Trapezoidal"

    # 3. Re-sample onto a uniform dt grid (profiles produce non-uniform spacing)
    t_stamps = np.array([t for t, _ in timed])
    q_matrix = np.array([q for _, q in timed])
    t_uni    = np.arange(t_stamps[0], t_stamps[-1], args.dt)
    n_joints = q_matrix.shape[1]
    traj_uni = [
        [float(np.interp(t, t_stamps, q_matrix[:, j])) for j in range(n_joints)]
        for t in t_uni
    ]

    print(f"  {len(raw_traj)} waypoints → {len(traj_uni)} servo ticks  "
          f"({args.dt * 1000:.1f} ms/tick, total {t_uni[-1]:.2f} s)")

    # 4. Record PID simulation
    print(f"\nRunning PID simulation "
          f"(kp={args.kp}, ki={args.ki}, kd={args.kd})...")
    robot = UR5Sim(); robot.connect_()
    pid   = JointPID(kp=args.kp, ki=args.ki, kd=args.kd, dt=args.dt)
    data  = record_control_run(traj_uni, robot=robot, pid=pid, dt=args.dt)

    # 5. Print per-joint error summary
    max_err = np.abs(data["error"]).max(axis=0)
    rms_err = np.sqrt(np.mean(data["error"] ** 2, axis=0))
    print("\nTracking error per joint:")
    print(f"  {'Joint':<16} {'Max (rad)':>12} {'Max (°)':>10} {'RMS (rad)':>12}")
    print("  " + "-" * 54)
    for j, lbl in enumerate(JOINT_LABELS):
        print(f"  {lbl:<16} {max_err[j]:>12.5f} "
              f"{np.degrees(max_err[j]):>10.3f} {rms_err[j]:>12.5f}")

    # 6. Animate
    joints = args.joints if args.joints else list(range(6))
    title  = (f"UR5 — Control Effects  |  {profile_name} Profile  |  "
              f"PID  kp={args.kp}  ki={args.ki}  kd={args.kd}")
    print("\nLaunching animation. Close the window to exit.\n")
    ani = animate_control(data, title=title, joints=joints)  # noqa: F841


if __name__ == "__main__":
    main()
