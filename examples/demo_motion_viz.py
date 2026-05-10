"""
demo_motion_viz.py — UR5 motion visualization.

Demonstrates a pick-and-place sequence:
  Home → Approach → Pick → Retreat → Place → Home

The script uses:
  - UR5 standard DH parameters for 3-D forward kinematics
  - Linear joint interpolation (matching MotionPlanner.plan_joint_motion)
  - Matplotlib 3-D animation

Run from the project root:
    python examples/demo_motion_viz.py
"""

import numpy as np
import matplotlib
# Auto-select backend; override with MPLBACKEND env var if needed
# e.g.: MPLBACKEND=Qt5Agg python examples/demo_motion_viz.py
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (registers 3-D projection)
from collections import namedtuple

# ---------------------------------------------------------------------------
# Minimal type definitions (mirrors ur5lib.types.common_types)
# ---------------------------------------------------------------------------
JointAngles = namedtuple("JointAngles", ["joints"])
Pose = namedtuple("Pose", ["x", "y", "z", "rx", "ry", "rz"])

# ---------------------------------------------------------------------------
# UR5 DH parameters  (matches ur5lib/io/ur_rtde.py robot model)
# Each row: [a (m), d (m), alpha (rad)]
# ---------------------------------------------------------------------------
UR5_DH = np.array([
    [0.0,      0.1625,   np.pi / 2],   # joint 1 — base rotation
    [-0.425,   0.0,      0.0      ],   # joint 2 — shoulder
    [-0.3922,  0.0,      0.0      ],   # joint 3 — elbow
    [0.0,      0.1333,   np.pi / 2],   # joint 4 — wrist 1
    [0.0,      0.0997,  -np.pi / 2],   # joint 5 — wrist 2
    [0.0,      0.0996,   0.0      ],   # joint 6 — wrist 3
])


def dh_matrix(theta: float, a: float, d: float, alpha: float) -> np.ndarray:
    """4×4 homogeneous transformation for one DH joint."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.,  sa,       ca,      d     ],
        [0.,  0.,       0.,      1.    ],
    ])


def forward_kinematics(joints: list) -> np.ndarray:
    """
    Compute (x, y, z) of every frame origin for a joint configuration.
    Returns shape (7, 3): base + 6 joint frame origins.
    """
    T = np.eye(4)
    positions = [T[:3, 3].copy()]   # base at origin
    for i, theta in enumerate(joints):
        a, d, alpha = UR5_DH[i]
        T = T @ dh_matrix(theta, a, d, alpha)
        positions.append(T[:3, 3].copy())
    return np.array(positions)


# ---------------------------------------------------------------------------
# Motion planner — mirrors MotionPlanner.plan_joint_motion
# ---------------------------------------------------------------------------
def plan_joint_motion(start: JointAngles, goal: JointAngles,
                      num_points: int = 40) -> list:
    """Linearly interpolate between two joint configurations."""
    s = np.array(start.joints)
    g = np.array(goal.joints)
    return [(s + alpha * (g - s)).tolist()
            for alpha in np.linspace(0, 1, num_points)]


# ---------------------------------------------------------------------------
# Named waypoints for the pick-and-place sequence
# (angles in radians, UR5 joint order: base, shoulder, elbow, wrist1-3)
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
    """Concatenate interpolated segments into a single trajectory."""
    traj = []
    for i in range(len(SEQUENCE) - 1):
        segment = plan_joint_motion(
            WAYPOINTS[SEQUENCE[i]],
            WAYPOINTS[SEQUENCE[i + 1]],
            num_points=POINTS_PER_SEGMENT,
        )
        traj.extend(segment[:-1])   # drop last to avoid duplicates at junctions
    traj.append(WAYPOINTS[SEQUENCE[-1]].joints)
    return traj


# ---------------------------------------------------------------------------
# Animation
# ---------------------------------------------------------------------------
LINK_COLORS = ["#e74c3c", "#e67e22", "#f1c40f",
               "#2ecc71", "#3498db", "#9b59b6"]


def run_animation(trajectory: list):
    # Pre-compute all frame positions
    frames = [forward_kinematics(q) for q in trajectory]

    all_pts = np.vstack(frames)
    margin = 0.08
    lim = max(float(np.abs(all_pts).max()), 0.55) + margin

    fig = plt.figure(figsize=(11, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_zlim(0, lim * 1.6)
    ax.set_xlabel("X (m)", labelpad=8)
    ax.set_ylabel("Y (m)", labelpad=8)
    ax.set_zlabel("Z (m)", labelpad=8)
    ax.set_title("UR5 Pick-and-Place Motion", fontsize=13, pad=14)
    ax.view_init(elev=22, azim=-55)

    # Ground plane grid
    g = np.linspace(-lim, lim, 6)
    for v in g:
        ax.plot([v, v], [-lim, lim], [0, 0], color="#cccccc", lw=0.6)
        ax.plot([-lim, lim], [v, v], [0, 0], color="#cccccc", lw=0.6)

    # Link lines (one per segment)
    link_lines = [
        ax.plot([], [], [], lw=5, color=LINK_COLORS[i],
                solid_capstyle="round")[0]
        for i in range(6)
    ]

    # Joint spheres
    joint_pts = ax.scatter([], [], [], s=55, color="black", zorder=5,
                           depthshade=False)

    # TCP trace
    tcp_x, tcp_y, tcp_z = [], [], []
    tcp_trace, = ax.plot([], [], [], color="#7f8c8d", lw=1.0, alpha=0.6,
                         label="TCP path")

    # TCP star marker
    tcp_star, = ax.plot([], [], [], "r*", markersize=14, label="TCP",
                        zorder=6)

    # Text overlays
    seg_text  = ax.text2D(0.02, 0.96, "", transform=ax.transAxes,
                          fontsize=10, color="#2c3e50",
                          bbox=dict(boxstyle="round,pad=0.3",
                                    fc="white", alpha=0.7))
    frame_text = ax.text2D(0.02, 0.90, "", transform=ax.transAxes,
                           fontsize=8, color="#7f8c8d")

    # Legend with colour-coded link labels
    for i, name in enumerate(["Base", "Shoulder", "Elbow",
                               "Wrist 1", "Wrist 2", "Wrist 3"]):
        ax.plot([], [], lw=3, color=LINK_COLORS[i], label=name)
    ax.plot([], [], "r*", markersize=10, label="TCP")
    ax.plot([], [], color="#7f8c8d", lw=1.0, label="TCP path")
    ax.legend(loc="upper right", fontsize=7, ncol=2, framealpha=0.8)

    n_total = len(trajectory)
    seg_frames = POINTS_PER_SEGMENT - 1

    def _seg_label(idx: int) -> str:
        seg = min(idx // seg_frames, len(SEQUENCE) - 2)
        return (f"{SEQUENCE[seg].capitalize()}  →  "
                f"{SEQUENCE[min(seg + 1, len(SEQUENCE) - 1)].capitalize()}")

    def init():
        for ln in link_lines:
            ln.set_data([], [])
            ln.set_3d_properties([])
        tcp_trace.set_data([], [])
        tcp_trace.set_3d_properties([])
        tcp_star.set_data([], [])
        tcp_star.set_3d_properties([])
        return (*link_lines, tcp_trace, tcp_star, seg_text, frame_text)

    def update(idx: int):
        pts = frames[idx]           # (7, 3)

        # Links
        for i, ln in enumerate(link_lines):
            ln.set_data([pts[i, 0], pts[i + 1, 0]],
                        [pts[i, 1], pts[i + 1, 1]])
            ln.set_3d_properties([pts[i, 2], pts[i + 1, 2]])

        # Joint spheres
        joint_pts._offsets3d = (pts[:, 0], pts[:, 1], pts[:, 2])

        # TCP trace (accumulate)
        tcp_x.append(pts[-1, 0])
        tcp_y.append(pts[-1, 1])
        tcp_z.append(pts[-1, 2])
        tcp_trace.set_data(tcp_x, tcp_y)
        tcp_trace.set_3d_properties(tcp_z)

        # TCP marker
        tcp_star.set_data([pts[-1, 0]], [pts[-1, 1]])
        tcp_star.set_3d_properties([pts[-1, 2]])

        seg_text.set_text(f"Segment: {_seg_label(idx)}")
        frame_text.set_text(f"Frame {idx + 1} / {n_total}")

        return (*link_lines, joint_pts, tcp_trace, tcp_star,
                seg_text, frame_text)

    ani = animation.FuncAnimation(
        fig, update,
        frames=n_total,
        init_func=init,
        interval=50,        # 20 fps
        blit=False,
        repeat=True,
    )

    plt.tight_layout()
    plt.show()
    return ani   # keep reference alive


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    print("Building pick-and-place trajectory ...")
    traj = build_trajectory()
    print(f"  {len(SEQUENCE) - 1} segments × {POINTS_PER_SEGMENT} pts "
          f"= {len(traj)} total frames")
    print("Close the plot window to exit.")
    run_animation(traj)
