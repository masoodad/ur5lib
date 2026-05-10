"""
demo_motion_viz.py — UR5 motion visualization using the simulator.

Shows the robot moving through a pick-and-place sequence:
  Home → Approach → Pick → Retreat → Place → Home

Forward kinematics are computed from the standard UR5 DH parameters so
every joint configuration from the planner is rendered as a 3-D stick figure.

Run:
    python examples/demo_motion_viz.py
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

from ur5lib.io.simulator import UR5Sim
from ur5lib.motion.planner import MotionPlanner
from ur5lib.types.common_types import JointAngles


# ---------------------------------------------------------------------------
# UR5 DH parameters (standard, metres / radians)
# ---------------------------------------------------------------------------
# Each row: [a, d, alpha]
UR5_DH = np.array([
    [0.0,      0.1625,  np.pi / 2],   # joint 1
    [-0.425,   0.0,     0.0],          # joint 2
    [-0.3922,  0.0,     0.0],          # joint 3
    [0.0,      0.1333,  np.pi / 2],    # joint 4
    [0.0,      0.0997, -np.pi / 2],    # joint 5
    [0.0,      0.0996,  0.0],          # joint 6
])


def dh_matrix(theta: float, a: float, d: float, alpha: float) -> np.ndarray:
    """Return the 4×4 homogeneous DH transformation matrix."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,   sa,       ca,      d     ],
        [0,   0,        0,       1     ],
    ])


def forward_kinematics(joint_angles: list) -> np.ndarray:
    """
    Compute the (x, y, z) position of every frame origin for a given
    joint configuration.

    Returns an array of shape (7, 3):
      row 0  — robot base (always [0, 0, 0])
      rows 1–6 — joint frame origins
    """
    T = np.eye(4)
    positions = [T[:3, 3].copy()]           # base

    for i, theta in enumerate(joint_angles):
        a, d, alpha = UR5_DH[i]
        T = T @ dh_matrix(theta, a, d, alpha)
        positions.append(T[:3, 3].copy())

    return np.array(positions)             # shape (7, 3)


# ---------------------------------------------------------------------------
# Motion sequence: named waypoints
# ---------------------------------------------------------------------------
WAYPOINTS = {
    "home":     JointAngles([0.0,  -np.pi / 2,  0.0, -np.pi / 2,  0.0, 0.0]),
    "approach": JointAngles([0.3,  -1.0,         0.8, -1.4,         0.3, 0.0]),
    "pick":     JointAngles([0.3,  -0.7,         1.2, -2.1,         0.3, 0.0]),
    "retreat":  JointAngles([0.3,  -1.0,         0.8, -1.4,         0.3, 0.0]),
    "place":    JointAngles([-0.8, -0.9,         0.9, -1.6,        -0.8, 0.0]),
    "home_end": JointAngles([0.0,  -np.pi / 2,  0.0, -np.pi / 2,  0.0, 0.0]),
}

SEQUENCE = ["home", "approach", "pick", "retreat", "place", "home_end"]
SEGMENT_POINTS = 30     # waypoints per segment


# ---------------------------------------------------------------------------
# Build the full trajectory
# ---------------------------------------------------------------------------
def build_trajectory() -> list:
    """
    Use MotionPlanner to interpolate between every consecutive pair of
    named waypoints and concatenate into a single list of joint configs.
    """
    planner = MotionPlanner(num_points=SEGMENT_POINTS)
    trajectory = []

    for i in range(len(SEQUENCE) - 1):
        start = WAYPOINTS[SEQUENCE[i]]
        goal  = WAYPOINTS[SEQUENCE[i + 1]]
        segment = planner.plan_joint_motion(start, goal)
        # Drop the last point of each segment to avoid duplicates at joints
        trajectory.extend(segment[:-1])

    trajectory.append(WAYPOINTS[SEQUENCE[-1]].joints)  # final pose
    return trajectory


# ---------------------------------------------------------------------------
# Animation
# ---------------------------------------------------------------------------
def animate(trajectory: list):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    # Pre-compute all frame positions
    frames = [forward_kinematics(q) for q in trajectory]

    # Work out axis limits from all positions
    all_pts = np.vstack(frames)
    margin = 0.05
    lim = max(np.abs(all_pts).max(), 0.5) + margin

    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_zlim(0, lim * 1.5)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("UR5 Motion — Pick-and-Place Sequence")

    # Draw a simple ground plane grid
    grid_range = np.linspace(-lim, lim, 5)
    for gx in grid_range:
        ax.plot([gx, gx], [-lim, lim], [0, 0], color="lightgrey", lw=0.5)
    for gy in grid_range:
        ax.plot([-lim, lim], [gy, gy], [0, 0], color="lightgrey", lw=0.5)

    # Colours for each link segment
    LINK_COLORS = ["#e74c3c", "#e67e22", "#f1c40f", "#2ecc71", "#3498db", "#9b59b6"]

    # Persistent TCP trace
    tcp_trace_x, tcp_trace_y, tcp_trace_z = [], [], []
    tcp_trace, = ax.plot([], [], [], color="gray", lw=0.8, alpha=0.5,
                         label="TCP path")

    # Robot links (one line per segment)
    link_lines = [
        ax.plot([], [], [], lw=4, color=LINK_COLORS[i], solid_capstyle="round")[0]
        for i in range(6)
    ]

    # Joint spheres
    joint_scatter = ax.scatter([], [], [], s=60, color="black", zorder=5)

    # TCP marker
    tcp_marker, = ax.plot([], [], [], "r*", markersize=14, label="TCP",
                          zorder=6)

    # Segment label
    total_frames = len(trajectory)
    frames_per_seg = SEGMENT_POINTS - 1          # because last pt was dropped

    def segment_name(frame_idx: int) -> str:
        seg = min(frame_idx // frames_per_seg, len(SEQUENCE) - 2)
        a = SEQUENCE[seg]
        b = SEQUENCE[min(seg + 1, len(SEQUENCE) - 1)]
        return f"{a}  →  {b}"

    seg_text = ax.text2D(0.02, 0.95, "", transform=ax.transAxes,
                         fontsize=10, color="#2c3e50")
    frame_text = ax.text2D(0.02, 0.91, "", transform=ax.transAxes,
                           fontsize=8, color="gray")

    ax.legend(loc="upper right", fontsize=8)

    def init():
        for ln in link_lines:
            ln.set_data([], [])
            ln.set_3d_properties([])
        tcp_trace.set_data([], [])
        tcp_trace.set_3d_properties([])
        tcp_marker.set_data([], [])
        tcp_marker.set_3d_properties([])
        return link_lines + [tcp_trace, tcp_marker, seg_text, frame_text]

    def update(idx: int):
        pts = frames[idx]          # (7, 3)

        # Update links
        for i, ln in enumerate(link_lines):
            xs = [pts[i, 0], pts[i + 1, 0]]
            ys = [pts[i, 1], pts[i + 1, 1]]
            zs = [pts[i, 2], pts[i + 1, 2]]
            ln.set_data(xs, ys)
            ln.set_3d_properties(zs)

        # Update joint positions
        joint_scatter._offsets3d = (pts[:, 0], pts[:, 1], pts[:, 2])

        # TCP trace
        tcp_trace_x.append(pts[-1, 0])
        tcp_trace_y.append(pts[-1, 1])
        tcp_trace_z.append(pts[-1, 2])
        tcp_trace.set_data(tcp_trace_x, tcp_trace_y)
        tcp_trace.set_3d_properties(tcp_trace_z)

        # TCP marker
        tcp_marker.set_data([pts[-1, 0]], [pts[-1, 1]])
        tcp_marker.set_3d_properties([pts[-1, 2]])

        seg_text.set_text(f"Segment: {segment_name(idx)}")
        frame_text.set_text(f"Frame {idx + 1}/{total_frames}")

        return link_lines + [joint_scatter, tcp_trace, tcp_marker,
                              seg_text, frame_text]

    interval_ms = 50   # 20 fps
    ani = animation.FuncAnimation(
        fig, update,
        frames=total_frames,
        init_func=init,
        interval=interval_ms,
        blit=False,
        repeat=True,
    )

    plt.tight_layout()
    plt.show()
    return ani   # keep reference to prevent GC


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    print("Building trajectory via UR5Sim + MotionPlanner ...")

    # Connect the simulator (validates the library plumbing)
    robot = UR5Sim()
    robot.connect_()

    trajectory = build_trajectory()
    print(f"Trajectory ready: {len(trajectory)} frames across "
          f"{len(SEQUENCE) - 1} segments.")
    print("Close the plot window to exit.")

    animate(trajectory)
