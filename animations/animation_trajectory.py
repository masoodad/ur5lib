# ur5lib/animations/animation_trajectory.py
"""
Reusable kinematics and animation helpers for any serial manipulator.

DH parameters come from ``ur5lib.robots`` — never hardcoded here.
``forward_kinematics`` is DOF-agnostic (n joints → (n+1, 3) origins).
``animate_trajectories`` renders any number of pre-built frame sequences
side-by-side in one call — no animation boilerplate needed in demos.

Public API
----------
  dh_matrix(theta, a, d, alpha)  → 4×4 ndarray
  forward_kinematics(joints, dh_params=None)  → (n+1, 3) ndarray
  animate_trajectories(results, title, interval)
  LINK_COLORS                    — 6-colour list (cycles for higher DOF)
"""

import itertools

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

from ur5lib.robots import UR5_DH

LINK_COLORS = ["#e74c3c", "#e67e22", "#f1c40f",
               "#2ecc71", "#3498db", "#9b59b6"]


# ---------------------------------------------------------------------------
# Kinematics
# ---------------------------------------------------------------------------

def dh_matrix(theta: float, a: float, d: float, alpha: float) -> np.ndarray:
    """Return the 4×4 homogeneous transformation matrix for one DH joint."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.,  sa,       ca,      d     ],
        [0.,  0.,       0.,      1.    ],
    ])


def forward_kinematics(joints: list,
                       dh_params: np.ndarray = None) -> np.ndarray:
    """
    Compute frame origins for a given joint configuration.

    DOF is derived automatically from *dh_params*, so this works for any
    serial manipulator. Defaults to ``UR5_DH`` from ``ur5lib.robots``.

    Parameters
    ----------
    joints    : sequence of float (radians), length == DOF
    dh_params : (n, 3) array of [a, d, alpha].  None → use UR5_DH.

    Returns
    -------
    np.ndarray, shape (n+1, 3) — base origin + one origin per joint frame.
    """
    if dh_params is None:
        dh_params = UR5_DH
    T = np.eye(4)
    positions = [T[:3, 3].copy()]
    for theta, (a, d, alpha) in zip(joints, dh_params):
        T = T @ dh_matrix(theta, a, d, alpha)
        positions.append(T[:3, 3].copy())
    return np.array(positions)


# ---------------------------------------------------------------------------
# Animation
# ---------------------------------------------------------------------------

def _link_colors_for(n_links: int) -> list:
    """Return a list of *n_links* colours, cycling if needed."""
    cycle = itertools.cycle(LINK_COLORS)
    return [next(cycle) for _ in range(n_links)]


def _make_panel(fig, pos, data: dict, lim: float):
    """Build one dark-theme 3-D subplot.  Returns (ax, artists_dict)."""
    ax = fig.add_subplot(*pos, projection="3d")
    ax.set_facecolor("#0d0d1a")
    ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_zlim(0, lim * 1.7)
    ax.set_xlabel("X (m)", color="#888888", labelpad=5, fontsize=7)
    ax.set_ylabel("Y (m)", color="#888888", labelpad=5, fontsize=7)
    ax.set_zlabel("Z (m)", color="#888888", labelpad=5, fontsize=7)
    ax.tick_params(colors="#555555", labelsize=6)
    for pane in (ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane):
        pane.fill = False
        pane.set_edgecolor("#1e1e3a")
    ax.view_init(elev=24, azim=-50)

    # Ground grid
    g = np.linspace(-lim, lim, 5)
    for v in g:
        ax.plot([v, v], [-lim, lim], [0, 0], color="#1e1e3a", lw=0.5, zorder=0)
        ax.plot([-lim, lim], [v, v], [0, 0], color="#1e1e3a", lw=0.5, zorder=0)

    # Optional metadata in title
    name       = data.get("name", "Trajectory")
    best_for   = data.get("best_for", "")
    complexity = data.get("complexity", "")
    smoothness = data.get("smoothness", "")
    desc       = data.get("desc", "")
    accent     = data.get("accent", LINK_COLORS[0])

    title_lines = [f"$\\bf{{{name.replace(' ', '\\ ')}}}$"]
    if best_for:
        title_lines.append(f"Best for: {best_for}")
    if complexity or smoothness:
        title_lines.append(
            "  |  ".join(filter(None, [
                f"Complexity: {complexity}" if complexity else "",
                f"Smoothness: {smoothness}" if smoothness else "",
            ]))
        )
    ax.set_title("\n".join(title_lines),
                 color="white", fontsize=8.5, pad=10, linespacing=1.6)

    if desc:
        ax.text2D(0.02, 0.01, desc, transform=ax.transAxes,
                  fontsize=6, color="#888888", va="bottom",
                  bbox=dict(boxstyle="round,pad=0.3",
                            fc="#0a0a15", alpha=0.7, ec="#333355"))

    # n_links derived from frame shape (n+1 points → n links)
    n_links = data["frames"][0].shape[0] - 1
    colors  = _link_colors_for(n_links)

    link_lines = [
        ax.plot([], [], [], lw=4, color=colors[j],
                solid_capstyle="round")[0]
        for j in range(n_links)
    ]
    joint_sph = ax.scatter([], [], [], s=35, color="white",
                           zorder=5, depthshade=False)
    tcp_trace, = ax.plot([], [], [], color=accent, lw=1.8, alpha=0.75, zorder=4)
    tcp_star,  = ax.plot([], [], [], "*", color=accent, markersize=13, zorder=6)
    ftxt = ax.text2D(0.97, 0.01, "", transform=ax.transAxes,
                     fontsize=6, color="#555555", ha="right")

    return ax, dict(
        links=link_lines, joints=joint_sph,
        trace=tcp_trace, star=tcp_star, ftxt=ftxt,
        hist={"x": [], "y": [], "z": []},
        frames=data["frames"], n=len(data["frames"]),
    )


def animate_trajectories(results: list,
                         title: str = "Trajectory Animation",
                         interval: int = 40):
    """
    Animate one or more pre-built trajectory frame sequences side-by-side.

    Each entry in *results* is a dict with:

    Required
    --------
    ``frames`` : list of (n+1, 3) ndarrays  — output of ``forward_kinematics``
    ``name``   : str — panel label

    Optional (all default to empty / auto)
    --------
    ``accent``     : hex colour for the TCP trace & star marker
    ``best_for``   : str
    ``complexity`` : str
    ``smoothness`` : str
    ``desc``       : str — description shown in a text box

    Parameters
    ----------
    results  : list[dict]  — one dict per trajectory panel
    title    : str  — figure suptitle
    interval : int  — milliseconds between frames (~fps = 1000/interval)

    Returns
    -------
    matplotlib.animation.FuncAnimation  (keep the reference alive)

    Example
    -------
    >>> from ur5lib.animations import forward_kinematics, animate_trajectories
    >>> frames = [forward_kinematics(q) for q in my_trajectory]
    >>> animate_trajectories([{"name": "My Traj", "frames": frames}])
    """
    n_panels = len(results)

    fig = plt.figure(figsize=(7 * n_panels, 7.5))
    fig.patch.set_facecolor("#0a0a15")
    fig.suptitle(title, color="white", fontsize=15, fontweight="bold", y=0.99)

    all_pts = np.vstack([np.vstack(d["frames"]) for d in results])
    lim = max(float(np.abs(all_pts).max()), 0.55) + 0.1

    panels = []
    for i, data in enumerate(results):
        _, artists = _make_panel(fig, (1, n_panels, i + 1), data, lim)
        panels.append(artists)

    n_max = max(p["n"] for p in panels)

    def init():
        out = []
        for p in panels:
            for ln in p["links"]:
                ln.set_data([], []); ln.set_3d_properties([])
            p["trace"].set_data([], []); p["trace"].set_3d_properties([])
            p["star"].set_data([], []);  p["star"].set_3d_properties([])
            p["hist"]["x"].clear(); p["hist"]["y"].clear(); p["hist"]["z"].clear()
            out += p["links"] + [p["trace"], p["star"]]
        return out

    def update(frame):
        out = []
        for p in panels:
            idx = frame % p["n"]
            pts = p["frames"][idx]

            for j, ln in enumerate(p["links"]):
                ln.set_data([pts[j, 0], pts[j+1, 0]],
                            [pts[j, 1], pts[j+1, 1]])
                ln.set_3d_properties([pts[j, 2], pts[j+1, 2]])

            p["joints"]._offsets3d = (pts[:, 0], pts[:, 1], pts[:, 2])

            p["hist"]["x"].append(pts[-1, 0])
            p["hist"]["y"].append(pts[-1, 1])
            p["hist"]["z"].append(pts[-1, 2])
            p["trace"].set_data(p["hist"]["x"], p["hist"]["y"])
            p["trace"].set_3d_properties(p["hist"]["z"])

            p["star"].set_data([pts[-1, 0]], [pts[-1, 1]])
            p["star"].set_3d_properties([pts[-1, 2]])

            p["ftxt"].set_text(f"{idx+1}/{p['n']}")
            out += p["links"] + [p["trace"], p["star"], p["ftxt"]]
        return out

    ani = animation.FuncAnimation(
        fig, update, frames=n_max,
        init_func=init, interval=interval,
        blit=False, repeat=True,
    )
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.show()
    return ani
