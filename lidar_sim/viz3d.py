"""3D plotting: range image, 3D scatter, bird's-eye view, rotating animation."""

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon

from . import viz


def _cmap():
    return plt.get_cmap("Blues").with_extremes(bad="#d9d8d4")


def range_image(ax, s3, vmax=6.0, title="range image (rings × azimuth)"):
    """Rows = rings (elevation), columns = azimuth; gray = no return."""
    el = np.degrees(s3.elevations)
    az = np.degrees(s3.azimuths)
    half = (el[1] - el[0]) / 2 if len(el) > 1 else 1.0
    im = ax.imshow(np.ma.masked_invalid(s3.ranges), aspect="auto", origin="lower", cmap=_cmap(), vmin=0, vmax=vmax,
                   extent=[az[0], az[-1], el[0] - half, el[-1] + half], interpolation="nearest")
    ax.set_xlabel("azimuth φ [deg] (0 = heading)")
    ax.set_ylabel("elevation ε [deg]")
    ax.set_title(title, fontsize=10)
    ax.grid(False)
    return im


def scatter3d(ax, s3, color_by="height", s=2.0):
    det = s3.detected
    P = s3.points_world[det]
    if color_by == "ghost":
        g = s3.is_ghost[det]
        ax.scatter(*P[~g].T, s=s, color=viz.C_REAL, linewidths=0, alpha=0.7, label="real")
        ax.scatter(*P[g].T, s=s * 2, color=viz.C_GHOST, linewidths=0, label="ghost")
    else:
        ax.scatter(*P.T, s=s, c=P[:, 2], cmap="Blues", vmin=-0.5, vmax=3.0, linewidths=0)
    ax.scatter(*s3.origin, s=40, color=viz.INK, marker="^")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    try:
        lo, hi = np.nanmin(P, 0), np.nanmax(P, 0)
        ax.set_box_aspect(np.maximum(hi - lo, 0.5))
    except ValueError:
        pass


def draw_footprints(ax, scene):
    seen = set()
    for name, xy in scene.footprints:
        ax.add_patch(Polygon(xy, closed=True, facecolor="none", edgecolor=viz.INK2, lw=1.0, ls=(0, (2, 1.5)), zorder=4))
        if name not in seen:
            seen.add(name)
            ax.annotate(name, xy.mean(0), xytext=(6, -10), textcoords="offset points", fontsize=7.5, color=viz.INK2)


def birds_eye(ax, scene, s3, highlight_ids=(), s=4):
    """Top-down view: walls, 3D-only object footprints (dashed), points coloured by height,
    points on highlighted surfaces in orange."""
    viz.draw_world(ax, scene.world2d)
    draw_footprints(ax, scene)
    det = s3.detected
    P = s3.points_world[det]
    sid = s3.surface_id[det]
    hl = np.isin(sid, list(highlight_ids))
    sc = ax.scatter(P[~hl, 0], P[~hl, 1], s=s, c=P[~hl, 2], cmap="Blues", vmin=-0.5, vmax=3.0, linewidths=0, zorder=5)
    if hl.any():
        ax.scatter(P[hl, 0], P[hl, 1], s=s * 2.5, color=viz.ORANGE, linewidths=0, zorder=6, label="low box / table")
    viz.draw_sensor(ax, s3.pose)
    return sc


def rotate_animation(s3, path, n_frames=72, title="", color_by="ghost"):
    fig = plt.figure(figsize=(6.4, 5.2))
    ax = fig.add_subplot(projection="3d")
    scatter3d(ax, s3, color_by=color_by, s=3)
    ax.set_title(title, fontsize=10)

    def update(k):
        ax.view_init(elev=28, azim=-60 + k * 360 / n_frames)

    return viz.animate(fig, update, n_frames, path, fps=15)
