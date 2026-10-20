"""Plotting helpers: scene, rays, points, FOV wedge, polar ranges, animations.

Colours follow a validated categorical order (blue, orange, aqua, ...). Scatter plots
use at most three categorical colours so they stay colour-blind safe.
"""

from pathlib import Path

import matplotlib

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon, Wedge

from .materials import GLASS, MIRROR

# --- palette (light mode, print) -------------------------------------------------
BLUE, ORANGE, AQUA, YELLOW, MAGENTA, GREEN, VIOLET, RED = (
    "#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6", "#e34948")
SERIES = [BLUE, ORANGE, AQUA, YELLOW, MAGENTA, GREEN, VIOLET, RED]
INK, INK2, MUTED, GRID, SURFACE = "#0b0b0b", "#52514e", "#8a8984", "#e4e3df", "#ffffff"

# roles
C_REAL, C_GHOST, C_THIRD = BLUE, ORANGE, AQUA
C_MIRROR, C_GLASS = VIOLET, AQUA


def set_style():
    """Consistent figure style for every experiment."""
    try:
        import imageio_ffmpeg

        plt.rcParams["animation.ffmpeg_path"] = imageio_ffmpeg.get_ffmpeg_exe()
    except Exception:  # pragma: no cover - MP4 export just becomes unavailable
        pass
    plt.rcParams.update({
        "figure.dpi": 110,
        "savefig.dpi": 200,
        "savefig.bbox": "tight",
        "figure.facecolor": SURFACE,
        "axes.facecolor": SURFACE,
        "axes.edgecolor": MUTED,
        "axes.labelcolor": INK,
        "axes.titlecolor": INK,
        "axes.titlesize": 11,
        "axes.labelsize": 10,
        "axes.grid": True,
        "grid.color": GRID,
        "grid.linewidth": 0.6,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.prop_cycle": matplotlib.cycler(color=SERIES),
        "xtick.color": INK2,
        "ytick.color": INK2,
        "font.size": 9.5,
        "legend.frameon": False,
        "legend.fontsize": 8.5,
        "lines.linewidth": 1.6,
        "lines.markersize": 5,
        "image.cmap": "Blues",
    })


set_style()


def wall_gray(rho):
    """Darker = less reflective (rho 0.1 -> near black, 0.9 -> light gray)."""
    g = 0.15 + 0.6 * float(rho)
    return (g, g, g)


def draw_world(ax, world, t=0.0, labels=True, equal=True, pad=0.3, lw=2.2):
    """Top-down drawing of a 2D world: matte walls in gray (by reflectivity), mirror violet,
    glass aqua dashed. Direct labels on mirror/glass/box so colour isn't the only cue."""
    w = world.at_time(t) if world.movers else world
    labelled = set()
    for s in w.segments:
        code = s.material.code
        xs, ys = [s.a[0], s.b[0]], [s.a[1], s.b[1]]
        if code == MIRROR:
            ax.plot(xs, ys, color=C_MIRROR, lw=lw + 1.6, solid_capstyle="round", zorder=4)
        elif code == GLASS:
            ax.plot(xs, ys, color=C_GLASS, lw=lw + 0.6, ls=(0, (4, 2)), zorder=4)
        else:
            ax.plot(xs, ys, color=wall_gray(s.material.rho), lw=lw, solid_capstyle="round", zorder=3)
        if labels and s.name in ("mirror", "glass", "box", "object", "dark wall", "bright wall") \
                and s.name not in labelled:
            labelled.add(s.name)
            mid = 0.5 * (s.a + s.b)
            ax.annotate(s.name, mid, xytext=(4, 4), textcoords="offset points", fontsize=7.5,
                        color=INK2, zorder=6)
    if equal:
        ax.set_aspect("equal")
        lo, hi = w.bounds(pad)
        ax.set_xlim(lo[0], hi[0])
        ax.set_ylim(lo[1], hi[1])
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    return ax


def draw_sensor(ax, pose, size=0.18, color=INK):
    x, y, th = pose[:3]
    tri = np.array([[size, 0], [-size * 0.6, size * 0.5], [-size * 0.6, -size * 0.5]])
    c, s = np.cos(th), np.sin(th)
    tri = tri @ np.array([[c, s], [-s, c]]) + [x, y]
    ax.add_patch(Polygon(tri, closed=True, color=color, zorder=8))


def draw_rays(ax, origin, points, color=BLUE, alpha=0.25, lw=0.6):
    """Thin rays from origin to each point (NaN points are skipped)."""
    pts = np.asarray(points)
    ok = np.all(np.isfinite(pts), axis=1)
    if not ok.any():
        return
    o = np.asarray(origin)[:2]
    segs = np.stack([np.broadcast_to(o, pts[ok].shape), pts[ok]], axis=1)
    from matplotlib.collections import LineCollection

    ax.add_collection(LineCollection(segs, colors=color, alpha=alpha, linewidths=lw, zorder=2))


def draw_fov(ax, pose, fov_deg, radius, color=BLUE):
    """FOV wedge outline (dashed): the range/angle part of S(x), before line of sight."""
    x, y, th = pose[:3]
    th_deg = np.degrees(th)
    ax.add_patch(Wedge((x, y), radius, th_deg - fov_deg / 2, th_deg + fov_deg / 2, facecolor="none",
                       edgecolor=color, lw=0.9, ls=(0, (3, 2)), alpha=0.7, zorder=1))


def draw_visibility(ax, poly, color=BLUE, alpha=0.09):
    """Filled visibility polygon: FOV wedge minus everything hidden (glass is transparent)."""
    ax.add_patch(Polygon(poly, closed=True, facecolor=color, alpha=alpha, edgecolor="none", zorder=1))


def save(fig, out, name):
    """Save a figure to out/name (.png unless an extension is given); returns the path."""
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    path = out / (name if Path(name).suffix else f"{name}.png")
    fig.savefig(path)
    return path


def draw_scan(ax, scan, show_rays=True, show_true=True, show_dropouts=True, r_drop=None, ms=9):
    """Points the LiDAR reports: real (blue dots), ghosts (orange x) linked by a dotted line to
    where the light really came from (aqua circles). Dropout beams are faint gray rays."""
    p = scan.pose[:2]
    det = scan.detected
    real, ghost = det & ~scan.is_ghost, det & scan.is_ghost
    if show_rays:
        draw_rays(ax, p, scan.points_world[det], color=BLUE, alpha=0.18, lw=0.5)
    if show_dropouts and (~det).any():
        r = 1.2 if r_drop is None else r_drop
        u = np.stack([np.cos(scan.pose[2] + scan.beam_angles), np.sin(scan.pose[2] + scan.beam_angles)], 1)
        draw_rays(ax, p, p + r * u[~det], color=MUTED, alpha=0.35, lw=0.5)
    ax.scatter(*scan.points_world[real].T, s=ms, color=C_REAL, zorder=6, label="reported point", linewidths=0)
    if ghost.any():
        ax.scatter(*scan.points_world[ghost].T, s=ms * 2.2, color=C_GHOST, marker="x", zorder=7,
                   label="ghost (reported)", linewidths=1.3)
        if show_true:
            th = scan.true_hits[ghost]
            ax.scatter(*th.T, s=ms * 1.6, facecolors="none", edgecolors=C_THIRD, zorder=7,
                       label="true source of ghost", linewidths=1.1)
            for a, b in zip(scan.points_world[ghost], th):
                ax.plot([a[0], b[0]], [a[1], b[1]], color=C_THIRD, lw=0.5, ls=":", alpha=0.6, zorder=5)
    draw_sensor(ax, scan.pose)


def polar_ranges(ax, scan, r_max=None, title="range profile (body frame)"):
    """Polar plot of z_i vs beam angle beta_i; ghosts in orange."""
    det = scan.detected
    real, ghost = det & ~scan.is_ghost, det & scan.is_ghost
    ax.scatter(scan.beam_angles[real], scan.ranges[real], s=7, color=C_REAL, linewidths=0, label="reported")
    if ghost.any():
        ax.scatter(scan.beam_angles[ghost], scan.ranges[ghost], s=14, color=C_GHOST, marker="x",
                   linewidths=1.0, label="ghost")
    ax.set_theta_zero_location("N")
    lim = np.degrees(np.max(np.abs(scan.beam_angles))) if len(scan.beam_angles) else 180
    ax.set_thetalim(-np.radians(lim), np.radians(lim))
    if r_max:
        ax.set_rmax(r_max)
    ax.set_title(title, fontsize=10)
    ax.tick_params(labelsize=7.5)


def legend_below(ax, ncol=3, **kw):
    handles, labels = ax.get_legend_handles_labels()
    seen = dict(zip(labels, handles))
    if seen:
        ax.legend(seen.values(), seen.keys(), loc="upper center", bbox_to_anchor=(0.5, -0.12), ncol=ncol, **kw)


def animate(fig, update, n_frames, out_path, fps=10, dpi=None):
    """Render update(k) for k in range(n_frames) to a GIF (pillow) or MP4 (bundled ffmpeg).

    GIFs default to a lower dpi (80) to keep files small; MP4s default to 130.
    """
    from matplotlib.animation import FFMpegWriter, FuncAnimation, PillowWriter

    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    anim = FuncAnimation(fig, update, frames=n_frames, blit=False)
    writer = FFMpegWriter(fps=fps, bitrate=2400) if out_path.suffix == ".mp4" else PillowWriter(fps=fps)
    if dpi is None:
        dpi = 130 if out_path.suffix == ".mp4" else 80
    anim.save(out_path, writer=writer, dpi=dpi)
    return out_path
