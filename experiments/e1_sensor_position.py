"""E1 — sensor POSITION -> measurements (D3).

The sensor translates along a straight path with a fixed heading. For selected beams we compare
the simulated range with the closed form R = (d - n.p) / (n.u) for a flat wall, and show that
moving parallel to a wall leaves that wall's range unchanged (dR/dp is proportional to n).

Run: uv run python -m experiments.e1_sensor_position --person ammar|gal [--fast]
"""

import matplotlib.pyplot as plt
import numpy as np

from experiments.common import finish, load_person, parser, resolve, scene_of
from lidar_sim import viz
from lidar_sim.sensor import LidarParams, beam_directions, scan


def _closed_form(p, u, n, d):
    n = np.asarray(n, float)
    return (d - p @ n) / (u @ n)


def run(person="ammar", seed=None, fast=False, animate_to=None):
    cfg = load_person(person)
    seed = cfg["seed"] if seed is None else seed
    pr = cfg["e1"]
    world = scene_of(cfg, with_moving_box=False)
    params = LidarParams()
    rng = np.random.default_rng(seed)
    n = 31 if fast else 61
    start, end = np.array(pr["start"], float), np.array(pr["end"], float)
    xy = start + np.linspace(0, 1, n)[:, None] * (end - start)
    th = np.radians(pr["heading_deg"])
    poses = np.column_stack([xy, np.full(n, th)])
    axis = int(np.argmax(np.abs(end - start)))  # coordinate that changes along the path
    coord = xy[:, axis]
    cname = "xy"[axis]

    scans = [scan(world, p, params, rng=rng) for p in poses]
    true = np.array([s.true_ranges for s in scans])
    z = np.array([s.ranges for s in scans])
    beta_deg = np.degrees(params.beam_angles())

    # --- fig 1: range of selected beams vs position, with the closed form --------------------
    fig1, ax = plt.subplots(figsize=(7.2, 4.2))
    rows = []
    for k, (b, nrm, d, label) in enumerate(pr["beams"]):
        i = int(np.argmin(np.abs(beta_deg - b)))
        u = beam_directions(poses, params)[:, i]
        cf = np.array([_closed_form(p[:2], uu, nrm, d) for p, uu in zip(poses, u)])
        col = viz.SERIES[k]
        ax.plot(coord, cf, color=col, lw=1.6, ls="--", label=f"{label} (closed form)")
        det = np.isfinite(z[:, i])
        ax.scatter(coord[det], z[det, i], s=16, color=col, zorder=4, linewidths=0, label=f"{label} (measured)")
        if (~det).any():
            ax.scatter(coord[~det], np.full((~det).sum(), 0.15), marker="|", s=50, color=col, zorder=4,
                       label=f"{label}: no return")
        hits = np.array([sc.true_hits[i] for sc in scans])
        on_wall = np.isfinite(true[:, i]) & (np.abs(hits @ np.asarray(nrm, float) - d) < 1e-6)
        err = np.abs(true[on_wall, i] - cf[on_wall]).max() if on_wall.any() else np.nan
        dm = det & on_wall
        rms = np.sqrt(np.mean((z[dm, i] - cf[dm]) ** 2)) if dm.any() else np.nan
        slope = np.polyfit(coord, cf, 1)[0]
        rows.append([label, f"{slope:+.3f}", float(err), float(rms), f"{det.mean():.0%}",
                     int((~on_wall & np.isfinite(true[:, i])).sum())])
    ax.set_xlabel(f"sensor {cname} [m]")
    ax.set_ylabel("range z [m]")
    ax.set_ylim(bottom=0)
    ax.set_title("E1: range vs sensor position (dashed = closed form D3)")
    ax.legend(fontsize=7.5, ncol=2, loc="upper center", bbox_to_anchor=(0.5, -0.16))

    # --- fig 2: full range profile vs position -------------------------------------------------
    fig2, ax2 = plt.subplots(figsize=(7.2, 4.2))
    zz = np.ma.masked_invalid(z.T)
    cmap = plt.get_cmap("Blues").with_extremes(bad="#d9d8d4")
    im = ax2.imshow(zz, aspect="auto", origin="lower", cmap=cmap, vmin=0, vmax=6,
                    extent=[coord[0], coord[-1], beta_deg[0], beta_deg[-1]])
    fig2.colorbar(im, ax=ax2, label="measured range z [m] (gray = no return)")
    ax2.set_xlabel(f"sensor {cname} [m]")
    ax2.set_ylabel("beam angle β [deg]")
    ax2.set_title("E1: every beam's range as the sensor moves")
    ax2.grid(False)

    # --- fig 3: the setup ------------------------------------------------------------------------
    fig3, ax3 = plt.subplots(figsize=(6.4, 5))
    viz.draw_world(ax3, world)
    ax3.plot(xy[:, 0], xy[:, 1], color=viz.INK, lw=1.2, ls=":", label="sensor path")
    for p in (poses[0], poses[-1]):
        viz.draw_fov(ax3, p, params.fov_deg, params.r_max_ref)
        viz.draw_sensor(ax3, p)
    for k, (b, *_rest) in enumerate(pr["beams"]):
        i = int(np.argmin(np.abs(beta_deg - b)))
        s0 = scans[n // 2]
        if np.isfinite(s0.true_ranges[i]):
            ax3.plot([s0.pose[0], s0.true_hits[i, 0]], [s0.pose[1], s0.true_hits[i, 1]], color=viz.SERIES[k], lw=1.6)
    ax3.set_title("E1 setup: path, FOV at both ends, tracked beams (mid-path)")
    viz.legend_below(ax3, ncol=2)

    media = []
    if animate_to:
        media.append(_animate(world, poses, params, seed, animate_to))
    header = ["beam", "dR/d(position) closed form", "max |sim − closed form| on wall (noise-free) [m]",
              "RMS measured error on wall [m]", "detected", "samples blocked by another object"]
    return {"figs": {"ranges": fig1, "profile": fig2, "setup": fig3}, "tables": {"closed_form": (header, rows)},
            "data": {"coord": coord, "z": z, "true": true}, "media": media}


def _animate(world, poses, params, seed, path):
    rng = np.random.default_rng(seed)
    fig = plt.figure(figsize=(10, 4.4))
    ax = fig.add_subplot(1, 2, 1)
    axp = fig.add_subplot(1, 2, 2, projection="polar")

    def update(k):
        ax.clear()
        axp.clear()
        s = scan(world, poses[k], params, rng=rng)
        viz.draw_world(ax, world)
        viz.draw_fov(ax, poses[k], params.fov_deg, params.r_max_ref)
        viz.draw_scan(ax, s, show_dropouts=False)
        ax.set_title(f"sensor at ({poses[k, 0]:.2f}, {poses[k, 1]:.2f})", fontsize=10)
        viz.polar_ranges(axp, s, r_max=6.0)

    fig.tight_layout()
    return viz.animate(fig, update, len(poses), path, fps=8)


def main():
    args = resolve(parser(__doc__).parse_args())
    anim = None if args.no_anim else f"{args.out}/e1_drive.gif"
    finish(run(args.person, args.seed, args.fast, animate_to=anim), args.out, "e1")


if __name__ == "__main__":
    main()
