"""E3 — TARGET STATE (moving box) -> measurements (D4 LOS, D11 dz/dx_T).

The sensor is fixed; the box moves through one full back-and-forth period. The prediction uses the
box geometry alone (a world containing only the box); the simulation uses the full scene, so the
two differ exactly where something else blocks the line of sight.

Run: uv run python -m experiments.e3_moving_target --person ammar|gal [--fast]
"""

import matplotlib.pyplot as plt
import numpy as np

from experiments.common import finish, load_person, parser, resolve, scene_of
from lidar_sim import viz
from lidar_sim.sensor import LidarParams, scan
from lidar_sim.world import World


def _intervals(mask, t):
    out, on = [], None
    for k, m in enumerate(mask):
        if m and on is None:
            on = t[k]
        if not m and on is not None:
            out.append((on, t[k - 1]))
            on = None
    if on is not None:
        out.append((on, t[-1]))
    return out


def run(person="ammar", seed=None, fast=False, animate_to=None):
    cfg = load_person(person)
    seed = cfg["seed"] if seed is None else seed
    pr = cfg["e3"]
    world = scene_of(cfg)
    box = world.movers[0]
    box_only = World([], [box])
    params = LidarParams()
    rng = np.random.default_rng(seed)
    pose = np.array([pr["pose"][0], pr["pose"][1], np.radians(pr["pose"][2])])
    period = _period(box)
    times = np.linspace(0, period, 60 if fast else 160)
    beta = np.degrees(params.beam_angles())
    i = int(np.argmin(np.abs(beta - pr["beam_deg"])))
    box_ids = world.at_time(0).surface_ids("box")

    z_beam, pred_beam, full, n_box = [], [], [], []
    for t in times:
        s = scan(world, pose, params, rng=rng, t=t)
        sb = scan(box_only, pose, params, noise=False, t=t)
        z_beam.append(s.ranges[i])
        pred_beam.append(sb.true_ranges[i])
        full.append(scan(world, pose, params, noise=False, t=t).ranges)
        n_box.append(int((s.detected & np.isin(s.surface_id, box_ids)).sum()))
    z_beam, pred_beam, full, n_box = map(np.array, (z_beam, pred_beam, full, n_box))
    static = scan(scene_of(cfg, with_moving_box=False), pose, params, noise=False).true_ranges[i]

    fig1, ax = plt.subplots(figsize=(7.2, 4.0))
    ax.axhline(static, color=viz.MUTED, lw=1.2, ls="--", label="background (no box)")
    ax.plot(times, pred_beam, color=viz.ORANGE, lw=2.6, alpha=0.85, zorder=5, label="predicted from box geometry alone")
    det = np.isfinite(z_beam)
    ax.scatter(times[det], z_beam[det], s=12, color=viz.BLUE, zorder=4, linewidths=0, label="measured z (full scene)")
    ax.set_xlabel("time t [s]")
    ax.set_ylabel(f"range of beam β = {beta[i]:.0f}° [m]")
    ax.set_ylim(bottom=0)
    ax.set_title("E3: the moving box enters and leaves one beam")
    ax.legend(fontsize=7.5, loc="lower right")

    fig2, ax2 = plt.subplots(figsize=(7.2, 4.2))
    cmap = plt.get_cmap("Blues").with_extremes(bad="#d9d8d4")
    im = ax2.imshow(np.ma.masked_invalid(full.T), aspect="auto", origin="lower", cmap=cmap, vmin=0, vmax=6,
                    extent=[times[0], times[-1], beta[0], beta[-1]])
    fig2.colorbar(im, ax=ax2, label="range [m] (gray = no return)")
    ax2.axhline(beta[i], color=viz.ORANGE, lw=0.8, ls=":")
    ax2.set_xlabel("time t [s]")
    ax2.set_ylabel("beam angle β [deg]")
    ax2.set_title("E3: the box sweeps across the beams (short-range streak)")
    ax2.grid(False)

    fig3, ax3 = plt.subplots(figsize=(6.4, 5))
    viz.draw_world(ax3, scene_of(cfg, with_moving_box=False))
    traj = np.array([box.trajectory(t)[:2] for t in times])
    ax3.plot(*traj.T, color=viz.ORANGE, lw=1.2, ls=":", label="box path")
    for t in (times[0], times[len(times) // 4]):
        for seg in box.segments_at(t):
            ax3.plot([seg.a[0], seg.b[0]], [seg.a[1], seg.b[1]], color=viz.ORANGE, lw=2)
    viz.draw_fov(ax3, pose, params.fov_deg, params.r_max_ref)
    u = np.array([np.cos(pose[2] + np.radians(beta[i])), np.sin(pose[2] + np.radians(beta[i]))])
    ax3.plot(*np.array([pose[:2], pose[:2] + 6 * u]).T, color=viz.BLUE, lw=1.2, label=f"tracked beam β={beta[i]:.0f}°")
    viz.draw_sensor(ax3, pose)
    ax3.set_title("E3 setup")
    viz.legend_below(ax3, ncol=2)

    pi = _intervals(np.isfinite(pred_beam), times)
    si = _intervals(np.isfinite(z_beam) & (np.abs(z_beam - np.nan_to_num(pred_beam, nan=-9)) < 0.1), times)
    rows = [[f"{a:.2f}–{b:.2f}" for a, b in pi], [f"{a:.2f}–{b:.2f}" for a, b in si]]
    width = max(len(r) for r in rows)
    rows = [["predicted (box geometry)"] + rows[0] + [""] * (width - len(rows[0])),
            ["simulated (full scene)"] + rows[1] + [""] * (width - len(rows[1]))]
    header = ["box on tracked beam: time intervals [s]"] + [f"#{k + 1}" for k in range(width)]

    media = []
    if animate_to:
        media.append(_animate(world, pose, params, times, seed, animate_to))
    return {"figs": {"beam_vs_time": fig1, "beams_vs_time": fig2, "setup": fig3},
            "tables": {"intervals": (header, rows)},
            "data": {"times": times, "z": z_beam, "pred": pred_beam, "n_box": n_box}, "media": media}


def _period(box):
    """Period of the box trajectory (ping_pong trajectories carry it)."""
    return float(getattr(box.trajectory, "period", 20.0))


def _animate(world, pose, params, times, seed, path):
    rng = np.random.default_rng(seed)
    fig, ax = plt.subplots(figsize=(6, 4.8))

    def update(k):
        ax.clear()
        s = scan(world, pose, params, rng=rng, t=times[k])
        viz.draw_world(ax, world, t=times[k])
        viz.draw_scan(ax, s, show_dropouts=False)
        ax.set_title(f"t = {times[k]:.1f} s", fontsize=10)

    return viz.animate(fig, update, len(times), path, fps=12)


def main():
    args = resolve(parser(__doc__).parse_args())
    anim = None if args.no_anim else f"{args.out}/e3_moving_box.gif"
    finish(run(args.person, args.seed, args.fast, animate_to=anim), args.out, "e3")


if __name__ == "__main__":
    main()
