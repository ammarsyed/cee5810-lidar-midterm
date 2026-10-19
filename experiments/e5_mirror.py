"""E5 — MIRROR: ghost points and the ghost-point theorem (D8).

(a) One scan with ghost points, their true sources, and the "virtual room" (every wall reflected
    across the mirror line) — the ghosts lie on the virtual walls.
(b) Mirror-angle sweep: how many ghosts and head-on mirror returns the sensor sees.
(c) The theorem check: |reported point - Ref(true hit)| is ~1e-15 m noise-free, and with noise the
    error along the beam is N(0, sigma_R^2).

Run: uv run python -m experiments.e5_mirror [--person partner] [--fast]
"""

import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import norm

from experiments.common import finish, parser, preset, resolve
from lidar_sim import viz
from lidar_sim.geometry import reflect_point, segment_normal
from lidar_sim.materials import MIRROR
from lidar_sim.scenes import get_scene
from lidar_sim.sensor import LidarParams, scan


def _mirror(world):
    return [s for s in world.segments if s.material.code == MIRROR][0]


def run(scene="default", seed=0, fast=False):
    pr = preset(scene, "e5")
    world = get_scene(scene, with_moving_box=False)
    params = LidarParams()
    rng = np.random.default_rng(seed)
    pose = np.array([pr["pose"][0], pr["pose"][1], np.radians(pr["pose"][2])])
    m = _mirror(world)
    n_hat = segment_normal(m.a, m.b)

    s = scan(world, pose, params, rng=rng)
    fig1, ax = plt.subplots(figsize=(7.4, 5.6))
    for seg in world.segments:  # the virtual room behind the mirror
        if seg is m:
            continue
        a, b = reflect_point(seg.a, m.a, n_hat), reflect_point(seg.b, m.a, n_hat)
        ax.plot([a[0], b[0]], [a[1], b[1]], color=viz.MUTED, lw=1.0, ls=(0, (2, 2)), zorder=2)
    ax.plot([], [], color=viz.MUTED, lw=1.0, ls=(0, (2, 2)), label="virtual room (walls reflected in mirror)")
    viz.draw_world(ax, world, equal=False)
    viz.draw_scan(ax, s, show_dropouts=False)
    ax.set_aspect("equal")
    allpts = np.vstack([np.array([p for seg in world.segments for p in (seg.a, seg.b)]),
                        np.array([reflect_point(p, m.a, n_hat) for seg in world.segments for p in (seg.a, seg.b)])])
    lo, hi = allpts.min(0) - 0.4, allpts.max(0) + 0.4
    ax.set_xlim(lo[0], hi[0])
    ax.set_ylim(lo[1], hi[1])
    ax.set_title(f"E5a: {int((s.detected & s.is_ghost).sum())} ghost points lie on the virtual room")
    viz.legend_below(ax, ncol=2)

    # (b) mirror-angle sweep
    angles = np.arange(0, 180, 15 if fast else 5)
    ghosts, head_on = [], []
    builder = get_scene.__globals__["SCENES"][scene]
    for a in angles:
        w = builder(with_moving_box=False, mirror_angle_deg=float(a))
        sc = scan(w, pose, params, noise=False)
        mid = w.surface_ids("mirror")
        ghosts.append(int((sc.detected & sc.is_ghost).sum()))
        head_on.append(int((sc.detected & ~sc.is_ghost & np.isin(sc.surface_id, mid)).sum()))
    fig2, ax2 = plt.subplots(figsize=(6.8, 3.8))
    ax2.plot(angles, ghosts, color=viz.ORANGE, lw=1.8, marker="o", ms=4, label="ghost points")
    ax2.plot(angles, head_on, color=viz.VIOLET, lw=1.8, marker="s", ms=4, label="mirror seen head-on (true position)")
    ax2.set_xlabel("mirror line angle [deg]")
    ax2.set_ylabel("# beams")
    ax2.set_title("E5b: what the LiDAR sees of the mirror depends on its orientation")
    ax2.legend(fontsize=7.5)

    # (c) theorem check
    clean = scan(world, pose, params, noise=False)
    r = clean.returns
    mirror_id = world.surface_ids(m.name)
    sel = (r.reflections == 1) & (r.kind == 0) & np.isin(r.reflector, mirror_id)
    err_clean = np.linalg.norm(r.apparent_point[sel] - reflect_point(r.hit_point[sel], m.a, n_hat), axis=1)
    n_rep = 30 if fast else 150
    along = []
    for _ in range(n_rep):
        sn = scan(world, pose, params, rng=rng)
        g = sn.detected & sn.is_ghost
        along.append(sn.ranges[g] - sn.true_ranges[g])  # noise along the beam, same for mirror/glass ghosts
    along = np.concatenate(along) if along else np.zeros(0)
    fig3, ax3 = plt.subplots(figsize=(6.4, 3.8))
    if len(along):
        ax3.hist(along * 100, bins=40, density=True, color=viz.BLUE, alpha=0.75, edgecolor="white", linewidth=0.5,
                 label=f"ghost range error ({len(along)} ghosts)")
        x = np.linspace(-4, 4, 300)
        ax3.plot(x, norm.pdf(x, 0, params.sigma_r * 100), color=viz.ORANGE, lw=1.8, label="N(0, σ_R²), σ_R = 1 cm")
    ax3.set_xlabel("reported − true path length along the beam [cm]")
    ax3.set_ylabel("density")
    ax3.set_title("E5c: ghosts carry the same range noise as real points")
    ax3.legend(fontsize=7.5)

    rows = [["ghost returns checked (noise-free)", int(sel.sum())],
            ["max |reported − Ref(true hit)| (noise-free) [m]", float(err_clean.max()) if len(err_clean) else np.nan],
            ["ghost range error mean [cm]", float(along.mean() * 100) if len(along) else np.nan],
            ["ghost range error std [cm]", float(along.std() * 100) if len(along) else np.nan],
            ["mirror angle with most ghosts [deg]", int(angles[int(np.argmax(ghosts))])]]
    return {"figs": {"ghosts": fig1, "angle_sweep": fig2, "theorem_check": fig3},
            "tables": {"ghost_theorem": (["quantity", "value"], rows)},
            "data": {"angles": angles, "ghosts": ghosts, "head_on": head_on}}


def main():
    args = resolve(parser(__doc__).parse_args())
    finish(run(args.scene, args.seed, args.fast), args.out, "e5")


if __name__ == "__main__":
    main()
