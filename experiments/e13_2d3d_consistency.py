"""E13 — validation: the 2D model is the ε = 0 slice of the 3D model (D14e). Upgrade U2.

On the extruded room (walls, mirror and glass all vertical; floor and ceiling added):
(a) the ε = 0 ring of the 3D sensor vs the 2D sensor, beam by beam (noise-free), incl. ghosts;
(b) R_3D / R_2D for every ring on wall hits vs 1/cos ε.

Run: uv run python -m experiments.e13_2d3d_consistency --person ammar|gal [--fast]
"""

import matplotlib.pyplot as plt
import numpy as np

from experiments.common import finish, load_person, parser, resolve, scene_of
from lidar_sim import viz
from lidar_sim.scenes3d import extrude
from lidar_sim.sensor import scan
from lidar_sim.sensor3d import LidarParams3D, scan3d


def run(person="ammar", seed=None, fast=False):
    cfg = load_person(person)
    seed = cfg["seed"] if seed is None else seed
    pr = cfg["e13"]
    w2 = scene_of(cfg, with_moving_box=False)
    sc = extrude(w2, 3.0)
    pose = np.array([pr["pose"][0], pr["pose"][1], np.radians(pr["pose"][2])])
    s2 = scan(w2, pose, noise=False)
    beta = np.degrees(s2.beam_angles)

    flat = scan3d(sc, pose, LidarParams3D(elevations_override=(0.0,)), noise=False)
    az = np.degrees(flat.azimuths)
    idx = np.array([int(np.argmin(np.abs(az - b))) for b in beta])
    r3 = flat.true_ranges[0, idx]
    ok = np.isfinite(s2.true_ranges) & np.isfinite(r3)
    diff = np.abs(r3 - s2.true_ranges)
    same_detect = float(np.mean((flat.p_detect[0, idx] >= 0.5) == (s2.p_detect >= 0.5)))

    fig1, (a1, a2) = plt.subplots(2, 1, figsize=(7.4, 5.2), sharex=True, gridspec_kw={"height_ratios": [2, 1]})
    a1.plot(beta, s2.true_ranges, color=viz.BLUE, lw=2.4, label="2D sensor")
    a1.plot(beta, r3, color=viz.ORANGE, lw=1.0, ls="--", label="3D sensor, ε = 0 ring")
    g = s2.is_ghost & np.isfinite(s2.true_ranges)
    a1.scatter(beta[g], s2.true_ranges[g], s=14, color=viz.ORANGE, marker="x", zorder=5, label="ghost (both)")
    a1.set_ylabel("noise-free range [m]")
    a1.set_title("E13a: the horizontal ring reproduces the 2D scan, ghosts included")
    a1.legend(fontsize=7.5)
    a2.semilogy(beta[ok], np.maximum(diff[ok], 1e-9), color=viz.BLUE, lw=1.0, marker=".", ms=3)
    a2.set_ylabel("|R_3D − R_2D| [m]")
    a2.set_xlabel("beam angle β [deg]")
    a2.set_ylim(1e-9, 1e-2)
    fig1.tight_layout()

    # (b) every ring
    p3 = LidarParams3D()
    s3 = scan3d(sc, pose, p3, noise=False)
    walls = len(w2.segments)  # surface ids < walls are extruded 2D segments
    fig2, ax = plt.subplots(figsize=(6.8, 3.8))
    eps_fine = np.linspace(-16, 16, 200)
    ax.plot(eps_fine, 1 / np.cos(np.radians(eps_fine)), color=viz.ORANGE, lw=1.8, label="1 / cos ε")
    worst = 0.0
    for k, e in enumerate(np.degrees(s3.elevations)):
        r = s3.true_ranges[k, idx]
        ok_k = (s3.surface_id[k, idx] < walls) & (s3.surface_id[k, idx] >= 0) & np.isfinite(s2.true_ranges) \
            & (s3.surface_id[k, idx] == s2.surface_id) & ~s2.is_ghost
        ratio = r[ok_k] / s2.true_ranges[ok_k]
        if len(ratio):
            worst = max(worst, float(np.max(np.abs(ratio - 1 / np.cos(np.radians(e))))))
            ax.scatter(np.full(len(ratio), e), ratio, s=8, color=viz.BLUE, alpha=0.4, linewidths=0,
                       label="simulated (direct wall hits)" if k == 0 else None)
    ax.set_xlabel("ring elevation ε [deg]")
    ax.set_ylabel("R_3D / R_2D")
    ax.set_title("E13b: tilted rings see the same wall 1/cos ε farther")
    ax.legend(fontsize=7.5)
    rows = [["max |R_3D(ε=0) − R_2D| [m]", float(np.max(diff[ok]))],
            ["beams compared", int(ok.sum())],
            ["same detection decision (P_D ≥ 0.5)", f"{same_detect:.1%}"],
            ["ghost beams in both", int(g.sum())],
            ["max |R_3D/R_2D − 1/cos ε| over all rings", worst]]
    return {"figs": {"slice": fig1, "ratio": fig2}, "tables": {"consistency": (["quantity", "value"], rows)},
            "data": {"diff": diff}}


def main():
    args = resolve(parser(__doc__).parse_args())
    finish(run(args.person, args.seed, args.fast), args.out, "e13")


if __name__ == "__main__":
    main()
