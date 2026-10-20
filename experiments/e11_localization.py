"""E11 — LOCALIZATION: the sensor model used for estimation (D13). Upgrade U1.

A robot drives a loop past mirrors and glass. Three particle filters receive *identical* scans and
odometry and differ only in the measurement model they believe:
  N1  naive model (mirror and glass treated as opaque matte walls), no robust mixture
  N2  naive model + robust mixture (the standard robotics fix)
  P   physics-aware model (our simulator: ghosts and see-through glass predicted) + mixture
Repeated over many seeds; we report position/heading error, divergence rate and N_eff.

Scenarios: the glass corridor (parallel walls, so along-corridor information mostly comes through
the glass) and the person's own room.

Run: uv run python -m experiments.e11_localization --person ammar|gal [--fast]
"""

import matplotlib.pyplot as plt
import numpy as np
from tqdm import tqdm

from experiments.common import finish, load_person, parser, resolve
from lidar_sim import viz
from lidar_sim.geometry import wrap_angle
from lidar_sim.localization.likelihood import make_model
from lidar_sim.localization.particle_filter import ParticleFilter, init_particles
from lidar_sim.paths import pure_pursuit
from lidar_sim.scenes import default_room, glass_corridor, variant_b
from lidar_sim.sensor import LidarParams, expected_scan, scan

SCENARIOS = {
    "glass_corridor": dict(build=lambda: glass_corridor(),
                           waypoints=[(1.2, 1.0), (12.8, 1.0), (13.3, 1.3), (12.8, 1.6), (1.2, 1.6)]),
    "default": dict(build=lambda: default_room(with_moving_box=False),
                    waypoints=[(1.2, 1.5), (3.6, 3.0), (5.8, 3.0), (6.6, 2.2), (5.0, 3.6), (2.0, 3.6), (1.2, 2.0)]),
    "variant_b": dict(build=lambda: variant_b(with_moving_box=False),
                      waypoints=[(2.5, 1.2), (4.5, 2.5), (4.0, 4.6), (2.4, 4.8), (2.2, 2.8), (2.5, 1.4)]),
}
FILTERS = {"N1 naive": ("naive", False), "N2 naive + mixture": ("naive", True), "P physics-aware": ("physics", True)}
COLORS = {"N1 naive": viz.ORANGE, "N2 naive + mixture": viz.YELLOW, "P physics-aware": viz.BLUE}

DT = 0.2
BEAMS = np.arange(0, 241, 8)  # 31 beams
SIGMA_LIK = 0.05  # inflated likelihood sigma (DECISIONS #7)
ODO_NOISE = (0.04, 0.04)  # odometry noise on (v, w)


def simulate(world, waypoints, seed, M=300, record_steps=()):
    rng = np.random.default_rng(seed)
    params = LidarParams()
    _, poses, u = pure_pursuit(waypoints, v=0.5, dt=DT, w_max=1.2)
    K = len(u)
    z = np.array([scan(world, poses[k + 1], params, rng=rng).ranges[BEAMS] for k in range(K)])
    u_meas = u + rng.normal(0.0, ODO_NOISE, u.shape)
    naive_world = make_model(world, "naive")
    R_naive, PD_naive = expected_scan(naive_world, poses[1:], params, BEAMS)
    # beams where the naive model's prediction at the TRUE pose is wrong (wrong range, or a return
    # predicted where the sensor got none, or none predicted where it got one)
    fin = np.isfinite(z)
    wrong_range = fin & ~(np.abs(z - np.nan_to_num(R_naive, nan=1e6)) < 3 * SIGMA_LIK)
    missed = ~fin & (PD_naive > 0.5)
    out = {"poses": poses, "naive_outlier_frac": float((wrong_range | missed).mean())}
    for i, (name, (kind, mix)) in enumerate(FILTERS.items()):
        frng = np.random.default_rng(seed * 101 + i)
        pf = ParticleFilter(make_model(world, kind), init_particles(poses[0], (0.2, 0.2, np.radians(5)), M, frng),
                            frng, params, BEAMS, lik=dict(sigma=SIGMA_LIK, eps=0.05, use_mixture=mix))
        est, snaps = [], {}
        for k in range(K):
            pf.predict(u_meas[k], DT)
            pf.update(z[k])
            est.append(pf.estimate())
            if k in record_steps:
                snaps[k] = pf.particles.copy()
        est = np.array(est)
        out[name] = dict(est=est, pos_err=np.linalg.norm(est[:, :2] - poses[1:, :2], axis=1),
                         head_err=np.abs(wrap_angle(est[:, 2] - poses[1:, 2])), neff=np.array(pf.n_eff_history),
                         snaps=snaps)
    return out


def run_scenario(name, seeds, M=300, progress=True):
    sc = SCENARIOS[name]
    world = sc["build"]()
    runs = []
    it = tqdm(seeds, desc=f"E11 {name}", leave=False) if progress else seeds
    for s in it:
        K = len(pure_pursuit(sc["waypoints"], v=0.5, dt=DT, w_max=1.2)[2])
        rec = (0, K // 3, (2 * K) // 3, K - 1) if s == seeds[0] else ()
        runs.append(simulate(world, sc["waypoints"], s, M, rec))
    return world, runs


def summarise(name, world, runs):
    t = np.arange(1, len(runs[0]["poses"])) * DT
    figs, rows = {}, []
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(7.4, 5.4), sharex=True)
    for f in FILTERS:
        P = np.array([r[f]["pos_err"] for r in runs])
        H = np.degrees(np.array([r[f]["head_err"] for r in runs]))
        c = COLORS[f]
        a1.fill_between(t, np.percentile(P, 25, 0), np.percentile(P, 75, 0), color=c, alpha=0.18, linewidth=0)
        a1.plot(t, np.median(P, 0), color=c, lw=1.8, label=f)
        a2.fill_between(t, np.percentile(H, 25, 0), np.percentile(H, 75, 0), color=c, alpha=0.18, linewidth=0)
        a2.plot(t, np.median(H, 0), color=c, lw=1.8)
        final = P[:, -1]
        rows.append([f, float(np.mean(np.sqrt(np.mean(P**2, axis=1)))), float(np.median(final)),
                     f"{np.mean(final > 0.5):.0%}", float(np.mean(np.sqrt(np.mean(H**2, axis=1)))),
                     float(np.mean([r[f]["neff"].mean() for r in runs]))])
    a1.set_yscale("log")
    a1.set_ylabel("position error [m]")
    a1.set_title(f"E11 ({name}): median and interquartile band over {len(runs)} runs", fontsize=10)
    a1.legend(fontsize=7.5, loc="lower left", ncol=3, frameon=True, facecolor="white", edgecolor="none",
              framealpha=0.9)
    a2.set_yscale("log")
    a2.set_ylabel("heading error [deg]")
    a2.set_xlabel("time [s]")
    fig.tight_layout()
    figs[f"{name}_error"] = fig

    fig2, ax = plt.subplots(figsize=(7.0, 3.4))
    for f in FILTERS:
        ax.plot(t, np.mean([r[f]["neff"] for r in runs], 0), color=COLORS[f], lw=1.5, label=f)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("N_eff before resampling")
    ax.set_title(f"E11 ({name}): effective sample size", fontsize=10)
    ax.legend(fontsize=7.5)
    figs[f"{name}_neff"] = fig2

    r0 = runs[0]
    steps = sorted(r0["P physics-aware"]["snaps"])
    fig3, axes = plt.subplots(len(FILTERS), len(steps), figsize=(3.1 * len(steps), 2.6 * len(FILTERS)))
    for i, f in enumerate(FILTERS):
        for j, k in enumerate(steps):
            ax = axes[i, j]
            viz.draw_world(ax, world, labels=False, lw=1.4)
            P = r0[f]["snaps"][k]
            ax.scatter(P[:, 0], P[:, 1], s=2, color=COLORS[f], alpha=0.5, linewidths=0)
            ax.plot(*r0["poses"][: k + 2, :2].T, color=viz.INK, lw=0.8)
            viz.draw_sensor(ax, r0["poses"][k + 1], size=0.22)
            ax.set_xlabel("")
            ax.set_ylabel("")
            ax.tick_params(labelsize=6)
            if i == 0:
                ax.set_title(f"t = {(k + 1) * DT:.1f} s", fontsize=9)
            if j == 0:
                ax.set_ylabel(f, fontsize=8.5)
    fig3.suptitle(f"E11 ({name}): particle clouds (seed 0); black = true path", fontsize=11)
    fig3.tight_layout()
    figs[f"{name}_snapshots"] = fig3
    rows.append(["beams the naive model mispredicts at the true pose", f"{np.mean([r['naive_outlier_frac'] for r in runs]):.0%}",
                 "", "", "", ""])
    return figs, rows


def run(person="ammar", seed=None, fast=False, animate_to=None):
    cfg = load_person(person)
    seed = cfg["seed"] if seed is None else seed
    scene = cfg["scene"]
    n_seeds = 3 if fast else cfg["e11"]["n_seeds"]
    seeds = list(range(seed, seed + n_seeds))
    M = 150 if fast else 300
    figs, tables, data = {}, {}, {}
    header = ["filter", "mean RMSE position [m]", "median final error [m]", "diverged (final > 0.5 m)",
              "mean RMSE heading [deg]", "mean N_eff"]
    for name in ("glass_corridor", scene if scene in SCENARIOS else "default"):
        world, runs = run_scenario(name, seeds, M, progress=not fast)
        f, rows = summarise(name, world, runs)
        figs.update(f)
        tables[f"{name}_summary"] = (header, rows)
        data[name] = runs
    media = []
    if animate_to:
        media.append(_animate("glass_corridor", seed, M, animate_to))
    return {"figs": figs, "tables": tables, "data": data, "media": media}


def _animate(name, seed, M, path):
    sc = SCENARIOS[name]
    world = sc["build"]()
    K = len(pure_pursuit(sc["waypoints"], v=0.5, dt=DT, w_max=1.2)[2])
    r = simulate(world, sc["waypoints"], seed, M, record_steps=tuple(range(0, K, 2)))
    steps = sorted(r["P physics-aware"]["snaps"])
    fig, axes = plt.subplots(1, 3, figsize=(13, 3.6))

    def update(j):
        k = steps[j]
        for ax, f in zip(axes, FILTERS):
            ax.clear()
            viz.draw_world(ax, world, labels=False, lw=1.4)
            P = r[f]["snaps"][k]
            ax.scatter(P[:, 0], P[:, 1], s=2, color=COLORS[f], alpha=0.5, linewidths=0)
            ax.plot(*r["poses"][: k + 2, :2].T, color=viz.INK, lw=0.8)
            viz.draw_sensor(ax, r["poses"][k + 1], size=0.22)
            ax.set_title(f"{f}   error {r[f]['pos_err'][k]:.2f} m", fontsize=9)

    fig.tight_layout()
    return viz.animate(fig, update, len(steps), path, fps=10)


def main():
    args = resolve(parser(__doc__).parse_args())
    anim = None if args.no_anim else f"{args.out}/e11_particles.gif"
    finish(run(args.person, args.seed, args.fast, animate_to=anim), args.out, "e11")


if __name__ == "__main__":
    main()
