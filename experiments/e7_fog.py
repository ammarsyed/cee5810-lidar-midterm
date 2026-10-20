"""E7 — OPERATING CONDITIONS (fog) -> measurements (D5, D10, D11).

(a) The same scan at four fog densities: the point cloud thins from the far end.
(b) Effective max range vs extinction coefficient α: simulated (bisection on the traced power) vs
    the closed form R_max = W0(α R0)/α for three reflectivities.
(c) Fraction of beams detected in the room vs α (Monte Carlo mean ± std).

Run: uv run python -m experiments.e7_fog --person ammar|gal [--fast]
"""

import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import brentq

from experiments.common import finish, load_person, parser, resolve, scene_of
from lidar_sim import viz
from lidar_sim.physics import max_range
from lidar_sim.scenes import single_wall
from lidar_sim.sensor import LidarParams, scan, trace_beams

ONE = LidarParams(fov_deg=0.0, n_beams=1)


def _sim_max_range(rho, alpha):
    f = lambda r: trace_beams(single_wall(r, rho, fog_alpha=alpha), [0, 0, 0], ONE)["power"][0, 0] - 1.0  # noqa: E731
    return brentq(f, 0.05, 30)


def run(person="ammar", seed=None, fast=False):
    cfg = load_person(person)
    seed = cfg["seed"] if seed is None else seed
    pr = cfg["e7"]
    rng = np.random.default_rng(seed)
    base = scene_of(cfg, with_moving_box=False)
    pose = np.array([pr["pose"][0], pr["pose"][1], np.radians(pr["pose"][2])])

    fig1, axes = plt.subplots(2, 2, figsize=(9.5, 7.6))
    for ax, a in zip(axes.flat, (0.0, 0.1, 0.2, 0.4)):
        w = base.with_fog(a)
        s = scan(w, pose, rng=rng)
        viz.draw_world(ax, w)
        viz.draw_fov(ax, pose, 240, float(max_range(0.9, alpha=a)))
        viz.draw_scan(ax, s, show_dropouts=True, show_true=False, r_drop=0.6)
        ax.set_title(f"α = {a} m⁻¹ (visibility ≈ {3.912 / a:.0f} m)   detected {s.detected.sum()}/241" if a else
                     f"clear air   detected {s.detected.sum()}/241", fontsize=10)
    fig1.suptitle("E7a: fog shrinks the dashed max-range circle (white wall) and thins the cloud", fontsize=11)
    fig1.tight_layout()

    alphas = np.linspace(0, 0.5, 11 if fast else 26)
    fig2, ax2 = plt.subplots(figsize=(6.8, 4.0))
    rows = []
    fine = np.linspace(0, 0.5, 200)
    for k, rho in enumerate(float(r) for r in pr["rhos"]):
        ax2.plot(fine, max_range(rho, alpha=fine), color=viz.SERIES[k], lw=1.8, label=f"ρ = {rho}: W₀(αR₀)/α")
        sim = np.array([_sim_max_range(rho, a) for a in alphas])
        ax2.scatter(alphas, sim, s=16, color=viz.SERIES[k], zorder=4, linewidths=0)
        rows.append([rho, float(np.max(np.abs(sim - max_range(rho, alpha=alphas))))])
    ax2.set_xlabel("fog extinction coefficient α [1/m]")
    ax2.set_ylabel("effective max range [m]")
    ax2.set_title("E7b: max range vs fog — dots simulated, lines closed form")
    ax2.legend(fontsize=7.5)

    n_mc = 8 if fast else 30
    alphas_c = np.linspace(0, 0.5, 11)
    mean, std = [], []
    for a in alphas_c:
        w = base.with_fog(a)
        frac = [scan(w, pose, rng=rng).detected.mean() for _ in range(n_mc)]
        mean.append(np.mean(frac))
        std.append(np.std(frac))
    mean, std = np.array(mean), np.array(std)
    fig3, ax3 = plt.subplots(figsize=(6.4, 3.6))
    ax3.fill_between(alphas_c, mean - std, mean + std, color=viz.BLUE, alpha=0.2, linewidth=0)
    ax3.plot(alphas_c, mean, color=viz.BLUE, lw=1.8, marker="o", ms=4, label=f"mean ± std over {n_mc} scans")
    ax3.set_xlabel("fog extinction coefficient α [1/m]")
    ax3.set_ylabel("fraction of beams detected")
    ax3.set_ylim(0, 1)
    ax3.set_title("E7c: fewer returns as the fog thickens")
    ax3.legend(fontsize=7.5)
    header = ["ρ", "max |sim − W₀(αR₀)/α| over α [m]"]
    rows.append(["room detection fraction, clear → α=0.5", f"{mean[0]:.2f} → {mean[-1]:.2f}"])
    return {"figs": {"clouds": fig1, "max_range": fig2, "detected_fraction": fig3},
            "tables": {"fog": (header, rows)}, "data": {"alphas": alphas_c, "mean": mean}}


def main():
    args = resolve(parser(__doc__).parse_args())
    finish(run(args.person, args.seed, args.fast), args.out, "e7")


if __name__ == "__main__":
    main()
