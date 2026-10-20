"""E4 — TARGET FEATURES (reflectivity, orientation) -> detection (D5, D6).

(a) Detection probability vs range for several reflectivities: Monte Carlo through the sensor
    model vs the analytic P_D = Phi((p - 1)/sigma_P), with the analytic max ranges marked.
(b) Per-beam detection rate over repeated scans near a dark wall vs the model's P_D.
(c) Max range vs incidence angle: simulated (bisection on the traced power) vs R0 sqrt(cos θ).

Run: uv run python -m experiments.e4_reflectivity [--person partner] [--fast]
"""

import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import brentq

from experiments.common import finish, parser, preset, resolve
from lidar_sim import viz
from lidar_sim.noise import detection_prob, sample_measurement
from lidar_sim.physics import diffuse_power, max_range
from lidar_sim.scenes import get_scene, single_wall
from lidar_sim.sensor import LidarParams, scan, trace_beams

ONE = LidarParams(fov_deg=0.0, n_beams=1)


def _power(r, rho, angle_deg=0.0):
    return trace_beams(single_wall(r, rho, angle_deg), [0, 0, 0], ONE)["power"][0, 0]


def run(scene="default", seed=0, fast=False):
    rng = np.random.default_rng(seed)
    rhos = [0.1, 0.2, 0.5, 0.9]
    ranges = np.linspace(0.3, 6.5, 22 if fast else 45)
    n_mc = 80 if fast else 300

    fig1, ax = plt.subplots(figsize=(7.2, 4.2))
    rows = []
    for k, rho in enumerate(rhos):
        col = viz.SERIES[k]
        fine = np.linspace(0.3, 6.5, 400)
        ax.plot(fine, detection_prob(diffuse_power(fine, rho, 1.0)), color=col, lw=1.6, label=f"ρ = {rho} (model)")
        rate = []
        for r in ranges:
            tb = trace_beams(single_wall(r, rho), [0, 0, 0], ONE)
            _, det = sample_measurement(np.repeat(tb["true_range"][0], n_mc), np.repeat(tb["p_detect"][0], n_mc), rng)
            rate.append(det.mean())
        ax.scatter(ranges, rate, s=14, color=col, zorder=4, linewidths=0)
        R = float(max_range(rho))
        ax.axvline(R, color=col, lw=0.8, ls=":")
        sim_R = brentq(lambda r, rho=rho: _power(r, rho) - 1.0, 0.05, 30)
        rows.append([rho, R, sim_R, abs(R - sim_R)])
    ax.set_xlabel("range to wall [m]")
    ax.set_ylabel("detection probability")
    ax.set_title(f"E4a: P_D vs range — dots: Monte Carlo ({n_mc} shots), lines: model; dotted: R_max(ρ)")
    ax.legend(fontsize=7.5, loc="upper right")

    # (b) repeated scans near a dark wall
    pr = preset(scene, "e4")
    world = get_scene(scene, with_moving_box=False)
    pose = np.array([pr["pose"][0], pr["pose"][1], np.radians(pr["pose"][2])])
    n_scans = 40 if fast else 150
    det = np.array([scan(world, pose, rng=rng).detected for _ in range(n_scans)])
    model = scan(world, pose, noise=False)
    beta = np.degrees(model.beam_angles)
    fig2, (a1, a2) = plt.subplots(1, 2, figsize=(10.5, 4.2), gridspec_kw={"width_ratios": [1, 1.25]})
    viz.draw_world(a1, world)
    viz.draw_fov(a1, pose, 240, 5.6)
    many = [scan(world, pose, rng=rng) for _ in range(12)]
    for s in many:
        a1.scatter(*s.points_world[s.detected].T, s=5, color=viz.BLUE, alpha=0.35, linewidths=0)
    viz.draw_sensor(a1, pose)
    a1.set_title("12 overlaid scans (dark section flickers)", fontsize=10)
    a2.plot(beta, model.p_detect, color=viz.ORANGE, lw=1.6, label="model P_D per beam")
    a2.scatter(beta, det.mean(axis=0), s=9, color=viz.BLUE, zorder=4, linewidths=0, label=f"measured rate ({n_scans} scans)")
    a2.set_xlabel("beam angle β [deg]")
    a2.set_ylabel("detection rate")
    a2.set_title("E4b: per-beam detection vs model", fontsize=10)
    a2.legend(fontsize=7.5, loc="lower left")
    fig2.tight_layout()
    rms_b = float(np.sqrt(np.mean((det.mean(axis=0) - model.p_detect) ** 2)))

    # (c) orientation: max range vs incidence angle
    angles = np.linspace(0, 80, 9 if fast else 17)
    rho = 0.5
    sim = [brentq(lambda r, a=a: _power(r, rho, a) - 1.0, 0.05, 30) for a in angles]
    fig3, ax3 = plt.subplots(figsize=(6.4, 3.8))
    fine = np.linspace(0, 85, 200)
    ax3.plot(fine, max_range(rho, np.radians(fine)), color=viz.BLUE, lw=1.6, label="R₀·√cos θ (D5)")
    ax3.scatter(angles, sim, s=18, color=viz.ORANGE, zorder=4, linewidths=0, label="simulated (traced power = threshold)")
    ax3.set_xlabel("incidence angle θ [deg]")
    ax3.set_ylabel("max range [m]")
    ax3.set_title(f"E4c: a tilted wall is seen less far (ρ = {rho})")
    ax3.legend(fontsize=7.5)
    err_c = float(np.max(np.abs(np.array(sim) - max_range(rho, np.radians(angles)))))

    rows.append(["per-beam detection RMS (model vs MC)", rms_b, "", ""])
    rows.append(["max |sim − analytic| over incidence angles [m]", err_c, "", ""])
    header = ["ρ", "analytic R_max [m]", "simulated R_max [m]", "|difference| [m]"]
    return {"figs": {"pd_vs_range": fig1, "dark_wall": fig2, "incidence": fig3},
            "tables": {"max_range": (header, rows)}, "data": {"det": det, "model": model}}


def main():
    args = resolve(parser(__doc__).parse_args())
    finish(run(args.scene, args.seed, args.fast), args.out, "e4")


if __name__ == "__main__":
    main()
