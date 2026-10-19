"""E8 — NOISE MODEL check by Monte Carlo (D1, D6).

(a) Histogram of repeated measurements of one beam on a white wall at 3 m vs N(3, sigma_R^2),
    with a Kolmogorov–Smirnov test.
(b) Estimated sigma_R vs range with 95% chi-square confidence intervals vs c sigma_t / 2.
(c) Bias (mean error) vs range with 95% intervals.

Run: uv run python -m experiments.e8_noise_mc [--person partner] [--fast]
"""

import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import chi2, kstest, norm

from experiments.common import finish, parser, resolve
from lidar_sim import viz
from lidar_sim.noise import sample_measurement
from lidar_sim.scenes import single_wall
from lidar_sim.sensor import LidarParams, trace_beams

ONE = LidarParams(fov_deg=0.0, n_beams=1)


def _measure(r, n, rng, rho=0.9):
    tb = trace_beams(single_wall(r, rho), [0, 0, 0], ONE)
    z, det = sample_measurement(np.repeat(tb["true_range"][0], n), np.repeat(tb["p_detect"][0], n), rng,
                                ONE.sigma_r, ONE.r_min)
    return z[det], tb["true_range"][0, 0]


def run(scene="default", seed=0, fast=False):
    rng = np.random.default_rng(seed + (0 if scene == "default" else 1000))
    sigma = ONE.sigma_r
    n = 300 if fast else 1000
    z, R = _measure(3.0, n, rng)
    ks = kstest(z, norm(R, sigma).cdf)

    fig1, ax = plt.subplots(figsize=(6.4, 3.8))
    ax.hist((z - R) * 100, bins=35, density=True, color=viz.BLUE, alpha=0.75, edgecolor="white", linewidth=0.5,
            label=f"{len(z)} measurements, wall at {R:.1f} m")
    x = np.linspace(-4, 4, 300)
    ax.plot(x, norm.pdf(x, 0, sigma * 100), color=viz.ORANGE, lw=1.8, label="model N(0, σ_R²), σ_R = cσ_t/2 = 1 cm")
    ax.set_xlabel("z − R [cm]")
    ax.set_ylabel("density")
    ax.set_title(f"E8a: range noise matches the model (KS p = {ks.pvalue:.2f})")
    ax.legend(fontsize=7.5)

    ranges = np.linspace(0.5, 5.0, 10)
    m = 200 if fast else 600
    s_hat, lo, hi, bias, bias_ci = [], [], [], [], []
    for r in ranges:
        zz, RR = _measure(r, m, rng)
        k = len(zz)
        s = zz.std(ddof=1)
        s_hat.append(s)
        lo.append(s * np.sqrt((k - 1) / chi2.ppf(0.975, k - 1)))
        hi.append(s * np.sqrt((k - 1) / chi2.ppf(0.025, k - 1)))
        bias.append(zz.mean() - RR)
        bias_ci.append(1.96 * s / np.sqrt(k))
    s_hat, lo, hi, bias, bias_ci = map(np.array, (s_hat, lo, hi, bias, bias_ci))

    fig2, (a1, a2) = plt.subplots(1, 2, figsize=(10, 3.8))
    a1.fill_between(ranges, lo * 100, hi * 100, color=viz.BLUE, alpha=0.2, linewidth=0, label="95% CI")
    a1.plot(ranges, s_hat * 100, color=viz.BLUE, lw=1.6, marker="o", ms=4, label="estimated σ_R")
    a1.axhline(sigma * 100, color=viz.ORANGE, lw=1.6, ls="--", label="model σ_R = 1 cm")
    a1.set_xlabel("range [m]")
    a1.set_ylabel("σ_R [cm]")
    a1.set_ylim(0.8, 1.2)
    a1.set_title("E8b: noise level is the same at every range", fontsize=10)
    a1.legend(fontsize=7.5)
    a2.errorbar(ranges, bias * 1000, yerr=bias_ci * 1000, fmt="o", ms=4, color=viz.BLUE, capsize=2, lw=1.2,
                label="mean error ± 95% CI")
    a2.axhline(0, color=viz.ORANGE, lw=1.6, ls="--", label="unbiased")
    a2.set_xlabel("range [m]")
    a2.set_ylabel("bias [mm]")
    a2.set_title("E8c: no systematic bias", fontsize=10)
    a2.legend(fontsize=7.5)
    fig2.tight_layout()

    inside = np.mean((lo <= sigma) & (sigma <= hi))
    rows = [["samples at 3 m", len(z)], ["KS statistic", float(ks.statistic)], ["KS p-value", float(ks.pvalue)],
            ["mean estimated σ_R [cm]", float(s_hat.mean() * 100)],
            ["ranges whose 95% CI contains 1 cm", f"{inside:.0%}"],
            ["max |bias| [mm]", float(np.max(np.abs(bias)) * 1000)]]
    return {"figs": {"histogram": fig1, "sigma_bias": fig2}, "tables": {"noise": (["quantity", "value"], rows)},
            "data": {"z": z, "ranges": ranges, "s_hat": s_hat}}


def main():
    args = resolve(parser(__doc__).parse_args())
    finish(run(args.scene, args.seed, args.fast), args.out, "e8")


if __name__ == "__main__":
    main()
