"""E9 (stretch) — continuous-wave phase ranging and its ambiguity (D2).

The Hokuyo measures range from the phase shift of an amplitude-modulated laser. One frequency
rolls over every R_amb = c/(2f) (3.22 m at 46.55 MHz); two frequencies (46.55 and 53.2 MHz) unwrap it
up to c/(2|f1 - f2|) = 22.5 m. Phase noise is set so the f1 range noise is sigma_R = 1 cm.

Run: uv run python -m experiments.e9_cw_phase --person ammar|gal [--fast]
"""

import matplotlib.pyplot as plt
import numpy as np

from experiments.common import finish, load_person, parser, resolve
from lidar_sim import config, viz
from lidar_sim.phase import F1, F2, ambiguity_range, phase_of, phase_range, two_frequency_range


def run(person="ammar", seed=None, fast=False):
    cfg = load_person(person)
    seed = cfg["seed"] if seed is None else seed
    rng = np.random.default_rng(seed)
    R = np.linspace(0.05, float(cfg["e9"]["range_span"]), 600 if fast else 2500)
    sigma_phi = 4 * np.pi * F1 * config.SIGMA_R / config.C  # phase noise giving 1 cm at f1
    phi1 = np.mod(phase_of(R, F1) + rng.normal(0, sigma_phi, R.shape), 2 * np.pi)
    phi2 = np.mod(phase_of(R, F2) + rng.normal(0, sigma_phi * F2 / F1, R.shape), 2 * np.pi)
    single1, single2 = phase_range(phi1, F1), phase_range(phi2, F2)
    both = two_frequency_range(phi1, phi2)
    R_beat = config.C / (2 * abs(F1 - F2))
    ok = R < R_beat  # noqa: SIM300 (R is the range array, not a constant)

    fig, (a1, a2) = plt.subplots(2, 1, figsize=(7.4, 5.6), sharex=True, gridspec_kw={"height_ratios": [2, 1]})
    a1.plot(R, R, color=viz.MUTED, lw=1.0, ls="--", label="true range")
    a1.plot(R, single1, color=viz.ORANGE, lw=1.2, label=f"f₁ = {F1 / 1e6:.2f} MHz alone (wraps every {ambiguity_range(F1):.2f} m)")
    a1.plot(R, single2, color=viz.AQUA, lw=1.0, alpha=0.8, label=f"f₂ = {F2 / 1e6:.1f} MHz alone (wraps every {ambiguity_range(F2):.2f} m)")
    a1.plot(R, both, color=viz.BLUE, lw=1.8, label="two-frequency unwrap")
    a1.axvline(R_beat, color=viz.INK2, lw=0.8, ls=":")
    a1.annotate(f"beat ambiguity {R_beat:.1f} m", (R_beat, 2), xytext=(-110, 0), textcoords="offset points",
                fontsize=7.5, color=viz.INK2)
    a1.set_ylabel("measured range [m]")
    a1.set_title("E9: phase ranging rolls over; two frequencies fix it")
    a1.legend(fontsize=7.5, loc="upper left")
    a2.plot(R[ok], (both[ok] - R[ok]) * 100, color=viz.BLUE, lw=0.8)
    a2.set_ylabel("error [cm]")
    a2.set_xlabel("true range R [m]")
    a2.set_ylim(-6, 6)
    fig.tight_layout()

    err = both[ok] - R[ok]
    fail = np.abs(err) > 0.5
    where = R[ok][fail]
    rows = [["R_amb at f₁ [m]", float(ambiguity_range(F1))], ["R_amb at f₂ [m]", float(ambiguity_range(F2))],
            ["beat ambiguity c/2|f₁−f₂| [m]", float(R_beat)], ["phase noise σ_φ [rad]", float(sigma_phi)],
            ["two-frequency RMS error, successful unwraps [cm]", float(np.sqrt(np.mean(err[~fail] ** 2)) * 100)],
            ["unwrap failures (|error| > 0.5 m)", f"{fail.mean():.2%}"],
            ["failures occur at true ranges [m]",
             ", ".join(f"{r:.2f}" for r in where[:8]) + (" …" if len(where) > 8 else "") if len(where) else "none"]]
    return {"figs": {"phase": fig}, "tables": {"cw": (["quantity", "value"], rows)}, "data": {"R": R, "both": both}}


def main():
    args = resolve(parser(__doc__).parse_args())
    finish(run(args.person, args.seed, args.fast), args.out, "e9")


if __name__ == "__main__":
    main()
