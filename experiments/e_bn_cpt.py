"""Bayesian-network CPT learned from the simulator by Monte Carlo (D7, lecture L4).

Discretise   material ∈ {matte, mirror, glass},  fog ∈ {clear, light, heavy},  range ∈ {near, mid, far}
and estimate two conditional probability tables by counting simulated shots at a single surface
(random distance inside the bin, random incidence angle 0–60°):
  P(D = T | material, fog, range)        — any return detected
  P(S = target | material, fog, range)   — the detected return comes from the target surface itself

Run: uv run python -m experiments.e_bn_cpt --person ammar|gal [--fast]
"""

import matplotlib.pyplot as plt
import numpy as np

from experiments.common import finish, load_person, parser, resolve
from lidar_sim import viz
from lidar_sim.materials import glass, matte, mirror
from lidar_sim.noise import sample_measurement
from lidar_sim.sensor import LidarParams, trace_beams
from lidar_sim.world import Segment, World

ONE = LidarParams(fov_deg=0.0, n_beams=1)
MATERIALS = {"matte": matte(0.7, "target"), "mirror": mirror(name="target"), "glass": glass(name="target")}
FOGS = {"clear": 0.0, "light": 0.1, "heavy": 0.3}
RANGES = {"near": (0.5, 2.0), "mid": (2.0, 4.0), "far": (4.0, 6.0)}


def _world(material, r, inc_deg, alpha):
    d = np.array([np.cos(np.radians(90 + inc_deg)), np.sin(np.radians(90 + inc_deg))])
    c = np.array([r, 0.0])
    target = Segment(c - 1.5 * d, c + 1.5 * d, material, "target")
    room = [Segment(a, b, matte(0.7, "room"), "room") for a, b in
            [((-15, -15), (15, -15)), ((15, -15), (15, 15)), ((15, 15), (-15, 15)), ((-15, 15), (-15, -15))]]
    return World([target] + room, [], alpha, "cpt")


def run(person="ammar", seed=None, fast=False):
    cfg = load_person(person)
    seed = cfg["seed"] if seed is None else seed
    rng = np.random.default_rng(seed)
    n = 120 if fast else 500
    shape = (len(MATERIALS), len(FOGS), len(RANGES))
    p_det, p_tgt = np.zeros(shape), np.zeros(shape)
    rows = []
    for i, (mname, mat) in enumerate(MATERIALS.items()):
        for j, (fname, alpha) in enumerate(FOGS.items()):
            for k, (rname, (r0, r1)) in enumerate(RANGES.items()):
                det_n = tgt_n = 0
                for _ in range(n):
                    w = _world(mat, rng.uniform(r0, r1), rng.uniform(0, 60), alpha)
                    tb = trace_beams(w, [0, 0, 0], ONE)
                    _, det = sample_measurement(tb["true_range"][0], tb["p_detect"][0], rng)
                    if det[0]:
                        det_n += 1
                        tgt_n += int(tb["surface_id"][0, 0] == 0)
                p_det[i, j, k], p_tgt[i, j, k] = det_n / n, tgt_n / n
                rows.append([mname, fname, rname, p_det[i, j, k], p_tgt[i, j, k], n])

    fig, axes = plt.subplots(1, 3, figsize=(10.5, 3.6), sharey=True)
    for i, (ax, mname) in enumerate(zip(axes, MATERIALS)):
        im = ax.imshow(p_det[i], cmap="Blues", vmin=0, vmax=1, origin="upper")
        for j in range(len(FOGS)):
            for k in range(len(RANGES)):
                v = p_det[i, j, k]
                ax.text(k, j, f"{v:.2f}", ha="center", va="center", fontsize=8.5,
                        color="white" if v > 0.6 else viz.INK)
        ax.set_xticks(range(len(RANGES)), list(RANGES))
        ax.set_yticks(range(len(FOGS)), list(FOGS))
        ax.set_title(f"material = {mname}", fontsize=10)
        ax.grid(False)
        ax.set_xlabel("range bin")
    axes[0].set_ylabel("fog")
    fig.colorbar(im, ax=axes, label="P(D = T | material, fog, range)", shrink=0.85)
    fig.suptitle(f"BN CPT estimated from {n} simulated shots per cell (incidence 0–60°)", fontsize=11)

    header = ["material", "fog", "range", "P(D=T)", "P(S=target)", "shots"]
    return {"figs": {"heatmap": fig}, "tables": {"table": (header, rows)},
            "data": {"p_det": p_det, "p_tgt": p_tgt}}


def main():
    args = resolve(parser(__doc__).parse_args())
    finish(run(args.person, args.seed, args.fast), args.out, "cpt")


if __name__ == "__main__":
    main()
