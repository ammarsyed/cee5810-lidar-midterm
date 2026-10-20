"""E10 (stretch) — beam divergence and mixed pixels at object edges (D12).

The beam is a cone (Heiden: 4 cm wide at 4 m, half-angle 5 mrad). Each beam is sampled by 3 sub-rays.
When the sub-rays split between a near object and a far wall, the reported range lands BETWEEN them:
power-weighted linear average vs the exact continuous-wave phasor sum.

Run: uv run python -m experiments.e10_divergence --person ammar|gal [--fast]
"""

import matplotlib.pyplot as plt
import numpy as np

from experiments.common import finish, load_person, parser, resolve
from lidar_sim import viz
from lidar_sim.materials import matte
from lidar_sim.phase import mixed_range
from lidar_sim.raytrace import select, trace
from lidar_sim.world import Segment, World, box_segments

HALF_ANGLE = 0.005  # rad (2 cm radius at 4 m)


def edge_scene(near=2.0, far=3.0):
    """A box whose left edge sits on the beam axis at `near`, in front of a wall at `far`."""
    segs = box_segments((near + 0.3, -0.4), (0.6, 0.8), 0.0, matte(0.7, "box"), "box")
    segs.append(Segment((far, -5), (far, 5), matte(0.7, "wall"), "wall"))
    return World(segs, [], 0.0, "edge")


def sub_ray_returns(world, angles, n_sub=3, half=HALF_ANGLE):
    """Chosen-return range and power for every (beam angle, sub-ray). Shapes (n_beams, n_sub)."""
    offs = np.linspace(-half, half, n_sub)
    a = (angles[:, None] + offs[None, :]).ravel()
    dirs = np.stack([np.cos(a), np.sin(a)], 1)
    ret = trace(world.backend(), np.zeros_like(dirs), dirs)
    ch = select(ret, len(a))
    has = ch >= 0
    R = np.where(has, ret.path_length[np.where(has, ch, 0)], np.nan).reshape(len(angles), n_sub)
    P = np.where(has, ret.power[np.where(has, ch, 0)], 0.0).reshape(len(angles), n_sub)
    return R, P


def run(person="ammar", seed=None, fast=False):
    cfg = load_person(person)
    seed = cfg["seed"] if seed is None else seed
    near, far = cfg["e10"]["near"], cfg["e10"]["far"]
    world = edge_scene(near, far)
    edge_angle = np.arctan2(0.0, near)  # box edge at y = 0
    angles = edge_angle + np.radians(np.linspace(-0.8, 0.8, 161 if fast else 641))
    R, P = sub_ray_returns(world, angles)
    single = sub_ray_returns(world, angles, n_sub=1, half=0.0)[0][:, 0]
    lin = mixed_range(R, P, "linear")
    pha = mixed_range(R, P, "phasor")

    fig1, ax = plt.subplots(figsize=(7.2, 4.0))
    deg = np.degrees(angles)
    ax.plot(deg, single, color=viz.MUTED, lw=1.6, ls="--", label="single thin ray")
    ax.plot(deg, lin, color=viz.BLUE, lw=1.8, label="3 sub-rays, linear (power-weighted)")
    ax.plot(deg, pha, color=viz.ORANGE, lw=1.6, label="3 sub-rays, phasor (exact CW)")
    ax.axhline(near, color=viz.AQUA, lw=0.8, ls=":")
    ax.axhline(far, color=viz.AQUA, lw=0.8, ls=":")
    ax.set_xlabel("beam angle [deg] (box edge at 0°)")
    ax.set_ylabel("reported range [m]")
    ax.set_title("E10: at an edge the divergent beam reports ranges between box and wall")
    ax.legend(fontsize=7.5)

    fig2, ax2 = plt.subplots(figsize=(6.4, 4.4))
    viz.draw_world(ax2, world, labels=True)
    pts_lin = np.stack([lin * np.cos(angles), lin * np.sin(angles)], 1)
    mixed = np.isfinite(lin) & (lin > near + 0.02) & (lin < far - 0.02)
    ax2.scatter(*pts_lin[~mixed].T, s=6, color=viz.BLUE, linewidths=0, label="normal points")
    ax2.scatter(*pts_lin[mixed].T, s=14, color=viz.ORANGE, linewidths=0, label="mixed pixels (spurious)")
    ax2.set_xlim(-0.2, far + 0.5)
    ax2.set_ylim(-0.6, 0.6)
    ax2.set_title("E10: spurious points float between the box edge and the wall (Heiden Fig. 3)", fontsize=10)
    viz.legend_below(ax2, ncol=2)

    gap = far - near
    rows = [["beam half-angle [mrad]", HALF_ANGLE * 1000], ["near / far [m]", f"{near} / {far}"],
            ["beams with mixed pixels (linear)", int(mixed.sum())],
            ["max |linear − phasor| [m]", float(np.nanmax(np.abs(lin - pha)))],
            ["D12 example (1 m, 2 m; w 0.7/0.3): linear vs phasor [m]",
             f"{float(mixed_range([1.0, 2.0], [0.7, 0.3], 'linear')):.2f} vs {float(mixed_range([1.0, 2.0], [0.7, 0.3], 'phasor')):.2f}"],
            ["gap between surfaces [m]", gap]]
    return {"figs": {"edge_profile": fig1, "mixed_points": fig2}, "tables": {"divergence": (["quantity", "value"], rows)},
            "data": {"angles": angles, "linear": lin, "phasor": pha}}


def main():
    args = resolve(parser(__doc__).parse_args())
    finish(run(args.person, args.seed, args.fast), args.out, "e10")


if __name__ == "__main__":
    main()
