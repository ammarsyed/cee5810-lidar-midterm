"""E2 — sensor HEADING -> FOV -> measurements (D4).

The sensor rotates in place. The FOV membership test x_T in S(x) (range window from the target's
own max range, 240 deg angular window, line of sight) predicts when the box should be seen; the
simulation reports whether any beam actually returns from the box.

Run: uv run python -m experiments.e2_sensor_heading --person ammar|gal [--fast]
"""

import matplotlib.pyplot as plt
import numpy as np

from experiments.common import finish, load_person, parser, resolve, scene_of
from lidar_sim import viz
from lidar_sim.physics import max_range
from lidar_sim.sensor import LidarParams, in_fov, scan, visibility_polygon


def run(person="ammar", seed=None, fast=False):
    cfg = load_person(person)
    seed = cfg["seed"] if seed is None else seed
    pr = cfg["e2"]
    world = scene_of(cfg).at_time(0.0)  # box frozen at its t = 0 position
    params = LidarParams()
    box_ids = world.surface_ids("box")
    box = scene_of(cfg).movers[0]
    cx, cy, _ = box.trajectory(0.0)
    r_box = float(max_range(box.material.rho))  # the box's own max range (target features!)
    xy = np.array(pr["pose"], float)
    step = 6 if fast else 2
    thetas = np.arange(0, 360, step)
    # target ∩ S(x) ≠ ∅: sample points on the box outline (its centre is hidden by its own faces)
    outline = np.array([seg.a + f * (seg.b - seg.a) for seg in box.segments_at(0.0) for f in np.linspace(0, 1, 9)])
    predicted, n_hit = [], []
    for th in thetas:
        pose = np.array([*xy, np.radians(th)])
        predicted.append(bool(in_fov(outline, pose, params, world=world, r_max=r_box).any()))
        s = scan(world, pose, params, noise=False)
        n_hit.append(int((s.detected & np.isin(s.surface_id, box_ids)).sum()))
    predicted, n_hit = np.array(predicted), np.array(n_hit)
    seen = n_hit > 0
    agree = float(np.mean(predicted == seen))

    fig1, axes = plt.subplots(2, 2, figsize=(9.5, 7.6))
    for ax, th in zip(axes.flat, (0, 90, 180, 270)):
        pose = np.array([*xy, np.radians(th)])
        s = scan(world, pose, params, rng=np.random.default_rng(seed))
        viz.draw_visibility(ax, visibility_polygon(pose, world, params))
        viz.draw_fov(ax, pose, params.fov_deg, params.r_max_ref)
        viz.draw_world(ax, world)
        viz.draw_scan(ax, s, show_dropouts=False, show_true=False)
        inside = in_fov(outline, pose, params, world=world, r_max=r_box).any()
        ax.plot(cx, cy, marker="*", ms=12, color=viz.INK, zorder=9)
        ax.set_title(f"heading θ = {th}°   box in S(x): {'yes' if inside else 'no'}", fontsize=10)
    fig1.suptitle("E2: the FOV wedge and visible region rotate with the sensor", fontsize=11)
    fig1.tight_layout()

    fig2, (a1, a2) = plt.subplots(2, 1, figsize=(7.2, 4.6), sharex=True, gridspec_kw={"height_ratios": [1, 1.4]})
    a1.fill_between(thetas, 0, predicted.astype(float), step="mid", color=viz.BLUE, alpha=0.25,
                    label="predicted: box outline ∩ S(x) ≠ ∅")
    a1.step(thetas, seen.astype(float) * 0.85, where="mid", color=viz.ORANGE, lw=1.6,
            label="simulated: ≥1 beam returns from box")
    a1.set_yticks([0, 1], ["no", "yes"])
    a1.set_title(f"E2: FOV prediction vs simulation (agreement {agree:.0%})")
    a1.legend(loc="upper right", fontsize=7.5)
    a2.plot(thetas, n_hit, color=viz.ORANGE, lw=1.6)
    a2.set_ylabel("# beams on box")
    a2.set_xlabel("sensor heading θ [deg]")
    fig2.tight_layout()

    edges = np.flatnonzero(np.diff(predicted.astype(int)) != 0)
    rows = [["agreement (fraction of headings)", f"{agree:.3f}"],
            ["box max range from its ρ (D5) [m]", f"{r_box:.2f}"],
            ["distance sensor → box centre [m]", f"{np.hypot(cx - xy[0], cy - xy[1]):.2f}"],
            ["predicted FOV entry/exit headings [deg]", ", ".join(str(int(thetas[e])) for e in edges)]]
    return {"figs": {"snapshots": fig1, "fov_vs_heading": fig2}, "tables": {"summary": (["quantity", "value"], rows)},
            "data": {"thetas": thetas, "predicted": predicted, "n_hit": n_hit}}


def main():
    args = resolve(parser(__doc__).parse_args())
    finish(run(args.person, args.seed, args.fast), args.out, "e2")


if __name__ == "__main__":
    main()
