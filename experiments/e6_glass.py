"""E6 — GLASS: Fresnel reflection, see-through, and the head-on blip (D9).

A glass pane in front of a matte wall is rotated from head-on to grazing incidence.
(a) Fresnel R_F(θ) and the round-trip pane transmission T_pane² — theory curves vs the simulated
    wall-return power ratio (with / without the pane).
(b) Power of the glass return and of the wall return vs θ, with the detection threshold.
(c) Which surface is reported: glass only within a few degrees of head-on.
(d) In the room: points seen through the glass.

Run: uv run python -m experiments.e6_glass --person ammar|gal [--fast]
"""

import matplotlib.pyplot as plt
import numpy as np

from experiments.common import finish, load_person, parser, resolve, scene_of
from lidar_sim import config, viz
from lidar_sim.physics import fresnel_deg, pane_transmission
from lidar_sim.scenes import glass_test
from lidar_sim.sensor import LidarParams, scan, trace_beams

ONE = LidarParams(fov_deg=0.0, n_beams=1)


def run(person="ammar", seed=None, fast=False):
    cfg = load_person(person)
    seed = cfg["seed"] if seed is None else seed
    pr = cfg["e6"]
    gd, wd = pr["glass_dist"], pr["wall_dist"]
    angles = np.arange(float(pr["angle_offset_deg"]), 86, 5 if fast else 1).astype(float)
    p_glass, p_wall, ratio, rep_strong, rep_first = [], [], [], [], []
    p_bare = config.K_OVER_PTH * 0.7 / wd**2
    for a in angles:
        r = trace_beams(glass_test(a, gd, wd), [0, 0, 0], ONE)["returns"]
        g = (r.kind == 1) & (r.reflections == 0) & (r.interactions == 0)
        w = (r.kind == 0) & (r.interactions == 1) & (r.reflections == 0)
        p_glass.append(r.power[g].max() if g.any() else 0.0)
        p_wall.append(r.power[w].max() if w.any() else 0.0)
        ratio.append(p_wall[-1] / p_bare)
        rep_strong.append(scan(glass_test(a, gd, wd), [0, 0, 0], ONE, noise=False).ranges[0])
        rep_first.append(scan(glass_test(a, gd, wd), [0, 0, 0], ONE.with_(return_policy="first"), noise=False).ranges[0])
    p_glass, p_wall, ratio = map(np.array, (p_glass, p_wall, ratio))
    rep_strong, rep_first = np.array(rep_strong), np.array(rep_first)

    fine = np.linspace(0, 89.5, 400)
    RF = fresnel_deg(fine)
    fig1, ax = plt.subplots(figsize=(6.8, 4.0))
    ax.plot(fine, RF, color=viz.ORANGE, lw=1.8, label="R_F(θ) reflected (Fresnel, unpolarised)")
    ax.plot(fine, pane_transmission(RF) ** 2, color=viz.BLUE, lw=1.8, label="T_pane² round-trip transmission")
    ax.scatter(angles, ratio, s=14, color=viz.BLUE, zorder=4, linewidths=0, label="simulated wall power with / without pane")
    ax.axvline(56.31, color=viz.MUTED, lw=0.8, ls=":")
    ax.annotate("Brewster 56.3°", (56.31, 0.55), xytext=(4, 0), textcoords="offset points", fontsize=7.5, color=viz.INK2)
    ax.set_xlabel("incidence angle on the glass θ [deg]")
    ax.set_ylabel("fraction")
    ax.set_title("E6a: glass reflects little until grazing incidence")
    ax.legend(fontsize=7.5, loc="center left")

    fig2, ax2 = plt.subplots(figsize=(6.8, 4.0))
    ax2.semilogy(angles, np.maximum(p_glass, 1e-6), color=viz.AQUA, lw=1.8, marker="o", ms=3, label="glass (specular) return")
    ax2.semilogy(angles, np.maximum(p_wall, 1e-6), color=viz.BLUE, lw=1.8, marker="s", ms=3, label="wall behind glass")
    ax2.axhline(1.0, color=viz.INK, lw=1.0, ls="--", label="detection threshold (P_r = P_th)")
    ax2.set_ylim(1e-3, 1e4)
    ax2.set_xlabel("incidence angle θ [deg]")
    ax2.set_ylabel("received power / P_th")
    ax2.set_title("E6b: the glass return is above threshold only near head-on")
    ax2.legend(fontsize=7.5)

    fig3, ax3 = plt.subplots(figsize=(6.8, 3.6))
    ax3.step(angles, rep_strong, where="mid", color=viz.BLUE, lw=1.8, label="reported range (strongest return)")
    ax3.step(angles, rep_first, where="mid", color=viz.ORANGE, lw=1.4, ls="--", label="reported range (first return)")
    ax3.axhline(gd, color=viz.AQUA, lw=0.8, ls=":")
    ax3.axhline(wd, color=viz.MUTED, lw=0.8, ls=":")
    ax3.annotate("glass", (angles[-1], gd), xytext=(-30, 4), textcoords="offset points", fontsize=7.5, color=viz.INK2)
    ax3.annotate("wall", (angles[-1], wd), xytext=(-30, -12), textcoords="offset points", fontsize=7.5, color=viz.INK2)
    ax3.set_xlim(-1, 20)
    ax3.set_xlabel("incidence angle θ [deg] (zoomed to 0–20°)")
    ax3.set_ylabel("reported range [m]")
    ax3.set_title("E6c: the LiDAR sees the glass only within ~5° of head-on")
    ax3.legend(fontsize=7.5, loc="center right")

    world = scene_of(cfg, with_moving_box=False)
    pose = np.array([pr["room_pose"][0], pr["room_pose"][1], np.radians(pr["room_pose"][2])])
    s = scan(world, pose, rng=np.random.default_rng(seed))
    fig4, ax4 = plt.subplots(figsize=(6.4, 5))
    viz.draw_world(ax4, world)
    viz.draw_fov(ax4, pose, 240, 5.6)
    through = s.detected & s.transmitted & ~s.is_ghost
    normal = s.detected & ~s.transmitted & ~s.is_ghost
    ax4.scatter(*s.points_world[normal].T, s=9, color=viz.BLUE, linewidths=0, label="direct")
    ax4.scatter(*s.points_world[through].T, s=16, color=viz.ORANGE, marker="D", linewidths=0, label="seen through the glass")
    viz.draw_rays(ax4, pose, s.points_world[through], color=viz.ORANGE, alpha=0.25)
    viz.draw_sensor(ax4, pose)
    ax4.set_title(f"E6d: {int(through.sum())} points seen through the glass")
    viz.legend_below(ax4, ncol=2)

    sel = ratio > 0
    tp = pane_transmission(fresnel_deg(angles[sel])) ** 2
    lim_strong = angles[np.flatnonzero(np.isclose(rep_strong, gd, atol=0.01))]
    lim_first = angles[np.flatnonzero(np.isclose(rep_first, gd, atol=0.01))]
    rows = [["max |sim T_pane² − theory|", float(np.max(np.abs(ratio[sel] - tp)))],
            ["glass reported (strongest) up to [deg]", float(lim_strong.max()) if len(lim_strong) else np.nan],
            ["glass reported (first) up to [deg]", float(lim_first.max()) if len(lim_first) else np.nan],
            ["round-trip pane transmission head-on", float(tp[0])],
            ["points seen through glass in room scan", int(through.sum())]]
    return {"figs": {"fresnel": fig1, "powers": fig2, "reported": fig3, "room": fig4},
            "tables": {"glass": (["quantity", "value"], rows)},
            "data": {"angles": angles, "p_glass": p_glass, "p_wall": p_wall}}


def main():
    args = resolve(parser(__doc__).parse_args())
    finish(run(args.person, args.seed, args.fast), args.out, "e6")


if __name__ == "__main__":
    main()
