"""E12 — 3D LiDAR: what a planar LiDAR cannot see (D14). Upgrade U2.

(a) 2D scan plane (the ε = 0 ring) vs the 16-ring 3D scan in the same 3D room: the low box and
    the table top are invisible to the plane but seen by the rings.
(b) Floor ring vs mount tilt: simulated floor-hit distance vs the exact formula for ring −15°.
(c) Range image of the room scan (rings × azimuth).
(d) Mirror ghosts in 3D: the whole virtual room behind the mirror.
(e) Fog in 3D: fraction of rays detected vs α.

Run: uv run python -m experiments.e12_3d_fov --person ammar|gal [--fast]
"""

import matplotlib.pyplot as plt
import numpy as np

from experiments.common import finish, load_person, parser, resolve
from lidar_sim import viz, viz3d
from lidar_sim.geometry import rot3d
from lidar_sim.materials import matte
from lidar_sim.scenes3d import Scene3D, get_scene3d
from lidar_sim.sensor3d import LidarParams3D, scan3d


def floor_distance(azimuth, elev_deg, tilt_deg, h):
    """Exact horizontal distance where a ring hits the floor, for a mount pitched by tilt_deg."""
    e = np.radians(elev_deg)
    b = np.stack([np.cos(e) * np.cos(azimuth), np.cos(e) * np.sin(azimuth), np.full_like(azimuth, np.sin(e))], -1)
    d = b @ rot3d(0.0, np.radians(tilt_deg), 0.0).T
    with np.errstate(divide="ignore", invalid="ignore"):
        return np.where(d[:, 2] < 0, h * np.linalg.norm(d[:, :2], axis=1) / -d[:, 2], np.nan)


def _floor_only():
    sc = Scene3D(name="floor")
    f = sc.new_surface(matte(0.9, "floor"), "floor")
    sc.add_quad((-30, -30, 0), (30, -30, 0), (30, 30, 0), (-30, 30, 0), f)
    return sc


def run(person="ammar", seed=None, fast=False, animate_to=None):
    cfg = load_person(person)
    seed = cfg["seed"] if seed is None else seed
    scene = cfg["scene"]
    pr = cfg["e12"]
    rng = np.random.default_rng(seed)
    sc = get_scene3d(scene)
    p3 = LidarParams3D()
    pose = np.array([pr["pose"][0], pr["pose"][1], np.radians(pr["pose"][2])])
    obj_ids = sc.surface_ids("low box") + sc.surface_ids("table")

    s3 = scan3d(sc, pose, p3, rng=rng)
    plane = scan3d(sc, pose, p3.with_(elevations_override=(0.0,)), rng=rng)
    on3 = int((s3.detected & np.isin(s3.surface_id, sc.surface_ids("low box"))).sum())
    on3t = int((s3.detected & np.isin(s3.surface_id, sc.surface_ids("table"))).sum())
    on2 = int((plane.detected & np.isin(plane.surface_id, sc.surface_ids("low box"))).sum())
    on2t = int((plane.detected & np.isin(plane.surface_id, sc.surface_ids("table"))).sum())

    fig1, (a1, a2) = plt.subplots(1, 2, figsize=(11, 4.8))
    viz3d.birds_eye(a1, sc, plane, obj_ids, s=8)
    a1.set_title(f"2D scan plane at h = {p3.height} m: low box {on2}, table {on2t} points", fontsize=10)
    m = viz3d.birds_eye(a2, sc, s3, obj_ids)
    a2.set_title(f"16-ring 3D scan: low box {on3}, table {on3t} points", fontsize=10)
    fig1.colorbar(m, ax=[a1, a2], label="point height z [m]", shrink=0.8)
    viz.legend_below(a2, ncol=1)
    fig1.suptitle("E12a: obstacles below or above the scan plane are invisible to a 2D LiDAR", fontsize=11)

    # (b) floor ring vs tilt
    fl = _floor_only()
    fig2, ax2 = plt.subplots(figsize=(7.0, 3.8))
    rows = []
    for k, tilt in enumerate((0.0, float(cfg["e12"]["tilt_deg"]))):
        s = scan3d(fl, [0, 0, 0], p3.with_(elevations_override=(-15.0,), tilt_deg=tilt), noise=False)
        sim = np.linalg.norm(s.true_hits[0, :, :2], axis=1)
        az = s.azimuths
        exact = floor_distance(az, -15.0, tilt, p3.height)
        ax2.plot(np.degrees(az), exact, color=viz.SERIES[k], lw=1.8, label=f"tilt {tilt:.0f}°: exact")
        ax2.scatter(np.degrees(az[::6]), sim[::6], s=12, color=viz.SERIES[k], zorder=4, linewidths=0,
                    label=f"tilt {tilt:.0f}°: simulated")
        rows.append([f"floor ring (ε = −15°), tilt {tilt:.0f}°: forward distance [m]",
                     float(sim[np.argmin(np.abs(az))]), float(np.nanmax(np.abs(sim - exact)))])
    ax2.set_xlabel("azimuth φ [deg]")
    ax2.set_ylabel("floor-hit distance [m]")
    ax2.set_title("E12b: tilting the mount pulls the floor ring in ahead and pushes it out behind", fontsize=10)
    ax2.legend(fontsize=7.5, ncol=2)

    # (c) range image
    fig3, ax3 = plt.subplots(figsize=(9, 3.4))
    im = viz3d.range_image(ax3, s3, title="E12c: range image of the 3D scan (gray = no return)")
    fig3.colorbar(im, ax=ax3, label="range [m]")

    # (d) ghosts in 3D
    gp = np.array([pr["ghost_pose"][0], pr["ghost_pose"][1], np.radians(pr["ghost_pose"][2])])
    sg = scan3d(sc, gp, p3, rng=rng)
    fig4 = plt.figure(figsize=(6.8, 5.4))
    ax4 = fig4.add_subplot(projection="3d")
    viz3d.scatter3d(ax4, sg, color_by="ghost", s=3)
    ax4.view_init(elev=28, azim=-60)
    ax4.set_title(f"E12d: {int((sg.detected & sg.is_ghost).sum())} ghost points — the room behind the mirror",
                  fontsize=10)
    ax4.legend(fontsize=7.5, loc="upper left")

    # (e) fog
    alphas = np.linspace(0, 0.4, 5 if fast else 9)
    n_mc = 3 if fast else 8
    frac = [np.mean([scan3d(sc.with_fog(a), pose, p3, rng=rng).detected.mean() for _ in range(n_mc)]) for a in alphas]
    fig5, ax5 = plt.subplots(figsize=(6.2, 3.4))
    ax5.plot(alphas, frac, color=viz.BLUE, lw=1.8, marker="o", ms=4)
    ax5.set_xlabel("fog extinction coefficient α [1/m]")
    ax5.set_ylabel("fraction of 5,760 rays detected")
    ax5.set_ylim(0, None)
    ax5.set_title("E12e: fog in 3D", fontsize=10)

    blind = (p3.height - 0.3) / np.tan(np.radians(15))
    rows += [["points on low box: 2D plane / 3D", f"{on2} / {on3}", ""],
             ["points on table: 2D plane / 3D", f"{on2t} / {on3t}", ""],
             ["blind zone for a 0.3 m obstacle (h − z)/tan 15° [m]", float(blind), ""],
             ["3D ghost points (mirror pose)", int((sg.detected & sg.is_ghost).sum()), ""],
             ["3D detected fraction, clear → α=0.4", f"{frac[0]:.2f} → {frac[-1]:.2f}", ""]]
    media = []
    if animate_to:
        media.append(viz3d.rotate_animation(sg, animate_to, n_frames=48 if fast else 90,
                                            title="3D scan near the mirror (orange = ghosts)"))
    return {"figs": {"plane_vs_3d": fig1, "floor_ring": fig2, "range_image": fig3, "ghosts3d": fig4, "fog3d": fig5},
            "tables": {"summary": (["quantity", "value", "max |sim − exact| [m]"], rows)},
            "data": {"scan3d": s3, "plane": plane}, "media": media}


def main():
    args = resolve(parser(__doc__).parse_args())
    anim = None if args.no_anim else f"{args.out}/e12_rotate.gif"
    finish(run(args.person, args.seed, args.fast, animate_to=anim), args.out, "e12")


if __name__ == "__main__":
    main()
