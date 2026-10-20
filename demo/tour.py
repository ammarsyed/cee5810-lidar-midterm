"""Room tour: one annotated scan (snapshot) and an animated drive through the room.

The snapshot shows reported points, mirror ghosts with their true sources, see-through glass, the
FOV wedge and the visible region, next to the polar range profile. The animation drives the sensor
along a loop past the mirror and the glass while the box moves.

Run: uv run python -m demo.tour --person ammar|gal [--no-anim]

The snapshot pose and the drive loop come from the [tour] table in people/<person>.toml.
"""

import matplotlib.pyplot as plt
import numpy as np

from experiments.common import finish, load_person, parser, pose_rad, resolve, scene_of
from lidar_sim import viz
from lidar_sim.paths import waypoint_path
from lidar_sim.sensor import LidarParams, scan, visibility_polygon


def snapshot(person="ammar", seed=None, pose=None, t=0.0):
    """One scan with everything annotated. Returns {"figs": {"snapshot": fig}, "data": {"scan": ...}}.

    pose = [x, y, heading_deg]; default: the person's [tour] pose.
    """
    cfg = load_person(person)
    seed = cfg["seed"] if seed is None else seed
    world = scene_of(cfg)
    params = LidarParams()
    pose = pose_rad(cfg["tour"]["pose"] if pose is None else pose)
    s = scan(world, pose, params, rng=np.random.default_rng(seed), t=t)
    fig = plt.figure(figsize=(11, 4.8))
    ax = fig.add_subplot(1, 2, 1)
    viz.draw_fov(ax, pose, params.fov_deg, params.r_max_ref)
    viz.draw_visibility(ax, visibility_polygon(pose, world, params, t=t))
    viz.draw_world(ax, world, t=t)
    viz.draw_scan(ax, s)
    lo, hi = world.bounds(0.3)
    ax.set_xlim(lo[0], hi[0] + 2.5)
    ax.set_title(f"One scan: {int((s.detected & s.is_ghost).sum())} ghosts, "
                 f"{int((s.detected & s.transmitted).sum())} points through glass")
    viz.legend_below(ax)
    axp = fig.add_subplot(1, 2, 2, projection="polar")
    viz.polar_ranges(axp, s)
    fig.tight_layout()
    return {"figs": {"snapshot": fig}, "data": {"scan": s}}


def animate(out_paths, person="ammar", seed=None, speed=0.5, dt=0.2, fps=8):
    """Drive the person's tour loop and write it to each path (.gif or .mp4). Returns the paths."""
    cfg = load_person(person)
    seed = cfg["seed"] if seed is None else seed
    world = scene_of(cfg)
    params = LidarParams()
    times, poses = waypoint_path(cfg["tour"]["waypoints"], speed, dt)
    rng = np.random.default_rng(seed)
    fig = plt.figure(figsize=(10.5, 4.6))
    ax = fig.add_subplot(1, 2, 1)
    axp = fig.add_subplot(1, 2, 2, projection="polar")
    lo, hi = world.bounds(0.3)

    def update(k):
        ax.clear()
        axp.clear()
        s = scan(world, poses[k], params, rng=rng, t=times[k])
        viz.draw_world(ax, world, t=times[k])
        viz.draw_fov(ax, poses[k], params.fov_deg, params.r_max_ref)
        viz.draw_scan(ax, s, show_dropouts=False)
        ax.set_xlim(lo[0], hi[0] + 2.5)
        ax.set_title(f"t = {times[k]:4.1f} s   detected {s.detected.sum()}/{len(s.detected)}   "
                     f"ghosts {int((s.detected & s.is_ghost).sum())}", fontsize=10)
        viz.polar_ranges(axp, s, r_max=6.0)

    fig.tight_layout()
    paths = [viz.animate(fig, update, len(times), p, fps=fps) for p in out_paths]
    plt.close(fig)
    return paths


def main():
    args = resolve(parser(__doc__).parse_args())
    finish(snapshot(args.person, args.seed), args.out, "tour")
    if not args.no_anim:
        for p in animate([f"{args.out}/tour.gif"], args.person, args.seed):
            print(p)


if __name__ == "__main__":
    main()
