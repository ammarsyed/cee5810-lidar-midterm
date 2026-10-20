"""Room tour: one annotated scan (snapshot) and an animated drive through the room.

The snapshot shows reported points, mirror ghosts with their true sources, see-through glass, the
FOV wedge and the visible region, next to the polar range profile. The animation drives the sensor
along a loop past the mirror and the glass while the box moves.

Run: uv run python -m demo.tour [--person partner] [--no-anim]
"""

import matplotlib.pyplot as plt
import numpy as np

from experiments.common import finish, parser, resolve
from lidar_sim import viz
from lidar_sim.paths import waypoint_path
from lidar_sim.scenes import get_scene
from lidar_sim.sensor import LidarParams, scan, visibility_polygon

# Per scene: a snapshot pose (x, y, heading_deg) close enough to the mirror to show ghosts, and a
# drive loop that passes the mirror and the glass.
TOURS = {
    "default": dict(pose=(4.6, 3.2, 14.0),
                    waypoints=[(1.2, 1.5), (4.0, 3.2), (6.0, 2.6), (4.0, 3.6), (1.5, 3.0)]),
    "variant_b": dict(pose=(0.9, 2.1, 60.0),
                      waypoints=[(2.2, 1.2), (4.4, 2.4), (4.2, 4.4), (2.6, 4.8), (2.0, 2.6)]),
}


def snapshot(scene="default", seed=0, pose=None, t=0.0):
    """One scan with everything annotated. Returns {"figs": {"snapshot": fig}, "data": {"scan": ...}}."""
    world = get_scene(scene)
    params = LidarParams()
    px, py, ph = TOURS[scene]["pose"] if pose is None else pose
    pose = np.array([px, py, np.radians(ph)])
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


def animate(out_paths, scene="default", seed=0, speed=0.5, dt=0.2, fps=8):
    """Drive the tour loop and write it to each path (.gif or .mp4). Returns the paths."""
    world = get_scene(scene)
    params = LidarParams()
    times, poses = waypoint_path(TOURS[scene]["waypoints"], speed, dt)
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
    finish(snapshot(args.scene, args.seed), args.out, "tour")
    if not args.no_anim:
        for p in animate([f"{args.out}/tour.gif"], args.scene, args.seed):
            print(p)


if __name__ == "__main__":
    main()
