"""Stage 5 demo: the LiDAR drives through the default room while the box moves.

Run: uv run python -m experiments.demo_animation
"""

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from lidar_sim import viz  # noqa: E402
from lidar_sim.paths import waypoint_path  # noqa: E402
from lidar_sim.scenes import default_room  # noqa: E402
from lidar_sim.sensor import LidarParams, scan  # noqa: E402


def frames(world=None, seed=0, speed=0.5, dt=0.2):
    world = default_room() if world is None else world
    times, poses = waypoint_path([(1.2, 1.5), (4.0, 3.2), (6.0, 2.6), (4.0, 3.6), (1.5, 3.0)], speed, dt)
    rng = np.random.default_rng(seed)
    return world, times, poses, rng


def render(out_paths, world=None, seed=0, dt=0.2, fps=8):
    world, times, poses, rng = frames(world, seed, dt=dt)
    params = LidarParams()
    fig = plt.figure(figsize=(10.5, 4.6))
    ax = fig.add_subplot(1, 2, 1)
    axp = fig.add_subplot(1, 2, 2, projection="polar")

    def update(k):
        ax.clear()
        axp.clear()
        s = scan(world, poses[k], params, rng=rng, t=times[k])
        viz.draw_world(ax, world, t=times[k])
        viz.draw_fov(ax, poses[k], params.fov_deg, params.r_max_ref)
        viz.draw_scan(ax, s, show_dropouts=False)
        ax.set_xlim(-0.5, 10.5)
        ax.set_title(f"t = {times[k]:4.1f} s   detected {s.detected.sum()}/241   "
                     f"ghosts {int((s.detected & s.is_ghost).sum())}", fontsize=10)
        viz.polar_ranges(axp, s, r_max=6.0)

    fig.tight_layout()
    return [viz.animate(fig, update, len(times), p, fps=fps) for p in out_paths]


def main():
    for p in render(["figures/demo_drive.gif", "figures/demo_drive.mp4"]):
        print(p)


if __name__ == "__main__":
    main()
