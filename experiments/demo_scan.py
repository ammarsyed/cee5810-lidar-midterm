"""Stage 2-4 demo: one scan in the default room with FOV, visibility, ghosts and glass.

Run: uv run python -m experiments.demo_scan
"""

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from lidar_sim import viz  # noqa: E402
from lidar_sim.scenes import default_room  # noqa: E402
from lidar_sim.sensor import LidarParams, scan, visibility_polygon  # noqa: E402


def run(seed=0, pose=(4.6, 3.2, 0.25), t=0.0):
    world = default_room()
    params = LidarParams()
    s = scan(world, pose, params, rng=np.random.default_rng(seed), t=t)
    fig = plt.figure(figsize=(11, 4.8))
    ax = fig.add_subplot(1, 2, 1)
    viz.draw_fov(ax, pose, params.fov_deg, params.r_max_ref)
    viz.draw_visibility(ax, visibility_polygon(pose, world, params, t=t))
    viz.draw_world(ax, world, t=t)
    viz.draw_scan(ax, s)
    ax.set_xlim(-0.5, 10.5)
    ax.set_title("One scan: reported points, mirror ghosts, see-through glass")
    viz.legend_below(ax)
    axp = fig.add_subplot(1, 2, 2, projection="polar")
    viz.polar_ranges(axp, s)
    fig.tight_layout()
    return {"figs": {"demo_scan": fig}, "data": {"scan": s}}


def main():
    res = run()
    s = res["data"]["scan"]
    print(viz.save(res["figs"]["demo_scan"], "figures", "demo_scan"))
    print(f"detected {s.detected.sum()}/{len(s.detected)}, ghosts {int((s.detected & s.is_ghost).sum())}")


if __name__ == "__main__":
    main()
