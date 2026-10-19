"""Stage 1 smoke figure: a fan of 241 rays from the room centre hitting matte walls.

Run: uv run python -m experiments.smoke
"""

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from lidar_sim import viz  # noqa: E402
from lidar_sim.scenes import default_room  # noqa: E402


def run():
    world = default_room(with_mirror=False, with_glass=False, with_behind_glass=False, with_moving_box=False)
    origin = np.array([4.0, 3.0])
    angles = np.radians(np.linspace(-120, 120, 241))
    dirs = np.stack([np.cos(angles), np.sin(angles)], axis=1)
    hits = world.backend().cast(np.tile(origin, (len(dirs), 1)), dirs)
    fig, ax = plt.subplots(figsize=(6.4, 5))
    viz.draw_world(ax, world)
    viz.draw_rays(ax, origin, hits.point)
    ax.scatter(hits.point[:, 0], hits.point[:, 1], s=6, color=viz.BLUE, zorder=5, label="hit points")
    viz.draw_sensor(ax, (*origin, 0.0))
    ax.set_title("Stage 1: 241 beams over 240° hitting matte walls")
    ax.legend(loc="lower left")
    return {"figs": {"smoke": fig}, "data": {"t": hits.t}}


def main():
    res = run()
    print(viz.save(res["figs"]["smoke"], "figures", "smoke"))


if __name__ == "__main__":
    main()
