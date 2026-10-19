"""Stage 5: paths, unicycle model, moving targets."""

import numpy as np

from lidar_sim.localization.motion import sample_motion, unicycle_step
from lidar_sim.paths import pure_pursuit, waypoint_path
from lidar_sim.scenes import default_room
from lidar_sim.sensor import LidarParams, scan


def test_waypoint_path_constant_speed():
    t, p = waypoint_path([(0, 0), (2, 0), (2, 2)], speed=0.5, dt=0.1)
    step = np.linalg.norm(np.diff(p[:, :2], axis=0), axis=1)
    assert np.allclose(step[:15], 0.05)
    assert np.isclose(t[-1], 8.0, atol=0.1)
    assert np.isclose(p[5, 2], 0.0) and np.isclose(p[-1, 2], np.pi / 2)


def test_pure_pursuit_controls_reproduce_poses():
    t, poses, u = pure_pursuit([(1, 1), (5, 1), (5, 4), (1, 4)], v=0.5, dt=0.1)
    X = poses[0]
    for k in range(len(u)):
        X = unicycle_step(X, u[k], 0.1)
    assert np.allclose(X, poses[-1])
    assert np.linalg.norm(poses[-1, :2] - [1, 4]) < 0.4


def test_sample_motion_noise_scales():
    rng = np.random.default_rng(0)
    X = np.zeros((20000, 3))
    Y = sample_motion(X, (1.0, 0.0), 1.0, rng, sigma_v=0.1, sigma_w=0.05)
    assert abs(Y[:, 0].std() - 0.1) < 0.005
    assert abs(Y[:, 2].std() - 0.05) < 0.003


def test_moving_box_changes_the_scan():
    world = default_room()
    p = LidarParams()
    a = scan(world, [3.2, 2.0, np.pi / 2], p, noise=False, t=0.0).ranges
    b = scan(world, [3.2, 2.0, np.pi / 2], p, noise=False, t=4.0).ranges  # box has moved 2 m
    assert np.nanmax(np.abs(np.nan_to_num(a) - np.nan_to_num(b))) > 0.5
