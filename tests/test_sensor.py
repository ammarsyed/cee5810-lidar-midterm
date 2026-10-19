"""Stage 2: sensor pose, frames, FOV and LOS (D3, D4)."""

import numpy as np

from lidar_sim.scenes import default_room
from lidar_sim.sensor import LidarParams, beam_directions, in_fov, scan, to_world, visibility_polygon

MATTE_ROOM = default_room(with_mirror=False, with_glass=False, with_behind_glass=False, with_moving_box=False)


def test_in_fov_angle_wrap_and_range():
    p = LidarParams()
    pose = np.array([0.0, 0.0, np.pi])  # facing -x: the wedge straddles +-180 deg
    pts = np.array([[-2.0, 0.0], [-2.0, 0.1], [-1.0, 1.6], [2.0, 0.0], [-9.0, 0.0], [-0.01, 0.0]])
    # behind-left at 122 deg off the heading is outside the 240 deg wedge
    assert in_fov(pts, pose, p).tolist() == [True, True, True, False, False, False]


def test_in_fov_line_of_sight():
    world = default_room(with_moving_box=False)
    pose = np.array([1.5, 2.5, 0.0])
    behind_glass = np.array([[5.0, 1.2]])  # glass is transparent for LOS
    behind_object = np.array([[6.2, 1.2]])  # hidden behind the box that sits behind the glass
    assert in_fov(behind_glass, pose, world=world)[0]
    assert not in_fov(behind_object, pose, world=world)[0]


def test_to_world_round_trip_and_beam_directions():
    pose = np.array([4.0, 3.0, 0.3])
    s = scan(MATTE_ROOM, pose, noise=False)
    assert np.allclose(to_world(s.true_ranges, pose), s.true_hits)
    u = beam_directions(pose)
    assert np.allclose(np.linalg.norm(u, axis=1), 1.0)
    assert np.isclose(np.arctan2(u[120, 1], u[120, 0]), 0.3)  # middle beam = heading


def test_rotation_shifts_profile_by_whole_beams():
    p = LidarParams()
    base = scan(MATTE_ROOM, [3.0, 2.0, 0.0], p, noise=False).true_ranges
    k = 10  # beams = 10 deg at 1 deg spacing
    rot = scan(MATTE_ROOM, [3.0, 2.0, np.radians(k)], p, noise=False).true_ranges
    assert np.allclose(rot[:-k], base[k:])


def test_noise_free_matte_scan_matches_backend():
    pose = np.array([2.0, 4.0, -0.5])
    s = scan(MATTE_ROOM, pose, noise=False)
    h = MATTE_ROOM.backend().cast(np.tile(pose[:2], (241, 1)), beam_directions(pose))
    det = s.detected
    assert np.allclose(s.ranges[det], h.t[det])
    assert not s.is_ghost.any()


def test_visibility_polygon():
    world = default_room(with_moving_box=False)
    poly = visibility_polygon([1.5, 2.5, 0.0], world, n_rays=500)
    assert poly.shape == (501, 2)
    assert np.allclose(poly[0], [1.5, 2.5])
    assert np.all(np.linalg.norm(poly - [1.5, 2.5], axis=1) <= 5.6 + 1e-9)
