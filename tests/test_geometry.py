"""Geometry + 2D backend tests (D3)."""

import numpy as np

from lidar_sim.geometry import (ray_segment_intersect, ray_triangle_intersect, reflect_point, rot2d,
                                rot3d, wrap_angle)
from lidar_sim.scenes import default_room, single_wall


def _cast_one(A, B, o, u):
    t, s = ray_segment_intersect(np.array([o], float), np.array([u], float), np.array([A], float),
                                 np.array([B], float))
    return t[0, 0], s[0, 0]


def test_intersection_hit_miss_parallel_behind():
    A, B = (2.0, -1.0), (2.0, 1.0)
    t, s = _cast_one(A, B, (0, 0), (1, 0))
    assert np.isclose(t, 2.0) and np.isclose(s, 0.5)
    assert np.isinf(_cast_one(A, B, (0, 0), (0, 1))[0])  # miss (parallel to y, never reaches x=2)
    assert np.isinf(_cast_one(A, B, (0, 5), (1, 0))[0])  # miss (passes above the segment)
    assert np.isinf(_cast_one((0, 1), (4, 1), (0, 0), (1, 0))[0])  # parallel
    assert np.isinf(_cast_one(A, B, (0, 0), (-1, 0))[0])  # segment is behind the ray


def test_closed_form_wall_range_matches_backend():
    """R_i = (d - n.p) / (n.u_i) for a flat wall (D3b)."""
    rng = np.random.default_rng(0)
    world = single_wall(distance=3.0, length=200.0)  # wall x = 3: n = (1, 0), d = 3
    n, d = np.array([1.0, 0.0]), 3.0
    p = rng.uniform(-1, 1, size=(500, 2))
    ang = rng.uniform(-1.2, 1.2, size=500)
    u = np.stack([np.cos(ang), np.sin(ang)], axis=1)
    hits = world.backend().cast(p, u)
    expected = (d - p @ n) / (u @ n)
    assert np.allclose(hits.t, expected, atol=1e-9)
    assert np.all(hits.surface_id == 0)
    assert np.allclose(hits.normal, [-1.0, 0.0])  # flipped to face the rays


def test_batched_equals_one_at_a_time():
    world = default_room(with_moving_box=False)
    be = world.backend()
    rng = np.random.default_rng(1)
    o = rng.uniform([0.5, 0.5], [7.5, 5.5], size=(300, 2))
    a = rng.uniform(-np.pi, np.pi, 300)
    u = np.stack([np.cos(a), np.sin(a)], axis=1)
    batch = be.cast(o, u)
    for i in range(0, 300, 37):
        one = be.cast(o[i:i + 1], u[i:i + 1])
        assert np.isclose(one.t[0], batch.t[i]) and one.surface_id[0] == batch.surface_id[i]


def test_chunking_gives_same_answer():
    world = default_room(with_moving_box=False)
    be = world.backend()
    be_small = world.backend()
    be_small.max_pairs = 7  # forces many chunks
    rng = np.random.default_rng(2)
    o = rng.uniform([0.5, 0.5], [7.5, 5.5], size=(50, 2))
    a = rng.uniform(-np.pi, np.pi, 50)
    u = np.stack([np.cos(a), np.sin(a)], axis=1)
    assert np.allclose(be.cast(o, u).t, be_small.cast(o, u).t)


def test_closed_room_every_ray_hits():
    world = default_room(with_mirror=False, with_glass=False, with_behind_glass=False, with_moving_box=False)
    a = np.linspace(-np.pi, np.pi, 3601)
    hits = world.backend().cast(np.tile([4.0, 3.0], (len(a), 1)), np.stack([np.cos(a), np.sin(a)], 1))
    assert np.all(hits.hit)  # no ray escapes through a corner


def test_rotations_and_helpers():
    assert np.allclose(rot2d(np.pi / 2) @ [1, 0], [0, 1])
    assert np.allclose(rot3d(yaw=np.pi / 2) @ [1, 0, 0], [0, 1, 0])
    assert rot3d(pitch=np.radians(10))[2] @ [1, 0, 0] < 0  # positive pitch tilts forward axis down
    assert np.allclose(wrap_angle([np.pi, -np.pi, 3 * np.pi]), [-np.pi, -np.pi, -np.pi])
    m, n = np.array([1.0, 0.0]), np.array([1.0, 0.0])
    assert np.allclose(reflect_point([3.0, 2.0], m, n), [-1.0, 2.0])


def test_moller_trumbore_against_plane():
    v0, v1, v2 = np.array([[3.0, -1, 0]]), np.array([[3.0, 1, 0]]), np.array([[3.0, -1, 3]])
    o = np.array([[0.0, 0.0, 0.6]])
    d = np.array([[np.cos(0.1), 0.0, np.sin(-0.05)]])
    d /= np.linalg.norm(d)
    t = ray_triangle_intersect(o, d, v0[0], v1[0], v2[0])
    assert np.isclose(t[0, 0], 3.0 / d[0, 0])
