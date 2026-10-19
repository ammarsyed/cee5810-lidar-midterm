"""Stage 8: 3D backend and sensor (D14)."""

import time

import numpy as np

from lidar_sim.geometry import ray_triangle_intersect, reflect_point
from lidar_sim.materials import matte
from lidar_sim.scenes import default_room
from lidar_sim.scenes3d import Scene3D, default_room_3d, extrude
from lidar_sim.sensor import scan
from lidar_sim.sensor3d import LidarParams3D, scan3d


def test_embree_matches_moller_trumbore():
    sc = default_room_3d()
    be = sc.backend()
    rng = np.random.default_rng(0)
    o = np.column_stack([rng.uniform(0.5, 7.5, 3000), rng.uniform(0.5, 5.5, 3000), rng.uniform(0.2, 2.8, 3000)])
    d = rng.normal(size=(3000, 3))
    d /= np.linalg.norm(d, axis=1, keepdims=True)
    h = be.cast(o, d)
    V, F = np.asarray(sc.vertices, float), np.asarray(sc.faces)
    t_mt = ray_triangle_intersect(o, d, V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]).min(axis=1)
    both = np.isfinite(h.t) & np.isfinite(t_mt)
    assert np.mean(np.isfinite(h.t) == np.isfinite(t_mt)) > 0.999
    assert np.max(np.abs(h.t[both] - t_mt[both])) < 1e-4


def _flat(world2d):
    return extrude(world2d, 3.0)


def test_horizontal_ring_equals_2d_scan():
    """D14e: on an extruded scene, the eps = 0 ring reproduces the 2D sensor (incl. mirror + glass)."""
    w2 = default_room(with_moving_box=False)
    sc = _flat(w2)
    pose = np.array([4.6, 3.2, 0.25])
    s2 = scan(w2, pose, noise=False)
    s3 = scan3d(sc, pose, LidarParams3D(elevations_override=(0.0,), az_step_deg=1.0), noise=False)
    az = np.degrees(s3.azimuths)
    idx = np.array([int(np.argmin(np.abs(az - b))) for b in np.degrees(s2.beam_angles)])
    r3, r2 = s3.true_ranges[0, idx], s2.true_ranges
    ok = np.isfinite(r2)
    assert np.array_equal(np.isfinite(r3), ok)
    assert np.max(np.abs(r3[ok] - r2[ok])) < 1e-3
    assert np.allclose(s3.power[0, idx][ok], s2.power[ok], rtol=2e-3)
    assert np.array_equal(s3.is_ghost[0, idx], s2.is_ghost)


def test_tilted_ring_range_scales_with_1_over_cos():
    w2 = default_room(with_mirror=False, with_glass=False, with_behind_glass=False, with_moving_box=False)
    sc = _flat(w2)
    pose = np.array([4.0, 3.0, 0.0])
    s2 = scan(w2, pose, noise=False)
    eps = 10.0
    s3 = scan3d(sc, pose, LidarParams3D(elevations_override=(eps,)), noise=False)
    az = np.degrees(s3.azimuths)
    idx = np.array([int(np.argmin(np.abs(az - b))) for b in np.degrees(s2.beam_angles)])
    walls = s3.surface_id[0, idx] < len(w2.segments)  # hit a wall, not floor/ceiling
    ratio = s3.true_ranges[0, idx][walls] / s2.true_ranges[walls]
    assert walls.sum() > 50
    assert np.allclose(ratio, 1 / np.cos(np.radians(eps)), atol=1e-4)


def _floor_only():
    sc = Scene3D(name="floor")
    f = sc.new_surface(matte(0.9, "floor"), "floor")
    sc.add_quad((-20, -20, 0), (20, -20, 0), (20, 20, 0), (-20, 20, 0), f)
    return sc


def test_floor_ring_radius_and_tilt():
    sc = _floor_only()
    s = scan3d(sc, [0, 0, 0], LidarParams3D(elevations_override=(-15.0,)), noise=False)
    horiz = np.linalg.norm(s.true_hits[0, :, :2], axis=1)
    assert np.allclose(horiz, 0.6 / np.tan(np.radians(15)), atol=0.01)  # 2.24 m
    st = scan3d(sc, [0, 0, 0], LidarParams3D(elevations_override=(-15.0,), tilt_deg=5.0), noise=False)
    fwd = int(np.argmin(np.abs(st.azimuths)))
    assert np.isclose(np.linalg.norm(st.true_hits[0, fwd, :2]), 0.6 / np.tan(np.radians(20)), atol=0.01)  # 1.65 m


def test_low_obstacle_blind_zone():
    """A 0.3 m curb is invisible to the 2D plane (h = 0.6 m) and to 3D closer than 1.12 m (D14b)."""
    def curb_at(r):
        sc = _floor_only()
        c = sc.new_surface(matte(0.8, "curb"), "curb")
        sc.add_quad((r, -1, 0), (r, 1, 0), (r, 1, 0.3), (r, -1, 0.3), c)
        return sc, c

    for r, visible in [(1.0, False), (1.2, True), (2.0, True)]:
        sc, c = curb_at(r)
        s3 = scan3d(sc, [0, 0, 0], noise=False)
        fwd = int(np.argmin(np.abs(s3.azimuths)))  # straight ahead: horizontal distance = r
        assert bool((s3.detected[:, fwd] & (s3.surface_id[:, fwd] == c)).any()) == visible, r
        flat = scan3d(sc, [0, 0, 0], LidarParams3D(elevations_override=(0.0,)), noise=False)
        assert not (flat.surface_id == c).any()


def test_ghost_theorem_in_3d():
    w2 = default_room(with_glass=False, with_moving_box=False)
    sc = _flat(w2)
    s = scan3d(sc, [4.6, 3.2, 0.25], noise=False)
    m = [seg for seg in w2.segments if seg.name == "mirror"][0]
    e = m.b - m.a
    n_hat = np.array([-e[1], e[0], 0.0]) / np.linalg.norm(e)
    r = s.returns
    sel = (r.reflections == 1) & (r.kind == 0) & np.isin(r.reflector, sc.surface_ids("mirror"))
    assert sel.sum() > 50
    assert np.allclose(r.apparent_point[sel], reflect_point(r.hit_point[sel], np.array([*m.a, 0.0]), n_hat),
                       atol=1e-4)


def test_3d_scan_speed():
    sc = default_room_3d()
    scan3d(sc, [4.6, 3.2, 0.25], noise=False)  # warm-up (builds the Embree scene)
    t0 = time.perf_counter()
    s = scan3d(sc, [4.6, 3.2, 0.25], rng=np.random.default_rng(0))
    dt = time.perf_counter() - t0
    print(f"3D scan, 16 x 360 rays with mirror + glass: {dt * 1e3:.1f} ms, detected {s.detected.sum()}")
    assert s.ranges.shape == (16, 360)
    assert dt < 0.3
