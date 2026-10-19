"""Stage 4: batched tracer with mirror + glass (D5, D8, D9)."""

import time

import numpy as np

from lidar_sim import config
from lidar_sim.geometry import reflect_point, segment_normal
from lidar_sim.materials import matte, mirror
from lidar_sim.physics import fresnel
from lidar_sim.scenes import default_room, glass_test
from lidar_sim.sensor import LidarParams, expected_scan, scan, trace_beams
from lidar_sim.world import Segment, World

ONE_BEAM = LidarParams(fov_deg=0.0, n_beams=1)


def test_ghost_point_is_reflection_of_true_hit():
    world = default_room(with_glass=False, with_moving_box=False)
    s = scan(world, [4.6, 3.2, 0.25], noise=False)
    m = [seg for seg in world.segments if seg.name == "mirror"][0]
    n_hat = segment_normal(m.a, m.b)
    r = s.returns
    sel = (r.reflections == 1) & (r.kind == 0)
    assert sel.sum() > 10  # plenty of ghost returns
    assert np.all(np.isin(r.reflector[sel], world.surface_ids("mirror")))
    assert np.allclose(r.apparent_point[sel], reflect_point(r.hit_point[sel], m.a, n_hat), atol=1e-9)
    assert s.is_ghost.any()


def _mirror_scene():
    return World([Segment((2.0, -1.0), (2.0, 1.0), mirror(), "mirror"),
                  Segment((-3.0, -20), (-3.0, 20), matte(0.7), "back wall")])


def test_head_on_mirror_returns_its_true_distance():
    s = scan(_mirror_scene(), [0, 0, 0], ONE_BEAM, noise=False)
    assert np.isclose(s.ranges[0], 2.0)
    assert not s.is_ghost[0]


def test_mirror_bounce_power_uses_R_m_squared():
    world = _mirror_scene()
    r = trace_beams(world, [0, 0, 0], ONE_BEAM)["returns"]
    diffuse = (r.kind == 0) & (r.reflections == 1)
    L = 2.0 + 5.0  # to the mirror, then back past the sensor to the wall at x = -3
    expect = config.K_OVER_PTH * 0.7 * 1.0 * config.R_MIRROR**2 / L**2
    assert np.isclose(r.path_length[diffuse][0], L)
    assert np.isclose(r.power[diffuse][0], expect)


def test_glass_head_on_is_seen_but_45_deg_is_invisible():
    head_on = scan(glass_test(0.0), [0, 0, 0], ONE_BEAM, noise=False)
    assert np.isclose(head_on.ranges[0], 2.0)  # the specular blip off the glass (Heiden Fig. 1(4))
    oblique = scan(glass_test(45.0), [0, 0, 0], ONE_BEAM, noise=False)
    assert np.isclose(oblique.ranges[0], 4.0)  # sees straight through to the wall behind
    assert not oblique.is_ghost[0]


def test_transmitted_power_through_pane():
    r = trace_beams(glass_test(0.0), [0, 0, 0], ONE_BEAM)["returns"]
    wall = (r.kind == 0) & (r.interactions == 1)
    T = (1 - fresnel(1.0)) ** 2
    assert np.isclose(r.power[wall][0], config.K_OVER_PTH * 0.7 * T**2 / 16.0)


def test_first_vs_strongest_policy():
    strongest = scan(glass_test(0.0), [0, 0, 0], ONE_BEAM, noise=False)
    first = scan(glass_test(0.0), [0, 0, 0], ONE_BEAM.with_(return_policy="first"), noise=False)
    assert np.isclose(first.ranges[0], 2.0) and np.isclose(strongest.ranges[0], 2.0)
    # between ~4.19 and ~4.33 deg the glass lobe is weaker than the wall behind it but still above
    # threshold, so the two policies disagree. 4.32 deg = theta_w sqrt(2 ln((K_s/K)(K/P_th) R_F / R_g^2)) (D5)
    a = 4.25
    s_ = scan(glass_test(a), [0, 0, 0], ONE_BEAM, noise=False)
    f_ = scan(glass_test(a), [0, 0, 0], ONE_BEAM.with_(return_policy="first"), noise=False)
    assert s_.ranges[0] > 3.9
    assert abs(f_.ranges[0] - 2.0) < 0.01


def test_batch_of_poses_equals_one_at_a_time():
    world = default_room(with_moving_box=False)
    rng = np.random.default_rng(0)
    poses = np.column_stack([rng.uniform(0.5, 7.5, 40), rng.uniform(0.5, 5.5, 40), rng.uniform(-3, 3, 40)])
    idx = np.arange(0, 241, 4)
    R, PD = expected_scan(world, poses, beam_idx=idx)
    for i in range(0, 40, 9):
        r1, p1 = expected_scan(world, poses[i], beam_idx=idx)
        assert np.allclose(np.nan_to_num(R[i], nan=-1), np.nan_to_num(r1[0], nan=-1))
        assert np.allclose(PD[i], p1[0])


def test_batched_tracing_speed():
    world = default_room(with_moving_box=False)
    rng = np.random.default_rng(1)
    poses = np.column_stack([rng.uniform(0.5, 7.5, 500), rng.uniform(0.5, 5.5, 500), rng.uniform(-3, 3, 500)])
    idx = np.arange(0, 241, 4)  # 61 beams
    expected_scan(world, poses[:10], beam_idx=idx)  # warm-up
    t0 = time.perf_counter()
    expected_scan(world, poses, beam_idx=idx)
    dt = time.perf_counter() - t0
    print(f"500 poses x 61 beams with mirror + glass: {dt * 1e3:.1f} ms")
    assert dt < 0.5


def test_ghost_theorem_with_glass_present_selects_mirror_reflections():
    world = default_room(with_moving_box=False)  # glass reflections exist too
    s = scan(world, [6.9, 2.4, np.radians(150)], noise=False)
    m = [seg for seg in world.segments if seg.name == "mirror"][0]
    r = s.returns
    sel = (r.reflections == 1) & (r.kind == 0) & np.isin(r.reflector, world.surface_ids("mirror"))
    assert sel.sum() > 10
    assert np.allclose(r.apparent_point[sel], reflect_point(r.hit_point[sel], m.a, segment_normal(m.a, m.b)),
                       atol=1e-9)


def test_glass_blip_edge_matches_closed_form():
    """theta_max = theta_w sqrt(2 ln((K_s/K)(K/P_th) R_F / R_g^2)) for glass at R_g = 2 m (D5)."""
    from lidar_sim.physics import fresnel_deg

    w = np.radians(config.SPEC_LOBE_DEG)
    th = np.degrees(w * np.sqrt(2 * np.log(config.SPEC_GAIN * config.K_OVER_PTH * fresnel_deg(4.3) / 4.0)))
    first = ONE_BEAM.with_(return_policy="first")
    assert np.isclose(scan(glass_test(th - 0.02), [0, 0, 0], first, noise=False).ranges[0], 2.0, atol=0.01)
    assert scan(glass_test(th + 0.02), [0, 0, 0], first, noise=False).ranges[0] > 3.9
    assert np.isclose(config.SPEC_GAIN, 182.4, atol=0.1)
