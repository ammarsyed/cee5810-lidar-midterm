"""Stage 7: particle-filter localization (D13)."""

import time

import numpy as np

from lidar_sim.geometry import wrap_angle
from lidar_sim.localization.likelihood import beam_log_likelihood, make_model, mixture_gap
from lidar_sim.localization.particle_filter import (ParticleFilter, circular_mean, init_particles,
                                                    low_variance_resample)
from lidar_sim.paths import pure_pursuit
from lidar_sim.scenes import default_room
from lidar_sim.sensor import LidarParams, expected_scan, scan

BEAMS = np.arange(0, 241, 8)


def test_filter_converges_in_matte_room():
    world = default_room(with_mirror=False, with_glass=False, with_behind_glass=False, with_moving_box=False)
    rng = np.random.default_rng(0)
    _, poses, u = pure_pursuit([(1.5, 1.5), (6.0, 1.5), (6.0, 4.5), (2.0, 4.5)], v=0.5, dt=0.2)
    poses, u = poses[:61], u[:60]
    start = poses[0] + np.array([0.3, -0.3, np.radians(10)])
    pf = ParticleFilter(world, init_particles(start, (0.3, 0.3, np.radians(10)), 300, rng), rng, beam_idx=BEAMS,
                        lik=dict(sigma=0.05, eps=0.05))
    errs = []
    for k in range(60):
        pf.predict(u[k] + rng.normal(0, [0.03, 0.03]), 0.2)
        z = scan(world, poses[k + 1], rng=rng).ranges[BEAMS]
        pf.update(z)
        errs.append(np.linalg.norm(pf.estimate()[:2] - poses[k + 1, :2]))
    assert np.mean(errs[-10:]) < 0.06


def test_low_variance_resampling_is_unbiased():
    rng = np.random.default_rng(1)
    w = rng.random(50)
    w /= w.sum()
    counts = np.zeros(50)
    for _ in range(3000):
        counts += np.bincount(low_variance_resample(w, rng), minlength=50)
    assert np.allclose(counts / 3000, 50 * w, atol=0.05)


def test_naive_equals_physics_without_mirror_or_glass():
    world = default_room(with_mirror=False, with_glass=False, with_moving_box=False)
    poses = np.array([[2.0, 2.0, 0.3], [5.0, 4.0, -2.0]])
    a = expected_scan(make_model(world, "naive"), poses, beam_idx=BEAMS)
    b = expected_scan(make_model(world, "physics"), poses, beam_idx=BEAMS)
    assert np.allclose(np.nan_to_num(a[0], nan=-1), np.nan_to_num(b[0], nan=-1)) and np.allclose(a[1], b[1])


def test_naive_model_turns_glass_and_mirror_opaque():
    world = default_room(with_moving_box=False)
    naive = make_model(world, "naive")
    assert not naive.has_specular() and world.has_specular()


def test_mixture_bounds_the_per_beam_penalty():
    z = np.array([3.0])
    R_hat = np.array([[3.0], [2.0], [np.nan]])  # perfect, 1 m off, no return expected
    P_D = np.array([[1.0], [1.0], [0.0]])
    with_mix = beam_log_likelihood(z, R_hat, P_D, sigma=0.05, eps=0.05)[:, 0]
    no_mix = beam_log_likelihood(z, R_hat, P_D, sigma=0.05, use_mixture=False)[:, 0]
    assert np.isclose(with_mix[0] - with_mix[1], mixture_gap(0.05), rtol=1e-6)
    assert np.isclose(mixture_gap(0.05), 6.745, atol=0.01)
    assert np.isclose(mixture_gap(0.01), 8.35, atol=0.01)  # D13: sigma = 1 cm
    assert no_mix[0] - no_mix[1] > 150  # (1 m / 5 cm)^2 / 2 = 200 nats without the mixture


def test_circular_mean_across_pi():
    th = np.array([np.pi - 0.1, -np.pi + 0.1])
    assert np.isclose(abs(wrap_angle(circular_mean(th, np.array([0.5, 0.5])))), np.pi)


def test_physics_update_speed():
    world = default_room(with_moving_box=False)
    rng = np.random.default_rng(2)
    pf = ParticleFilter(world, init_particles([3, 3, 0], (0.3, 0.3, 0.2), 300, rng), rng, beam_idx=BEAMS)
    z = scan(world, [3, 3, 0], rng=rng).ranges[BEAMS]
    pf.update(z)
    t0 = time.perf_counter()
    pf.update(z)
    dt = time.perf_counter() - t0
    print(f"physics-aware update, 300 particles x 31 beams: {dt * 1e3:.1f} ms")
    assert dt < 0.3
