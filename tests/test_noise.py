"""Stage 3: probabilistic measurement model (D1, D6)."""

import numpy as np

from lidar_sim import config
from lidar_sim.noise import detection_prob, range_noise_std, sample_measurement
from lidar_sim.scenes import single_wall
from lidar_sim.sensor import LidarParams, scan


def test_sigma_r_from_timing_jitter():
    assert np.isclose(range_noise_std(config.SIGMA_T), 0.01)
    assert np.isclose(config.SIGMA_T, 66.7e-12, rtol=1e-3)  # 67 ps for 1 cm (D1)


def test_sampled_noise_std():
    rng = np.random.default_rng(0)
    z, det = sample_measurement(np.full(10_000, 3.0), np.ones(10_000), rng)
    assert det.all()
    assert abs(np.std(z) / 0.01 - 1) < 0.03
    assert abs(np.mean(z) - 3.0) < 0.001


def test_noise_through_scan():
    rng = np.random.default_rng(1)
    p = LidarParams(fov_deg=0.0, n_beams=1)
    z = np.array([scan(single_wall(3.0), [0, 0, 0], p, rng=rng).ranges[0] for _ in range(2000)])
    assert np.all(np.isfinite(z))  # 3 m white wall: always detected
    assert abs(np.std(z) / 0.01 - 1) < 0.05


def test_detection_prob_shape():
    assert np.isclose(detection_prob(1.0), 0.5)
    assert detection_prob(2.0) > 0.999 and detection_prob(0.5) < 1e-3
    assert detection_prob(np.nan) < 1e-12


def test_dropouts_are_nan():
    rng = np.random.default_rng(2)
    z, det = sample_measurement(np.array([1.0, 2.0, np.nan]), np.array([0.0, 1.0, 1.0]), rng)
    assert np.isnan(z[0]) and np.isfinite(z[1]) and np.isnan(z[2])
    assert det.tolist() == [False, True, False]
