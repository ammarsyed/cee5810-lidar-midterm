"""Stage 3/4 physics: power, max range, fog, Fresnel, reflection (D5, D8, D9, D10, D11)."""

import numpy as np
import pytest
from scipy.optimize import brentq

from lidar_sim import config
from lidar_sim.physics import (dmax_range_dalpha, diffuse_power, fresnel_deg, max_range, pane_transmission,
                               reflect)
from lidar_sim.scenes import single_wall
from lidar_sim.sensor import LidarParams, scan

ONE_BEAM = LidarParams(fov_deg=0.0, n_beams=1)


def test_calibration():
    assert np.isclose(config.K_OVER_PTH, 34.8444, atol=1e-3)
    assert np.isclose(max_range(0.9), 5.6)
    assert np.isclose(max_range(0.7), 4.94, atol=0.005)
    assert np.isclose(max_range(0.2), 2.64, atol=0.005)
    assert np.isclose(max_range(0.1), 1.87, atol=0.005)


@pytest.mark.parametrize("rho", [0.1, 0.5, 0.9])
def test_simulated_threshold_crossing_matches_analytic_max_range(rho):
    R = float(max_range(rho))
    p_at = scan(single_wall(R, rho), [0, 0, 0], ONE_BEAM, noise=False).power[0]
    assert np.isclose(p_at, 1.0, rtol=1e-9)  # P_D = 0.5 exactly at R_max
    near = scan(single_wall(0.99 * R, rho), [0, 0, 0], ONE_BEAM, noise=False).p_detect[0]
    far = scan(single_wall(1.01 * R, rho), [0, 0, 0], ONE_BEAM, noise=False).p_detect[0]
    assert near > 0.5 > far
    rng = np.random.default_rng(3)
    hits = [scan(single_wall(R, rho), [0, 0, 0], ONE_BEAM, rng=rng).detected[0] for _ in range(2000)]
    assert abs(np.mean(hits) - 0.5) < 0.04


def test_fog_lambert_w_matches_root_find_and_table():
    table = {0.9: [5.60, 4.48, 3.82, 3.05, 2.58, 2.03], 0.7: [4.94, 4.04, 3.49, 2.81, 2.40, 1.91],
             0.2: [2.64, 2.35, 2.13, 1.83, 1.62, 1.35], 0.1: [1.87, 1.71, 1.59, 1.41, 1.27, 1.09]}
    alphas = [0, 0.05, 0.1, 0.2, 0.3, 0.5]
    for rho, row in table.items():
        for a, expect in zip(alphas, row):
            R = float(max_range(rho, alpha=a))
            assert round(R, 2) == pytest.approx(expect, abs=0.005)
            if a > 0:
                root = brentq(lambda r: diffuse_power(r, rho, 1.0, alpha=a) - 1.0, 0.01, 50)
                assert np.isclose(R, root, rtol=1e-9)


def test_fog_limit_and_derivative():
    assert np.isclose(max_range(0.9, alpha=1e-9), 5.6, rtol=1e-6)
    for a in (0.05, 0.2, 0.4):
        h = 1e-6
        fd = (max_range(0.9, alpha=a + h) - max_range(0.9, alpha=a - h)) / (2 * h)
        assert np.isclose(dmax_range_dalpha(0.9, a), fd, rtol=1e-5)
    assert np.isclose(dmax_range_dalpha(0.9, 0.2), -5.76, atol=0.01)


def test_fresnel_table():
    angles = [0, 30, 45, 56.31, 70, 80, 89]
    expect = [0.0400, 0.0415, 0.0502, 0.0740, 0.1710, 0.3877, 0.9042]
    assert np.allclose(fresnel_deg(np.array(angles)), expect, atol=6e-5)
    assert np.isclose(pane_transmission(0.04), 0.9216)
    assert np.isclose(pane_transmission(0.04, incoherent=True), 0.923, atol=5e-4)


def test_reflect_length_and_angle():
    rng = np.random.default_rng(0)
    u = rng.normal(size=(100, 3))
    u /= np.linalg.norm(u, axis=1, keepdims=True)
    n = rng.normal(size=(100, 3))
    n /= np.linalg.norm(n, axis=1, keepdims=True)
    r = reflect(u, n)
    assert np.allclose(np.linalg.norm(r, axis=1), 1.0)
    assert np.allclose(np.sum(r * n, axis=1), -np.sum(u * n, axis=1))  # normal component flips
    tang_u = u - np.sum(u * n, 1, keepdims=True) * n
    tang_r = r - np.sum(r * n, 1, keepdims=True) * n
    assert np.allclose(tang_u, tang_r)  # tangential component unchanged
