"""Unicycle motion model with noisy odometry, vectorised over particles (D13a).

x_{k+1} = x_k + dt v cos(theta_k),  y_{k+1} = y_k + dt v sin(theta_k),  theta_{k+1} = theta_k + dt w
(the nonholonomic unicycle model from lecture L2, discretised with forward Euler).
"""

import numpy as np

from ..geometry import wrap_angle


def unicycle_step(X, u, dt):
    """Deterministic step for one pose (3,) or many poses (P, 3); u = (v, w)."""
    X = np.asarray(X, dtype=float)
    v, w = np.asarray(u, dtype=float)[..., 0], np.asarray(u, dtype=float)[..., 1]
    th = X[..., 2]
    out = np.empty_like(X)
    out[..., 0] = X[..., 0] + dt * v * np.cos(th)
    out[..., 1] = X[..., 1] + dt * v * np.sin(th)
    out[..., 2] = wrap_angle(th + dt * w)
    return out


def sample_motion(X, u, dt, rng, sigma_v=0.05, sigma_w=0.05):
    """Sample x_{k+1} ~ p(x_{k+1} | x_k, u_k): perturb (v, w) per particle, then step."""
    X = np.atleast_2d(np.asarray(X, dtype=float))
    n = len(X)
    v = u[0] + rng.normal(0.0, sigma_v, n)
    w = u[1] + rng.normal(0.0, sigma_w, n)
    return unicycle_step(X, np.stack([v, w], axis=1), dt)
