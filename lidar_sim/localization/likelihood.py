"""Naive vs physics-aware expected scans and the robust log-likelihood (D6, D13d).

physics-aware model: the real world (mirrors reflect, glass transmits)
naive model:         every mirror and glass surface is an opaque matte wall (what a standard
                     occupancy map / Gazebo-style ray-cast model assumes)

Per beam (D6 + Thrun-style uniform "junk" component):
  z finite:  p = (1 - eps) P_D N(z; R_hat, sigma^2) + eps / r_max
  z = ∅:     p = (1 - eps) (1 - P_D) + eps_null
Without the mixture (eps = 0) a single mismatched beam can cost thousands of nats.
"""

import numpy as np

from ..sensor import expected_scan

LOG_FLOOR = np.log(1e-300)


def make_model(world, kind):
    if kind == "physics":
        return world
    if kind == "naive":
        return world.naive()
    raise ValueError(kind)


def expected(model_world, poses, params, beam_idx):
    """Noise-free expected ranges and detection probabilities, shape (P, nb)."""
    return expected_scan(model_world, poses, params, beam_idx)


def beam_log_likelihood(z, R_hat, P_D, sigma=0.05, eps=0.05, r_max=5.6, eps_null=0.01, use_mixture=True):
    """Per-beam log p(z_i | x) for every particle. z: (nb,), R_hat/P_D: (P, nb). Returns (P, nb)."""
    z = np.asarray(z, dtype=float)[None, :]
    finite = np.isfinite(z)
    R = np.where(np.isfinite(R_hat), R_hat, 1e6)
    with np.errstate(divide="ignore"):
        log_pd = np.log(P_D)
        log_miss = np.log1p(-np.clip(P_D, 0.0, 1.0 - 1e-16))
    log_gauss = -0.5 * ((np.nan_to_num(z) - R) / sigma) ** 2 - np.log(sigma * np.sqrt(2 * np.pi))
    log_hit = np.maximum(log_pd + log_gauss, LOG_FLOOR)
    log_miss = np.maximum(log_miss, LOG_FLOOR)
    if use_mixture:
        log_hit = np.logaddexp(np.log1p(-eps) + log_hit, np.log(eps / r_max))
        log_miss = np.logaddexp(np.log1p(-eps) + log_miss, np.log(eps_null))
    return np.where(finite, log_hit, log_miss)


def log_likelihood(z, R_hat, P_D, temper=1.0, **kw):
    """Scan log-likelihood (beams conditionally independent, D7), optionally tempered. Returns (P,)."""
    return temper * beam_log_likelihood(z, R_hat, P_D, **kw).sum(axis=1)


def mixture_gap(sigma=0.05, eps=0.05, r_max=5.6):
    """Largest possible per-beam log-likelihood difference with the mixture (D13d)."""
    peak = np.log((1 - eps) / (sigma * np.sqrt(2 * np.pi)) + eps / r_max)
    return peak - np.log(eps / r_max)
