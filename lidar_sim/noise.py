"""Probabilistic measurement model: detection probability and range noise (D1, D6).

z_i = R_i + n_i with n_i ~ N(0, sigma_R^2) if the beam is detected (prob. P_D), else ∅ (NaN).
"""

import numpy as np
from scipy.special import ndtr

from . import config


def range_noise_std(sigma_t=config.SIGMA_T):
    """sigma_R = c sigma_t / 2 (D1)."""
    return config.C * sigma_t / 2.0


def detection_prob(p_norm, sigma_p=config.SIGMA_P_REL):
    """P_D = Phi((P_r - P_th) / sigma_P) with powers normalised by P_th (D6)."""
    p_norm = np.nan_to_num(np.asarray(p_norm, dtype=float), nan=0.0)
    return ndtr((p_norm - 1.0) / sigma_p)


def sample_measurement(true_range, p_detect, rng, sigma_r=config.SIGMA_R, r_min=config.R_MIN):
    """Draw z: detected with prob. P_D, then Gaussian range noise; NaN = no return (∅)."""
    true_range = np.asarray(true_range, dtype=float)
    detected = (rng.random(true_range.shape) < p_detect) & np.isfinite(true_range)
    z = true_range + rng.normal(0.0, sigma_r, size=true_range.shape)
    z = np.where(detected, np.maximum(z, r_min), np.nan)
    return z, detected
