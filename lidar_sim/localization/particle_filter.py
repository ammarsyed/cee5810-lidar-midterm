"""Particle filter (Monte Carlo localization): predict, weight, low-variance resample, estimate (D13c).

bel(x_k) ≈ sum_m w_m δ(x - x_m).  Predict with the unicycle motion model, weight by the scan
likelihood p(z_k | x_m), resample with the low-variance scheme when N_eff < M/2, then roughen.
"""

from dataclasses import dataclass, field

import numpy as np
from scipy.special import logsumexp

from ..geometry import wrap_angle
from ..sensor import LidarParams
from .likelihood import expected, log_likelihood
from .motion import sample_motion


def low_variance_resample(weights, rng):
    """Systematic resampling: one random offset, M evenly spaced pointers into the CDF."""
    M = len(weights)
    positions = (rng.random() + np.arange(M)) / M
    cdf = np.cumsum(weights)
    cdf[-1] = 1.0
    return np.searchsorted(cdf, positions)


def circular_mean(theta, w):
    return float(np.arctan2(np.sum(w * np.sin(theta)), np.sum(w * np.cos(theta))))


@dataclass
class ParticleFilter:
    model_world: object  # the world the filter *believes* in (naive or physics)
    particles: np.ndarray  # (M, 3)
    rng: np.random.Generator
    params: LidarParams = field(default_factory=LidarParams)
    beam_idx: np.ndarray = None
    sigma_v: float = 0.06
    sigma_w: float = 0.06
    lik: dict = field(default_factory=dict)  # kwargs for log_likelihood (sigma, eps, use_mixture, temper...)
    roughen: tuple = (0.01, 0.01, np.radians(0.5))
    resample_frac: float = 0.5

    def __post_init__(self):
        self.particles = np.array(self.particles, dtype=float)
        self.logw = np.full(len(self.particles), -np.log(len(self.particles)))
        self.n_eff_history = []

    @property
    def weights(self):
        return np.exp(self.logw - logsumexp(self.logw))

    @property
    def n_eff(self):
        w = self.weights
        return float(1.0 / np.sum(w * w))

    def predict(self, u, dt):
        self.particles = sample_motion(self.particles, u, dt, self.rng, self.sigma_v, self.sigma_w)

    def update(self, z):
        """z: measured ranges for the beams in beam_idx (NaN = no return)."""
        R_hat, P_D = expected(self.model_world, self.particles, self.params, self.beam_idx)
        self.logw = self.logw + log_likelihood(z, R_hat, P_D, **self.lik)
        self.logw -= logsumexp(self.logw)
        self.n_eff_history.append(self.n_eff)
        if self.n_eff < self.resample_frac * len(self.particles):
            self.resample()

    def resample(self):
        idx = low_variance_resample(self.weights, self.rng)
        M = len(idx)
        self.particles = self.particles[idx] + self.rng.normal(0.0, 1.0, (M, 3)) * np.asarray(self.roughen)
        self.particles[:, 2] = wrap_angle(self.particles[:, 2])
        self.logw = np.full(M, -np.log(M))

    def estimate(self):
        w = self.weights
        return np.array([np.sum(w * self.particles[:, 0]), np.sum(w * self.particles[:, 1]),
                         circular_mean(self.particles[:, 2], w)])


def init_particles(mean, std, M, rng):
    P = np.asarray(mean, float) + rng.normal(0.0, 1.0, (M, 3)) * np.asarray(std, float)
    P[:, 2] = wrap_angle(P[:, 2])
    return P
