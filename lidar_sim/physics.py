"""Light physics: reflection, Fresnel, received power, fog, max range (D5, D8, D9, D10).

All functions are dimension-agnostic and vectorised. Powers are normalised by the
detection threshold P_th (see config.py).
"""

import numpy as np
from scipy.special import lambertw

from . import config


def reflect(u, n_hat):
    """Law of reflection u' = u - 2 (u.n) n (D8a). Works in 2D and 3D."""
    return u - 2.0 * np.sum(u * n_hat, axis=-1, keepdims=True) * n_hat


def fresnel(cos_i, n1=1.0, n2=config.N_GLASS):
    """Unpolarised Fresnel reflectance R_F = (r_s^2 + r_p^2)/2 (D9b).

    cos_i is the cosine of the incidence angle. Total internal reflection gives R_F = 1.
    """
    cos_i = np.clip(np.asarray(cos_i, dtype=float), 0.0, 1.0)
    n1 = np.asarray(n1, dtype=float)
    n2 = np.asarray(n2, dtype=float)
    sin_t = n1 / n2 * np.sqrt(1.0 - cos_i**2)
    tir = sin_t >= 1.0
    cos_t = np.sqrt(np.clip(1.0 - sin_t**2, 0.0, 1.0))
    with np.errstate(divide="ignore", invalid="ignore"):
        rs = (n1 * cos_i - n2 * cos_t) / (n1 * cos_i + n2 * cos_t)
        rp = (n1 * cos_t - n2 * cos_i) / (n1 * cos_t + n2 * cos_i)
    R = 0.5 * (rs**2 + rp**2)
    R = np.where(tir, 1.0, R)
    return np.where(cos_i == 0.0, 1.0, R)  # grazing incidence reflects everything


def fresnel_deg(theta_deg, n1=1.0, n2=config.N_GLASS):
    return fresnel(np.cos(np.radians(theta_deg)), n1, n2)


def pane_transmission(R_F, incoherent=False):
    """Single-pass transmission through a thin pane: (1-R)^2, or (1-R)/(1+R) with
    incoherent internal bounces (D9c)."""
    R_F = np.asarray(R_F, dtype=float)
    return (1.0 - R_F) / (1.0 + R_F) if incoherent else (1.0 - R_F) ** 2


def fog_transmission(L, alpha):
    """Two-way Beer-Lambert loss e^(-2 alpha L) (D10)."""
    return np.exp(-2.0 * alpha * np.asarray(L, dtype=float))


def diffuse_power(L, rho, cos_inc, T=1.0, alpha=0.0, k=config.K_OVER_PTH):
    """Normalised received power from a Lambertian surface (D5):
    p = (K/P_th) rho cos(theta) T^2 e^(-2 alpha L) / L^2, with T the one-way throughput."""
    L = np.asarray(L, dtype=float)
    return k * rho * cos_inc * np.square(T) * fog_transmission(L, alpha) / L**2


def specular_power(L, R_s, theta_inc, T=1.0, alpha=0.0, k=config.K_OVER_PTH,
                   gain=config.SPEC_GAIN, lobe_deg=config.SPEC_LOBE_DEG):
    """Normalised power of a head-on specular return (D5 specular lobe, modelling assumption):
    p = (K_s/P_th) R_s exp(-theta^2 / (2 theta_w^2)) T^2 e^(-2 alpha L) / L^2."""
    L = np.asarray(L, dtype=float)
    w = np.radians(lobe_deg)
    lobe = np.exp(-np.square(theta_inc) / (2.0 * w * w))
    return k * gain * R_s * lobe * np.square(T) * fog_transmission(L, alpha) / L**2


def clear_max_range(rho, theta_inc=0.0, T_path=1.0, k=config.K_OVER_PTH):
    """R0 = sqrt(K rho cos(theta) T_path / P_th) (D5). T_path is the round-trip factor."""
    return np.sqrt(k * rho * np.cos(theta_inc) * T_path)


def max_range(rho, theta_inc=0.0, alpha=0.0, T_path=1.0, k=config.K_OVER_PTH):
    """Range where p = 1 (P_D = 0.5): R0 in clear air, W0(alpha R0)/alpha in fog (D5, D10)."""
    R0 = clear_max_range(rho, theta_inc, T_path, k)
    alpha = np.asarray(alpha, dtype=float)
    with np.errstate(divide="ignore", invalid="ignore"):
        fog = np.real(lambertw(alpha * R0, 0)) / alpha
    return np.where(alpha > 0, fog, R0)


def dmax_range_dalpha(rho, alpha, theta_inc=0.0, T_path=1.0, k=config.K_OVER_PTH):
    """dR_max/d alpha = -R_max^2 / (1 + alpha R_max) (D11)."""
    R = max_range(rho, theta_inc, alpha, T_path, k)
    return -(R**2) / (1.0 + alpha * R)
