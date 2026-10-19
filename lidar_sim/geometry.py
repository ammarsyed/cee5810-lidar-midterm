"""Vectors, rotations and intersection math (D3, D14).

Everything is vectorised: functions take (..., d) arrays and broadcast.
"""

import numpy as np


def rot2d(theta):
    """2D rotation matrix Rot(theta) (body -> world), D3(a)."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])


def rot3d(yaw=0.0, pitch=0.0, roll=0.0):
    """ZYX Euler rotation R_z(yaw) R_y(pitch) R_x(roll) (body -> world), D14(a).

    Positive pitch tilts the sensor's forward axis downward.
    """
    cz, sz = np.cos(yaw), np.sin(yaw)
    cy, sy = np.cos(pitch), np.sin(pitch)
    cx, sx = np.cos(roll), np.sin(roll)
    Rz = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]])
    Ry = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]])
    Rx = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]])
    return Rz @ Ry @ Rx


def cross2(a, b):
    """2D cross product a x b = a_x b_y - a_y b_x (broadcasts over leading axes)."""
    return a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]


def wrap_angle(a):
    """Wrap angles to [-pi, pi)."""
    return (np.asarray(a) + np.pi) % (2.0 * np.pi) - np.pi


def unit(v):
    """Normalise vectors along the last axis."""
    v = np.asarray(v, dtype=float)
    return v / np.linalg.norm(v, axis=-1, keepdims=True)


def ray_segment_intersect(origins, dirs, A, B, t_min=1e-9):
    """Ray-segment intersection for every (ray, segment) pair, D3(c).

    Ray p + t u, segment a + s (b - a). With e = b - a:
        t = ((a - p) x e) / (u x e),   s = ((a - p) x u) / (u x e)
    hit iff u x e != 0, t > t_min, 0 <= s <= 1.

    Returns (t, s), each of shape (M, S); t is +inf where there is no hit.
    """
    origins = np.asarray(origins, dtype=float)
    dirs = np.asarray(dirs, dtype=float)
    E = B - A  # (S, 2)
    denom = cross2(dirs[:, None, :], E[None, :, :])  # (M, S)
    AO = A[None, :, :] - origins[:, None, :]  # (M, S, 2)
    with np.errstate(divide="ignore", invalid="ignore"):
        t = cross2(AO, E[None, :, :]) / denom
        s = cross2(AO, dirs[:, None, :]) / denom
    ok = (np.abs(denom) > 1e-12) & (t > t_min) & (s >= -1e-9) & (s <= 1.0 + 1e-9)
    return np.where(ok, t, np.inf), s


def ray_triangle_intersect(origins, dirs, v0, v1, v2, t_min=1e-9):
    """Moller-Trumbore ray-triangle intersection for every (ray, triangle) pair, D14(c).

    Solves o + t d = (1 - u - v) v0 + u v1 + v v2 by Cramer's rule.
    Returns t of shape (M, T), +inf where there is no hit.
    """
    o = np.asarray(origins, dtype=float)[:, None, :]
    d = np.asarray(dirs, dtype=float)[:, None, :]
    e1 = (v1 - v0)[None]
    e2 = (v2 - v0)[None]
    P = np.cross(d, e2)
    det = np.sum(e1 * P, axis=-1)
    T = o - v0[None]
    Q = np.cross(T, e1)
    with np.errstate(divide="ignore", invalid="ignore"):
        inv = 1.0 / det
        u = np.sum(T * P, axis=-1) * inv
        v = np.sum(d * Q, axis=-1) * inv
        t = np.sum(e2 * Q, axis=-1) * inv
    ok = (np.abs(det) > 1e-12) & (u >= 0) & (v >= 0) & (u + v <= 1) & (t > t_min)
    return np.where(ok, t, np.inf)


def reflect_point(y, m, n_hat):
    """Mirror image of point(s) y across the line/plane through m with unit normal n_hat (D8)."""
    y = np.asarray(y, dtype=float)
    return y - 2.0 * np.sum((y - m) * n_hat, axis=-1, keepdims=True) * n_hat


def segment_normal(a, b):
    """Unit normal of segment a->b (rotated +90 deg)."""
    e = np.asarray(b, dtype=float) - np.asarray(a, dtype=float)
    n = np.array([-e[..., 1], e[..., 0]]).T if e.ndim > 1 else np.array([-e[1], e[0]])
    return n / np.linalg.norm(n, axis=-1, keepdims=True)
