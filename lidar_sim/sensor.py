"""2D LiDAR sensor: parameters, pose and frames, FOV and LOS, scan() (D1, D3, D4, D6).

Poses are (x, y, theta) in the world frame. Beam i points at beta_i in the body frame,
u_i = Rot(theta) [cos beta_i, sin beta_i] in the world frame (D3a).
"""

from dataclasses import dataclass, replace

import numpy as np

from . import config
from .geometry import wrap_angle
from .noise import detection_prob, sample_measurement
from .raytrace import Returns, select, trace


@dataclass(frozen=True)
class LidarParams:
    fov_deg: float = config.FOV_DEG
    n_beams: int = config.N_BEAMS
    r_min: float = config.R_MIN
    sigma_t: float = config.SIGMA_T
    sigma_p_rel: float = config.SIGMA_P_REL
    max_bounces: int = config.MAX_BOUNCES
    return_policy: str = "strongest"  # or "first"
    spec_lobe_deg: float = config.SPEC_LOBE_DEG
    spec_gain: float = config.SPEC_GAIN
    k_over_pth: float = config.K_OVER_PTH
    r_max_ref: float = config.R_MAX_REF  # white-wall clear-air max range (for FOV drawing)

    @property
    def sigma_r(self):
        return config.C * self.sigma_t / 2.0

    def beam_angles(self):
        return np.radians(np.linspace(-self.fov_deg / 2, self.fov_deg / 2, self.n_beams))

    def with_(self, **kw):
        return replace(self, **kw)


@dataclass
class ScanResult:
    pose: np.ndarray
    beam_angles: np.ndarray
    ranges: np.ndarray  # z_i, NaN = ∅
    true_ranges: np.ndarray  # noise-free path length of the chosen return, NaN if none
    detected: np.ndarray
    power: np.ndarray  # normalised (P_r / P_th) power of the chosen return
    p_detect: np.ndarray
    points_world: np.ndarray  # where the LiDAR reports points (ghosts included)
    true_hits: np.ndarray  # where the chosen return really came from
    is_ghost: np.ndarray  # chosen return involved >= 1 mirror/glass reflection
    surface_id: np.ndarray  # surface of the chosen return (-1 if none)
    returns: Returns
    transmitted: np.ndarray = None  # chosen return passed through >= 1 glass pane


def _poses(poses):
    p = np.asarray(poses, dtype=float)
    return p[None, :] if p.ndim == 1 else p


def beam_directions(poses, params=None, beam_idx=None):
    """World-frame beam directions, shape (P, nb, 2) (or (nb, 2) for a single pose)."""
    params = params or LidarParams()
    single = np.asarray(poses).ndim == 1
    P = _poses(poses)
    beta = params.beam_angles() if beam_idx is None else params.beam_angles()[beam_idx]
    ang = P[:, 2:3] + beta[None, :]
    u = np.stack([np.cos(ang), np.sin(ang)], axis=-1)
    return u[0] if single else u


def to_world(ranges, pose, params=None, beam_idx=None):
    """Body-frame ranges -> world points q = p + z u (NaN ranges stay NaN)."""
    params = params or LidarParams()
    u = beam_directions(pose, params, beam_idx)
    return np.asarray(pose)[:2] + np.asarray(ranges)[:, None] * u


def chosen_returns(ret, n, params):
    """Per-beam chosen return (by policy) + detection probability, as flat arrays of length n.

    Shared by the 2D sensor (trace_beams) and the 3D sensor (sensor3d.scan3d).
    """
    ch = select(ret, n, params.return_policy)
    has = ch >= 0
    g = np.where(has, ch, 0)

    def pick(arr, fill):
        out = arr[g] if len(arr) else np.zeros((n,) + arr.shape[1:])
        return np.where(has.reshape((-1,) + (1,) * (out.ndim - 1)), out, fill)

    true_range = pick(ret.path_length, np.nan)
    power = pick(ret.power, 0.0)
    p_det = detection_prob(power, params.sigma_p_rel)
    p_det = np.where(np.isfinite(true_range) & (true_range >= params.r_min), p_det, 0.0)
    return {
        "true_range": true_range,
        "power": power,
        "p_detect": p_det,
        "true_hit": pick(ret.hit_point, np.nan),
        "is_ghost": pick(ret.reflections, 0) > 0,
        "transmitted": pick(ret.interactions - ret.reflections, 0) > 0,
        "surface_id": np.where(has, pick(ret.surface_id, -1), -1),
    }


def trace_beams(world, poses, params=None, beam_idx=None, t=0.0):
    """Noise-free per-beam result for one or many poses. Arrays have shape (P, nb)."""
    params = params or LidarParams()
    w = world.at_time(t) if world.movers else world
    P = _poses(poses)
    u = beam_directions(P, params, beam_idx)  # (P, nb, 2)
    nP, nb = u.shape[:2]
    ret = trace(w.backend(), np.repeat(P[:, :2], nb, axis=0), u.reshape(-1, 2), fog_alpha=w.fog_alpha,
                max_bounces=params.max_bounces, k=params.k_over_pth, spec_gain=params.spec_gain,
                lobe_deg=params.spec_lobe_deg)
    out = {k: v.reshape((nP, nb) + v.shape[1:]) for k, v in chosen_returns(ret, nP * nb, params).items()}
    out["dirs"] = u
    out["returns"] = ret
    return out


def scan(world, pose, params=None, rng=None, noise=True, t=0.0, beam_idx=None):
    """One LiDAR scan at `pose` (time t for moving targets).

    noise=True draws detections (prob. P_D) and Gaussian range noise (D6); noise=False reports
    every return with P_D >= 0.5 at its exact range.
    """
    params = params or LidarParams()
    pose = np.asarray(pose, dtype=float)
    r = trace_beams(world, pose, params, beam_idx, t)
    true_range, p_det = r["true_range"][0], r["p_detect"][0]
    if noise:
        rng = np.random.default_rng() if rng is None else rng
        z, det = sample_measurement(true_range, p_det, rng, params.sigma_r, params.r_min)
    else:
        det = p_det >= 0.5
        z = np.where(det, true_range, np.nan)
    u = r["dirs"][0]
    beta = params.beam_angles() if beam_idx is None else params.beam_angles()[beam_idx]
    return ScanResult(pose=pose, beam_angles=beta, ranges=z, true_ranges=true_range, detected=det,
                      power=r["power"][0], p_detect=p_det, points_world=pose[:2] + z[:, None] * u,
                      true_hits=r["true_hit"][0], is_ghost=r["is_ghost"][0], surface_id=r["surface_id"][0],
                      returns=r["returns"], transmitted=r["transmitted"][0])


def expected_scan(world, poses, params=None, beam_idx=None, t=0.0):
    """Noise-free expected ranges and P_D for many poses (used by the particle filter)."""
    params = params or LidarParams()
    r = trace_beams(world, poses, params, beam_idx, t)
    return r["true_range"], r["p_detect"]


# --- field of view & line of sight (D4) ---------------------------------------------------

def in_fov(points, pose, params=None, world=None, r_max=None, t=0.0):
    """Membership test q in S(x): range window, angular window, and (if world given) LOS.

    Glass is transparent for LOS; everything else is opaque.
    """
    params = params or LidarParams()
    pts = np.atleast_2d(np.asarray(points, dtype=float))
    pose = np.asarray(pose, dtype=float)
    r_max = params.r_max_ref if r_max is None else r_max
    v = pts - pose[:2]
    dist = np.linalg.norm(v, axis=1)
    ang = wrap_angle(np.arctan2(v[:, 1], v[:, 0]) - pose[2])
    ok = (dist >= params.r_min) & (dist <= r_max) & (np.abs(ang) <= np.radians(params.fov_deg) / 2 + 1e-12)
    if world is not None and ok.any():
        w = world.at_time(t) if world.movers else world
        dirs = v[ok] / dist[ok, None]
        h = w.backend(opaque_only=True).cast(np.tile(pose[:2], (ok.sum(), 1)), dirs)
        ok[np.flatnonzero(ok)] = h.t >= dist[ok] - 1e-6
    return ok


def visibility_polygon(pose, world, params=None, n_rays=1000, r_max=None, t=0.0):
    """Polygon of everything inside the FOV wedge with a clear line of sight (glass transparent)."""
    params = params or LidarParams()
    pose = np.asarray(pose, dtype=float)
    r_max = params.r_max_ref if r_max is None else r_max
    w = world.at_time(t) if world.movers else world
    ang = pose[2] + np.radians(np.linspace(-params.fov_deg / 2, params.fov_deg / 2, n_rays))
    dirs = np.stack([np.cos(ang), np.sin(ang)], axis=1)
    h = w.backend(opaque_only=True).cast(np.tile(pose[:2], (n_rays, 1)), dirs)
    reach = np.minimum(h.t, r_max)
    ends = pose[:2] + reach[:, None] * dirs
    return np.vstack([pose[:2], ends]) if params.fov_deg < 360 else ends
