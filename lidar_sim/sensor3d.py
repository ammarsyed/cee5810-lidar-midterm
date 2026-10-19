"""3D LiDAR: 16-ring geometry, Euler-angle ring directions, scan3d() (D14).

Ring k has elevation eps_k, column j has azimuth phi_j (body frame). World-frame direction:
    u_jk = R_z(yaw) R_y(tilt) R_x(0) [cos eps cos phi, cos eps sin phi, sin eps]
The sensor sits at height h above the robot's (x, y). Same tracer, power and noise model as 2D.
"""

from dataclasses import dataclass

import numpy as np

from . import config
from .geometry import rot3d
from .noise import detection_prob, sample_measurement
from .raytrace import Returns, select, trace
from .sensor import LidarParams


@dataclass(frozen=True)
class LidarParams3D(LidarParams):
    n_rings: int = config.N_RINGS
    elev_min_deg: float = config.ELEV_MIN_DEG
    elev_max_deg: float = config.ELEV_MAX_DEG
    az_step_deg: float = config.AZ_STEP_DEG
    height: float = config.SENSOR_HEIGHT
    tilt_deg: float = 0.0  # pitch of the mount; positive tilts the forward axis down
    elevations_override: tuple = None  # e.g. (0.0,) for the 2D-slice check

    def elevations(self):
        if self.elevations_override is not None:
            return np.radians(np.asarray(self.elevations_override, float))
        return np.radians(np.linspace(self.elev_min_deg, self.elev_max_deg, self.n_rings))

    def azimuths(self):
        return np.radians(np.arange(-180.0, 180.0, self.az_step_deg))


@dataclass
class ScanResult3D:
    pose: np.ndarray  # (x, y, yaw)
    origin: np.ndarray  # (x, y, h)
    elevations: np.ndarray
    azimuths: np.ndarray
    ranges: np.ndarray  # (n_rings, n_az), NaN = no return
    true_ranges: np.ndarray
    detected: np.ndarray
    power: np.ndarray
    p_detect: np.ndarray
    points_world: np.ndarray  # (n_rings, n_az, 3)
    true_hits: np.ndarray
    is_ghost: np.ndarray
    transmitted: np.ndarray
    surface_id: np.ndarray
    returns: Returns


def ring_directions(pose, params=LidarParams3D()):
    """World-frame unit directions, shape (n_rings, n_az, 3)."""
    E, A = np.meshgrid(params.elevations(), params.azimuths(), indexing="ij")
    b = np.stack([np.cos(E) * np.cos(A), np.cos(E) * np.sin(A), np.sin(E)], axis=-1)
    R = rot3d(yaw=pose[2], pitch=np.radians(params.tilt_deg))
    return b @ R.T


def scan3d(scene, pose, params=LidarParams3D(), rng=None, noise=True):
    pose = np.asarray(pose, dtype=float)
    origin = np.array([pose[0], pose[1], params.height])
    u = ring_directions(pose, params)
    shape = u.shape[:2]
    dirs = u.reshape(-1, 3)
    n = len(dirs)
    ret = trace(scene.backend(), np.tile(origin, (n, 1)), dirs, fog_alpha=scene.fog_alpha,
                max_bounces=params.max_bounces, k=params.k_over_pth, spec_gain=params.spec_gain,
                lobe_deg=params.spec_lobe_deg)
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
    if noise:
        rng = np.random.default_rng() if rng is None else rng
        z, det = sample_measurement(true_range, p_det, rng, params.sigma_r, params.r_min)
    else:
        det = p_det >= 0.5
        z = np.where(det, true_range, np.nan)
    pts = origin + z[:, None] * dirs
    return ScanResult3D(
        pose=pose, origin=origin, elevations=params.elevations(), azimuths=params.azimuths(),
        ranges=z.reshape(shape), true_ranges=true_range.reshape(shape), detected=det.reshape(shape),
        power=power.reshape(shape), p_detect=p_det.reshape(shape), points_world=pts.reshape(shape + (3,)),
        true_hits=pick(ret.hit_point, np.nan).reshape(shape + (3,)),
        is_ghost=(pick(ret.reflections, 0) > 0).reshape(shape),
        transmitted=(pick(ret.interactions - ret.reflections, 0) > 0).reshape(shape),
        surface_id=np.where(has, pick(ret.surface_id, -1), -1).reshape(shape), returns=ret)
