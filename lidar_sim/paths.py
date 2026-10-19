"""Sensor/robot paths: constant-speed waypoint paths and a pure-pursuit unicycle driver.

The pure-pursuit driver produces poses AND the controls (v, w) that generate them through the
unicycle model, so localization (Stage 7) gets perfectly consistent ground truth + odometry.
"""

import numpy as np

from .geometry import wrap_angle
from .localization.motion import unicycle_step


def waypoint_path(waypoints, speed=0.5, dt=0.1, heading="follow_path", theta0=0.0, spin_rate=0.6):
    """Constant-speed piecewise-linear path through the waypoints.

    heading: "follow_path" (face along the segment), "fixed" (always theta0) or "spin" (theta0 + spin_rate t).
    Returns (times (K,), poses (K, 3)).
    """
    wp = np.asarray(waypoints, dtype=float)
    seg = np.diff(wp, axis=0)
    seg_len = np.linalg.norm(seg, axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg_len)])
    times = np.arange(0.0, cum[-1] / speed + 1e-9, dt)
    s = np.minimum(times * speed, cum[-1])
    i = np.clip(np.searchsorted(cum, s, side="right") - 1, 0, len(seg) - 1)
    frac = (s - cum[i]) / seg_len[i]
    xy = wp[i] + frac[:, None] * seg[i]
    if heading == "follow_path":
        th = np.arctan2(seg[i, 1], seg[i, 0])
    elif heading == "fixed":
        th = np.full(len(times), theta0)
    elif heading == "spin":
        th = wrap_angle(theta0 + spin_rate * times)
    else:
        raise ValueError(heading)
    return times, np.column_stack([xy, th])


def pure_pursuit(waypoints, start_pose=None, v=0.5, dt=0.1, k_heading=2.5, w_max=1.2, reach=0.35,
                 max_steps=5000):
    """Drive a unicycle through the waypoints in order (steer toward the next one).

    Returns (times (K+1,), poses (K+1, 3), controls (K, 2)) with poses[k+1] = unicycle_step(poses[k], controls[k]).
    """
    wp = np.asarray(waypoints, dtype=float)
    if start_pose is None:
        d = wp[1] - wp[0]
        start_pose = (wp[0, 0], wp[0, 1], np.arctan2(d[1], d[0]))
        target = 1
    else:
        target = 0
    poses = [np.asarray(start_pose, dtype=float)]
    controls = []
    while target < len(wp) and len(controls) < max_steps:
        x = poses[-1]
        to = wp[target] - x[:2]
        if np.linalg.norm(to) < reach:
            target += 1
            continue
        err = wrap_angle(np.arctan2(to[1], to[0]) - x[2])
        w = float(np.clip(k_heading * err, -w_max, w_max))
        vv = v * max(0.2, np.cos(err))  # slow down in sharp turns
        controls.append((vv, w))
        poses.append(unicycle_step(x, (vv, w), dt))
    poses = np.array(poses)
    controls = np.array(controls).reshape(-1, 2)
    return np.arange(len(poses)) * dt, poses, controls
