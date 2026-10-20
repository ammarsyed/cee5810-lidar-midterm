"""Scene geometry: Segment, Box, World, and moving targets (2D)."""

from collections.abc import Callable
from dataclasses import dataclass, field, replace

import numpy as np

from . import config
from .materials import GLASS, MIRROR, Material, MaterialArrays, matte


@dataclass
class Segment:
    a: np.ndarray
    b: np.ndarray
    material: Material
    name: str = ""

    def __post_init__(self):
        self.a = np.asarray(self.a, dtype=float)
        self.b = np.asarray(self.b, dtype=float)
        if not self.name:
            self.name = self.material.name


def box_segments(center, size, angle, material, name="box"):
    """The 4 sides of a rectangle with the given centre, (w, h) size and rotation [rad]."""
    w, h = size
    local = np.array([[-w / 2, -h / 2], [w / 2, -h / 2], [w / 2, h / 2], [-w / 2, h / 2]])
    c, s = np.cos(angle), np.sin(angle)
    corners = local @ np.array([[c, s], [-s, c]]) + np.asarray(center, dtype=float)
    return [Segment(corners[i], corners[(i + 1) % 4], material, name) for i in range(4)]


def ping_pong(p0, p1, speed, angle=0.0):
    """Trajectory moving back and forth between p0 and p1 at constant speed."""
    p0, p1 = np.asarray(p0, dtype=float), np.asarray(p1, dtype=float)
    length = np.linalg.norm(p1 - p0)
    period = 2.0 * length / speed

    def traj(t):
        phase = (t % period) / period
        frac = 2 * phase if phase < 0.5 else 2 - 2 * phase
        p = p0 + frac * (p1 - p0)
        return float(p[0]), float(p[1]), angle

    traj.period = period
    return traj


@dataclass
class MovingBox:
    size: tuple
    material: Material
    trajectory: Callable[[float], tuple]  # t -> (x, y, angle)
    name: str = "box"

    def segments_at(self, t):
        x, y, a = self.trajectory(t)
        return box_segments((x, y), self.size, a, self.material, self.name)


@dataclass
class World:
    segments: list
    movers: list = field(default_factory=list)
    fog_alpha: float = 0.0
    name: str = ""

    # --- time ----------------------------------------------------------------
    def at_time(self, t):
        """Static world with every moving target placed at time t (ids stay stable)."""
        segs = list(self.segments) + [s for m in self.movers for s in m.segments_at(t)]
        return World(segs, [], self.fog_alpha, self.name)

    def _static(self):
        return self.at_time(0.0) if self.movers else self

    # --- queries -----------------------------------------------------------------
    @property
    def materials(self):
        return [s.material for s in self._static().segments]

    def surface_ids(self, name):
        return [i for i, s in enumerate(self._static().segments) if s.name == name]

    def bounds(self, pad=0.0):
        pts = np.array([p for s in self._static().segments for p in (s.a, s.b)])
        return pts.min(axis=0) - pad, pts.max(axis=0) + pad

    def backend(self, opaque_only=False):
        """Segments2D backend. opaque_only drops glass (used for line-of-sight)."""
        from .backends.segments2d import Segments2D

        segs = self._static().segments
        mat = MaterialArrays.from_list([s.material for s in segs])
        ids = [i for i, s in enumerate(segs) if not (opaque_only and s.material.code == GLASS)]
        A = np.array([segs[i].a for i in ids]).reshape(-1, 2)
        B = np.array([segs[i].b for i in ids]).reshape(-1, 2)
        return Segments2D(A, B, mat, ids=ids)

    # --- variants ------------------------------------------------------------------
    def with_fog(self, alpha):
        return replace(self, fog_alpha=float(alpha))

    def naive(self, rho=config.RHO_WALL):
        """What a naive model assumes: every mirror and glass surface is an opaque matte wall."""

        def fix(seg):
            if seg.material.code in (MIRROR, GLASS):
                return Segment(seg.a, seg.b, matte(rho, name=seg.name), seg.name)
            return seg

        movers = [replace(m, material=matte(rho, m.name)) if m.material.code in (MIRROR, GLASS) else m
                  for m in self.movers]
        return World([fix(s) for s in self.segments], movers, self.fog_alpha, self.name + " (naive)")

    def has_specular(self):
        return any(m.code in (MIRROR, GLASS) for m in self.materials)
