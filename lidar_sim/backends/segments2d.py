"""2D backend: vectorised numpy ray-segment intersection (D3c)."""

import numpy as np

from ..geometry import ray_segment_intersect
from ..materials import MaterialArrays
from .base import Hits


class Segments2D:
    """First-hit queries against a set of 2D segments.

    `ids` maps local segment index -> global surface id, so a filtered backend
    (e.g. opaque surfaces only, for LOS) still reports the world's surface ids.
    `mat` is indexed by *global* surface id.
    """

    dim = 2

    def __init__(self, A, B, mat: MaterialArrays, ids=None, max_pairs=2_000_000):
        self.A = np.asarray(A, dtype=float).reshape(-1, 2)
        self.B = np.asarray(B, dtype=float).reshape(-1, 2)
        self.ids = np.arange(len(self.A)) if ids is None else np.asarray(ids, dtype=int)
        self.mat = mat
        E = self.B - self.A
        self.normals = np.stack([-E[:, 1], E[:, 0]], axis=1)
        self.normals /= np.linalg.norm(self.normals, axis=1, keepdims=True)
        self.max_pairs = max_pairs

    def cast(self, origins, dirs, t_min=1e-9):
        origins = np.asarray(origins, dtype=float).reshape(-1, 2)
        dirs = np.asarray(dirs, dtype=float).reshape(-1, 2)
        M, S = len(origins), len(self.A)
        t_out = np.full(M, np.inf)
        sid = np.full(M, -1, dtype=int)
        normal = np.full((M, 2), np.nan)
        if S == 0 or M == 0:
            return Hits(t_out, np.full((M, 2), np.nan), normal, sid)
        rows = max(1, self.max_pairs // S)
        for start in range(0, M, rows):
            sl = slice(start, min(M, start + rows))
            t, _ = ray_segment_intersect(origins[sl], dirs[sl], self.A, self.B, t_min)
            idx = np.argmin(t, axis=1)
            tt = t[np.arange(len(idx)), idx]
            hit = np.isfinite(tt)
            t_out[sl] = tt
            sid[sl] = np.where(hit, self.ids[idx], -1)
            n = self.normals[idx]
            flip = np.sum(n * dirs[sl], axis=1) > 0
            n[flip] *= -1.0
            n[~hit] = np.nan
            normal[sl] = n
        point = origins + np.where(np.isfinite(t_out), t_out, np.nan)[:, None] * dirs
        return Hits(t_out, point, normal, sid)
