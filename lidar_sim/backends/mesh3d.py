"""3D backend: trimesh + Embree first-hit ray casting (D14c).

Embree computes the Moller-Trumbore intersections in float32, so hits are accurate to ~1e-5 m at
room scale; secondary rays are offset by config.EPS_SELF_3D.
"""

import numpy as np
import trimesh
from trimesh.ray.ray_pyembree import RayMeshIntersector

from .base import Hits


class Mesh3D:
    dim = 3

    def __init__(self, vertices, faces, tri_surface_id, mat):
        self.mesh = trimesh.Trimesh(vertices=np.asarray(vertices, float), faces=np.asarray(faces, int), process=False)
        self.tri_sid = np.asarray(tri_surface_id, dtype=int)
        self.mat = mat
        self.normals = np.asarray(self.mesh.face_normals, dtype=float)
        self.isect = RayMeshIntersector(self.mesh)

    def cast(self, origins, dirs):
        origins = np.asarray(origins, dtype=float).reshape(-1, 3)
        dirs = np.asarray(dirs, dtype=float).reshape(-1, 3)
        M = len(origins)
        t = np.full(M, np.inf)
        sid = np.full(M, -1, dtype=int)
        point = np.full((M, 3), np.nan)
        normal = np.full((M, 3), np.nan)
        if M == 0:
            return Hits(t, point, normal, sid)
        tri, ray, loc = self.isect.intersects_id(origins, dirs, multiple_hits=False, return_locations=True)
        if len(ray):
            tt = np.sum((loc - origins[ray]) * dirs[ray], axis=1)
            t[ray] = tt
            point[ray] = origins[ray] + tt[:, None] * dirs[ray]
            sid[ray] = self.tri_sid[tri]
            n = self.normals[tri].copy()
            flip = np.sum(n * dirs[ray], axis=1) > 0
            n[flip] *= -1.0
            normal[ray] = n
        return Hits(t, point, normal, sid)
