"""3D scenes: extrude a 2D world into walls, add floor/ceiling, a low box and a table (D14).

Every 2D segment becomes a vertical quad (2 triangles) with the same surface id and material, so
the horizontal (elevation 0) ring of the 3D sensor reproduces the 2D sensor exactly (D14e).
"""

from dataclasses import dataclass, field, replace

import numpy as np

from .materials import MaterialArrays, matte
from .scenes import default_room, variant_b


@dataclass
class Scene3D:
    vertices: list = field(default_factory=list)
    faces: list = field(default_factory=list)
    tri_sid: list = field(default_factory=list)
    materials: list = field(default_factory=list)  # indexed by surface id
    names: list = field(default_factory=list)  # indexed by surface id
    fog_alpha: float = 0.0
    world2d: object = None  # the 2D world it was extruded from (for drawing)
    name: str = ""
    footprints: list = field(default_factory=list)  # (name, 4x2 corners) of 3D-only objects

    def new_surface(self, material, name):
        self.materials.append(material)
        self.names.append(name)
        return len(self.materials) - 1

    def add_quad(self, p0, p1, p2, p3, sid):
        base = len(self.vertices)
        self.vertices += [p0, p1, p2, p3]
        self.faces += [[base, base + 1, base + 2], [base, base + 2, base + 3]]
        self.tri_sid += [sid, sid]

    def add_box(self, center, size, yaw, material, name, z0=0.0):
        """Axis-aligned-in-yaw box: centre (x, y), size (w, d, h), bottom at z0."""
        sid = self.new_surface(material, name)
        w, d, h = size
        c, s = np.cos(yaw), np.sin(yaw)
        local = np.array([[-w / 2, -d / 2], [w / 2, -d / 2], [w / 2, d / 2], [-w / 2, d / 2]])
        xy = local @ np.array([[c, s], [-s, c]]) + np.asarray(center, float)
        lo = [(*p, z0) for p in xy]
        hi = [(*p, z0 + h) for p in xy]
        for i in range(4):
            j = (i + 1) % 4
            self.add_quad(lo[i], lo[j], hi[j], hi[i], sid)
        self.add_quad(*lo, sid)
        self.add_quad(*hi, sid)
        self.footprints.append((name, xy))
        return sid

    def surface_ids(self, name):
        return [i for i, n in enumerate(self.names) if n == name]

    def with_fog(self, alpha):
        out = replace(self, fog_alpha=float(alpha))
        out._backend = getattr(self, "_backend", None)
        return out

    def backend(self):
        be = getattr(self, "_backend", None)
        if be is None:
            from .backends.mesh3d import Mesh3D

            be = Mesh3D(self.vertices, self.faces, self.tri_sid, MaterialArrays.from_list(self.materials))
            self._backend = be
        return be


def extrude(world2d, height=3.0, floor_rho=0.5, ceiling_rho=0.7, floor=True):
    w = world2d.at_time(0.0) if world2d.movers else world2d
    sc = Scene3D(fog_alpha=w.fog_alpha, world2d=w, name=w.name + "_3d")
    for seg in w.segments:
        sid = sc.new_surface(seg.material, seg.name)
        a, b = seg.a, seg.b
        sc.add_quad((a[0], a[1], 0.0), (b[0], b[1], 0.0), (b[0], b[1], height), (a[0], a[1], height), sid)
    if floor:
        lo, hi = w.bounds()
        f = sc.new_surface(matte(floor_rho, "floor"), "floor")
        sc.add_quad((lo[0], lo[1], 0.0), (hi[0], lo[1], 0.0), (hi[0], hi[1], 0.0), (lo[0], hi[1], 0.0), f)
        c = sc.new_surface(matte(ceiling_rho, "ceiling"), "ceiling")
        sc.add_quad((lo[0], lo[1], height), (hi[0], lo[1], height), (hi[0], hi[1], height), (lo[0], hi[1], height), c)
    return sc


def add_table(sc, center, top_size=(1.0, 0.6), top_z=0.75, thickness=0.04, leg=0.05, rho=0.6, yaw=0.0):
    """Table: a thin top at top_z plus four legs (the 2D scan plane at 0.6 m only sees the legs)."""
    m = matte(rho, "table")
    sc.add_box(center, (*top_size, thickness), yaw, m, "table", z0=top_z - thickness)
    w, d = top_size
    c, s = np.cos(yaw), np.sin(yaw)
    for dx, dy in [(-1, -1), (1, -1), (1, 1), (-1, 1)]:
        off = np.array([dx * (w / 2 - leg), dy * (d / 2 - leg)]) @ np.array([[c, s], [-s, c]])
        sc.add_box(np.asarray(center) + off, (leg, leg, top_z - thickness), yaw, m, "table")
    return sc


def default_room_3d(height=3.0, fog_alpha=0.0):
    """Default room extruded to 3 m, plus a 0.3 m low box and a table (top at 0.75 m)."""
    sc = extrude(default_room(with_moving_box=False, fog_alpha=fog_alpha), height)
    sc.add_box((3.2, 2.0), (0.6, 0.6, 0.3), 0.0, matte(0.6, "low box"), "low box")
    add_table(sc, (3.0, 4.2))
    return sc


def variant_b_3d(height=3.0, fog_alpha=0.0):
    sc = extrude(variant_b(with_moving_box=False, fog_alpha=fog_alpha), height)
    sc.add_box((4.0, 4.4), (0.7, 0.5, 0.3), 0.4, matte(0.6, "low box"), "low box")
    add_table(sc, (6.8, 4.6), yaw=0.3)
    return sc


SCENES3D = {"default": default_room_3d, "variant_b": variant_b_3d}


def get_scene3d(name="default", **kw):
    return SCENES3D[name](**kw)
