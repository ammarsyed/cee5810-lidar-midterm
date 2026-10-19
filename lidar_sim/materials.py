"""Surface materials: matte (Lambertian), mirror, glass.

Mirrors and glass are modelled as two-sided thin surfaces (see DECISIONS_NEEDED.md).
"""

from dataclasses import dataclass

import numpy as np

from . import config

MATTE, MIRROR, GLASS = 0, 1, 2
KIND_CODES = {"matte": MATTE, "mirror": MIRROR, "glass": GLASS}


@dataclass(frozen=True)
class Material:
    kind: str  # "matte" | "mirror" | "glass"
    rho: float = config.RHO_WALL  # diffuse (Lambertian) reflectivity
    reflectance: float = config.R_MIRROR  # mirror only (R_m)
    n: float = config.N_GLASS  # glass only
    name: str = ""

    @property
    def code(self):
        return KIND_CODES[self.kind]


def matte(rho=config.RHO_WALL, name="wall"):
    return Material("matte", rho=rho, name=name)


def mirror(reflectance=config.R_MIRROR, rho=0.0, name="mirror"):
    return Material("mirror", rho=rho, reflectance=reflectance, name=name)


def glass(n=config.N_GLASS, rho=0.0, name="glass"):
    return Material("glass", rho=rho, n=n, name=name)


@dataclass
class MaterialArrays:
    """Per-surface material properties, indexed by surface id."""

    code: np.ndarray
    rho: np.ndarray
    reflectance: np.ndarray
    n: np.ndarray
    names: list

    @classmethod
    def from_list(cls, materials):
        return cls(
            code=np.array([m.code for m in materials], dtype=int),
            rho=np.array([m.rho for m in materials], dtype=float),
            reflectance=np.array([m.reflectance for m in materials], dtype=float),
            n=np.array([m.n for m in materials], dtype=float),
            names=[m.name for m in materials],
        )
