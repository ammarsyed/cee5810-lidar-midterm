"""Hits dataclass and the SceneBackend protocol every backend implements."""

from dataclasses import dataclass
from typing import Protocol

import numpy as np

from ..materials import MaterialArrays


@dataclass
class Hits:
    """First-hit results, one entry per input ray."""

    t: np.ndarray  # (M,) distance to first hit, +inf if none
    point: np.ndarray  # (M, d) hit point, NaN if none
    normal: np.ndarray  # (M, d) unit normal flipped so dot(dir, normal) <= 0
    surface_id: np.ndarray  # (M,) global surface id, -1 if none

    @property
    def hit(self):
        return self.surface_id >= 0


class SceneBackend(Protocol):
    dim: int
    mat: MaterialArrays  # indexed by global surface id

    def cast(self, origins: np.ndarray, dirs: np.ndarray) -> Hits: ...
