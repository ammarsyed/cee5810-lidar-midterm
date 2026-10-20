"""Smoke tests: every module imports, and the scientific stack (incl. MP4 export, Embree) works."""

import importlib

import pytest

def _all_modules():
    """Every module in the three packages, discovered automatically."""
    import pkgutil

    import demo
    import experiments
    import lidar_sim

    names = []
    for pkg in (lidar_sim, experiments, demo):
        names.append(pkg.__name__)
        names += [m.name for m in pkgutil.walk_packages(pkg.__path__, pkg.__name__ + ".")]
    return names


MODULES = _all_modules()


@pytest.mark.parametrize("name", MODULES)
def test_module_imports(name):
    importlib.import_module(name)


def test_scientific_stack():
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    from scipy.special import lambertw

    # Lambert W is needed for the fog max-range formula (D5/D10): W(x) e^W(x) = x.
    w = lambertw(2.0).real
    assert np.isclose(w * np.exp(w), 2.0)

    fig, ax = plt.subplots()
    ax.plot([0, 1], [0, 1])
    plt.close(fig)


def test_video_export_available():
    import os

    import imageio_ffmpeg
    import tqdm  # noqa: F401  (Monte Carlo progress bars)

    # Bundled ffmpeg lets matplotlib write MP4s with no system install.
    assert os.path.exists(imageio_ffmpeg.get_ffmpeg_exe())


def test_embree_ray_casting():
    import numpy as np
    import trimesh
    from trimesh.ray.ray_pyembree import RayMeshIntersector

    # 8 x 6 x 3 m room; one horizontal ray from the centre at 0.6 m must hit the x = +4 wall.
    room = trimesh.creation.box(extents=(8, 6, 3))
    room.apply_translation((0, 0, 1.5))
    locs, ray_idx, _ = RayMeshIntersector(room).intersects_location(
        np.array([[0.0, 0.0, 0.6]]), np.array([[1.0, 0.0, 0.0]]), multiple_hits=False
    )
    assert list(ray_idx) == [0]
    assert np.isclose(locs[0][0], 4.0, atol=1e-4)
