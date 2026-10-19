"""Stage 0 smoke test: the environment and package skeleton import cleanly."""

import importlib

import pytest

MODULES = [
    "lidar_sim",
    "lidar_sim.config",
    "lidar_sim.geometry",
    "lidar_sim.materials",
    "lidar_sim.world",
    "lidar_sim.scenes",
    "lidar_sim.physics",
    "lidar_sim.raytrace",
    "lidar_sim.sensor",
    "lidar_sim.noise",
    "lidar_sim.viz",
    "experiments",
    "experiments.common",
]


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
