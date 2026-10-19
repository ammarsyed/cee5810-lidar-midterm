"""Every experiment's run() works in fast mode for both teammates' scenes (no files written)."""

import importlib

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import pytest  # noqa: E402

EXPERIMENTS = ["e1_sensor_position", "e2_sensor_heading", "e3_moving_target", "e4_reflectivity", "e5_mirror",
               "e6_glass", "e7_fog", "e8_noise_mc", "e_bn_cpt"]


@pytest.mark.parametrize("scene", ["default", "variant_b"])
@pytest.mark.parametrize("name", EXPERIMENTS)
def test_experiment_runs(name, scene):
    mod = importlib.import_module(f"experiments.{name}")
    res = mod.run(scene=scene, seed=0, fast=True)
    assert res["figs"], name
    for header, rows in res.get("tables", {}).values():
        assert rows and all(len(r) <= len(header) for r in rows)
    plt.close("all")
