"""Every experiment's run() works in fast mode for both people's configs (no files written)."""

import importlib

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import pytest  # noqa: E402

EXPERIMENTS = ["e1_sensor_position", "e2_sensor_heading", "e3_moving_target", "e4_reflectivity", "e5_mirror",
               "e6_glass", "e7_fog", "e8_noise_mc", "e_bn_cpt", "e12_3d_fov", "e13_2d3d_consistency", "e9_cw_phase", "e10_divergence"]


@pytest.mark.parametrize("person", ["ammar", "gal"])
@pytest.mark.parametrize("name", EXPERIMENTS)
def test_experiment_runs(name, person):
    mod = importlib.import_module(f"experiments.{name}")
    res = mod.run(person=person, fast=True)
    assert res["figs"], name
    for header, rows in res.get("tables", {}).values():
        assert rows and all(len(r) <= len(header) for r in rows)
    plt.close("all")


def test_localization_experiment_fast():
    mod = importlib.import_module("experiments.e11_localization")
    res = mod.run(person="ammar", fast=True)
    header, rows = res["tables"]["glass_corridor_summary"]
    rmse = {r[0]: r[1] for r in rows[:3]}
    assert rmse["P physics-aware"] < rmse["N1 naive"]  # the physics-aware filter beats the naive one
    plt.close("all")
