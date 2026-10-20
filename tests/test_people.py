"""Person configs (people/*.toml): both exist, stay in sync, and actually differ."""

import pytest

from experiments.common import load_person, people


def test_both_people_exist():
    assert {"ammar", "gal"} <= set(people())


def test_configs_have_the_same_structure():
    a, g = load_person("ammar"), load_person("gal")
    assert set(a) == set(g)
    for key, val in a.items():
        if isinstance(val, dict):
            assert set(val) == set(g[key]), key  # same experiment parameters on both sides


def test_configs_differ_where_it_matters():
    a, g = load_person("ammar"), load_person("gal")
    assert a["scene"] != g["scene"] and a["seed"] != g["seed"] and a["out"] != g["out"]


def test_unknown_person_is_a_clear_error():
    with pytest.raises(SystemExit, match="Unknown person"):
        load_person("bob")
