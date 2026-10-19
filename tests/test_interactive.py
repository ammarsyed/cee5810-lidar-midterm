"""Stage 9a: the slider demo builds headless and responds to its controls."""

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from experiments.interactive import build, set_values  # noqa: E402


def test_build_and_fog_slider_reduces_detections():
    fig, c = build("default")
    clear = c["state"]["scan"].detected.sum()
    set_values(c, fog=0.4)
    foggy = c["state"]["scan"].detected.sum()
    assert foggy < clear
    plt.close(fig)


def test_moving_the_sensor_changes_the_scan_and_variant_b_builds():
    fig, c = build("default")
    a = c["state"]["scan"].ranges.copy()
    set_values(c, x=2.0, y=2.0)
    b = c["state"]["scan"].ranges
    assert not ((a == b) | (a != a) & (b != b)).all()
    plt.close(fig)
    fig, c = build("variant_b")
    assert c["state"]["scan"] is not None
    plt.close(fig)
