"""Demo tools: the slider app builds headless and responds to its controls; the room tour renders."""

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from demo.interactive import build, build_for, set_values  # noqa: E402


def test_build_and_fog_slider_reduces_detections():
    fig, c = build("default")
    clear = c["state"]["scan"].detected.sum()
    set_values(c, fog=0.4)
    foggy = c["state"]["scan"].detected.sum()
    assert foggy < clear
    plt.close(fig)


def test_moving_the_sensor_changes_the_scan_and_gal_builds():
    fig, c = build_for("ammar")  # starts at Ammar's tour pose
    a = c["state"]["scan"].ranges.copy()
    set_values(c, x=2.0, y=2.0)
    b = c["state"]["scan"].ranges
    assert not ((a == b) | (a != a) & (b != b)).all()
    plt.close(fig)
    fig, c = build_for("gal")
    assert c["state"]["scan"] is not None
    plt.close(fig)


def test_room_tour_snapshot_for_both_people():
    from demo import tour

    for person in ("ammar", "gal"):
        res = tour.snapshot(person)
        s = res["data"]["scan"]
        assert (s.detected & s.is_ghost).any(), person  # the tour pose is chosen to show mirror ghosts
        plt.close("all")
