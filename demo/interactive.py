"""Interactive slider demo: drag the sensor, fog, mirror and box; watch the scan update.

Desktop window:   uv run python -m demo.interactive [--scene variant_b]
Record a video:   uv run python -m demo.interactive --record figures/ammar/demo.mp4
In the notebook:  %matplotlib widget  then  from demo.interactive import launch; launch()
"""

import argparse

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Button, CheckButtons, RadioButtons, Slider

from lidar_sim import viz
from lidar_sim.scenes import SCENES
from lidar_sim.sensor import LidarParams, scan, visibility_polygon

DEFAULT_POSE = {"default": (4.6, 3.2, 14.0), "variant_b": (2.6, 3.0, 60.0), "glass_corridor": (3.0, 1.2, 0.0)}
DEFAULT_MIRROR = {"default": 45.0, "variant_b": 135.0}


def build(scene="default", params=None, seed=0):
    """Create the demo figure. Returns (fig, controls); controls["state"]["scan"] is the latest scan."""
    params = params or LidarParams()
    builder = SCENES[scene]
    probe = builder()
    lo, hi = probe.bounds()
    box = probe.movers[0] if probe.movers else None
    period = float(getattr(box.trajectory, "period", 16.0)) if box else 16.0
    x0, y0, th0 = DEFAULT_POSE.get(scene, (2.0, 2.0, 0.0))

    fig = plt.figure(figsize=(12.5, 7.2))
    ax = fig.add_axes([0.04, 0.34, 0.50, 0.62])
    axp = fig.add_axes([0.60, 0.40, 0.34, 0.52], projection="polar")
    readout = fig.text(0.60, 0.355, "", fontsize=9.5, color=viz.INK)

    def sax(row):
        return fig.add_axes([0.10, 0.25 - 0.035 * row, 0.40, 0.022])

    sl = {
        "x": Slider(sax(0), "sensor x [m]", lo[0] + 0.2, hi[0] - 0.2, valinit=x0),
        "y": Slider(sax(1), "sensor y [m]", lo[1] + 0.2, hi[1] - 0.2, valinit=y0),
        "theta": Slider(sax(2), "heading θ [deg]", -180, 180, valinit=th0),
        "fog": Slider(sax(3), "fog α [1/m]", 0.0, 0.5, valinit=0.0),
        "mirror": Slider(sax(4), "mirror angle [deg]", 0, 180, valinit=DEFAULT_MIRROR.get(scene, 45.0)),
        "box": Slider(sax(5), "box position", 0.0, 1.0, valinit=0.0),
        "rho": Slider(sax(6), "box reflectivity ρ", 0.05, 1.0, valinit=0.5),
    }
    radio = RadioButtons(fig.add_axes([0.60, 0.10, 0.12, 0.10]), ("strongest", "first"))
    fig.text(0.60, 0.205, "return policy", fontsize=8.5, color=viz.INK2)
    checks = CheckButtons(fig.add_axes([0.75, 0.06, 0.13, 0.17]),
                          ("rays", "FOV", "visible region", "ghost sources", "noise"), (True, True, True, True, True))
    button = Button(fig.add_axes([0.89, 0.10, 0.09, 0.05]), "new noise")
    state = {"seed": seed, "batch": False, "scan": None}
    has_mirror = any(s.name == "mirror" for s in probe.segments)

    def world_now():
        kw = dict(fog_alpha=sl["fog"].val, box_rho=sl["rho"].val) if box else dict(fog_alpha=sl["fog"].val)
        if has_mirror and scene in DEFAULT_MIRROR:
            kw["mirror_angle_deg"] = sl["mirror"].val
        return builder(**kw)

    def redraw(_=None):
        if state["batch"]:
            return
        on = dict(zip(("rays", "FOV", "visible region", "ghost sources", "noise"), checks.get_status()))
        w = world_now()
        t = sl["box"].val * period / 2
        pose = np.array([sl["x"].val, sl["y"].val, np.radians(sl["theta"].val)])
        p = params.with_(return_policy=radio.value_selected)
        s = scan(w, pose, p, rng=np.random.default_rng(state["seed"]), noise=on["noise"], t=t)
        state["scan"] = s
        ax.clear()
        axp.clear()
        if on["visible region"]:
            viz.draw_visibility(ax, visibility_polygon(pose, w, p, t=t, n_rays=600))
        if on["FOV"]:
            viz.draw_fov(ax, pose, p.fov_deg, p.r_max_ref)
        viz.draw_world(ax, w, t=t)
        viz.draw_scan(ax, s, show_rays=on["rays"], show_true=on["ghost sources"], show_dropouts=on["rays"])
        lo2, hi2 = w.bounds(0.3)
        ax.set_xlim(lo2[0], hi2[0] + 2.5)
        ax.set_ylim(lo2[1], hi2[1])
        ax.legend(loc="upper right", fontsize=7.5)
        viz.polar_ranges(axp, s, r_max=6.0)
        n_ghost = int((s.detected & s.is_ghost).sum())
        nearest = np.nanmin(s.ranges) if s.detected.any() else np.nan
        readout.set_text(f"detected {int(s.detected.sum())}/{len(s.detected)}   dropouts {int((~s.detected).sum())}   "
                         f"ghosts {n_ghost}   through glass {int((s.detected & s.transmitted).sum())}   "
                         f"nearest {nearest:.2f} m")
        fig.canvas.draw_idle()

    for s_ in sl.values():
        s_.on_changed(redraw)
    radio.on_clicked(redraw)
    checks.on_clicked(redraw)

    def new_noise(_):
        state["seed"] += 1
        redraw()

    button.on_clicked(new_noise)
    redraw()
    return fig, {"sliders": sl, "radio": radio, "checks": checks, "button": button, "state": state, "redraw": redraw}


def set_values(controls, **vals):
    """Set several sliders at once with a single redraw."""
    controls["state"]["batch"] = True
    for k, v in vals.items():
        controls["sliders"][k].set_val(v)
    controls["state"]["batch"] = False
    controls["redraw"]()


def launch(scene="default", **kw):
    fig, controls = build(scene, **kw)
    plt.show()
    return fig, controls


def record(path, scene="default", n=120, fps=15):
    """Scripted sweep -> video: drive the sensor, then thicken the fog, then rotate the mirror."""
    fig, c = build(scene)
    sl = c["sliders"]
    x0, x1 = sl["x"].valmin + 1.0, min(sl["x"].valmax - 1.5, sl["x"].valmin + 5.0)
    y_mid = 0.5 * (sl["y"].valmin + sl["y"].valmax)
    third = n // 3

    def update(k):
        if k < third:
            f = k / max(1, third - 1)
            set_values(c, x=x0 + f * (x1 - x0), y=y_mid, theta=10.0, fog=0.0, box=f)
        elif k < 2 * third:
            f = (k - third) / max(1, third - 1)
            set_values(c, fog=0.4 * f)
        else:
            f = (k - 2 * third) / max(1, n - 2 * third - 1)
            set_values(c, fog=0.0, mirror=20 + 140 * f)

    return viz.animate(fig, update, n, path, fps=fps)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--scene", default="default", choices=list(SCENES))
    ap.add_argument("--record", default=None, help="save a scripted sweep to this .mp4/.gif instead of opening a window")
    args = ap.parse_args()
    if args.record:
        matplotlib.use("Agg")
        print(record(args.record, args.scene))
    else:
        launch(args.scene)


if __name__ == "__main__":
    main()
