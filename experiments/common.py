"""Shared CLI arguments, per-scene presets and output helpers for the experiments.

Every experiment module exposes run(scene=..., seed=..., fast=...) -> dict with keys
  figs:   name -> matplotlib Figure
  tables: name -> (header list, rows list)
  data:   anything else (arrays) for the notebook
and a main() that parses the CLI, calls run() and saves everything with finish().

Use --person to get distinct media: ammar -> default scene, partner -> variant_b scene.
"""

import argparse
import csv
from pathlib import Path

import matplotlib

from lidar_sim import viz

PERSON = {
    "ammar": dict(scene="default", out="figures/ammar"),
    "partner": dict(scene="variant_b", out="figures/partner"),
}

# Poses are (x, y, heading_deg). Closed-form walls are (beam_deg, normal, offset d, label): n.q = d.
PRESETS = {
    "default": {
        "e1": dict(start=(2.8, 3.0), end=(5.8, 3.0), heading_deg=0.0,
                   beams=[(0.0, (1, 0), 8.0, "β=0°: right wall"), (90.0, (0, 1), 6.0, "β=+90°: top wall"),
                          (-90.0, (0, 1), 0.0, "β=−90°: bottom wall (dark 3–5 m)")]),
        "e2": dict(pose=(3.0, 3.0)),
        "e3": dict(pose=(3.2, 2.0, 90.0), beam_deg=0.0),
        "e4": dict(pose=(4.0, 1.75, -90.0)),
        "e5": dict(pose=(6.9, 2.4, 150.0)),
        "e6": dict(glass_dist=2.0, wall_dist=4.0, room_pose=(2.4, 1.5, -10.0)),
        "e7": dict(pose=(3.0, 3.2, 0.0)),
        "e12": dict(pose=(1.6, 2.8, 0.0), ghost_pose=(6.9, 2.4, 150.0)),
        "e13": dict(pose=(4.6, 3.2, 14.0)),
    },
    "variant_b": {
        "e1": dict(start=(2.6, 0.8), end=(2.6, 3.2), heading_deg=90.0,
                   beams=[(0.0, (0, 1), 6.0, "β=0°: top wall"), (90.0, (1, 0), 0.0, "β=+90°: left wall")]),
        "e2": dict(pose=(4.0, 3.5)),
        "e3": dict(pose=(1.5, 3.0, 0.0), beam_deg=0.0),
        "e4": dict(pose=(7.2, 3.5, 0.0)),
        "e5": dict(pose=(0.9, 2.1, 60.0)),
        "e6": dict(glass_dist=1.5, wall_dist=5.0, room_pose=(3.6, 1.6, 20.0)),
        "e7": dict(pose=(4.5, 4.2, -150.0)),
        "e12": dict(pose=(5.2, 3.0, 90.0), ghost_pose=(0.9, 2.1, 60.0)),
        "e13": dict(pose=(3.0, 3.0, 200.0)),
    },
}


def preset(scene, exp):
    return PRESETS.get(scene, PRESETS["default"])[exp]


def parser(description):
    ap = argparse.ArgumentParser(description=description)
    ap.add_argument("--person", choices=list(PERSON), default="ammar",
                    help="picks the default scene and output folder (distinct media per teammate)")
    ap.add_argument("--scene", default=None, help="scene name (default: from --person)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default=None, help="output folder (default: from --person)")
    ap.add_argument("--fast", action="store_true", help="fewer samples / frames (for quick checks)")
    ap.add_argument("--no-anim", action="store_true", help="skip GIF/MP4 rendering")
    return ap


def resolve(args):
    """Fill scene/out defaults from --person and switch to the non-interactive backend."""
    matplotlib.use("Agg")
    d = PERSON[args.person]
    args.scene = args.scene or d["scene"]
    args.out = args.out or d["out"]
    return args


def write_table(out, name, header, rows):
    """Write a table as CSV and as a Markdown file (for pasting into the report)."""
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    with open(out / f"{name}.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)

    def fmt(v):
        return f"{v:.4g}" if isinstance(v, float) else str(v)

    md = ["| " + " | ".join(header) + " |", "|" + "---|" * len(header)]
    md += ["| " + " | ".join(fmt(v) for v in r) + " |" for r in rows]
    (out / f"{name}.md").write_text("\n".join(md) + "\n")
    return out / f"{name}.csv"


def finish(res, out, prefix):
    """Save every figure (PNG) and table (CSV + MD) of an experiment result; print the paths."""
    paths = []
    for name, fig in res.get("figs", {}).items():
        paths.append(viz.save(fig, out, f"{prefix}_{name}"))
    for name, (header, rows) in res.get("tables", {}).items():
        paths.append(write_table(out, f"{prefix}_{name}", header, rows))
    for p in paths + res.get("media", []):
        print(p)
    import matplotlib.pyplot as plt

    plt.close("all")
    return paths
