"""Shared helpers for the experiments: person configs, CLI arguments, output writing.

Every experiment module exposes run(person="ammar", seed=None, fast=False) -> dict with keys
  figs:   name -> matplotlib Figure
  tables: name -> (header list, rows list)
  data:   anything else (arrays) for the notebook
and a main() that parses the CLI, calls run() and saves everything with finish().

Each teammate has a config file in people/<name>.toml (room, random seed, output folder, and every
experiment's poses/parameters), so the same simulator produces distinct media for each person.
"""

import argparse
import csv
import os
import sys
import tomllib
from pathlib import Path

import matplotlib
import numpy as np

from lidar_sim import viz
from lidar_sim.scenes import get_scene

PEOPLE_DIR = Path(__file__).resolve().parents[1] / "people"


def people():
    """Names of the available person configs (e.g. ["ammar", "gal"])."""
    return sorted(p.stem for p in PEOPLE_DIR.glob("*.toml"))


def load_person(name):
    """Load people/<name>.toml as a dict (adds "key" = the file name)."""
    key = str(name).strip().lower()
    path = PEOPLE_DIR / f"{key}.toml"
    if not path.exists():
        raise SystemExit(f"Unknown person {name!r}. Choose one of: {', '.join(people())} (configs in {PEOPLE_DIR}).")
    with open(path, "rb") as f:
        cfg = tomllib.load(f)
    cfg["key"] = key
    return cfg


def scene_of(cfg, **overrides):
    """Build the person's room, with their scene_options plus any per-call overrides."""
    return get_scene(cfg["scene"], **{**cfg.get("scene_options", {}), **overrides})


def pose_rad(p):
    """[x, y, heading_deg] -> np.array([x, y, heading_rad])."""
    return np.array([p[0], p[1], np.radians(p[2])], dtype=float)


def ask_person():
    """Return the person from LIDAR_PERSON, or ask on an interactive terminal."""
    env = os.environ.get("LIDAR_PERSON")
    if env:
        return env
    if sys.stdin.isatty():
        names = people()
        answer = input(f"Who are you? ({' / '.join(n.capitalize() for n in names)}): ").strip().lower()
        if answer in names:
            return answer
    raise SystemExit(f"Pass --person ({' | '.join(people())}) or set LIDAR_PERSON.")


def parser(description):
    ap = argparse.ArgumentParser(description=description, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--person", default=None,
                    help=f"whose config to use: {' | '.join(people())} (default: $LIDAR_PERSON, else asks)")
    ap.add_argument("--seed", type=int, default=None, help="override the person's base seed")
    ap.add_argument("--out", default=None, help="output folder (default: from the person's config)")
    ap.add_argument("--fast", action="store_true", help="fewer samples / frames (for quick checks)")
    ap.add_argument("--no-anim", action="store_true", help="skip GIF/MP4 rendering")
    return ap


def resolve(args):
    """Fill person/out defaults, attach the config as args.cfg, switch to the non-interactive backend."""
    matplotlib.use("Agg")
    args.person = (args.person or ask_person()).lower()
    args.cfg = load_person(args.person)
    args.out = args.out or args.cfg["out"]
    return args


def markdown_table(header, rows):
    """Markdown text for a table (floats to 4 significant figures)."""

    def fmt(v):
        return f"{v:.4g}" if isinstance(v, float) else str(v)

    md = ["| " + " | ".join(header) + " |", "|" + "---|" * len(header)]
    md += ["| " + " | ".join(fmt(v) for v in r) + " |" for r in rows]
    return "\n".join(md) + "\n"


def write_table(out, name, header, rows):
    """Write a table as CSV and as a Markdown file (for pasting into the report)."""
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    with open(out / f"{name}.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)
    (out / f"{name}.md").write_text(markdown_table(header, rows))
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
