"""Regenerate every figure, table and video for one teammate, plus the demo board page.

    uv run python -m experiments.run_all --person ammar   # people/ammar.toml -> figures/ammar/
    uv run python -m experiments.run_all --person gal     # people/gal.toml   -> figures/gal/
    add --fast for a quick pass, --no-anim to skip GIF/MP4, --notebook to also export the notebook HTML

Every experiment uses the person's config (room, seeds, poses, parameters), so the two sets of media
are distinct. Without --person it uses $LIDAR_PERSON, or asks.
"""

import argparse
import importlib
import subprocess
import sys
import time
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

from experiments.common import ask_person, finish, load_person, people  # noqa: E402

# (module, output prefix, animation file or None)
EXPERIMENTS = [
    ("e1_sensor_position", "e1", "e1_drive.gif"),
    ("e2_sensor_heading", "e2", None),
    ("e3_moving_target", "e3", "e3_moving_box.gif"),
    ("e4_reflectivity", "e4", None),
    ("e5_mirror", "e5", None),
    ("e6_glass", "e6", None),
    ("e7_fog", "e7", None),
    ("e8_noise_mc", "e8", None),
    ("e_bn_cpt", "cpt", None),
    ("e9_cw_phase", "e9", None),
    ("e10_divergence", "e10", None),
    ("e11_localization", "e11", "e11_particles.gif"),
    ("e12_3d_fov", "e12", "e12_rotate.mp4"),
    ("e13_2d3d_consistency", "e13", None),
]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--person", choices=people(), default=None, help="default: $LIDAR_PERSON, else asks")
    ap.add_argument("--seed", type=int, default=None, help="override the person's base seed")
    ap.add_argument("--fast", action="store_true")
    ap.add_argument("--no-anim", action="store_true")
    ap.add_argument("--notebook", action="store_true", help="also execute the notebook to HTML")
    ap.add_argument("--only", nargs="*", help="run only these module names (e.g. e5_mirror e6_glass)")
    args = ap.parse_args()
    person = (args.person or ask_person()).lower()
    out = load_person(person)["out"]
    Path(out).mkdir(parents=True, exist_ok=True)
    t_all = time.perf_counter()

    from experiments import params_table

    (Path(out) / "params_table.md").write_text(params_table.run(person)["text"])
    print(Path(out) / "params_table.md")
    for mod_name, prefix, anim in EXPERIMENTS:
        if args.only and mod_name not in args.only:
            continue
        t0 = time.perf_counter()
        mod = importlib.import_module(f"experiments.{mod_name}")
        kw = {}
        if anim and not args.no_anim and "animate_to" in mod.run.__code__.co_varnames:
            kw["animate_to"] = f"{out}/{anim}"
        res = mod.run(person, args.seed, args.fast, **kw)
        finish(res, out, prefix)
        print(f"  {mod_name}: {time.perf_counter() - t0:.1f} s")
    from demo import tour
    from demo.board import build as build_board
    from demo.interactive import record

    if not args.only:
        finish(tour.snapshot(person, args.seed), out, "tour")
        if not args.no_anim:
            print(record(f"{out}/demo.mp4", person, n=60 if args.fast else 120))
            for p in tour.animate([f"{out}/tour.gif"], person, args.seed):
                print(p)
    print(build_board(person))
    if args.notebook:
        subprocess.run([sys.executable, "-m", "jupyter", "nbconvert", "--to", "html", "--execute",
                        "--ExecutePreprocessor.timeout=1800", "notebooks/lidar_walkthrough.ipynb"], check=True)
    print(f"done in {time.perf_counter() - t_all:.0f} s -> {out}/")


if __name__ == "__main__":
    main()
