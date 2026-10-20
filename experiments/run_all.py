"""Regenerate every figure, table and video for one teammate (Stage 11).

    uv run python -m experiments.run_all --person ammar      # default scene  -> figures/ammar/
    uv run python -m experiments.run_all --person partner    # variant_b scene -> figures/partner/
    add --fast for a quick pass, --no-anim to skip GIF/MP4, --notebook to also export the notebook HTML

Every experiment runs on the person's own scene, so the two sets of media are distinct.
"""

import argparse
import importlib
import subprocess
import sys
import time
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

from experiments.common import PERSON, finish  # noqa: E402

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
    ap.add_argument("--person", choices=list(PERSON), default="ammar")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--fast", action="store_true")
    ap.add_argument("--no-anim", action="store_true")
    ap.add_argument("--notebook", action="store_true", help="also execute the notebook to HTML")
    ap.add_argument("--only", nargs="*", help="run only these module names (e.g. e5_mirror e6_glass)")
    args = ap.parse_args()
    scene, out = PERSON[args.person]["scene"], PERSON[args.person]["out"]
    Path(out).mkdir(parents=True, exist_ok=True)
    t_all = time.perf_counter()

    from experiments import params_table

    (Path(out) / "params_table.md").write_text(params_table.run(scene)["text"])
    print(Path(out) / "params_table.md")
    for mod_name, prefix, anim in EXPERIMENTS:
        if args.only and mod_name not in args.only:
            continue
        t0 = time.perf_counter()
        mod = importlib.import_module(f"experiments.{mod_name}")
        kw = {}
        if anim and not args.no_anim and "animate_to" in mod.run.__code__.co_varnames:
            kw["animate_to"] = f"{out}/{anim}"
        res = mod.run(scene, args.seed, args.fast, **kw)
        finish(res, out, prefix)
        print(f"  {mod_name}: {time.perf_counter() - t0:.1f} s")
    if not args.no_anim and not args.only:
        from experiments.demo_animation import render
        from experiments.interactive import record

        print(record(f"{out}/demo.mp4", scene, n=60 if args.fast else 120))
        if scene == "default":
            for p in render([f"{out}/drive.gif"]):
                print(p)
    from experiments.demo_board import build as build_board

    print(build_board(args.person))
    if args.notebook:
        subprocess.run([sys.executable, "-m", "jupyter", "nbconvert", "--to", "html", "--execute",
                        "--ExecutePreprocessor.timeout=1800", "notebooks/lidar_walkthrough.ipynb"], check=True)
    print(f"done in {time.perf_counter() - t_all:.0f} s -> {out}/")


if __name__ == "__main__":
    main()
