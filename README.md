# LiDAR Simulator — CEE 5810 Robot Perception Midterm

A physics-based simulation of a scanning LiDAR (2D Hokuyo-like, plus a 16-ring 3D mode) in rooms with matte walls, a **mirror**, a **glass pane**, a **moving box** and **fog**. It also includes a **particle filter** that uses the sensor model to localize a robot.

Every experiment checks the simulation against a closed-form prediction from `plan/02_DERIVATIONS.md` (D1–D14), and shows one of the Fig. 1 dependencies:

| Fig. 1 variable | Varied in |
|---|---|
| sensor state | E1, E2, E12 |
| target state / features | E3, E4, E5, E6 |
| operating conditions | E7 |

- Project plan: [plan/01_PROJECT_PLAN.md](plan/01_PROJECT_PLAN.md)
- Derivations (D1–D14): [plan/02_DERIVATIONS.md](plan/02_DERIVATIONS.md)
- Build stages: [plan/03_CODE_PLAN.md](plan/03_CODE_PLAN.md)
- Build log: [PROGRESS.md](PROGRESS.md)
- **Calls I made that you may want to change:** [DECISIONS_NEEDED.md](DECISIONS_NEEDED.md)

## Setup

Requires [uv](https://docs.astral.sh/uv/). uv installs Python 3.12 and the dependencies into `.venv/`:
- core: numpy, scipy, matplotlib, pillow, imageio-ffmpeg (bundled ffmpeg for MP4), tqdm, trimesh + embreex (3D ray casting)
- development: pytest, jupyterlab, ipympl

```bash
uv sync
```

## Run

Everything runs from this folder.

| What | Command |
|---|---|
| All your figures, tables and videos (~4 min) | `uv run python -m experiments.run_all --person ammar` |
| Your partner's (different scene, so distinct media) | `uv run python -m experiments.run_all --person partner` |
| Quick pass | add `--fast` (and `--no-anim` to skip videos) |
| One experiment | `uv run python -m experiments.e5_mirror --person partner` |
| **Demo board** (one page: every video + key figures, in talk order) | `uv run python -m experiments.demo_board --open` |
| Interactive slider demo (desktop window) | `uv run python -m experiments.interactive` |
| Record the demo as a video | `uv run python -m experiments.interactive --record figures/ammar/demo.mp4` |
| Walkthrough notebook | `uv run jupyter lab notebooks/lidar_walkthrough.ipynb` |
| Notebook → HTML for the appendix (~25 s) | `uv run jupyter nbconvert --to html --execute notebooks/lidar_walkthrough.ipynb` |
| Tests (114, ~20 s) | `uv run pytest` |

Outputs go to `figures/ammar/` or `figures/partner/`: PNG figures, CSV + Markdown tables (paste-ready for the report), GIF/MP4 animations, and `params_table.md` for the Procedure section. GIF/MP4 files are git-ignored; regenerate them with `run_all`.

## Experiments and headline results (default scene)

| ID | Shows | Theory check | Result |
|---|---|---|---|
| E1 | range vs sensor **position** | closed form R = (d − n·p)/(n·u) (D3) | match to 4e-16 m; parallel motion gives constant range |
| E2 | FOV vs sensor **heading** | x_T ∈ S(x) membership (D4) | 100% agreement (99.4% in variant_b) |
| E3 | **moving target** enters/leaves a beam | box-only geometry prediction | identical time intervals |
| E4 | **reflectivity / orientation** → detection | P_D = Φ((p−1)/σ_P), R_max(ρ, θ) (D5, D6) | R_max to 1e-13 m; per-beam MC RMS 0.018 |
| E5 | **mirror ghosts** | ghost = mirror image of true hit (D8) | 2.5e-15 m; ghosts lie on the "virtual room" |
| E6 | **glass**: see-through + head-on blip | Fresnel R_F, T_pane² (D9); blip edge θ_max (D5) | 8e-16; glass seen only within ~4° of head-on |
| E7 | **fog** → range loss | R_max = W₀(αR₀)/α (D5, D10) | 5e-13 m |
| E8 | noise model | N(R, σ_R²), σ_R = cσ_t/2 (D1, D6) | KS p = 0.65, σ̂ = 1.003 cm |
| CPT | Bayesian-network table from simulation | P(D \| material, fog, range) (D7) | mirrors/glass detected ≈ 5–12% at random incidence |
| E9 | CW phase ranging | R_amb = c/2f; two-frequency unwrap (D2) | 3.22 m wrap; unwrap RMS 0.99 cm |
| E10 | beam divergence mixed pixels | linear vs exact phasor (D12) | 229 spurious beams at a box edge |
| **E11** | **localization**: naive vs physics-aware model | Bayes / particle filter (D13) | glass corridor RMSE (20 runs): N1 36 cm, N2 11.3 cm, **P 0.9 cm**; default room N2 0.98 vs P 0.70 cm |
| **E12** | **3D**: what a planar LiDAR misses | floor ring h/tan(15°+ϑ), blind zone (D14) | low box: 0 points in 2D vs 81 in 3D; ring exact to 1e-15 |
| **E13** | 2D is the ε = 0 slice of 3D | R_3D = R_2D / cos ε (D14e) | 1.3e-15 m; 100% identical detections and ghosts |

## Layout

```
lidar_sim/
  geometry.py         rotations (2D, ZYX 3D), ray–segment (D3c), Möller–Trumbore (D14c), reflect_point
  materials.py        matte / mirror / glass
  world.py, scenes.py 2D scenes (default, variant_b, glass_corridor, test scenes), moving targets
  backends/           Segments2D (numpy) and Mesh3D (trimesh + Embree): one vectorised first-hit query
  raytrace.py         batched breadth-first tracer: diffuse + specular returns, mirror/glass child rays
  physics.py          reflection, Fresnel, received power, fog, max range (D5, D8–D10)
  noise.py            detection probability and range noise (D1, D6)
  sensor.py           2D LiDAR: scan, expected_scan (many poses), FOV + LOS, visibility polygon
  sensor3d.py         16-ring 3D LiDAR; scenes3d.py extrudes 2D rooms and adds 3D-only objects
  phase.py            CW phase ranging and mixed pixels (D2, D12)
  localization/       unicycle motion model, robust likelihood, particle filter (D13)
  paths.py            waypoint paths and a pure-pursuit unicycle driver
  viz.py, viz3d.py    plotting and animation
experiments/          e1 … e13, e_bn_cpt, params_table, interactive, run_all, demo_* (each: run() + CLI)
notebooks/            lidar_walkthrough.ipynb (thin viewer) + make_notebook.py
tests/                pytest checks against the derivations
figures/ammar|partner generated media
plan/                 planning docs
```

## Notes

- `--person ammar` uses the 8 × 6 m default room; `--person partner` uses the 9 × 6 m `variant_b` room. Every experiment has per-scene presets (`experiments/common.py`), so the two reports get different figures from the same code. The assignment requires distinct media.
- The course PDFs in this folder are git-ignored, so the repo can be pushed without redistributing them.
