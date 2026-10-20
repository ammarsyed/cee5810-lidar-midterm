# LiDAR Perception Simulator — CEE 5810 Robot Perception Midterm

A physics-based simulation of a scanning **LiDAR**: a 2D Hokuyo-style sensor plus a 16-ring 3D (Velodyne-style) mode. It runs in rooms with matte walls, a **mirror**, a **glass pane**, a **moving box** and **fog**, and includes a **particle filter** that uses the sensor model to localize a robot.

Every experiment checks the simulation against a formula from [the derivations](docs/02_DERIVATIONS.md), and shows one dependency from Figure 1 of the assignment:

| Figure 1 variable | Shown in |
|---|---|
| sensor state → measurements | E1, E2, E12 |
| target state / features → measurements | E3–E6 |
| operating conditions → measurements | E7 |

**Status:** the simulator, all experiments and all media are done. What's left is the two reports; see [PROGRESS.md → What's left](PROGRESS.md#whats-left).

---

## Quick start (for the teammate)

**You need:** git, and [uv](https://docs.astral.sh/uv/), which installs the right Python and every library for you. Tested on macOS (Apple Silicon); Linux and Windows should work too, since all dependencies ship pre-built wheels.

```bash
# 1. install uv (once). macOS/Linux:
curl -LsSf https://astral.sh/uv/install.sh | sh        # or: brew install uv
# Windows (PowerShell):  powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"

# 2. get the code and install everything (~1 min)
git clone https://github.com/ammarsyed/cee5810-lidar-midterm.git
cd cee5810-lidar-midterm
uv sync

# 3. check it works (~20 s, 136 tests)
uv run pytest

# 4. make YOUR figures, tables and videos (~4.5 min) -> figures/partner/
uv run python -m experiments.run_all --person partner

# 5. open the demo board (all your videos + key figures on one page)
uv run python -m demo.board --person partner --open

# 6. play with the live slider app
uv run python -m demo.interactive --scene variant_b
```

`--person partner` uses a **different room (variant_b)** than `--person ammar` (the default room). The assignment requires each person's media to be distinct, and this gives you different figures from the same code. The course PDFs (assignment + papers) are not in the repo; you have them on Canvas.

---

## What you can demo

| Thing | What it is | How to open it |
|---|---|---|
| **Slider app** (the "front end") | A desktop window: drag the sensor, heading, fog, mirror angle, box position and box reflectivity, and watch the scan, ghosts and see-through glass update live. Includes a return-policy switch, show/hide toggles and a "new noise" button. | `uv run python -m demo.interactive` (add `--scene variant_b` for the partner's room) |
| **Demo board** | One local web page with every video and the key figures, in presentation order. The safest thing to present from. | `uv run python -m demo.board --open` → `figures/<person>/index.html` |
| **Videos** | `demo.mp4` (scripted slider sweep), `tour.gif` (drive through the room), `e1_drive.gif`, `e3_moving_box.gif`, `e11_particles.gif` (three particle filters racing), `e12_rotate.mp4` (rotating 3D point cloud) | made by `run_all` into `figures/<person>/`; also on the demo board |
| **Walkthrough notebook** | Every experiment inline, ending in a theory-vs-simulation summary table. The executed HTML doubles as the code appendix. | live: `uv run jupyter lab notebooks/lidar_walkthrough.ipynb`; static: `uv run jupyter nbconvert --to html --execute notebooks/lidar_walkthrough.ipynb` → `notebooks/lidar_walkthrough.html` |
| **Figures and tables for the report** | PNG figures, plus each table as CSV and paste-ready Markdown; `params_table.md` for the Procedure section | `figures/<person>/` |

Suggested 3–5 minute demo:
1. Slider app: walk toward the mirror (ghosts appear outside the room), then add fog.
2. Demo board: the particle-filter race and the corridor error plot.
3. Demo board: the rotating 3D cloud.
4. Notebook: the summary table.

---

## Results at a glance (default room)

| ID | What it shows | Checked against | Result |
|---|---|---|---|
| E1 | range vs sensor **position** | closed form R = (d − n·p)/(n·u) (D3) | agrees to 4e-16 m; moving parallel to a wall changes nothing |
| E2 | FOV vs sensor **heading** | x_T ∈ S(x) membership (D4) | 100% agreement (99.4% in variant_b) |
| E3 | **moving target** enters/leaves a beam | box-only geometry | identical time intervals |
| E4 | **reflectivity / tilt** → detection | P_D and R_max(ρ, θ) (D5, D6) | R_max to 1e-13 m |
| E5 | **mirror ghosts** | ghost = mirror image of the true hit (D8) | 2.5e-15 m; ghosts lie on a "virtual room" |
| E6 | **glass**: see-through + head-on blip | Fresnel (D9), blip edge θ_max (D5) | 8e-16; glass visible only within ~4° of head-on |
| E7 | **fog** → range loss | R_max = W₀(αR₀)/α (D5, D10) | 5e-13 m |
| E8 | noise model | N(R, σ_R²), σ_R = cσ_t/2 (D1, D6) | KS p = 0.65, σ̂ = 1.003 cm |
| CPT | Bayesian-network table learned from the sim | P(detect \| material, fog, range) (D7) | mirrors/glass detected ≈ 5–12% at random angles |
| E9 | CW phase ranging | R_amb = c/2f, two-frequency unwrap (D2) | wraps every 3.22 m; unwrap RMS 0.99 cm |
| E10 | beam divergence / mixed pixels | linear vs exact phasor (D12) | spurious points between box and wall |
| **E11** | **localization** with the sensor model | Bayes / particle filter (D13) | glass corridor RMSE (20 runs): naive 36 cm, naive + robust 11.3 cm, **physics-aware 0.9 cm** |
| **E12** | **3D**: what a planar LiDAR misses | floor ring, blind zone (D14) | low box: 0 points in 2D vs 81 in 3D |
| **E13** | 2D = horizontal slice of 3D | R_3D = R_2D / cos ε (D14e) | 1.3e-15 m; identical detections and ghosts |

---

## Documents

| Doc | What's in it |
|---|---|
| [docs/02_DERIVATIONS.md](docs/02_DERIVATIONS.md) | **The math**: D1–D14 (time of flight, frames, FOV/LOS, power and detection, the probabilistic model, the Bayesian network, mirror and glass physics, fog, the 5810 dependency analysis, beam divergence, the particle filter, 3D). Notation table included. A rendered version with real equations and diagrams: [partner derivations page](https://claude.ai/artifact/7VtNLZAhL791QDsZSSYCva) (private; the owner shares it from its Share menu). |
| [docs/01_PROJECT_PLAN.md](docs/01_PROJECT_PLAN.md) | What we're building and why: gaps vs. the rubric, the Figure 1 mapping, sensor parameters, scenes, experiments, **who does what**, the report outline, the timeline, course concepts used, references. |
| [docs/03_CODE_PLAN.md](docs/03_CODE_PLAN.md) | The original stage-by-stage build plan (history; the file layout has since changed, and this README is authoritative). |
| [DECISIONS.md](DECISIONS.md) | **Modelling calls made during the build**, each in plain English with the tradeoff and a recommendation. Review before writing the reports. |
| [PROGRESS.md](PROGRESS.md) | **Status, what's left**, and the full build log. |
| [AGENTS.md](AGENTS.md) | Instructions for AI coding assistants (Claude Code reads it through [CLAUDE.md](CLAUDE.md)): commands, conventions, rules. |

---

## How it works (architecture)

```
scene (2D segments or 3D triangle mesh, each surface has a material: matte / mirror / glass)
   │
   ▼
backend.cast(origins, dirs) ── one vectorised "first hit" query: Segments2D (numpy) or Mesh3D (Embree)
   │
   ▼
raytrace.trace ── batched, breadth-first: per bounce depth, one cast for ALL rays
   │                 matte → diffuse return (stop) · mirror → reflect · glass → transmit + reflect
   │                 every return gets path length + received power (D5)
   ▼
sensor.chosen_returns ── strongest/first return per beam, detection probability P_D (D6)
   │
   ▼
scan / scan3d ── Bernoulli detection + Gaussian range noise → ranges, reported points, ghosts
   │
   ├──► experiments/ (E1–E13, CPT) ── run() → figures + tables, compared with closed forms
   ├──► localization/ ── particle filter whose likelihood calls expected_scan() for 300 poses at once
   └──► demo/ ── slider app, demo board, room tour
```

Key ideas:
- **One backend interface for 2D and 3D.** Physics, noise and return selection are shared, which is why 2D is provably the ε = 0 slice of 3D (E13).
- **Batched tracing.** 500 poses × 61 beams with mirror + glass take ~19 ms, and one 3D scan (5,760 rays) ~3 ms. That's what makes the particle filter practical.
- **Per-person presets** (`experiments/common.py`) give each teammate a different room and different poses, so the media are distinct.

---

## Repository map

### Top level
| File | Purpose |
|---|---|
| [README.md](README.md) | this file |
| [AGENTS.md](AGENTS.md) / [CLAUDE.md](CLAUDE.md) | instructions for AI coding assistants (CLAUDE.md just imports AGENTS.md) |
| [PROGRESS.md](PROGRESS.md) | status, what's left, build log |
| [DECISIONS.md](DECISIONS.md) | modelling decisions with tradeoffs and recommendations |
| [pyproject.toml](pyproject.toml) | dependencies plus pytest and ruff settings |
| [uv.lock](uv.lock) | exact pinned versions (`uv sync` installs them) |
| [.python-version](.python-version) | Python 3.12 |
| [.gitignore](.gitignore) | ignores `.venv`, caches, generated videos, the notebook HTML and `course_materials/` |

### `lidar_sim/`: the simulator package
| File | Purpose |
|---|---|
| [`__init__.py`](lidar_sim/__init__.py) | package marker |
| [`config.py`](lidar_sim/config.py) | every default constant: noise, power calibration, specular gain, materials, tracing, sensor geometry |
| [`geometry.py`](lidar_sim/geometry.py) | 2D/3D rotations (ZYX Euler), ray–segment (D3c) and ray–triangle (Möller–Trumbore, D14c) intersection, point reflection |
| [`materials.py`](lidar_sim/materials.py) | `Material` (matte / mirror / glass) and per-surface property arrays |
| [`world.py`](lidar_sim/world.py) | 2D `World`: segments, boxes, moving targets (`MovingBox`, `ping_pong`), the "naive" variant, the backend factory |
| [`scenes.py`](lidar_sim/scenes.py) | 2D rooms: `default_room` (Ammar), `variant_b` (partner), `glass_corridor` (E11), and test scenes (`single_wall`, `glass_test`) |
| [`scenes3d.py`](lidar_sim/scenes3d.py) | 3D scenes: `extrude()` a 2D room into walls + floor + ceiling, add a low box and a table |
| [`backends/base.py`](lidar_sim/backends/base.py) | `Hits` result type and the `SceneBackend` interface |
| [`backends/segments2d.py`](lidar_sim/backends/segments2d.py) | vectorised numpy 2D first-hit backend |
| [`backends/mesh3d.py`](lidar_sim/backends/mesh3d.py) | 3D first-hit backend using trimesh + Intel Embree |
| [`physics.py`](lidar_sim/physics.py) | reflection, Fresnel, pane transmission, fog, diffuse and specular received power, max range (Lambert W) |
| [`raytrace.py`](lidar_sim/raytrace.py) | the batched breadth-first tracer (`trace`) and return selection (`select`) |
| [`noise.py`](lidar_sim/noise.py) | detection probability Φ((p−1)/σ_P), range noise, measurement sampling |
| [`sensor.py`](lidar_sim/sensor.py) | 2D LiDAR: `LidarParams`, `scan`, `expected_scan` (many poses), `in_fov` + LOS, `visibility_polygon`, shared `chosen_returns` |
| [`sensor3d.py`](lidar_sim/sensor3d.py) | 3D LiDAR: `LidarParams3D` (16 rings, tilt), `ring_directions`, `scan3d` |
| [`phase.py`](lidar_sim/phase.py) | continuous-wave phase ranging, two-frequency unwrapping, mixed-pixel models (D2, D12) |
| [`paths.py`](lidar_sim/paths.py) | constant-speed waypoint paths and a pure-pursuit unicycle driver |
| [`localization/motion.py`](lidar_sim/localization/motion.py) | unicycle motion model (deterministic + noisy sampling) |
| [`localization/likelihood.py`](lidar_sim/localization/likelihood.py) | naive vs. physics-aware models, robust beam log-likelihood, mixture bound |
| [`localization/particle_filter.py`](lidar_sim/localization/particle_filter.py) | particle filter: predict, update, low-variance resampling, estimate |
| [`viz.py`](lidar_sim/viz.py) | palette and style, scene/scan/FOV drawing, polar plots, GIF/MP4 export |
| [`viz3d.py`](lidar_sim/viz3d.py) | range image, 3D scatter, bird's-eye view, rotating animation |

### `experiments/`: report results (each exposes `run()` plus a command line)
| File | Purpose |
|---|---|
| [`common.py`](experiments/common.py) | shared CLI flags (`--person`, `--scene`, `--seed`, `--out`, `--fast`, `--no-anim`), per-scene presets, table writing |
| [`run_all.py`](experiments/run_all.py) | regenerates **everything** for one person (figures, tables, videos, demo board) |
| [`params_table.py`](experiments/params_table.py) | writes `params_table.md` (all parameters + scene surfaces) |
| [`e1_sensor_position.py`](experiments/e1_sensor_position.py) | E1: range vs position, closed form |
| [`e2_sensor_heading.py`](experiments/e2_sensor_heading.py) | E2: FOV vs heading |
| [`e3_moving_target.py`](experiments/e3_moving_target.py) | E3: moving box enters/leaves beams |
| [`e4_reflectivity.py`](experiments/e4_reflectivity.py) | E4: detection vs reflectivity and tilt |
| [`e5_mirror.py`](experiments/e5_mirror.py) | E5: mirror ghosts, virtual room, theorem check |
| [`e6_glass.py`](experiments/e6_glass.py) | E6: glass transmission, head-on blip, see-through points |
| [`e7_fog.py`](experiments/e7_fog.py) | E7: fog thinning and max range |
| [`e8_noise_mc.py`](experiments/e8_noise_mc.py) | E8: Monte Carlo check of the noise model |
| [`e9_cw_phase.py`](experiments/e9_cw_phase.py) | E9: phase ranging and ambiguity |
| [`e10_divergence.py`](experiments/e10_divergence.py) | E10: beam divergence, mixed pixels |
| [`e11_localization.py`](experiments/e11_localization.py) | E11: three particle filters, naive vs. physics-aware |
| [`e12_3d_fov.py`](experiments/e12_3d_fov.py) | E12: 3D vs 2D visibility, floor ring, 3D ghosts, 3D fog |
| [`e13_2d3d_consistency.py`](experiments/e13_2d3d_consistency.py) | E13: 2D = horizontal slice of 3D |
| [`e_bn_cpt.py`](experiments/e_bn_cpt.py) | Bayesian-network CPT estimated by Monte Carlo |

### `demo/`: presentation tools
| File | Purpose |
|---|---|
| [`interactive.py`](demo/interactive.py) | the slider app (`build`, `launch`, `record` → MP4) |
| [`board.py`](demo/board.py) | builds the demo-board web page `figures/<person>/index.html` |
| [`tour.py`](demo/tour.py) | room tour: one annotated scan (`tour_snapshot.png`) and the drive-through animation (`tour.gif`) |

### `notebooks/`, `tests/`, `figures/`
| Path | Purpose |
|---|---|
| [`notebooks/make_notebook.py`](notebooks/make_notebook.py) | generates the notebook (edit this, not the .ipynb) |
| [`notebooks/lidar_walkthrough.ipynb`](notebooks/lidar_walkthrough.ipynb) | walkthrough notebook: a thin viewer over the experiments |
| [`tests/test_geometry.py`](tests/test_geometry.py) | intersections, closed-form wall range, batching, rotations |
| [`tests/test_sensor.py`](tests/test_sensor.py) | FOV/LOS, frames, rotation shift, visibility polygon |
| [`tests/test_physics.py`](tests/test_physics.py) | calibration, max range, fog (incl. the D10 table), Fresnel table, reflection |
| [`tests/test_noise.py`](tests/test_noise.py) | noise std, detection probability, dropouts |
| [`tests/test_raytrace.py`](tests/test_raytrace.py) | ghost theorem, mirror/glass powers, return policies, glass-blip edge, speed |
| [`tests/test_paths.py`](tests/test_paths.py) | paths, unicycle model, moving box |
| [`tests/test_localization.py`](tests/test_localization.py) | filter convergence, resampling, naive vs physics, mixture bound |
| [`tests/test_backends.py`](tests/test_backends.py) | Embree vs Möller–Trumbore, 2D-slice identity, 1/cos ε, floor ring, blind zone, 3D ghosts |
| [`tests/test_phase.py`](tests/test_phase.py) | ambiguity ranges, unwrapping, mixed pixels |
| [`tests/test_demo.py`](tests/test_demo.py) | slider app (headless) and room tour |
| [`tests/test_experiments.py`](tests/test_experiments.py) | every experiment runs (fast mode, both scenes); E11 ordering |
| [`tests/test_smoke.py`](tests/test_smoke.py) | every module imports; MP4 export and Embree work |
| `figures/ammar/`, `figures/partner/` | generated outputs. Naming: `<exp>_<name>.png` figures, `<exp>_<table>.csv/.md` tables, `params_table.md`, `index.html` (demo board). Videos (`*.gif`, `*.mp4`) are regenerated by `run_all` and not committed. |

---

## Development

```bash
uv run pytest            # 136 tests, ~20 s
uvx ruff check .         # lint (config in pyproject.toml)
uv run python -m experiments.run_all --person ammar --fast --no-anim   # quick full pass
```

To add an experiment:
1. Write `experiments/eN_name.py` with `run(scene, seed, fast) -> {"figs", "tables", "data"}` and a `main()` that uses `common.parser/resolve/finish`.
2. Add presets for both scenes in `common.PRESETS`.
3. Register it in `run_all.EXPERIMENTS` and `tests/test_experiments.py`.
