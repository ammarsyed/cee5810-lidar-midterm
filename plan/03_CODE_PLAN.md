# Code Plan — 2D LiDAR Simulator (Python)

> **How to use this doc:** each **Stage** below is sized to fit in roughly one Claude Code session. Start a session, paste the stage's **Prompt**, and let it run. Check the **Done when** list, then commit. Every stage appends a short entry to `PROGRESS.md`, so the next session (or a fresh context after hitting usage limits) knows where things stand.
>
> Read `01_PROJECT_PLAN.md` for the *why* and `02_DERIVATIONS.md` for the equations (referred to as D1–D12). Experiments are E1–E10.

---

## 1. The big picture (what the code does, in one breath)

1. A **world** is a list of line segments (walls, mirror, glass, box sides). Each segment has a **material**.
2. A **sensor** has a pose `(x, y, θ)`. It fires N beams across a 240° fan.
3. Each beam is **ray-traced**: find the nearest segment it hits, then:
   - **matte** → bounce light back (a return)
   - **mirror** → reflect and keep going
   - **glass** → mostly pass through, partly reflect
   - repeat up to B bounces
4. For every return we know the **path length** and the **received power** (inverse-square, reflectivity, angle, Fresnel/mirror losses, fog).
5. The sensor picks the **strongest** (or first) return. It decides **detected or not** (power vs. threshold, with randomness) and adds **Gaussian range noise**.
6. Output: per-beam ranges (or ∅), the "apparent" points (where the LiDAR *thinks* things are, including ghost points), and the true hit points.
7. **Experiments** scripts vary one Fig. 1 variable at a time and save figures/animations, with theory curves overlaid.

---

## 2. Tech stack & setup

- **Python 3.12** via `uv` (`uv sync` installs everything). System Python here is 3.9 with no scipy, so always use the project env.
- Version control: git in `Midterm/`, commit after every stage.

| Library | Group | Used for |
|---|---|---|
| `numpy` | core | all vector math, ray–segment intersection, rotations |
| `scipy` | core | `special.lambertw` (fog max range, D5/D10), `stats.norm` (detection probability, D6) |
| `matplotlib` | core | every figure, `FuncAnimation` for videos, `matplotlib.widgets` for the slider demo |
| `pillow` | core | GIF export |
| `imageio-ffmpeg` | core | a bundled ffmpeg binary, so MP4 export works with no system install. Set `plt.rcParams["animation.ffmpeg_path"] = imageio_ffmpeg.get_ffmpeg_exe()` once in `viz.py` |
| `tqdm` | core | progress bars for Monte Carlo runs (E8, CPT estimation) |
| `pytest` | dev | tests against the derivations |
| `jupyterlab` | dev | the walkthrough notebook (same as HW 2); `nbconvert` comes with it, for the HTML appendix export |
| `ipympl` (brings `ipywidgets`) | dev | `%matplotlib widget`, so the **same** slider demo runs inside the notebook |

**Considered and skipped on purpose:**
- **shapely:** ray–segment intersection *is* derivation D3(c), so we write and test it ourselves.
- **pgmpy:** HW 2 did BNs by hand, and our CPT is just Monte Carlo counting.
- **PyBullet / Open3D / Gazebo ray casting:** no mirror/glass physics. Using an existing simulator would also put us on the assignment's "adopt an existing sensor sim" path, which needs extra validation.
- **Streamlit / web UI:** not graded, and the slider demo plus notebook cover it.
- **pandas / seaborn:** numpy + csv + matplotlib are enough.

## 3. Repo layout

```
Midterm/
├── plan/                     ← these docs
├── PROGRESS.md               ← running log, one entry per stage/session
├── pyproject.toml
├── lidar_sim/
│   ├── __init__.py
│   ├── geometry.py           ← vectors, rot2d, ray–segment intersection   (D3)
│   ├── materials.py          ← Material dataclass + presets (matte/mirror/glass)
│   ├── world.py              ← Segment, Box, World, moving targets
│   ├── scenes.py             ← scene builders: default_room(), variant_b(), …
│   ├── physics.py            ← reflect, fresnel, received_power, fog, max_range (D5, D8, D9, D10)
│   ├── raytrace.py           ← recursive tracer → list[Return] per beam
│   ├── sensor.py             ← LidarParams, LidarSensor.scan(), FOV, visibility (D1, D3, D4)
│   ├── noise.py              ← detection prob, range noise, sampling (D6)
│   ├── viz.py                ← drawing scene/rays/points/FOV, polar plots, animation helpers
│   └── config.py             ← default parameter values (from 01_PROJECT_PLAN §4)
├── experiments/
│   ├── e1_sensor_position.py
│   ├── e2_sensor_heading.py
│   ├── …                     ← one script per experiment, each exposes run(...) + a CLI
│   ├── interactive.py        ← slider demo window (Stage 7a), launch() also works in the notebook
│   └── common.py             ← shared CLI args (--scene, --seed, --out, --style)
├── notebooks/
│   └── lidar_walkthrough.ipynb  ← thin "viewer" over the package (Stage 7b), exported to HTML for the appendix
├── tests/
│   ├── test_geometry.py
│   ├── test_physics.py
│   ├── test_raytrace.py
│   └── test_noise.py
└── figures/                  ← generated outputs (png/gif/mp4), split by person
    ├── ammar/
    └── partner/
```

## 4. Key data structures (the contract between modules)

```python
@dataclass
class Material:
    kind: Literal["matte", "mirror", "glass"]
    rho: float = 0.7          # diffuse reflectivity (matte; small residual for mirror/glass)
    reflectance: float = 0.95 # mirror only
    n: float = 1.5            # glass only
    name: str = ""

@dataclass
class Segment:
    a: np.ndarray; b: np.ndarray   # endpoints (world frame)
    material: Material
    id: int

@dataclass
class World:
    segments: list[Segment]
    fog_alpha: float = 0.0         # Λ
    def at_time(self, t) -> "World": ...   # returns world with moving targets placed at time t

@dataclass
class Return:
    path_length: float        # one-way optical path R (sum of legs)  → measured range if chosen
    power: float              # received power (D5), includes out-and-back losses + fog
    hit_point: np.ndarray     # TRUE location of the final surface hit (world frame)
    apparent_point: np.ndarray# sensor_pos + path_length * initial_dir  (ghost if bounced)
    surface_id: int
    kind: str                 # "diffuse" or "specular"
    bounces: int              # number of mirror/glass interactions before this return
    theta_inc: float

@dataclass
class LidarParams:
    fov_deg: float = 240; n_beams: int = 241
    r_min: float = 0.02; r_max_ref: float = 5.6   # r_max_ref: clear-air max range for rho=0.9 head-on
    sigma_t: float = 67e-12      # → sigma_R ≈ 1 cm
    sigma_P_rel: float = 0.1     # power noise as fraction of P_th (for smooth P_D)
    max_bounces: int = 5
    return_policy: Literal["strongest", "first"] = "strongest"
    spec_lobe_deg: float = 1.5   # θ_w for specular head-on returns

@dataclass
class ScanResult:
    pose: np.ndarray             # (x, y, θ)
    beam_angles: np.ndarray      # β_i in body frame
    ranges: np.ndarray           # z_i, NaN = no return (∅)
    true_ranges: np.ndarray      # noise-free path length of chosen return, NaN if none
    detected: np.ndarray[bool]
    power: np.ndarray
    p_detect: np.ndarray         # P_D per beam
    points_world: np.ndarray     # apparent points from noisy ranges (what the LiDAR reports)
    true_hits: np.ndarray        # true hit points of chosen returns
    is_ghost: np.ndarray[bool]   # chosen return involved ≥1 mirror/glass reflection
    all_returns: list[list[Return]]  # for visualisation/debug
```

## 5. Physics decisions (so the code matches the math)

| Thing | Rule in code | Derivation |
|---|---|---|
| Range from time | `R = c·t/2`; noise is generated on `t` (σ_t) and converted, or directly `σ_R = c σ_t/2` | D1 |
| Beam directions | `u_i = rot2d(θ_s) @ [cos β_i, sin β_i]` | D3 |
| Intersection | cross-product formula; nearest `t > 1e-9` | D3(c) |
| Normal orientation | flip `n̂` so `dot(u, n̂) < 0`; `θ_inc = arccos(−u·n̂)` | — |
| Matte hit | diffuse return, **stop** tracing | D5 |
| Mirror hit | specular-lobe return using `R_m` (head-on only) + continue with `u' = u − 2(u·n̂)n̂`, throughput × R_m | D5, D8 |
| Glass hit | specular-lobe return with `R_F(θ)` + continue **straight** (thin pane) with throughput × (1−R_F)² + (if bounces left) a reflected branch with throughput × R_F | D9 |
| Power | `P = K·ρ·cosθ·T²·e^(−2αL)/L²` where T = one-way throughput, L = total path length | D5, D10 |
| Calibration | choose `K/P_th` so `R_max = 5.6 m` for ρ = 0.9 head-on, clear air | D5 |
| Detection | `P_D = Φ((P − P_th)/σ_P)`, sample Bernoulli | D6 |
| Range noise | `z = L + N(0, σ_R²)`, clip at `r_min` | D6 |
| Apparent point | `p + z·u_i` (this is where ghosts come from) | D8 |

Throughput is tracked **one-way** and squared for out-and-back. Glass's reflected branch is the "weak ghost" path. Prune any branch whose best-case power is below `1e-3·P_th`, to keep tracing fast.

---

## 6. Stages

Rough sizes: **S** ≈ small session, **M** ≈ medium, **L** ≈ big (maybe split it).

### Stage 0 — Setup (S)
**Goal:** a project skeleton that runs.
- `git init`, `.gitignore` (venv, `__pycache__`, `figures/**/*.gif` optional)
- `uv init` / `pyproject.toml` with deps; `uv sync`
- Empty modules from §3, `PROGRESS.md`, `README.md` with how-to-run
- One trivial test so `uv run pytest` passes

**Done when:** `uv run pytest` passes; first commit made.

> **Prompt:** *"Read plan/03_CODE_PLAN.md. Do Stage 0 only: set up the uv project (Python 3.11+, numpy, scipy, matplotlib, pillow, pytest), git init, the folder layout in §3 with empty modules, README and PROGRESS.md. Make `uv run pytest` pass, commit, and log it in PROGRESS.md."*

### Stage 1 — Geometry + matte room + first picture (M)
**Goal:** rays hitting plain walls, drawn on screen.
- `geometry.py`: `rot2d`, `cross2`, `ray_segment_intersect` (D3c), `nearest_hit(origin, dir, segments)`
- `materials.py`: presets `MATTE(ρ)`, `MIRROR`, `GLASS`
- `world.py`: `Segment`, `World`, `Box` → 4 segments
- `scenes.py`: `default_room()` (8×6 m, matte walls, one dark and one bright section; no mirror/glass yet)
- `viz.py`: `draw_world(ax, world)`, `draw_rays(ax, origin, hits)`
- Script `experiments/smoke.py`: 241 beams from the room centre, saves `figures/smoke.png`
- Tests: intersection hit/miss/parallel/behind-ray; closed-form wall range `R = (d − n̂·p)/(n̂·u)` matches the tracer

**Done when:** `figures/smoke.png` shows the room with a fan of rays ending on the walls; tests pass.

> **Prompt:** *"Read plan/03_CODE_PLAN.md and PROGRESS.md. Do Stage 1 only (geometry, materials, world, default_room with matte walls, viz basics, smoke script, tests against the closed-form wall range from plan/02_DERIVATIONS.md D3). Show me the smoke figure, commit, update PROGRESS.md."*

### Stage 2 — Sensor pose, frames, FOV & LOS (M)
**Goal:** a real sensor object with a pose, a FOV wedge and a visibility polygon.
- `sensor.py`: `LidarParams`, `LidarSensor.beam_directions(pose)`, `to_world(ranges, pose)`, `in_fov(point, pose)` (D4), `visibility_polygon(pose, world, n_rays=2000)` (glass treated as transparent, everything else opaque)
- `viz.py`: `draw_fov(ax, pose, params)` (wedge), `draw_visibility(ax, poly)`, `draw_sensor(ax, pose)`, `polar_ranges(ax, scan)`
- Noise-free `scan()` for matte only (power/noise come in Stage 3)
- Tests: `in_fov` edge cases (angle wrap at ±180°); `to_world` round-trips; rotating the sensor by Δθ shifts the range profile by Δθ in a circular room

**Done when:** a figure showing the sensor, its 240° wedge, the shaded visible region, and rays; a polar range plot.

> **Prompt:** *"Read plan/03_CODE_PLAN.md and PROGRESS.md. Do Stage 2 only: sensor pose, body↔world frames, FOV wedge, LOS visibility polygon, noise-free scan for matte walls, polar plot, tests. Save a demo figure, commit, update PROGRESS.md."*

### Stage 3 — Power, noise, dropouts, fog (M)
**Goal:** the probabilistic part.
- `physics.py`: `received_power(...)` (D5), `fog_transmission(L, α)` (D10), `max_range(ρ, θ, α)` with Lambert W (D5), calibration of `K/P_th`
- `noise.py`: `detection_prob(P)`, `sample_measurement(true_R, P, rng)` (D6)
- `scan()` now returns noisy ranges, detected mask, `p_detect`, NaN for ∅
- All randomness goes through a seeded `np.random.Generator`
- Tests:
  - 10k samples of one beam → std ≈ `cσ_t/2` within 3%
  - Simulated max range (where P_D crosses 0.5) ≈ the analytic `R_max(ρ)` for ρ ∈ {0.1, 0.5, 0.9}
  - Fog: analytic `W₀(αR₀)/α` matches a numeric root-find **and** the `R_max(α)` reference table in D10 (2 decimals)
  - `dR_max/dα` matches `−R_max²/(1+αR_max)` by finite differences (D11)
  - α = 0 reduces to the clear-air formula

**Done when:** tests pass; a quick figure of P_D vs. range for a few ρ and α values.

> **Prompt:** *"Read plan/03_CODE_PLAN.md, plan/02_DERIVATIONS.md (D1, D5, D6, D10) and PROGRESS.md. Do Stage 3 only: received power, calibration, fog, detection probability, range noise, seeded RNG, plus the tests listed. Show me a P_D-vs-range figure, commit, update PROGRESS.md."*

### Stage 4 — Mirror + glass ray tracing (L, split into 4a/4b if needed)
**Goal:** multi-bounce tracing with ghosts and see-through glass.
- `physics.py`: `reflect(u, n)`, `fresnel(θ, n1, n2)` (D9; unpolarised)
- `raytrace.py`: recursive `trace(origin, dir, world, params, depth, L_so_far, throughput)` → `list[Return]` per the rules in §5
- `sensor.scan()` uses the tracer and the return policy; fills `is_ghost`, `apparent_point`
- `scenes.default_room()` gets the mirror and glass pane (plus an object behind the glass)
- `viz.draw_scan(ax, scan)`: true path legs (thin), apparent points (ghosts in a distinct colour/marker), dropouts marked
- Tests:
  - **Ghost theorem:** apparent point == reflection of the true hit across the mirror line (D8), to 1e-9
  - `reflect` preserves length; angle in = angle out
  - Fresnel table from D9 (0.040, 0.050 @45°, 0.074 @Brewster, 0.388 @80°) to 3 decimals
  - Head-on mirror → return at the mirror's true distance
  - Glass at 45° → chosen return comes from the wall *behind* (strongest policy)

**Done when:** a figure where you can clearly see ghost points behind the mirror and the wall behind the glass being detected; all tests pass.

> **Prompt (4a):** *"Read plan/03_CODE_PLAN.md (§5 physics rules) and plan/02_DERIVATIONS.md D8–D9, plus PROGRESS.md. Do Stage 4a: reflect, fresnel, recursive raytracer with mirror support, ghost-theorem test. Commit, update PROGRESS.md."*
> **Prompt (4b):** *"Continue with Stage 4b: glass (transmission + weak reflected branch), return policy, scene with mirror + glass, draw_scan, remaining tests, demo figure. Commit, update PROGRESS.md."*

### Stage 5 — Moving target + sensor paths + animation (M)
**Goal:** time.
- `world.py`: `MovingBox(trajectory: Callable[[t], (x, y, angle)])`; `World.at_time(t)`
- Sensor paths: waypoint interpolation; heading modes (`follow_path`, `fixed`, `spin`)
- `viz.animate(frames_fn, out_path, fps)`: a two-panel figure (left: top-down scene + rays + points; right: polar range plot or range-vs-beam), saved as GIF (pillow) and MP4 (ffmpeg from `imageio-ffmpeg`, already installed and tested)
- Mirror pose can also be time-varying (rotating mirror)

**Done when:** a short GIF of the LiDAR driving through the room while the box moves, with ghosts and glass behaviour visible.

> **Prompt:** *"Read plan/03_CODE_PLAN.md and PROGRESS.md. Do Stage 5: moving box, time-varying world, sensor paths and heading modes, two-panel animation export (GIF and MP4). Produce one demo GIF + MP4, commit, update PROGRESS.md."*

### Stage 6 — Experiment scripts (L, split by person)
**Goal:** the report media. Each script takes `--scene`, `--seed`, `--out figures/<person>/`, so each teammate makes **distinct** figures.

**Structure rule (so the notebook can reuse it):** each script exposes
- `run(scene="default", seed=0, **overrides) -> dict` with keys `figs` (name → Figure) and `data` (arrays/tables), with no file I/O;
- a `main()` CLI that calls `run()` and saves to `--out`.

The notebook calls `run()` directly, so there's one source of truth for every figure.

| Script | Produces | Theory overlay |
|---|---|---|
| `e1_sensor_position.py` | animation + "range of beam i vs. sensor x" plot | closed-form wall range (D3) |
| `e2_sensor_heading.py` | FOV wedge at several headings; targets entering/leaving FOV | `in_fov` predictions (D4) |
| `e3_moving_target.py` | box crossing; range-vs-time on affected beams; occlusion of the wall behind | predicted FOV/LOS entry/exit times |
| `e4_reflectivity.py` | P_D vs. range for several ρ; dark vs. bright wall point clouds | analytic R_max(ρ) (D5) |
| `e5_mirror.py` | ghost points; sweep mirror angle; ghost vs. reflected-true overlay | ghost theorem (D8) |
| `e6_glass.py` | sweep incidence angle: glass return power and which surface is reported | Fresnel curve (D9) |
| `e7_fog.py` | point clouds at α ∈ {0, 0.1, 0.2, 0.4}; effective max range vs. α | `W(αR₀)/α` (D5+D10) |
| `e8_noise_mc.py` | 1000-scan histogram for one beam; σ_R vs. range | Gaussian overlay (D6) |
| `e_bn_cpt.py` | Monte Carlo table `P(detect \| material, fog, range-bin)` as CSV + heatmap | the Bayesian network CPT (D7) |

Also: a `figures/<person>/params_table.md` dump of all parameters used, for the report's Procedure section.

> **Prompt:** *"Read plan/03_CODE_PLAN.md §6 Stage 6 and PROGRESS.md. Implement experiments E1, E3, E5, E7 (+ the BN CPT script) with the shared CLI, theory overlays and consistent figure style, writing to figures/ammar/. Commit, update PROGRESS.md."* (Partner, or a later session: E2, E4, E6, E8 to `figures/partner/` with a different scene variant.)

### Stage 7a — Interactive slider demo (M)
**Goal:** a "UI" you can play with live and screen-record for the media. It's a desktop matplotlib window, not a website.

**What it looks like:**
- **Left panel:** the top-down room, with the sensor, FOV wedge, rays, reported points, and ghost points highlighted.
- **Right panel:** a polar range plot of the current scan.
- **Readout line:** # detected, # dropouts, # ghosts, nearest range.

**Controls (`matplotlib.widgets`):**
- Sliders: sensor `x`, `y`, heading `θ`; fog `α`; mirror angle; box position; box reflectivity `ρ`.
- Radio buttons: return policy (`strongest` / `first`).
- Check boxes: show rays / FOV / visibility polygon / true hit points / noise on-off.
- A "new noise sample" button (re-draws the scan with a fresh RNG seed).

**Code:**
- `experiments/interactive.py`: `build(world=None, params=None) -> (fig, controls)`, `launch()`, `main()`.
- Every slider change re-runs `sensor.scan()` and updates the artists in place (241 beams is fast enough; no threading).
- `--record out.mp4`: plays a scripted sweep of the sliders (e.g. drive the sensor, then ramp the fog, then rotate the mirror) and saves it with `FuncAnimation`. You get a **reproducible demo video** without manual screen recording. Manual capture on macOS is Cmd+Shift+5.
- Tests: `build()` works under the Agg backend, and setting a slider value changes the scan (e.g. raising `α` lowers the detection count).

**Done when:** `uv run python -m experiments.interactive` opens a working window, and `--record figures/ammar/demo.mp4` writes a video.

> **Prompt:** *"Read plan/03_CODE_PLAN.md (Stage 7a) and PROGRESS.md. Build the interactive slider demo in experiments/interactive.py with the listed controls, the --record scripted-sweep MP4 option, and the Agg-backend tests. Commit, update PROGRESS.md."*

### Stage 7b — Walkthrough notebook (M)
**Goal:** a notebook that tells the whole story with figures inline. Your partner can explore in it, and it can be exported as the code appendix (like HW 2).

**Rule:** the notebook is a **viewer**. It contains no physics. Every cell calls the package or an experiment's `run()`.

**`notebooks/lidar_walkthrough.ipynb` sections:**
1. **Setup:** imports, `%matplotlib inline`, and a sys.path fix so the kernel finds `lidar_sim` when started from `notebooks/`. Plus a global seed.
2. **The sensor & scene:** parameter table, scene drawing.
3. **One scan, explained:** rays, returns, ghost vs. true points, a dropout.
4. **Sensor state → Z:** E1, E2.
5. **Target state & features → Z:** E3, E4.
6. **Mirror & glass:** E5, E6.
7. **Operating conditions → Z:** E7.
8. **Noise model check:** E8.
9. **Bayesian network:** CPT table + heatmap from `e_bn_cpt.run()`.
10. **Play with it:** `%matplotlib widget`, then `interactive.launch()` (the same slider demo as 7a, via ipympl).
11. **Summary table:** each theory prediction vs. simulated result (numbers for the report).

**Export for the appendix:**
```bash
uv run jupyter nbconvert --to html --execute notebooks/lidar_walkthrough.ipynb
```
Open the HTML in a browser and print to PDF.

**Done when:** "Restart & Run All" works top to bottom, the nbconvert command above succeeds, and the figures match the scripts' outputs for the same seed.

> **Prompt:** *"Read plan/03_CODE_PLAN.md (Stage 7b) and PROGRESS.md. Create notebooks/lidar_walkthrough.ipynb following the section list, as a thin viewer over experiments' run() functions and interactive.launch(). Execute it with nbconvert to HTML to verify. Commit (with outputs cleared or kept, your call), update PROGRESS.md."*

### Stage 8 — Stretch (optional, M each)
- **E9 CW phase ranging (D2):** `sensor.phase_range(true_R, f)` with wrap-around; two-frequency unwrap; plot measured vs. true range showing the saw-tooth at 3.22 m and the fix.
- **E10 beam divergence (D12):** 3 sub-rays per beam; linear and phasor range models; show mixed pixels at the box edge.
- Add a slider and a notebook section for whichever of these gets built.

### Stage 9 — Polish (S)
- README: install, run every experiment, the slider demo, the notebook, and where outputs go
- One command, `uv run python -m experiments.run_all --person ammar`, regenerates everything (figures + demo MP4)
- Code cleaned for the report appendix; docstrings cite the D-numbers; executed notebook HTML/PDF ready
- Final test run, final commit, tag `v1-submission`

---

## 7. Splitting work across sessions & usage limits

- **One stage per session.** Stage 4 and Stage 6 are the big ones, so split them as shown.
- **Always start a session with:** "Read plan/03_CODE_PLAN.md and PROGRESS.md, then do Stage N." That's enough context. You don't need to re-explain the project.
- **PROGRESS.md entry format:** date, stage, what was done, files touched, test status, known issues, next step.
- **Commit after each stage**, so a bad session is easy to roll back.
- **Minimum viable project** if time gets tight: Stages 0–4 plus E1, E5, E6, E7, E8. That already covers sensor state, target features, operating conditions, noise, FOV/LOS, mirror and glass. E3 (moving box) is the next most valuable, because it's the target-state arrow in Fig. 1. Stages 7a/7b are cheap once Stage 6 exists, because they only reuse `scan()` and the `run()` functions. Do them before any Stage 8 stretch work.

## 8. Order vs. the math

| Code stage | Needs these derivations finalised (or at least agreed) |
|---|---|
| 1 | D3 |
| 2 | D3, D4 |
| 3 | D1, D5, D6, D10 |
| 4 | D8, D9 |
| 6 | all of the above (for theory overlays), D7 for the CPT script |
| 7a, 7b | nothing new (they reuse Stages 1–6) |
| 8 | D2, D12 |

The equations are already written in `02_DERIVATIONS.md`, so coding doesn't have to wait for the partner's write-up. The partner's job is to **derive and explain** them for the report, and to flag anything that should change.
