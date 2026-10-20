# Code Plan — LiDAR Simulator (2D + 3D) with Localization (Python)

> **Status (2026-10-19): all stages 0–11 are implemented, tested (114 tests) and committed.** Per-stage details and results are in `PROGRESS.md`; calls made along the way are in `DECISIONS_NEEDED.md`. The stage prompts below still work for re-doing or extending a stage.

> **Note (2026-10-20 cleanup):** this is the original build plan. Files were since reorganised (`plan/` → `docs/`, presentation tools → `demo/`). The README's **Repository map** is the authoritative description of the current files.
>
> **How to use this doc:** each **Stage** below is sized to fit in roughly one Claude Code session. Start a session, paste the stage's **Prompt**, and let it run. Check the **Done when** list, then commit. Every stage appends a short entry to `PROGRESS.md`, so the next session (or a fresh context after hitting usage limits) knows where things stand.
>
> Read `01_PROJECT_PLAN.md` for the *why* and `02_DERIVATIONS.md` for the equations (referred to as D1–D14). Experiments are E1–E13.

---

## 1. The big picture (what the code does, in one breath)

1. A **world** is a set of surfaces, each with a **material** (matte / mirror / glass):
   - in 2D, line segments (walls, mirror, glass, box sides)
   - in 3D, triangle meshes (the same room extruded, plus floor, ceiling, a low box, a table)
2. A **backend** answers one question, fast and for many rays at once: *"for each ray, what's the first surface it hits, how far, and what's the normal?"*
   - 2D backend: our own numpy segment intersection (D3c)
   - 3D backend: trimesh + Embree (D14c)
   - Everything above the backend is shared.
3. A **sensor** has a pose and fires its beams. The 2D sensor fires 241 beams over 240°; the 3D sensor fires 16 rings × 360°.
4. The **batched tracer** traces all rays together, one bounce depth at a time:
   - **matte** → bounce light back (a return)
   - **mirror** → reflect and keep going
   - **glass** → mostly pass through, partly reflect
   - repeat up to B bounces
5. For every return we know the **path length** and the **received power** (inverse-square, reflectivity, angle, Fresnel/mirror losses, fog).
6. The sensor picks the **strongest** (or first) return per beam. It decides **detected or not** (power vs. threshold, with randomness) and adds **Gaussian range noise**.
7. **Output:** per-beam ranges (or ∅), "apparent" points (where the LiDAR *thinks* things are, ghosts included), and true hit points.
8. **Localization** (2D): a particle filter uses the same simulator as its measurement model, either physics-aware or naive, to estimate the robot's pose from scans.
9. **Experiments** vary one Fig. 1 variable at a time and save figures/animations, with theory curves overlaid. A **slider demo** and a **notebook** sit on top for exploring and presenting.

---

## 2. Tech stack & setup

- **Python 3.12** via `uv` (`uv sync` installs everything). System Python here is 3.9 with no scipy, so always use the project env.
- Version control: git in `Midterm/`, commit after every stage.

| Library | Group | Used for |
|---|---|---|
| `numpy` | core | all vector math, 2D ray–segment intersection, rotations, particle filter |
| `scipy` | core | `special.lambertw` (fog max range, D5/D10), `stats.norm` (detection probability, D6), `spatial.cKDTree` (optional ICP baseline, D13e) |
| `matplotlib` | core | every figure (incl. `mplot3d` for 3D point clouds), `FuncAnimation` for videos, `matplotlib.widgets` for the slider demo |
| `pillow` | core | GIF export |
| `imageio-ffmpeg` | core | a bundled ffmpeg binary, so MP4 export works with no system install. Set `plt.rcParams["animation.ffmpeg_path"] = imageio_ffmpeg.get_ffmpeg_exe()` once in `viz.py` |
| `tqdm` | core | progress bars for Monte Carlo runs (E8, E11, CPT estimation) |
| `trimesh` | core | 3D meshes: building the 3D room, boxes, table; face normals |
| `embreex` | core | Intel Embree ray tracing behind `trimesh.ray.ray_pyembree`. Tested on an M4: 28,800 rays in ~5 ms |
| `pytest` | dev | tests against the derivations |
| `jupyterlab` | dev | the walkthrough notebook (same as HW 2); `nbconvert` comes with it, for the HTML appendix export |
| `ipympl` (brings `ipywidgets`) | dev | `%matplotlib widget`, so the **same** slider demo runs inside the notebook |

**Considered and skipped on purpose:**
- **Gazebo / Isaac Sim / CARLA / AirSim:** either won't run on Apple Silicon without an NVIDIA GPU, or (Gazebo) has weak macOS support. Their LiDARs are ray-cast + Gaussian noise with no mirror/glass physics, which is exactly the gap Heiden et al. point out.
- **Open3D:** its 0.20 Mac wheel needs Homebrew's `libusb` (tested: import fails without it), so it would break on a teammate's machine. trimesh + Embree does the same ray casting with plain pip.
- **PyBullet:** no Apple Silicon wheel for Python 3.12.
- **shapely:** 2D ray–segment intersection *is* derivation D3(c), so we write and test it ourselves.
- **pgmpy:** HW 2 did BNs by hand, and our CPT is just Monte Carlo counting.
- **Streamlit / web UI:** not graded; the slider demo plus notebook cover it.
- **pandas / seaborn:** numpy + csv + matplotlib are enough.

## 3. Repo layout

```
Midterm/
├── docs/                     ← these docs
├── PROGRESS.md               ← running log, one entry per stage/session
├── pyproject.toml
├── lidar_sim/
│   ├── __init__.py
│   ├── geometry.py           ← vectors, rot2d, rot3d (ZYX Euler), cross2, intersection math   (D3, D14)
│   ├── materials.py          ← Material dataclass + presets (matte/mirror/glass)
│   ├── world.py              ← Segment, Box, World (2D), moving targets
│   ├── scenes.py             ← 2D scene builders: default_room(), variant_b(), …
│   ├── scenes3d.py           ← extrude(world2d) → 3D scene; default_room_3d() (+ low box, table)
│   ├── backends/
│   │   ├── base.py           ← Hits dataclass + SceneBackend protocol
│   │   ├── segments2d.py     ← vectorised numpy ray–segment backend (D3c)
│   │   └── mesh3d.py         ← trimesh + Embree backend (D14c)
│   ├── physics.py            ← reflect, fresnel, received_power, fog, max_range (D5, D8, D9, D10)
│   ├── raytrace.py           ← batched, breadth-first, backend-agnostic tracer → Returns
│   ├── sensor.py             ← LidarParams, scan(), FOV, visibility (2D; D1, D3, D4)
│   ├── sensor3d.py           ← LidarParams3D, ring directions, scan3d() (D14)
│   ├── noise.py              ← detection prob, range noise, sampling (D6)
│   ├── localization/
│   │   ├── motion.py         ← unicycle motion model, vectorised over particles (D13a)
│   │   ├── likelihood.py     ← naive vs physics-aware expected scans; robust log-likelihood (D13d)
│   │   ├── particle_filter.py← predict / update / low-variance resample / estimate (D13c)
│   ├── viz.py                ← 2D drawing, polar plots, animation helpers, figure style
│   ├── viz3d.py              ← range image, 3D scatter, bird's-eye view, rotating-camera animation
│   └── config.py             ← default parameter values (from 01_PROJECT_PLAN §4)
├── experiments/
│   ├── e1_sensor_position.py … e8_noise_mc.py, e_bn_cpt.py
│   ├── e11_localization.py
│   ├── e12_3d_fov.py
│   ├── e13_2d3d_consistency.py
│   ├── interactive.py        ← slider demo window (Stage 9a); launch() also works in the notebook
│   └── common.py             ← shared CLI args (--scene, --seed, --out, --style)
├── notebooks/
│   └── lidar_walkthrough.ipynb  ← thin "viewer" over the package (Stage 9b), exported to HTML for the appendix
├── tests/
│   ├── test_geometry.py, test_physics.py, test_raytrace.py, test_noise.py
│   ├── test_backends.py      ← 2D vs 3D consistency, Embree vs Möller–Trumbore
│   └── test_localization.py
└── figures/                  ← generated outputs (png/gif/mp4), split by person
    ├── ammar/
    └── partner/
```

## 4. Key data structures (the contract between modules)

Arrays are numpy. "M" = number of rays in a batch, "d" = 2 or 3.

```python
@dataclass
class Material:
    kind: Literal["matte", "mirror", "glass"]
    rho: float = 0.7          # diffuse reflectivity (matte; small residual for mirror/glass)
    reflectance: float = 0.95 # mirror only (R_m)
    n: float = 1.5            # glass only
    name: str = ""

@dataclass
class Segment:                 # 2D surface
    a: np.ndarray; b: np.ndarray   # endpoints (world frame)
    material: Material
    id: int

@dataclass
class World:                   # 2D scene
    segments: list[Segment]
    fog_alpha: float = 0.0         # Λ
    def at_time(self, t) -> "World": ...   # world with moving targets placed at time t
    def backend(self) -> "SceneBackend": ...  # Segments2D(self)

@dataclass
class Scene3D:                 # 3D scene
    mesh: trimesh.Trimesh          # all triangles
    tri_surface_id: np.ndarray     # (n_tri,) surface id per triangle
    materials: dict[int, Material]
    fog_alpha: float = 0.0
    def backend(self) -> "SceneBackend": ...  # Mesh3D(self)

@dataclass
class Hits:                    # one entry per input ray
    t: np.ndarray              # (M,) distance to first hit, np.inf if none
    point: np.ndarray          # (M, d)
    normal: np.ndarray         # (M, d) unit normal, flipped so dot(dir, normal) < 0
    surface_id: np.ndarray     # (M,) int, -1 if no hit

class SceneBackend(Protocol):
    dim: int                                          # 2 or 3
    def cast(self, origins, dirs) -> Hits: ...        # vectorised first-hit query
    def material(self, surface_id: int) -> Material: ...

@dataclass
class Returns:                 # struct of arrays, one row per return (a beam can have several)
    beam_idx: np.ndarray       # which beam this return belongs to
    path_length: np.ndarray    # one-way optical path L → measured range if chosen
    power: np.ndarray          # received power (D5), out-and-back losses + fog
    hit_point: np.ndarray      # (K, d) TRUE location of the final surface hit
    apparent_point: np.ndarray # (K, d) sensor_pos + L * initial_dir (ghost if bounced)
    surface_id: np.ndarray
    kind: np.ndarray           # 0 = diffuse, 1 = specular
    bounces: np.ndarray        # mirror/glass interactions before this return
    theta_inc: np.ndarray

@dataclass
class LidarParams:             # 2D (Hokuyo-like)
    fov_deg: float = 240; n_beams: int = 241
    r_min: float = 0.02; r_max_ref: float = 5.6   # clear-air max range for rho=0.9 head-on
    sigma_t: float = 67e-12      # → sigma_R ≈ 1 cm
    sigma_P_rel: float = 0.1     # power noise as fraction of P_th (for smooth P_D)
    max_bounces: int = 5
    return_policy: Literal["strongest", "first"] = "strongest"
    spec_lobe_deg: float = 1.5   # θ_w for specular head-on returns

@dataclass
class LidarParams3D(LidarParams):  # 3D (VLP-16-like geometry, same power/noise model)
    n_rings: int = 16; elev_min_deg: float = -15; elev_max_deg: float = 15
    az_step_deg: float = 1.0      # 360° → 360 columns → 5,760 rays
    height: float = 0.6; tilt_deg: float = 0.0   # mount height h and pitch tilt ϑ

@dataclass
class ScanResult:              # 2D: arrays shaped (N,); 3D: shaped (n_rings, n_az)
    pose: np.ndarray             # (x, y, θ)
    ranges: np.ndarray           # z, NaN = no return (∅)
    true_ranges: np.ndarray      # noise-free path length of chosen return, NaN if none
    detected: np.ndarray[bool]
    power: np.ndarray
    p_detect: np.ndarray         # P_D per beam
    points_world: np.ndarray     # apparent points from noisy ranges (what the LiDAR reports)
    true_hits: np.ndarray        # true hit points of chosen returns
    is_ghost: np.ndarray[bool]   # chosen return involved ≥1 mirror/glass reflection
    returns: Returns             # everything, for visualisation/debug
```

## 5. Physics & tracing decisions (so the code matches the math)

| Thing | Rule in code | Derivation |
|---|---|---|
| Range from time | `R = c·t/2`; noise is generated on `t` (σ_t) and converted, or directly `σ_R = c σ_t/2` | D1 |
| Beam directions (2D) | `u_i = rot2d(θ_s) @ [cos β_i, sin β_i]` | D3 |
| Beam directions (3D) | `u_jk = R_z(ψ) R_y(ϑ) R_x(φ_r) @ [cos ε cos φ, cos ε sin φ, sin ε]` | D14a |
| Intersection | 2D: cross-product formula, vectorised over rays × segments. 3D: Embree. Nearest `t > ε_self` | D3(c), D14(c) |
| Self-intersection offset | secondary rays start at `hit + ε_self·dir`; `ε_self = 1e-9` (2D, float64), `1e-4 m` (3D; Embree is float32) | — |
| Normal orientation | flip `n̂` so `dot(u, n̂) < 0`; `θ_inc = arccos(−u·n̂)` | — |
| Matte hit | diffuse return, **stop** tracing that ray | D5 |
| Mirror hit | specular-lobe return using `R_m` (head-on only) + child ray `u' = u − 2(u·n̂)n̂`, throughput × R_m | D5, D8 |
| Glass hit | specular-lobe return with `R_F(θ)` + child continuing **straight** (thin pane) with throughput × (1−R_F)² + (if bounces left) a reflected child with throughput × R_F | D9 |
| Power | `P = K·ρ·cosθ·T²·e^(−2αL)/L²` where T = one-way throughput, L = total path length | D5, D10 |
| Calibration | choose `K/P_th` so `R_max = 5.6 m` for ρ = 0.9 head-on, clear air (2D and 3D share it) | D5 |
| Detection | `P_D = Φ((P − P_th)/σ_P)`, sample Bernoulli | D6 |
| Range noise | `z = L + N(0, σ_R²)`, clip at `r_min` | D6 |
| Apparent point | `p + z·u_i` (this is where ghosts come from) | D8 |

**Batched breadth-first tracing** (not per-beam recursion):
1. Put all rays of a scan, or of all particles at once, into one batch.
2. Call `backend.cast()` once per bounce depth.
3. Compute the returns for this depth.
4. Spawn the child rays for mirror and glass hits; they form the next batch.
5. Repeat until B bounces or no rays are left.

This is what makes the particle filter (≈30k rays per step) and 3D (5,760 rays per scan) fast. Throughput is tracked **one-way** and squared for out-and-back. Prune any child whose best-case power is below `1e-3·P_th`. Pick the chosen return per beam with a group-by on `beam_idx` (max power, or min path length for the `first` policy).

---

## 6. Stages

Rough sizes: **S** ≈ small session, **M** ≈ medium, **L** ≈ big (split it as shown).

### Stage 0 — Setup (S) ✅ done
uv project, deps, skeleton, README, PROGRESS, smoke test, git. See PROGRESS.md.

### Stage 1 — Geometry + 2D backend + matte room + first picture (M)
**Goal:** rays hitting plain walls, drawn on screen, through the backend interface from day one.
- `geometry.py`: `rot2d`, `cross2`, `ray_segment_intersect` (D3c, vectorised: M rays × S segments with broadcasting; chunk if large)
- `backends/base.py`: `Hits`, `SceneBackend`. `backends/segments2d.py`: `Segments2D(world).cast(origins, dirs)`
- `materials.py`: presets `MATTE(ρ)`, `MIRROR`, `GLASS`
- `world.py`: `Segment`, `World` (+ `.backend()`), `Box` → 4 segments
- `scenes.py`: `default_room()` (8×6 m, matte walls, one dark and one bright section; no mirror/glass yet)
- `viz.py`: figure style, `draw_world(ax, world)`, `draw_rays(ax, origin, hits)`
- Script `experiments/smoke.py` (removed in the cleanup): 241 beams from the room centre, saves `figures/smoke.png`
- Tests: intersection hit / miss / parallel / behind-ray; closed-form wall range `R = (d − n̂·p)/(n̂·u)` matches `cast()`; batched result == one-ray-at-a-time result

**Done when:** `figures/smoke.png` shows the room with a fan of rays ending on the walls; tests pass.

> **Prompt:** *"Read docs/03_CODE_PLAN.md and PROGRESS.md. Do Stage 1 only (geometry, the backend interface + vectorised Segments2D backend, materials, world, default_room with matte walls, viz basics, smoke script, tests against the closed-form wall range from docs/02_DERIVATIONS.md D3). Show me the smoke figure, commit, update PROGRESS.md."*

### Stage 2 — Sensor pose, frames, FOV & LOS (M)
**Goal:** a real sensor object with a pose, a FOV wedge and a visibility polygon.
- `sensor.py`: `LidarParams`, `beam_directions(pose)`, `to_world(ranges, pose)`, `in_fov(point, pose)` (D4), `visibility_polygon(pose, world, n_rays=2000)` (glass treated as transparent, everything else opaque)
- `viz.py`: `draw_fov(ax, pose, params)` (wedge), `draw_visibility(ax, poly)`, `draw_sensor(ax, pose)`, `polar_ranges(ax, scan)`
- Noise-free `scan()` for matte only (power/noise come in Stage 3)
- Tests: `in_fov` edge cases (angle wrap at ±180°); `to_world` round-trips; rotating the sensor by Δθ shifts the range profile by Δθ in a circular room

**Done when:** a figure showing the sensor, its 240° wedge, the shaded visible region, and rays; a polar range plot.

> **Prompt:** *"Read docs/03_CODE_PLAN.md and PROGRESS.md. Do Stage 2 only: sensor pose, body↔world frames, FOV wedge, LOS visibility polygon, noise-free scan for matte walls, polar plot, tests. Save a demo figure, commit, update PROGRESS.md."*

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

> **Prompt:** *"Read docs/03_CODE_PLAN.md, docs/02_DERIVATIONS.md (D1, D5, D6, D10) and PROGRESS.md. Do Stage 3 only: received power, calibration, fog, detection probability, range noise, seeded RNG, plus the tests listed. Show me a P_D-vs-range figure, commit, update PROGRESS.md."*

### Stage 4 — Batched tracer: mirror + glass (L, split into 4a/4b)
**Goal:** multi-bounce tracing with ghosts and see-through glass, backend-agnostic and batched (§5).
- `physics.py`: `reflect(u, n)`, `fresnel(θ, n1, n2)` (D9; unpolarised). Both dimension-agnostic.
- `raytrace.py`: `trace(backend, origins, dirs, params, fog) -> Returns` with the breadth-first loop in §5 and the per-beam return selection
- `sensor.scan()` uses the tracer and the return policy; fills `is_ghost`, `apparent_point`
- `scenes.default_room()` gets the mirror and glass pane (plus an object behind the glass)
- `viz.draw_scan(ax, scan)`: true path legs (thin), apparent points (ghosts in a distinct colour/marker), dropouts marked
- Tests:
  - **Ghost theorem:** apparent point == reflection of the true hit across the mirror line (D8), to 1e-9
  - `reflect` preserves length; angle in = angle out
  - Fresnel table from D9 (0.040, 0.050 @45°, 0.074 @Brewster, 0.388 @80°) to 3 decimals
  - Head-on mirror → return at the mirror's true distance
  - Glass at 45° → chosen return comes from the wall *behind* (strongest policy)
  - Batch of many scans (e.g. 500 poses) gives the same results as scanning one pose at a time
  - Speed: 500 poses × 61 beams with mirror + glass in < 50 ms

**Done when:** a figure where you can clearly see ghost points behind the mirror and the wall behind the glass being detected; all tests pass.

> **Prompt (4a):** *"Read docs/03_CODE_PLAN.md (§5 physics & tracing rules) and docs/02_DERIVATIONS.md D8–D9, plus PROGRESS.md. Do Stage 4a: reflect, fresnel, the batched breadth-first backend-agnostic tracer with mirror support, return selection, ghost-theorem test. Commit, update PROGRESS.md."*
> **Prompt (4b):** *"Continue with Stage 4b: glass (transmission + weak reflected branch), return policy switch, scene with mirror + glass, draw_scan, remaining tests including the multi-pose batch and speed tests, demo figure. Commit, update PROGRESS.md."*

### Stage 5 — Moving target + sensor paths + animation (M)
**Goal:** time.
- `world.py`: `MovingBox(trajectory: Callable[[t], (x, y, angle)])`; `World.at_time(t)`
- Sensor paths: waypoint interpolation; heading modes (`follow_path`, `fixed`, `spin`). Also export the path as unicycle controls `(v_k, ω_k)` for Stage 7.
- `viz.animate(frames_fn, out_path, fps)`: a two-panel figure saved as GIF (pillow) and MP4 (ffmpeg from `imageio-ffmpeg`, already installed and tested)
  - left panel: top-down scene + rays + points
  - right panel: polar range plot or range-vs-beam
- Mirror pose can also be time-varying (rotating mirror)

**Done when:** a short GIF of the LiDAR driving through the room while the box moves, with ghosts and glass behaviour visible.

> **Prompt:** *"Read docs/03_CODE_PLAN.md and PROGRESS.md. Do Stage 5: moving box, time-varying world, sensor paths and heading modes (with unicycle-control export), two-panel animation export (GIF and MP4). Produce one demo GIF + MP4, commit, update PROGRESS.md."*

### Stage 6 — Experiment scripts E1–E8 + CPT (L, split by person)
**Goal:** the core report media. Each script takes `--scene`, `--seed`, `--out figures/<person>/`, so each teammate makes **distinct** figures.

**Structure rule (so the notebook can reuse it):** each script exposes
- `run(scene="default", seed=0, **overrides) -> dict` with keys `figs` (name → Figure) and `data` (arrays/tables), with no file I/O;
- a `main()` CLI that calls `run()` and saves to `--out`.

The notebook calls `run()` directly, so there's one source of truth for every figure. The same rule applies to E11–E13.

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

> **Prompt:** *"Read docs/03_CODE_PLAN.md §6 Stage 6 and PROGRESS.md. Implement experiments E1, E3, E5, E7 (+ the BN CPT script) with run()/main(), the shared CLI, theory overlays and consistent figure style, writing to figures/ammar/. Commit, update PROGRESS.md."* (Partner, or a later session: E2, E4, E6, E8 to `figures/partner/` with a different scene variant.)

### Stage 7 — Localization: particle filter, naive vs physics-aware (L, split into 7a/7b) — Upgrade U1
**Goal:** the sensor model gets *used*: three particle filters estimate the robot's pose from scans, and we show where the naive model fails.

**7a — filter core**
- `localization/motion.py`: `unicycle_step(X, u, dt, rng, sigma_v, sigma_w)`, vectorised over particles (D13a)
- `localization/likelihood.py`:
  - `make_model(world, kind)`:
    - `"physics"` = the real world
    - `"naive"` = a copy where mirror and glass become opaque matte (ρ = default)
  - `expected_scan(model, poses, beam_subset)`: one batched trace for all particles → noise-free ranges + `P_D`
  - `log_likelihood(z, expected, p_detect, sigma_R, eps, R_max, temper, use_mixture)` (D6 + D13d)
  - Readings of ∅ use `p = (1−ε)(1−P_D) + ε_∅` with `ε_∅ = 0.01`
- `localization/particle_filter.py`: `ParticleFilter(model, M, rng, …)` with `predict(u)`, `update(z)`, `resample()` (low-variance), `estimate()` (weighted mean + circular mean for θ), `n_eff`
- Tests:
  - In a plain matte room the filter converges: error < 5 cm within 50 steps from a 0.5 m / 20° initial error
  - Low-variance resampling keeps expected copy counts ≈ `M·w` (statistical test)
  - Naive == physics when the scene has no mirror/glass
  - With the mixture, the per-beam log-likelihood gap is ≤ ≈ 8.35 nats (D13d)
  - The circular mean is correct across ±π
  - Speed: one physics-aware update (500 particles × 61 beams) < 50 ms

**7b — experiment E11**
- Ground truth: the Stage 5 path (a loop past the mirror and along the glass) → true poses and controls. True scans come from the physics simulator with noise; odometry = true controls + noise.
- Three filters on identical data:
  - **N1:** naive, no mixture
  - **N2:** naive + mixture
  - **P:** physics-aware + mixture
- 20 seeds each, with tqdm progress.
- Outputs:
  - particle-cloud animation for one seed (all three side by side)
  - position and heading error vs. time (mean ± std band)
  - N_eff vs. time
  - table (CSV + markdown): RMSE, final error, % runs diverged (final error > 0.5 m)
- Optional, not built (DECISIONS #8): `localization/icp.py` (cKDTree + Kabsch SVD) and an ICP-odometry drift plot near the mirror (D13e)

**Done when:** tests pass; E11 figures and table exist in `figures/ammar/`. P clearly beats N1, and N2 sits in between (or, if not, you understand and write up why).

> **Prompt (7a):** *"Read docs/03_CODE_PLAN.md Stage 7, docs/02_DERIVATIONS.md D13 and PROGRESS.md. Do Stage 7a: unicycle motion model, naive vs physics-aware likelihood models with the robust mixture and tempering, the particle filter with low-variance resampling, and the listed tests. Commit, update PROGRESS.md."*
> **Prompt (7b):** *"Continue with Stage 7b: experiment E11 (three filters N1/N2/P, 20 seeds, error/N_eff plots, results table, particle-cloud animation) with run()/main(). Optional ICP baseline only if time allows. Commit, update PROGRESS.md."*

### Stage 8 — 3D LiDAR mode (L, split into 8a/8b) — Upgrade U2
**Goal:** a 16-ring 3D LiDAR on the same physics, validated against the 2D model.

**8a — backend, scenes, sensor**
- `backends/mesh3d.py`: `Mesh3D(scene3d)` wrapping `trimesh.ray.ray_pyembree.RayMeshIntersector`
  - `cast()` uses `intersects_location(..., multiple_hits=False)` and maps `tri_idx → surface_id`
  - normals come from `face_normals`, flipped to face the ray
  - rays with no hit get `t = inf`
- `scenes3d.py`:
  - `extrude(world2d, height=3.0)`: each segment → 2 triangles; adds floor and ceiling; carries materials over
  - `default_room_3d()`: extruded room + low box (0.3 m) + table (top 0.75 m, 4 legs)
- `geometry.rot3d(yaw, pitch, roll)` (ZYX); `sensor3d.py`: `LidarParams3D`, `ring_directions(pose, params)`, `scan3d(scene, pose, params, rng)`. Same tracer and noise; results reshaped to `(16, 360)`.
- Tests:
  - Embree vs our numpy Möller–Trumbore on 10k random rays (agree to 1e-4 m)
  - **2D-slice identity:** ε = 0 ring == 2D scan on the extruded scene (noise-free, 1e-4 m)
  - `R_3D = R_2D / cos ε` for wall hits
  - Floor-ring radius = `h/tan 15°` (2.24 m) within 1 cm
  - Low box: never hit by the 2D scan; hit by 3D at 1.2 m but not at 1.0 m (blind zone 1.12 m)
  - Ghost theorem in 3D (reflection across the mirror plane)
  - Speed: one 3D scan with mirror + glass < 20 ms

**8b — viz + experiments E12, E13**
- `viz3d.py`:
  - `range_image(ax, scan3d)`: rings × azimuth heatmap; ∅ in grey
  - `scatter3d(ax, scan3d, color_by="height"|"ghost")`
  - `birds_eye(ax, scan3d)`
  - `rotate_animation(...)`: a rotating-camera GIF/MP4
- `e12_3d_fov.py`:
  - low box + table seen by 3D, missed by 2D (side-by-side)
  - floor ring vs. mount tilt (0°, 5°) with the `h/tan(15°+ϑ)` overlay
  - 3D ghost room behind the mirror
  - fog thinning in 3D
- `e13_2d3d_consistency.py`: ε = 0 ring vs 2D difference plot (should be ~0); `R_3D/R_2D` vs elevation with the `1/cos ε` overlay

**Done when:** tests pass; E12 and E13 figures (plus one rotating 3D point-cloud MP4) exist.

> **Prompt (8a):** *"Read docs/03_CODE_PLAN.md Stage 8, docs/02_DERIVATIONS.md D14 and PROGRESS.md. Do Stage 8a: add trimesh + embreex usage via the Mesh3D backend, scenes3d (extrude + default_room_3d with low box and table), rot3d and sensor3d, and the listed tests (Möller–Trumbore cross-check, 2D-slice identity, floor ring, blind zone, 3D ghost theorem, speed). Commit, update PROGRESS.md."*
> **Prompt (8b):** *"Continue with Stage 8b: viz3d (range image, 3D scatter, bird's-eye, rotating animation) and experiments E12 and E13 with run()/main() and theory overlays, writing to figures/partner/ (E12) and figures/ammar/ (E13). Commit, update PROGRESS.md."*

### Stage 9a — Interactive slider demo (M)
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
- Optional: a **2D/3D toggle** that swaps the right panel for the 3D range image.

**Code:**
- `demo/interactive.py`: `build(world=None, params=None) -> (fig, controls)`, `launch()`, `main()`.
- Every slider change re-runs `scan()` and updates the artists in place (fast enough; no threading).
- `--record out.mp4`: plays a scripted sweep of the sliders (e.g. drive the sensor, then ramp the fog, then rotate the mirror) and saves it with `FuncAnimation`. You get a **reproducible demo video** without manual screen recording. Manual capture on macOS is Cmd+Shift+5.
- Tests: `build()` works under the Agg backend, and setting a slider value changes the scan (e.g. raising `α` lowers the detection count).

**Done when:** `uv run python -m demo.interactive` opens a working window, and `--record figures/ammar/demo.mp4` writes a video.

> **Prompt:** *"Read docs/03_CODE_PLAN.md (Stage 9a) and PROGRESS.md. Build the interactive slider demo in demo/interactive.py with the listed controls, the --record scripted-sweep MP4 option, and the Agg-backend tests. Commit, update PROGRESS.md."*

### Stage 9b — Walkthrough notebook (M)
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
10. **Localization:** E11 (use fewer seeds in the notebook to keep it fast; note it).
11. **3D mode:** E12, E13.
12. **Play with it:** `%matplotlib widget`, then `interactive.launch()` (the same slider demo as 9a, via ipympl).
13. **Summary table:** each theory prediction vs. simulated result (numbers for the report).

**Export for the appendix:**
```bash
uv run jupyter nbconvert --to html --execute notebooks/lidar_walkthrough.ipynb
```
Open the HTML in a browser and print to PDF.

**Done when:** "Restart & Run All" works top to bottom, the nbconvert command above succeeds, and the figures match the scripts' outputs for the same seed.

> **Prompt:** *"Read docs/03_CODE_PLAN.md (Stage 9b) and PROGRESS.md. Create notebooks/lidar_walkthrough.ipynb following the section list, as a thin viewer over the experiments' run() functions and interactive.launch(). Execute it with nbconvert to HTML to verify. Commit (with outputs cleared or kept, your call), update PROGRESS.md."*

### Stage 10 — Stretch (optional, M each)
- **E9 CW phase ranging (D2):** `sensor.phase_range(true_R, f)` with wrap-around; two-frequency unwrap; plot measured vs. true range showing the saw-tooth at 3.22 m and the fix.
- **E10 beam divergence (D12):** 3 sub-rays per beam; linear and phasor range models; show mixed pixels at the box edge.
- Add a slider and a notebook section for whichever of these gets built.

### Stage 11 — Polish (S)
- README: install, run every experiment, the slider demo, the notebook, and where outputs go
- One command, `uv run python -m experiments.run_all --person ammar`, regenerates everything (figures + demo MP4)
- Code cleaned for the report appendix; docstrings cite the D-numbers; executed notebook HTML/PDF ready
- Final test run, final commit, tag `v1-submission`

---

## 7. Splitting work across sessions & usage limits

- **One stage (or half-stage) per session.** Stages 4, 6, 7 and 8 are the big ones, so split them as shown.
- **Always start a session with:** "Read docs/03_CODE_PLAN.md and PROGRESS.md, then do Stage N." That's enough context. You don't need to re-explain the project.
- **PROGRESS.md entry format:** date, stage, what was done, files touched, test status, known issues, next step.
- **Commit after each stage**, so a bad session is easy to roll back.
- **Priority if time gets tight** (see also the cut order in `01_PROJECT_PLAN.md` §9):
  1. **Core:** Stages 0–6. Never cut.
  2. **Localization (Stage 7):** the most rubric value per hour of the upgrades.
  3. **3D (Stage 8):** at minimum 8a + E13 + one E12 figure.
  4. **Notebook (9b), then slider demo (9a):** cheap because they only reuse `scan()` and the `run()` functions.
  5. **Stretch (Stage 10).**
- The batched-tracer design in Stages 1 and 4 is what makes Stages 7 and 8 cheap. Don't shortcut it with per-beam recursion.

## 8. Order vs. the math

| Code stage | Needs these derivations finalised (or at least agreed) |
|---|---|
| 1 | D3 |
| 2 | D3, D4 |
| 3 | D1, D5, D6, D10 |
| 4 | D8, D9 |
| 6 | all of the above (for theory overlays), D7 for the CPT script |
| 7 | D13 (and D6) |
| 8 | D14 |
| 9a, 9b | nothing new (they reuse earlier stages) |
| 10 | D2, D12 |

The equations are already written in `02_DERIVATIONS.md`, so coding doesn't have to wait for the partner's write-up. The partner's job is to **derive and explain** them for the report, and to flag anything that should change.
