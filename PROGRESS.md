# Progress Log

One entry per stage/session (see plan/03_CODE_PLAN.md §7). Newest at the bottom.

---

## 2026-10-19 — Stage 0: Setup ✅

**Done**
- uv project (`pyproject.toml`, `uv.lock`, `.python-version` = 3.12). Deps: numpy, scipy, matplotlib, pillow; dev: pytest.
- `[tool.uv] package = false` + pytest `pythonpath = ["."]`. Everything runs from `Midterm/` with `uv run …`; experiments run as `python -m experiments.<name>`.
- Skeleton: `lidar_sim/` modules (docstrings only, each says which stage and derivation fills it), `experiments/`, `tests/`, `figures/ammar/`, `figures/partner/`.
- `tests/test_smoke.py`: imports every module and checks the scientific stack (including `scipy.special.lambertw`, needed for D5/D10).
- README, `.gitignore` (venv, caches, `.DS_Store`, top-level course PDFs), `git init`, first commit.

**Tests:** `uv run pytest` → all pass.

**Notes**
- The course folder is now `CEE5810` (graduate level), so D11 (dependency analysis) in `plan/02_DERIVATIONS.md` is **required**, not stretch.
- No MP4 export yet: there's no system ffmpeg. GIFs via pillow will work; add `imageio-ffmpeg` in Stage 5 if MP4 is wanted.

**Next:** Stage 1. Geometry, materials, world, `default_room()` with matte walls, viz basics, smoke figure, and tests against the closed-form wall range (D3).

## 2026-10-19 — Derivations page + reconciliation ✅

**Done**
- Partner page published (private, needs sharing from its Share menu): https://claude.ai/artifact/7VtNLZAhL791QDsZSSYCva. It's built from `plan/02_DERIVATIONS.md`, with MathJax math and BN and ghost-point diagrams.
- The page session re-checked every number in D1–D11 (all headline values correct) and fixed 16 points. All 16 are ported into `plan/02_DERIVATIONS.md`:
  - D7 BN: missing R_i→z_i edge, grouped G_i = (R_i, θ_inc, T_path), corrected d-separation claims
  - D6: likelihood derivation from L3's joint
  - D5: mirror lobe uses R_m; target-specific R₀; principal branch W₀
  - D4: tilted-face beam-gap condition
  - D8: virtual-sensor caveats
  - D9: must/should split, polarisation assumption, glass range bias, "plastic sheet" correction
  - D10: R_max(α) table
  - D11: explicit ∂P_D/∂ρ, ∂P_D/∂α, dR_max/dα, box-face sensitivity, CRLB assumptions
  - D12: phasor (exact) vs. linear mixed-pixel model
  - Notation: 13 symbols + name clashes
- New values recomputed independently in this session before porting. `plan/03_CODE_PLAN.md`: mirror lobe uses R_m; Stage 3 tests check the D10 table and dR_max/dα.

**Next:** Stage 1 (unchanged).

## 2026-10-19 — Libraries + demo/notebook added to the plan ✅

**Done**
- New libraries:
  - core: `imageio-ffmpeg` (bundled ffmpeg, so MP4 export works; tested a matplotlib MP4 + GIF), `tqdm`
  - dev: `jupyterlab`, `ipympl` (+ `ipywidgets`)
- Resolves the Stage 0 "no MP4" note.
- `plan/03_CODE_PLAN.md`:
  - §2 is now a library table with the ones deliberately skipped
  - Stage 6 scripts must expose `run()` for reuse
  - new **Stage 7a** slider demo (`experiments/interactive.py`, `--record` → MP4)
  - new **Stage 7b** walkthrough notebook (`notebooks/lidar_walkthrough.ipynb`, nbconvert → HTML appendix)
  - stretch is now Stage 8, polish Stage 9
- `plan/01_PROJECT_PLAN.md` and README updated to match. Added `notebooks/`.

**Tests:** `uv run pytest` → all pass.

**Next:** Stage 1 (unchanged).

## 2026-10-19 — Upgrades added: localization (U1) + 3D mode (U2) ✅

**Decided:** no big simulator platform. Gazebo, Isaac, CARLA and AirSim either won't run on Apple Silicon or only do ray-cast + noise. Open3D's Mac wheel needs Homebrew `libusb`. We add depth instead.

**Done**
- `plan/02_DERIVATIONS.md`:
  - **D13** localization: unicycle model (L2), Bayes filter from Bayes + Markov (L4), particle filter, robust mixture likelihood, bounded penalty ≈ 8.35 nats vs 5000 without, tempering, optional ICP
  - **D14** 3D: Euler-angle ring directions (L5), 3D FOV, floor ring 2.24 m, low-box blind zone 1.12 m, Möller–Trumbore, physics carries over, 2D = ε=0 slice
  - Notation and checklist updated. All new numbers verified numerically.
- `plan/01_PROJECT_PLAN.md`: pitch, upgrades table, 3D + localization parameters, 3D scene objects, experiments **E11–E13**, work split (Ammar E11, partner E12, E13 shared), report outline, a 12-day timeline with cut order, course concepts, references.
- `plan/03_CODE_PLAN.md` rewritten around a **backend-agnostic, batched breadth-first tracer**:
  - backends: `Segments2D` (numpy) and `Mesh3D` (trimesh + Embree)
  - **new Stage 7** localization (7a/7b) and **Stage 8** 3D (8a/8b)
  - the slider demo and notebook move to **9a/9b**, stretch to **10**, polish to **11**
- Deps: `trimesh`, `embreex`. New skeleton modules: `backends/`, `localization/`, `scenes3d.py`, `sensor3d.py`, `viz3d.py`. Smoke test imports them and checks a real Embree hit.

**Tests:** `uv run pytest` → 28 passed.

**Note:** the partner page (artifact) was built before D13/D14 existed, so it doesn't include them yet.

**Next:** Stage 1. Geometry + the backend interface + the vectorised `Segments2D` backend, matte room, smoke figure, D3 tests.

## 2026-10-19 — Stage 1: geometry + 2D backend + matte room ✅

**Done**
- `config.py` (all constants)
- `geometry.py`: rot2d, rot3d (ZYX), cross2, wrap_angle, vectorised ray–segment (D3c), Möller–Trumbore (D14c), reflect_point
- `materials.py`: Material + presets + MaterialArrays
- `backends/base.py` (Hits, protocol) and `backends/segments2d.py` (vectorised, chunked, normals flipped to face the ray)
- `world.py`: Segment, box_segments, ping_pong, MovingBox, World with at_time, backend(opaque_only), naive(), with_fog
- `scenes.py`: default_room, variant_b, glass_corridor, single_wall, glass_test
- `viz.py`: style + palette, draw_world, draw_sensor, draw_rays, draw_fov, draw_visibility, save
- `experiments/smoke.py` → `figures/smoke.png`

**Tests:** 35 passed (D3 closed form, hit/miss/parallel/behind, batch == single, chunking, closed room, rotations, Möller–Trumbore).

**Next:** Stage 2.

## 2026-10-19 — Stages 2–4: sensor/FOV/LOS, power/noise/fog, batched mirror+glass tracer ✅

**Done**
- `physics.py`: reflect, fresnel (TIR-safe), pane_transmission, fog_transmission, diffuse/specular power, clear_max_range, max_range (Lambert W₀), dmax_range_dalpha
- `noise.py`: range_noise_std, detection_prob (Φ), sample_measurement
- `raytrace.py`:
  - `trace()` is batched breadth-first: one backend call per bounce depth, diffuse + specular-lobe returns, mirror/glass child rays, pruning
  - `select()` implements the strongest/first policies
- `sensor.py`: LidarParams, ScanResult, beam_directions (single or many poses), to_world, trace_beams, scan, expected_scan, in_fov (with LOS, glass transparent), visibility_polygon
- `viz.py`: draw_scan (real / ghost / true source / dropouts), polar_ranges, legend_below. The FOV is now an outline and the visibility polygon is filled.
- `experiments/demo_scan.py` → `figures/demo_scan.png`: 13 ghosts outside the room, linked to their true sources on the top wall
- Scene change: mirror at 45° (DECISIONS #3)

**Tests:** 62 passed. Speed: 500 poses × 61 beams with mirror + glass = **18.6 ms**.

**Next:** Stage 5.

## 2026-10-19 — Stage 5: moving target, paths, animation ✅

**Done**
- `localization/motion.py`: unicycle_step (deterministic, vectorised) + sample_motion (D13a)
- `paths.py`:
  - waypoint_path (follow_path / fixed / spin headings)
  - pure_pursuit: unicycle driver returning poses + controls that reproduce them exactly
- `viz.animate` (GIF via pillow at 80 dpi, MP4 via the bundled ffmpeg)
- `experiments/demo_animation.py` → `figures/demo_drive.{gif,mp4}`: driving through the default room with the moving box, ghosts and glass
- GIF/MP4 git-ignored (DECISIONS #6)

**Tests:** 66 passed. Includes pure-pursuit controls reproducing poses, motion noise std, and the moving box changing the scan.

**Next:** Stage 6.

## 2026-10-19 — Stage 6: experiments E1–E8 + BN CPT + params table ✅

**Done**
- `experiments/common.py`:
  - `--person ammar|partner` maps to the default / variant_b scene and `figures/<person>/`
  - per-scene presets, so media are distinct
  - CSV + Markdown tables; `finish()` saves everything
- Experiments. Each has `run()` for the notebook plus a CLI.

  | Exp. | What it shows |
  |---|---|
  | **E1** | closed form matches to 4e-16 m; parallel motion → constant range; dark-wall dropouts; occlusion |
  | **E2** | FOV prediction vs simulation: 100% agreement (default), 99.4% (variant_b: one discrete-beam edge case) |
  | **E3** | predicted box-on-beam intervals == simulated |
  | **E4** | analytic vs simulated R_max(ρ) to 1e-13; per-beam MC detection RMS 0.018; R₀√cos θ to 3e-13 |
  | **E5** | ghost theorem 2.5e-15 m (noise-free); ghost noise std 1.00 cm; mirror-angle sweep |
  | **E6** | T_pane² matches Fresnel to 8e-16; glass reported only ≤ 4–5° from head-on; through-glass points in the room |
  | **E7** | fog max range matches W₀(αR₀)/α to 5e-13 m; room detection 0.53 → 0 by α = 0.25 |
  | **E8** | KS p = 0.65; σ̂ = 1.003 cm; all 95% CIs contain 1 cm; \|bias\| ≤ 1 mm |
  | **CPT** | P(D \| material, fog, range) by Monte Carlo (mirror/glass ≈ 5–12% detected at random incidence) |
  | params_table | all parameters and scene surfaces, for the Procedure section |
- Tracer: records the `reflector` surface; child path length includes the self-offset (exact ghost theorem). Scans flag `transmitted` (seen through glass).
- Scenes: variant_b mirror re-angled to 135° (so it produces ghosts); trajectories carry their `period`.

**Tests:** 85 passed in ~5 s, including every experiment in fast mode for both scenes.

**Next:** Stage 7, localization.

## 2026-10-19 — Stage 7: particle-filter localization (E11) ✅

**Done**
- `localization/likelihood.py`:
  - make_model (naive = mirror/glass → opaque matte)
  - beam_log_likelihood in the log domain, robust mixture (ε = 0.05, ε_∅ = 0.01) with an exact no-mixture mode
  - log_likelihood (tempering); mixture_gap
- `localization/particle_filter.py`: ParticleFilter (predict, update with N_eff-triggered low-variance resampling + roughening, estimate with circular mean), init_particles
- `experiments/e11_localization.py`:
  - three filters on identical data (N1 naive, N2 naive + mixture, P physics-aware), 20 seeds, M = 300, 31 beams, σ_lik = 5 cm
  - figures: error vs time (median + IQR, log scale), N_eff, particle snapshots, `e11_particles.gif`
  - tables
- **Results** (mean position RMSE over 20 runs):

  | Scene | N1 | N2 | P | Naive mispredicts |
  |---|---|---|---|---|
  | glass corridor | 35 cm | 11.6 cm | **1.1 cm** | 40% of beams |
  | default room | 33 cm | 0.97 cm | **0.69 cm** | 15% of beams |

  The corridor error plot shows N2 drifting exactly where no end wall is within range: through-glass beams are its only along-corridor information.
- D13 updated (σ_lik) and D5 updated (K_s). DECISIONS #7–9.

**Tests:** 92 passed. Includes convergence, unbiased resampling, naive == physics without specular surfaces, mixture bound 6.745 / 8.35 nats, circular mean, and update speed (5.3 ms).

**Next:** Stage 8 (3D).

## 2026-10-19 — Stage 8: 3D LiDAR mode (E12, E13) ✅

**Done**
- `backends/mesh3d.py`: trimesh + Embree first-hit backend (normals flipped to face the ray)
- `scenes3d.py`:
  - Scene3D (quads/boxes with per-surface materials, cached backend)
  - `extrude()` (2D segments → vertical walls with the same surface ids, plus floor and ceiling)
  - add_table, default_room_3d / variant_b_3d (with a 0.3 m low box and a table, top at 0.75 m)
- `sensor3d.py`: LidarParams3D (16 rings ±15°, 1° azimuth, h = 0.6 m, mount tilt, elevation override), ring_directions (ZYX Euler), scan3d (same tracer, noise and selection as 2D)
- `viz3d.py`: range image, 3D scatter (height / ghost), bird's-eye view with 3D-object footprints, rotating animation
- **E12** (partner's):

  | Check | Result |
  |---|---|
  | points on low box, 2D plane vs 3D | 0 vs 76–81 |
  | points on table, 2D plane vs 3D | 6–8 vs 57–94 |
  | floor ring vs exact formula, tilt 0° / 5° | 2.239 / 1.648 m, matching to 1e-15 |
  | 3D ghosts at the mirror pose | 445–540 |
  | rotating MP4/GIF | rendered |
- **E13** (Ammar's):

  | Check | Result |
  |---|---|
  | ε = 0 ring vs 2D | match to 1e-15 m; 100% same detections; ghosts identical |
  | R_3D/R_2D vs 1/cos ε | match to 7e-16 |
- One 3D scan (5,760 rays, mirror + glass): **3.1 ms**.

**Tests:** 104 passed. 3D tests cover Embree vs Möller–Trumbore, the slice identity, 1/cos ε, floor ring + tilt, blind zone at 1.12 m, 3D ghost theorem and speed. Experiment smoke tests now include E11–E13.

**Next:** Stage 9 (slider demo + notebook).

## 2026-10-19 — Stage 9a: interactive slider demo ✅

**Done**
- `experiments/interactive.py`:
  - `build()`: scene + FOV + visible region + scan on the left, polar profile on the right, and a readout (detected / dropouts / ghosts / through-glass / nearest)
  - 7 sliders (x, y, θ, fog, mirror angle, box position, box ρ), a return-policy radio, check boxes (rays, FOV, visible region, ghost sources, noise) and a "new noise" button
  - `set_values()` (batched update), `launch()` (desktop or notebook via ipympl)
  - `record()` / `--record PATH`: scripted sweep (drive, then fog ramp, then mirror rotation) → MP4/GIF
- `figures/ammar/demo.mp4` recorded (120 frames, ~33 s to render)

**Tests:** 2 new (headless build, fog lowers detections, sensor move changes the scan, variant_b builds).

**Next:** Stage 9b (notebook).

## 2026-10-19 — Stage 9b: walkthrough notebook ✅

**Done**
- `notebooks/make_notebook.py` generates `notebooks/lidar_walkthrough.ipynb`, a thin viewer. Sections: setup, sensor & scene (params table), one scan explained, then E1–E8, CPT, E11 (3 seeds), E12–E13, the interactive cell (off by default), and a theory-vs-simulation summary table.
- `experiments/common.markdown_table` factored out for the notebook.
- Executes headless with `uv run jupyter nbconvert --to html --execute notebooks/lidar_walkthrough.ipynb` in **~24 s** with no errors. Summary: E1 4.4e-16 m, E2 1.000, E4 4.6e-13 m, E5 2.5e-15 m, E6 7.8e-16, E7 4.7e-13 m, E8 KS p 0.65, E11 corridor 5.4 vs 1.2 cm (3 seeds), E13 1.3e-15 m.

**Next:** Stage 10 stretch (E9 CW phase, E10 beam divergence) and Stage 11 polish.

## 2026-10-19 — Stage 10: stretch experiments E9 (CW phase) + E10 (beam divergence) ✅

**Done**
- `lidar_sim/phase.py`: ambiguity_range, phase_of, phase_range, two_frequency_range (beat unwrap + f1 refine), mixed_range (linear / exact phasor)
- **E9**:
  - R_amb = 3.22 / 2.82 m; beat 22.54 m
  - two-frequency unwrap RMS **0.99 cm** (the 1 cm noise)
  - 0.4% unwrap failures, only within 7 cm of 0 m and at 22.4–22.5 m (edges of the unambiguous interval)
- **E10**:
  - 3 sub-rays (5 mrad half-angle) at a box edge; 229 mixed-pixel beams between box and wall
  - linear vs phasor differ by up to 6.8 cm; D12 example reproduced (1.30 vs 1.23 m)

**Tests:** 113 passed.
