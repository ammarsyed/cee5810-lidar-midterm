# Midterm Project Plan — 2D LiDAR Simulation with Mirrors, Glass & Fog

> Team doc. Read this first. Then:
> - `02_DERIVATIONS.md` → the math (mainly partner)
> - `03_CODE_PLAN.md` → the build steps (mainly Ammar, executed with Claude Code)

---

## 1. One-paragraph pitch

We model and simulate a **2D scanning LiDAR**: a laser range-finder that sweeps a fan of beams and measures distance by time-of-flight. The sensor sits in a room with **matte walls, a mirror, a glass pane, and a moving box**, optionally filled with **fog**.

We derive a **physics-based measurement model**: time-of-flight, ray reflection and refraction (law of reflection, Snell, Fresnel), a received-power equation, and fog attenuation. We wrap it in an **estimation-theoretic model** (`z = h(x, x_T, Λ) + n`) and a **probabilistic model** (`p(z | x, x_T, Λ)`, including missed detections). We formally define the sensor's **FOV** and **line-of-sight visibility**, and draw the whole thing as a **Bayesian network** that mirrors Figure 1 of the assignment.

The simulation then shows, with plots and animations, how the readings change when we vary:
- the **sensor state** (move and rotate it)
- the **target state and features** (box position; matte vs. mirror vs. glass; dark vs. bright)
- the **operating conditions** (fog density)

It also shows the effects real LiDARs have: **ghost points behind mirrors**, **see-through glass**, and **range loss and dropouts in fog**.

---

## 2. What the original idea had vs. what was missing

Original idea: *2D room, rays, a mirror that bounces rays, glass that passes rays but weakens them, move the sensor and show readings change, plus the time-of-flight math.*

That's a solid core. The rubric gaps are below. Each one is filled using course material where possible.

| # | Gap | Why it matters (rubric / Fig. 1) | Fill (and where it comes from) |
|---|---|---|---|
| 1 | **No noise / probabilistic model.** It's pure geometry right now. | "Probabilistic measurement model" and rigor. Without noise the measurements are trivially exact. | Use `z = h(x) + n` from **L3**. Range noise comes from timing jitter (`σ_R = c·σ_t/2`). **Detection only if received power ≥ threshold**, which is L3's exact wording. Below threshold the reading is ∅ / "no return". Gives `p(z \| x, x_T, Λ)`. |
| 2 | **No target state x_T.** Only the sensor moves. | Fig. 1 has a whole branch for target state and features. | Add a **moving box** that crosses the room. Also allow **rotating the mirror** (the Heiden paper tracks a mirror, Sec. IV-B). Target *features* = material type + reflectivity ρ. |
| 3 | **No operating conditions Λ.** | Fig. 1 box "Operating Conditions Λ". | **Fog/smoke** with extinction coefficient α (Beer–Lambert: signal × e^(−2αR)). One knob that clearly shrinks max range and adds dropouts. |
| 4 | **"Weaker through glass" is hand-wavy.** | Rigor; physics-based model. | Use **Fresnel equations** (+ Snell). For n = 1.5, head-on reflection is 4% per surface, so about 92% passes through a pane. Reflection rises sharply at grazing angles. This *explains* why glass is invisible except near head-on (Heiden Fig. 1). |
| 5 | **Mirror ghost points not explained.** | Rigor. | Law of reflection in vector form. **Prove the ghost point is the mirror image of the true hit point** across the mirror line. Measured range = total path length L₁ + L₂. |
| 6 | **Only sensor *position* varies, not heading.** | "position and orientation" is in the first sentence of the assignment. | Sensor state = pose (x, y, θ). Use the **2D rotation matrix** body↔world (**L5**). Show the FOV wedge rotating. |
| 7 | **FOV and LOS not formally defined.** | Explicitly listed tools; L2/L6 definition of S. | Define S(x) = points within R_max, within the angular fan, **and** with a clear line of sight. Mirror the camera paper's style: z = h(·) if x_T ∈ S, ∅ otherwise. |
| 8 | **No "form from class" for the model.** | Required at every level: "graphical model, probabilistic model, …". | A **Bayesian network** (L4) whose nodes match Fig. 1: pose → geometry → true range/path → received power → detection → measurement, with fog and materials as parent nodes. |
| 9 | **No quantitative validation.** | "Effectiveness" criterion; guidelines say *"Verify your findings through the theory."* | Each experiment compares the **simulated** result to an **analytic prediction**: noise histogram vs. Gaussian, max range vs. formula, ghost point vs. reflection formula, glass return vs. Fresnel curve. |
| 10 | **Each teammate needs distinct media + report.** | Stated in bold on page 1 of the assignment. | Share the simulator. **Split the experiments** so each person's figures are their own, and write separately (see §7). |
| 11 | **Graduate-level (5810) analysis missing.** | 5810 full credit needs a math analysis proving the model captures the Fig. 1 dependencies. | **D11** in `02_DERIVATIONS.md`: partial derivatives / set-membership arguments for each arrow in Fig. 1. |

Also planned, for presenting and exploring:
- **Interactive slider demo:** a desktop matplotlib window. Drag the sensor, fog, mirror angle and box, and watch the scan update. It can record a scripted sweep straight to MP4, which is great media.
- **Walkthrough notebook:** a JupyterLab notebook (like HW 2) that runs every experiment inline and embeds the same slider demo. It exports to HTML/PDF for the code appendix.

Optional stretch (only if time):
- **CW phase ranging + ambiguity**: the Heiden paper's actual method; range "rolls over" past ~3.2 m.
- **Beam divergence / mixed pixels**: Heiden Fig. 3/5.

---

## 3. Mapping to Figure 1 (put a version of this in each report)

```
 Operating conditions Λ  ◄─ - - ─  Sensor state x
   (fog α, ambient)                  (x, y, θ)
         │                         ╱
         ▼                       ╱
 Sensor measurements Z  ◄────── Target features X  ◄─ - - ─ Target state x_T
 (range per beam, or ∅)         (material: matte/mirror/glass,     (box position/velocity,
                                 reflectivity ρ, refractive n)      mirror pose)
```

| Fig. 1 variable | Our symbol | Concretely in the sim |
|---|---|---|
| Sensor state **x** | `x = [x_s, y_s, θ_s]` | LiDAR pose; we translate it along a path and rotate it |
| Target state **x_T** | `x_T = [x_b, y_b, v_x, v_y]`, mirror pose | moving box; rotatable mirror |
| Target features **X** | `ρ`, material type, `n_glass` | matte (ρ = 0.1–0.9), mirror, glass |
| Operating conditions **Λ** | `α` (fog), `P_th`, `σ_t` | fog density sweep |
| Measurements **Z** | `z = [z_1 … z_N]`, each z_i ∈ ℝ or ∅ | one range per beam |

---

## 4. The sensor we're modelling (pick once, use everywhere)

Loosely based on the **Hokuyo URG-04LX** from the Heiden paper, so we have a real reference.

| Parameter | Symbol | Value (suggested) |
|---|---|---|
| Angular field of view | Φ | 240° |
| Number of beams | N | 241 (1° spacing; real Hokuyo has 682, use fewer for speed) |
| Max range (clear air, white wall) | R_max | ~5.6 m (Hokuyo spec), emerges from the power threshold |
| Min range | R_min | 0.02 m |
| Timing jitter | σ_t | chosen so σ_R ≈ 1 cm → σ_t ≈ 67 ps |
| Detection threshold | P_th | set so R_max ≈ 5.6 m for ρ = 0.9 at normal incidence |
| Max bounces | B | 4–5 (Heiden uses 5) |
| Glass refractive index | n | 1.5 |
| Mirror reflectance | R_m | 0.95 |

**Return policy:** if a beam produces several returns above threshold (e.g. a weak one off the glass and a strong one off the wall behind it), report the **strongest**. Make it a switch (`first` / `strongest`) because the difference is a nice discussion point.

---

## 5. The scene

```
 ┌──────────────────────────────────────────┐  8 m × 6 m room, matte walls (ρ≈0.7)
 │                                  ╲       │
 │                                   ╲ mirror (rotatable)
 │      ■ ← moving box (ρ adjustable)       │
 │                                          │
 │  ▲ LiDAR (moves along a path,            │
 │    rotates)          ┃ glass pane        │
 │                      ┃ (n = 1.5)         │
 │                      ┃       □ object behind glass
 └──────────────────────────────────────────┘
 + optional fog (α) everywhere
```

Include one dark (ρ ≈ 0.1) and one bright (ρ ≈ 0.9) wall section so reflectivity effects show up.

---

## 6. Experiments → the media (each = at least one figure or animation)

Each one varies **one** Fig. 1 variable and checks against theory.

| ID | Varies | What we show | Check against theory |
|---|---|---|---|
| E1 | Sensor **position** | Animation: LiDAR drives through the room; point cloud + polar range plot update | Range to a flat wall vs. closed-form `R(x)` (D3) |
| E2 | Sensor **heading** | FOV wedge rotates; objects enter/leave FOV | FOV membership test (D4) |
| E3 | **Target state** (moving box) | Box crosses the room; range on beams hitting it vs. time; occlusion of the wall behind (LOS) | Predicted entry/exit times of FOV & LOS |
| E4 | **Target features** (ρ, material) | Detection probability & noise vs. reflectivity; dark vs. bright walls | Power equation threshold → predicted max range (D5) |
| E5 | **Mirror** (pose / incidence) | Ghost points behind mirror; sweep mirror angle | Ghost = reflection of true point (D8) |
| E6 | **Glass** (incidence angle) | Glass invisible except near head-on; wall behind glass seen with less power | Fresnel curve R(θ) (D9) |
| E7 | **Fog** α | Effective max range & detection rate vs. α; point cloud thinning | Beer–Lambert max-range curve (D5 + D10) |
| E8 | Noise (Monte Carlo) | Histogram of 1000 repeated scans of one beam; σ_R vs. range | Gaussian N(R, σ_R²) overlay (D6) |
| E9* | CW phase (stretch) | Range wrap-around at 3.22 m; two-frequency fix | `R_amb = c/2f` (D2) |
| E10* | Beam divergence (stretch) | Mixed pixels at object edges | Linear vs. phasor mixed-pixel model (D12) |
| Demo | All of the above, live | Slider demo video (scripted sweep → MP4) + walkthrough notebook | Readouts match the per-experiment results |

---

## 7. Who does what (draft — edit freely)

The simulator is shared. The figures and reports are individual.

|  | Ammar | Partner |
|---|---|---|
| Main role | Code (`03_CODE_PLAN.md`) | Math (`02_DERIVATIONS.md`) |
| Report figures (own runs/scenes) | E1, E3, E5, E7 (+ E9 if done) | E2, E4, E6, E8 (+ E10 if done) |
| Must still understand | the derivations D1, D3–D8 | how the code computes each equation |
| Writes own | full report, own wording | full report, own wording |

**Shared contract between code and math:** the notation table in §4 and the equations in `02_DERIVATIONS.md`. Every equation the partner derives should map to a function in the code (the code plan lists which). If one side changes a symbol or assumption, update the doc.

To make the media "clearly distinct" even where experiments overlap, each person can use different scene layouts, sensor paths, colour schemes and parameter sweeps. The sim supports this by loading scenes and paths from config.

---

## 8. Report skeleton (each person writes their own, 5–10 pages)

Per the assignment and writing guidelines: single column, 1.5 or double spaced, 11–12 pt, 1" margins, Word or LaTeX, IEEE citations. Hand in the PDF plus original files and code.

1. **Title page**
2. **Abstract:** one paragraph: motivation, what we built, key results (with numbers).
3. **Introduction:** why LiDAR matters in robot perception (L1/L3); the problem (mirrors, glass and fog break naive LiDAR models; Heiden et al.).
4. **Theory / Methods** (the math: the most important section per the guidelines)
   - Sensor class (active, narrow-beam, L3), frames (L5)
   - Time-of-flight measurement equation
   - Light–surface interaction: Lambertian, mirror, glass (Fresnel/Snell)
   - Received power, fog, detection threshold
   - Estimation-theoretic & probabilistic model; FOV & LOS definitions
   - Bayesian network (Fig. 1 analogue)
5. **Procedure:** simulator design (ray tracing algorithm, parameters table, scene), step-by-step how to reproduce.
6. **Results** (the "meat"): your experiments; each figure compared to its theoretical prediction; tables of numbers.
7. **Conclusions:** contributions, findings, limitations (2D only, thin glass, no multipath beyond B bounces, idealised noise), future work.
8. **References:** IEEE format.
9. **Appendix:** code listing / repo link.

---

## 9. Timeline (fill in the real due date: ________)

Assumes about 7–10 days. Squeeze or stretch as needed.

| Day | Ammar (code) | Partner (math) | Together |
|---|---|---|---|
| 1 | Stage 0–1 (setup, geometry, matte room) | D1, D3, D4 | Agree on notation & parameters (§4) |
| 2 | Stage 2 (sensor pose, FOV, LOS) | D5, D6, D7 (BN diagram) |  |
| 3 | Stage 3 (power, noise, dropout, fog) | D8, D9, D10 | Check code matches D5/D6 |
| 4 | Stage 4 (mirror + glass) | Start report theory section | Check mirror/glass vs. D8/D9 |
| 5 | Stage 5 (moving box, animations) | Results "predictions" for each experiment |  |
| 6 | Stage 6 (experiment scripts + figures) | Write theory section | Divide figures |
| 7 | Stage 7a/7b (slider demo + notebook); Stage 8 stretch if time | D11 (required for 5810) | Each starts own report |
| 8–9 | Write own report | Write own report | Swap & proofread for clarity (not content copying) |
| 10 | Submit | Submit |  |

---

## 10. Course concepts we're explicitly using (cite the lectures by name in the report)

| Concept | Lecture | Where it shows up |
|---|---|---|
| Workspace W, FOV S, sensor platform | L2, L6 | FOV definition, scene |
| Active / narrow-beam sensor, detection threshold | L3 | sensor classification, dropout model |
| `z = h(x, n)` estimation-theoretic model | L3 | measurement equation |
| `p(z \| x, n)` probabilistic model | L3 | noise + detection model |
| Bayesian networks (DAG + conditional probabilities) | L4 | sensor BN mirroring Fig. 1 |
| Inertial vs. body frame, rotation matrices | L5 | pose, beam directions, point cloud |
| Line-of-sight visibility | L6 | occlusion, ray casting |
| "z = h(x) if in FOV, ∅ otherwise" structure | Pinhole camera handout (eq. 9) | same structure for LiDAR |

## 11. References to start from

Verify each before citing; format in IEEE style.

1. E. Heiden, Z. Liu, R. K. Ramachandran, G. S. Sukhatme, "Physics-based Simulation of Continuous-Wave LIDAR for Localization, Calibration and Tracking," *ICRA*, 2020. **(provided; main source)**
2. Course lecture notes L2, L3, L4, L5, L6 and the "Pinhole Camera Model and FOV" handout, MAE/CEE 5810, S. Ferrari.
3. S. Thrun, W. Burgard, D. Fox, *Probabilistic Robotics*, MIT Press, 2005. Ch. 6 (beam model of range finders); optional, for the probabilistic model.
4. E. Hecht, *Optics*, Pearson. Fresnel equations, Snell's law.
5. M. Pharr, W. Jakob, G. Humphreys, *Physically Based Rendering* (pbrt). Ray tracing, Fresnel; Heiden et al. built on it.
6. Any reputable source on fog/weather attenuation of LiDAR (e.g. Rasshofer et al., "Influences of weather phenomena on automotive laser radar systems," *Adv. Radio Sci.*, 2011). Optional; Beer–Lambert itself is textbook physics.
