# Derivations To Do — LiDAR Midterm (for the math side)

Context: we're simulating a **2D scanning LiDAR** in a room with matte walls, a **mirror**, a **glass pane**, a **moving box**, and optional **fog**. The full plan is in `01_PROJECT_PLAN.md`. This doc lists the math we need, in priority order. For each item you get:
- **Show:** what to derive
- **Hint/sketch:** enough to get going, with key results so you can check yourself
- **Used in:** which experiment/figure and which code function uses it
- **Course tie-in:** the lecture concept to name in the report

**Priority:** 🟥 must-have (needed for full credit) · 🟧 should-have (adds rigor/challenge) · 🟦 stretch

> Note on the requirements: we're in **CEE 5810 (graduate level)**. That needs everything the undergrad version does (a math model "in one of the forms described in class": graphical model, probabilistic model, …), **plus** a mathematical analysis that proves or shows by inspection *why* the model captures the sensor properties and the Fig. 1 dependencies. That's **D11, which is required for us**.

---

## Shared notation (keep the code and both reports consistent)

| Symbol | Meaning |
|---|---|
| `p = [x_s, y_s]ᵀ`, `θ_s` | sensor position and heading, world frame; sensor state `x = [x_s, y_s, θ_s]ᵀ` |
| `β_i` | angle of beam *i* in the sensor (body) frame, `β_i ∈ [−Φ/2, Φ/2]` |
| `u_i = [cos(θ_s+β_i), sin(θ_s+β_i)]ᵀ` | unit direction of beam *i* in world frame |
| `Φ, N` | angular FOV (240°), number of beams |
| `R_i` | true range (one-way optical path length) along beam *i* |
| `z_i` | measured range for beam *i* (a number, or ∅ = no return) |
| `c` | speed of light, 2.998×10⁸ m/s |
| `t_R`, `σ_t` | round-trip time, timing jitter std. dev. |
| `ρ` | diffuse (Lambertian) reflectivity of a surface, 0–1 |
| `θ_inc` | angle of incidence (between beam and surface normal) |
| `n̂` | unit surface normal |
| `n` | refractive index (air ≈ 1, glass = 1.5) |
| `α` | fog extinction coefficient [1/m] (operating condition Λ) |
| `P_t, P_r, P_th` | transmitted power, received power, detection threshold |
| `x_T` | target state (moving box position/velocity, mirror pose) |
| `X` | target features: material type, `ρ`, `n`, `R_m` |
| `Λ` | operating conditions (here mainly `α`) |
| `Δβ` | angular spacing between beams, `Φ/(N−1)` (1° for N = 241) |
| `σ_R`, `n_i` | range-noise std. dev.; noise sample on beam *i* |
| `σ_P` | std. dev. of received-power noise (smooths the detection threshold) |
| `P_D,i`, `D_i` | detection probability; detection indicator (T/F) for beam *i* |
| `R_F(θ)` | Fresnel reflectance of glass at incidence θ |
| `R_m` | mirror reflectance (≈ 0.95) |
| `T_pane` | single-pass transmission through one glass pane |
| `T_path` | product of all interface factors along a beam's path, out and back |
| `R₀` | clear-air max range for a given target (D5) |

**Name clashes to watch:** `R` (range) vs. `R_F`, `R_m` (reflectances); `n` (refractive index) vs. `n̂` (normal) vs. `n_i` (noise); `Φ` (FOV angle) vs. `Φ_N` (normal CDF).

---

## D1 🟥 Time-of-flight range equation + noise

**Show:** `R = c·t_R / 2`, and how timing noise turns into range noise.

**Hint:** light goes out *and* back, so the distance covered is `2R = c·t_R`. If `t_R` has Gaussian error with std. dev. `σ_t`, then `R` is a linear function of `t_R`, so `σ_R = (c/2)·σ_t`.

**Sanity numbers:** a 5 m wall gives `t_R ≈ 33.4 ns`. For `σ_R = 1 cm` you need `σ_t ≈ 67 ps`. That's why cheap LiDARs use phase measurement instead (D2).

**Used in:** E8 (noise histogram); code `sensor.tof_to_range`, `noise.range_noise`.
**Course tie-in:** physics-based model; "active sensor" (L3).

---

## D2 🟧 Continuous-wave (phase-shift) ranging + ambiguity

**Show:** for a laser amplitude-modulated at frequency `f`, the phase shift is `Δφ = 2πf·t_R = 4πfR/c`, so `R = c·Δφ/(4πf)`. Because `Δφ` is only known mod 2π, ranges are ambiguous beyond `R_amb = c/(2f)`. Two frequencies extend this to `c/(2|f₁−f₂|)`.

**Key numbers (Hokuyo, Heiden et al. eq. 3):**

| f | R_amb |
|---|---|
| 46.55 MHz | 3.22 m |
| 53.2 MHz | 2.82 m |
| beat, 6.65 MHz | 22.5 m |

**Used in:** E9 (stretch); code `sensor.phase_range`.
**Course tie-in:** physics-based model; connects to the provided paper.

---

## D3 🟥 Measurement equation vs. sensor pose (frames + closed form)

**Show (a): beam geometry and frames.** Beam direction in the world frame is `u_i = Rot(θ_s)·[cos β_i, sin β_i]ᵀ` with `Rot(θ) = [[cos θ, −sin θ],[sin θ, cos θ]]`. A measured point in the body frame `q^B = z_i[cos β_i, sin β_i]ᵀ` maps to the world frame as `q^W = p + Rot(θ_s) q^B`. (Note the pinhole handout writes the transposed/inverse rotation; state our convention clearly.)

**Show (b): closed-form range to a flat wall.** Wall = line `{q : n̂·q = d}`. Solve `n̂·(p + R u_i) = d`:

```
R_i(x) = (d − n̂·p) / (n̂·u_i)        valid if R_i > 0 and the hit is within the wall's endpoints
```

**Show (c): general ray–segment intersection** (this is literally what the code does). Ray `p + t·u`, segment `a + s·(b−a)`, `s ∈ [0,1]`. With the 2D cross product `a×b = a_x b_y − a_y b_x` and `e = b − a`:

```
t = ((a − p) × e) / (u × e),     s = ((a − p) × u) / (u × e)
hit iff  u × e ≠ 0,  t > 0,  0 ≤ s ≤ 1
```

**Show (d): sensitivity.** This is what makes "measurement depends on sensor state" concrete:

```
∂R_i/∂p   = −n̂ / (n̂·u_i)
∂R_i/∂θ_s = −R_i (n̂·u_i') / (n̂·u_i),   where u_i' = [−sin(θ_s+β_i), cos(θ_s+β_i)]ᵀ
```

**Insight worth a paragraph:** `∂R/∂p ∝ n̂`. Moving the sensor *parallel* to a single flat wall doesn't change that wall's readings at all. That's a fundamental limitation of range sensing (a corridor looks the same everywhere).

**Used in:** E1, E2; code `geometry.ray_segment_intersect`, `geometry.rot2d`, `sensor.beam_directions`, `sensor.to_world`.
**Course tie-in:** coordinate frames / rotation matrices (L5); estimation-theoretic model `z = h(x) + n` (L3).

---

## D4 🟥 Field-of-view (FOV) and line-of-sight (LOS)

**Show:** a formal definition of the FOV set, in the same style as the pinhole handout (its eqs. 8–9).

```
S(x) = { q ∈ W :  R_min ≤ ‖q − p‖ ≤ R_max,
                  |wrap(atan2(q_y − y_s, q_x − x_s) − θ_s)| ≤ Φ/2,
                  LOS(p, q) }

LOS(p, q) = true  iff no opaque segment intersects the open segment (p, q)

z_i = h_i(x, x_T, Λ) + n_i   if the beam's first (or strongest) return is detected
z_i = ∅                       otherwise
```

**Points to make:**
1. The FOV is a **240° wedge** of radius `R_max`, *minus* everything hidden behind obstacles (LOS). Draw it: it's a "visibility polygon."
2. `R_max` is not a fixed constant: it depends on target reflectivity `ρ` and fog `α` (see D5). So really `S = S(x, X, Λ)`, which touches every box in Fig. 1.
3. Discrete beams: a target face of width `w` at range `R`, tilted by `θ_inc`, is only guaranteed to be hit if `w·cos θ_inc ≳ R·Δβ`, with `Δβ = Φ/(N−1)`. Our 1° spacing means a gap of about 8.7 cm between beams at 5 m.
4. Glass is (mostly) *transparent* for LOS. A mirror *extends* the FOV around corners (see D8).

**Used in:** E2, E3; code `sensor.in_fov`, `sensor.visibility_polygon`.
**Course tie-in:** FOV and LOS visibility (L2, L6); pinhole handout eq. (9).

---

## D5 🟥 Received power, detection threshold, max range

**Show:** received power from a matte (Lambertian) surface, then the detection rule.

**Sketch:** a Lambertian surface lit by power `P_t` at incidence `θ_inc` re-radiates with intensity `I = ρ P_t cos θ_inc / π` [W/sr] back toward the sensor. A receiver of area `A_rx` at distance `R` collects `I·A_rx/R²`.

Why only **one** `cos θ_inc`: the beam is narrow and fully lands on the surface, so all of `P_t` is intercepted (no cosine there). The cosine comes from Lambert's law for light emitted back along the beam, at angle `θ_inc` from the normal.

Include the fog loss (D10) and any interface losses along the path:

```
P_r = K · ρ cos θ_inc · T_path · e^(−2αR) / R²       with K = η P_t A_rx / π   (η = optics efficiency)
detect  ⇔  P_r ≥ P_th
```

`T_path` = product of interface factors traversed out *and* back: `R_m²` per mirror bounce, `T_pane²` per glass pane (D8, D9).

**Max range (clear air, α = 0, T_path = 1):**
```
R_max(ρ, θ_inc) = sqrt( K ρ cos θ_inc / P_th )
```
We **calibrate** `K/P_th` so that `R_max = 5.6 m` for `ρ = 0.9` head-on (Hokuyo spec), which gives `K/P_th ≈ 34.84 m²`. Then:
- `ρ = 0.7` → `R_max ≈ 4.94 m`
- `ρ = 0.2` → `R_max ≈ 2.64 m`
- `ρ = 0.1` (dark wall) → `R_max ≈ 1.87 m`

This gives dark-vs-bright behaviour for free.

**Max range with fog (nice closed form):** set `R² e^(2αR) = R₀²`. Here `R₀` is the clear-air max range *for that particular target*, `R₀ = sqrt(K ρ cos θ_inc · T_path / P_th)`. Take square roots and multiply by α: `αR·e^(αR) = αR₀`, so

```
R_max(α) = W₀(α R₀) / α          (W₀ = principal branch of the Lambert W function; scipy.special.lambertw(·, 0))
```
Check that it → `R₀` as `α → 0`: use `W₀(y) = y − y² + …`.

**Specular surfaces (mirror/glass) hit head-on:** a perfectly smooth surface only sends light back to the sensor if `θ_inc ≈ 0`. We model this as a narrow "specular lobe". This is a **modelling assumption**, so state it as one:
```
P_r,spec = K_s · R_s(θ_inc) · exp(−θ_inc² / (2 θ_w²)) / R²,     θ_w ≈ 1–2°
R_s = R_m (≈ 0.95) for the mirror,   R_s = R_F(θ_inc) (Fresnel, D9) for glass
```

**Used in:** E4, E6, E7; code `physics.received_power`, `physics.max_range`.
**Course tie-in:** physics-based model; "detection declared when received power exceeds a threshold" (L3, slide 7).

---

## D6 🟥 Probabilistic measurement model `p(z | x, x_T, Λ)`

**Show:** the full likelihood of a scan.

**Model:**
- Range noise: `n_i ~ N(0, σ_R²)`, `σ_R = cσ_t/2` (D1). Optional and more realistic: noise grows as the signal weakens, `σ_R(P_r) = σ_0·sqrt(P_ref / P_r)`.
- Detection probability: the received power itself is noisy, `P̃_r ~ N(P_r, σ_P²)`, so `P_D = Pr(P̃_r ≥ P_th) = Φ_N((P_r − P_th)/σ_P)` (Φ_N = standard normal CDF). This smooths the hard threshold.
- Per beam:

```
p(z_i | x, x_T, Λ) = P_D,i · N(z_i ; R_i, σ_R²)      for z_i ∈ ℝ
Pr(z_i = ∅ | x, x_T, Λ) = 1 − P_D,i
```

- Whole scan (beams conditionally independent given the state):

```
p(z | x, x_T, Λ) = ∏_i p(z_i | x, x_T, Λ)
```

**Point to make:** this comes straight from the L3 form `p(z, x, n) = p(z | x, n) p(x) p(n)`.
- Given the noise, the measurement is deterministic: `p(z | x, n) = δ(z − h(x) − n)`.
- Integrate the noise out, `p(z | x) = ∫ δ(z − h(x) − n) p(n) dn`, and you get the Gaussian above.
- State which variables are random (`z`, `n`) and which are given (`x`, `x_T`, `Λ`). The materials `X` are treated as known and left out of the notation.
- Caveat: the split `p(x)p(n)` assumes the noise is independent of the state. That breaks for the optional `σ_R(P_r)`. In that case write `p(n | x, x_T, Λ)` instead.

**Optional extension:** the Thrun et al. "beam model" mixes four cases: correct hit, unexpected short reading, max-range failure, and random junk. Mention it as the standard robotics model and explain which parts we include.

**Used in:** E4, E7, E8; code `noise.sample_measurement`, `noise.detection_prob`.
**Course tie-in:** probabilistic & estimation-theoretic measurement models (L3).

---

## D7 🟥 Bayesian network (graphical model) of the sensor

**Show:** a DAG for one beam that mirrors Fig. 1 of the assignment, plus its factorization.

```
   x (pose) ──┐                          α (fog, Λ)
              ▼                              │
   x_T ──► G_i = [R_i, θ_inc,i, T_path,i] ──►├──► P_i ──► D_i (detected?) ──► z_i
   (target)   ▲  (ray tracing:   │           ▲                                ▲ ▲
              │   deterministic) └───────────┼────────── R_i ─────────────────┘ │
   X = materials/ρ/n ─────────────────────────┘                     n_i (noise) ┘
```
Edges: `x, x_T, X → G_i`; `G_i, X, α → P_i`; `P_i → D_i`; `R_i (part of G_i), D_i, n_i → z_i`.

**Factorization** (group the geometry outputs as `G_i = (R_i, θ_inc,i, T_path,i)`):
```
p(z_i, D_i, P_i, G_i | x, x_T, X, α) =
     p(z_i | R_i, D_i) · p(D_i | P_i) · p(P_i | G_i, X, α) · p(G_i | x, x_T, X)
```
- `G_i` and `P_i` (the **noise-free** received power from D5) are deterministic, so their factors are delta distributions.
- The randomness lives in `D_i` (Bernoulli with probability `P_D,i`, D6) and in `z_i` (noise `n_i` integrated out).
- `T_path` must feed into `P_i`: mirror and glass losses change the power.

**Nice L4 tie-in (optional but easy marks):** discretise `α ∈ {clear, light, heavy}`, material `∈ {matte, mirror, glass}`, range `∈ {near, mid, far}`. Then **estimate the CPT `P(D = T | material, α, range)` by Monte Carlo from our simulator**. That's literally "learning CPTs from data" from L4. Ammar's code can produce the table.

**d-separation, stated carefully:**
- Different beams are conditionally independent given **all** of `x, x_T, X, α`. They are *not* independent given the pose alone: `α` and `X` are shared parents (open forks) that link every beam.
- `z_i ⊥ x | G_i` needs `X` in the conditioning set too. `G_i` is a collider on the path `x → G_i ← X → P_i`, so conditioning on `G_i` alone opens it.

**Course tie-in:** Bayesian networks, CPTs, d-separation (L4).

---

## D8 🟥 Mirror: law of reflection + ghost-point theorem

**Show (a):** reflected direction `u' = u − 2(u·n̂)n̂`. Show `‖u'‖ = 1` and angle in = angle out.
- Length: `‖u'‖² = ‖u‖² − 4(u·n̂)² + 4(u·n̂)² = 1`.
- Angle: `u'·n̂ = u·n̂ − 2(u·n̂) = −(u·n̂)`. The normal component flips sign, and the tangential component is unchanged.

**Show (b): ghost-point theorem.** The beam leaves `p`, hits the mirror at `h = p + L₁u` and the wall at `q = h + L₂u'`. Diffuse light retraces the path, so the round trip is `2(L₁+L₂)` and `z = L₁ + L₂`. The LiDAR assumes a straight line, so it plots the point at

```
g = p + (L₁ + L₂)·u
```
**Claim:** `g = Ref(q)`, the mirror image of the true wall point across the mirror line, where `Ref(y) = y − 2((y − m)·n̂)n̂` for any point `m` on the mirror.

*Proof sketch:* `(h − m)·n̂ = 0` because `h` is on the mirror. So `(q − m)·n̂ = L₂(u'·n̂) = −L₂(u·n̂)`. Then `Ref(q) = h + L₂u' + 2L₂(u·n̂)n̂ = h + L₂u = g`. ∎

**Consequences to discuss:**
- The LiDAR sees a **"virtual room"** behind the mirror. Its FOV through the mirror equals the visibility of a **virtual sensor at Ref(p)**, but only for rays that pass through the mirror segment. The beam order is reversed, because a reflection flips orientation. Check: `Ref(p) + L₁u' = h`. So the mirror extends the FOV around corners (ties to D4).
- Head-on (`u·n̂ = −1`): `u' = −u`, so the beam returns straight back and the mirror itself shows up at its true position.
- Return power through the mirror is scaled by `R_m²` (out and back) and uses the total path length `L₁ + L₂` (D5).

**Used in:** E5; code `physics.reflect`, test `test_ghost_point_is_reflection`.
**Course tie-in:** LOS/FOV (L6), physics-based model; Heiden et al. Fig. 7–8.

---

## D9 🟥/🟧 Glass: Snell + Fresnel + thin-pane model

Parts (a), (b) and the "story" are 🟥 must-have. Parts (c) and (d) are 🟧 should-have.

**Show (a) 🟥 Snell:** `n₁ sin θ₁ = n₂ sin θ₂`.

**Show (b) 🟥 Fresnel (unpolarised):**
```
r_s = (n₁cos θ₁ − n₂cos θ₂)/(n₁cos θ₁ + n₂cos θ₂)
r_p = (n₁cos θ₂ − n₂cos θ₁)/(n₁cos θ₂ + n₂cos θ₁)
R_F = (r_s² + r_p²)/2,     T = 1 − R_F
```
Check: head-on, `R_F = ((n₁−n₂)/(n₁+n₂))² = 0.04` for glass.

**Reference values (n = 1.5), to check the code against:**

| θ_inc | 0° | 30° | 45° | 56.3° (Brewster) | 70° | 80° | 89° |
|---|---|---|---|---|---|---|---|
| R_F | 0.040 | 0.042 | 0.050 | 0.074 | 0.171 | 0.388 | 0.904 |

(At Brewster's angle `tan θ_B = n₂/n₁`, `r_p = 0`.)

**Show (c) 🟧 pane transmission:** two interfaces, so single-pass `T_pane ≈ (1 − R_F)² ≈ 0.92` head-on. Including incoherent internal bounces it's `(1−R_F)/(1+R_F) ≈ 0.923`, basically the same. The round trip through the pane is `T_pane² ≈ 0.85`.

*Assumption to state:* averaging `R_s` and `R_p` first (unpolarised light) is a modelling choice. Laser diodes are actually polarised. Doing the incoherent sum per polarisation and then averaging gives a slightly different oblique value: 0.907 vs. 0.904 at 45°. Head-on they agree.

**Show (d) 🟧 why we can treat glass as a thin line:**
- **Direction:** for a slab with parallel faces, Snell twice gives exit angle = entry angle. The ray is only shifted sideways, by `δ = t·sin(θ₁ − θ₂)/cos θ₂`. For `t = 6 mm` and `θ₁ = 45°`, `δ ≈ 2.0 mm`.
- **Timing:** light is slower in glass, so the pane also adds a range bias: `(t/cos θ₂)·(n − cos(θ₁ − θ₂))`. That's `(n−1)t = 3.0 mm` head-on and about 3.7 mm at 45°.

Both are smaller than our 1 cm noise, so it's justified to ignore the bending and the delay.

**Story for the report 🟥:** at most angles, about 92% passes through and the specular 4–5% bounces *away* from the sensor. So the LiDAR reports the wall **behind** the glass and the glass is "invisible". Only near head-on does the reflection come straight back. Heiden Fig. 1(3)–(4) shows the same thing for a **transparent plastic sheet**: it's not glass, but the physics is the same. At grazing angles, `R_F` rises and the glass starts acting like a weak mirror (weak ghost points).

**Used in:** E6; code `physics.fresnel`, `physics.refract`.
**Course tie-in:** physics-based model; Heiden et al. Sec. III-A (BTDF/BSDF).

---

## D10 🟧 Fog: Beer–Lambert attenuation

**Show:** `dI/dx = −αI` gives `I(x) = I₀e^(−αx)`. Out and back gives `e^(−2αR)`, which goes into D5. Combine with D5 for `R_max(α) = W(αR₀)/α`.

**Interpretation:** meteorological visibility `V ≈ 3.912/α` (Koschmieder; 3.912 = ln 50, i.e. a 2% contrast threshold). For room-scale effects, sweep `α ∈ [0, 0.5] m⁻¹`.

**Reference table: `R_max(α)` in metres** (head-on, `T_path = 1`, `K/P_th = 34.84 m²`). The experiment E7 figure should match these:

| ρ \ α [1/m] | 0 | 0.05 | 0.1 | 0.2 | 0.3 | 0.5 |
|---|---|---|---|---|---|---|
| 0.9 | 5.60 | 4.48 | 3.82 | 3.05 | 2.58 | 2.03 |
| 0.7 | 4.94 | 4.04 | 3.49 | 2.81 | 2.40 | 1.91 |
| 0.2 | 2.64 | 2.35 | 2.13 | 1.83 | 1.62 | 1.35 |
| 0.1 | 1.87 | 1.71 | 1.59 | 1.41 | 1.27 | 1.09 |

**Stretch:** fog also scatters light *back* early, producing false short readings. That's the "short reading" term of the Thrun beam model (D6). Mention as a limitation if we don't simulate it.

**Used in:** E7; code `physics.fog_transmission`.
**Course tie-in:** operating conditions Λ (Fig. 1).

---

## D11 🟥 (required for 5810) Dependency analysis "by inspection / proof"

**Show:** for each arrow in Fig. 1, a mathematical reason the measurement depends on it.
- **`x → Z`:** nonzero `∂R/∂p`, `∂R/∂θ_s` (D3). Note the degenerate direction parallel to a wall.
- **`x_T → Z`:** when the box enters a beam, `z_i` jumps from the wall range to the box range (an LOS switch). While a box face (line `n̂·q = d_b`) is being hit and translates with its centre `c_b`, then `∂R_i/∂c_b = n̂/(n̂·u_i) = −∂R_i/∂p`. Moving the target is equivalent to moving the sensor the opposite way.
- **`X → Z`:** with `P_D = Φ_N((P_r − P_th)/σ_P)` and `P_r ∝ ρ`:
  ```
  ∂P_D/∂ρ = φ_N((P_r − P_th)/σ_P) · (P_r/ρ) / σ_P  > 0        (φ_N = standard normal pdf)
  ```
  Material type also changes *which* surface the measurement comes from (mirror → ghost, glass → the wall behind).
- **`Λ → Z`:** since `P_r ∝ e^(−2αR)`:
  ```
  ∂P_D/∂α = −φ_N(·) · 2R·P_r / σ_P  < 0
  ```
  Implicitly differentiating `R e^(αR) = R₀` gives
  ```
  dR_max/dα = −R_max² / (1 + α R_max)  < 0
  ```
  Finite-difference check: −5.76 m² at α = 0.2, ρ = 0.9.
- **`x → Λ` (dashed arrow in Fig. 1):** the path length through fog depends on the pose.
- **Bonus: Cramér–Rao lower bound** on estimating a wall's distance `d` from the beams hitting it.
  - Since `∂z_i/∂d = 1/(n̂·u_i)`, the Fisher information is `Σ_i (n̂·u_i)⁻² / σ_R²`, so `var(d̂) ≥ σ_R² / Σ_i (n̂·u_i)⁻²`.
  - Because `|n̂·u_i| ≤ 1`, this bound is `≤ σ_R²/N_wall`: more beams on the wall means a tighter estimate.
  - Assumptions: known pose and wall normal, all beams detected, equal `σ`.
  - General form with dropouts and unequal noise: Fisher info `= Σ_i P_D,i / (σ_i² (n̂·u_i)²)`.
  - Monte Carlo check: least-squares variance 1.05×10⁻⁶ m² vs. bound 1.02×10⁻⁶ m² (81 beams, σ = 1 cm).

**Used in:** the Results/Analysis discussion of E1–E8; each figure is the numerical evidence for one bullet above.
**Course tie-in:** the 5810 requirement ("prove or describe by inspection why/how the sensor model captures … the dependencies in Figure 1"); estimation theory (L3).

---

## D12 🟦 Beam divergence / mixed pixels (only if time)

A beam is a cone (Heiden: 4 cm wide at 4 m). Sample it with 3 sub-rays with power weights `w_k`. At an object edge, the reported point lands *between* the object and the wall behind it ("spurious measurement", Heiden Fig. 3).
- **Simple model:** a power-weighted average, `z = Σ w_k R_k / Σ w_k`. This is only the small-separation approximation for a phase-measuring sensor.
- **Exact CW model:** the received waves add as phasors, `z = c/(4πf) · arg( Σ_k w_k e^(i 4πf R_k / c) )`.
- Example: returns at 1 m and 2 m with weights 0.7 and 0.3 give 1.30 m (linear) vs. 1.23 m (phasor, f = 46.55 MHz).

**Used in:** E10 (stretch).
**Course tie-in:** physics-based model; Heiden et al. Sec. III-B, Fig. 3 and Fig. 5.

---

## Checklist for the hand-off to code

When you finish a derivation, put the **final equation(s) + any constants** in a message or comment so the code can be checked against it. The code has unit tests for: D3(c) intersection, D8 ghost theorem, D9 Fresnel table values, D5 max-range formula, D1/D6 noise std, D10 Lambert-W max range.
