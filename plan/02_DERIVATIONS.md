# Derivations To Do — LiDAR Midterm (for the math side)

Context: we're simulating a **2D scanning LiDAR** in a room with matte walls, a **mirror**, a **glass pane**, a **moving box**, and optional **fog**. Two upgrades sit on top: the robot **localizes itself** with a particle filter (D13), and there's a **3D, 16-ring LiDAR mode** (D14). The full plan is in `01_PROJECT_PLAN.md`. This doc lists the math we need, in priority order (D1–D12 core, D13–D14 upgrades). For each item you get:
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

| `x_k, u_k = (v_k, ω_k)`, `Δt` | robot pose and odometry (speed, turn rate) at time step *k*; step length (D13) |
| `σ_v, σ_ω` | odometry noise std. devs (D13) |
| `M`, `x^[m]`, `w^[m]`, `N_eff` | number of particles, particle *m*, its weight, effective sample size (D13) |
| `ε` (mixture) | weight of the uniform "junk" component in the robust likelihood (D13) |
| `ε_k`, `φ_j` | elevation of ring *k*, azimuth of column *j* (3D sensor, D14) |
| `h` | sensor mounting height (0.6 m, D14) |
| `ψ, ϑ, φ_r` | yaw, pitch, roll of the 3D sensor (L5 Euler angles, D14) |

**Name clashes to watch:**
- `R` (range) vs. `R_F`, `R_m` (reflectances)
- `n` (refractive index) vs. `n̂` (normal) vs. `n_i` (noise)
- `Φ` (FOV angle) vs. `Φ_N` (normal CDF)
- `ε` (mixture weight) vs. `ε_k` (elevation)
- `φ_j` (azimuth) vs. `φ_r` (roll)
- `ε` a third time: the Möller–Trumbore tolerance (renamed `τ` in D14c)
- `η` (Bayes normaliser, D13) vs. `η` (optics efficiency, D5)
- `h` (mount height) vs. `h` (mirror hit point, D8) vs. `h(·)` (measurement function)
- `u` (beam direction `u_i`) vs. `u_k` (odometry) vs. `u` (barycentric, D14c); `v` (speed) vs. `v` (barycentric)
- `x_k` (pose) vs. `x` (x-coordinate)
- `R_spec` (D5 specular reflectance; written `R_spec` to avoid) `R_s` (s-polarised Fresnel reflectance, D9)

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

In the code, `K_s/K = 1/(8θ_w²) ≈ 182` for θ_w = 1.5°. Where that comes from:
- The exponent is written in the incidence angle, but the mirror direction leaves the surface `2θ_inc` away from the direction back to the sensor. So in outgoing angle the lobe is `2θ_w` wide.
- Normalising that Gaussian over solid angle (`∫ dΩ ≈ 2π(2θ_w)² = 8πθ_w²`) and comparing with the Lambertian `1/π` lobe gives `1/(8θ_w²)`.
- This keeps the model **energy-conserving**. An earlier draft used `1/(2θ_w²) ≈ 730`, which reflects 4× the incident power.

**Where the glass blip ends:** the glass return stays above threshold while
```
θ_inc < θ_max = θ_w · sqrt( 2 ln( (K_s/K)(K/P_th) R_F / R_g² ) )
```
with `R_g` the distance to the glass. For `R_g = 2 m`, `θ_max ≈ 4.3°`: 5.0° at 1 m, 3.9° at 3 m, so it depends on the scene. The glass is also the *strongest* return up to ≈ 4.2° (with a ρ = 0.7 wall 2 m behind it). So glass appears as a blip about 8–9 beams wide at 1° spacing, depending on how the beams line up with the glass (E6).
```
P_r,spec = K_s · R_spec(θ_inc) · exp(−θ_inc² / (2 θ_w²)) / R²,     θ_w ≈ 1–2°
R_spec = R_m (≈ 0.95) for the mirror,   R_spec = R_F(θ_inc) (Fresnel, D9) for glass
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

# Upgrades (planned): localization and 3D

D13 and D14 support the two upgrades we chose for technical challenge:
- **experiment E11:** the robot localizes itself with a particle filter, naive vs. physics-aware model
- **experiments E12–E13:** a 3D, 16-ring LiDAR mode

They're 🟥 because we're building both, so each report has to explain them.

---

## D13 🟥 Localization: Bayes filter, particle filter, and why the model matters

**Story:** a robot drives through the room and has to work out *where it is* from its LiDAR scans. That's the main experiment in Heiden et al. (Sec. IV-A). We use our own likelihood `p(z | x)` from D6 inside a **particle filter**. Then we show what happens when the filter uses a **naive** sensor model (every surface matte and opaque, i.e. what Gazebo or a standard occupancy map assumes) instead of our **physics-aware** one.

**Show (a) 🟥 motion model.** The robot follows the **unicycle model from L2** (slide 9: `ẋ = v cos θ, ẏ = v sin θ, θ̇ = ω`). Discretise with step `Δt` and noisy odometry `u_k = (v_k, ω_k)`:
```
x_{k+1} = x_k + Δt·v̂_k cos θ_k
y_{k+1} = y_k + Δt·v̂_k sin θ_k
θ_{k+1} = θ_k + Δt·ω̂_k
v̂_k = v_k + N(0, σ_v²),   ω̂_k = ω_k + N(0, σ_ω²)
```
This defines `p(x_{k+1} | x_k, u_k)`. Note: noise only on `(v, ω)` gives a rank-2 (degenerate) pose noise per step. That's fine for *sampling* (the density is never evaluated). Thrun's velocity motion model adds a third noise term if particles collapse; our roughening after resampling plays that role.

**Show (b) 🟥 the Bayes filter, from Bayes' theorem + the Markov property (L4):**
```
bel(x_k) = η · p(z_k | x_k) · ∫ p(x_k | x_{k−1}, u_k) · bel(x_{k−1}) dx_{k−1}
           ─────┬─────         ───────────────┬──────────────────────
            correct (D6)                 predict (motion model)
```
Derive it in three lines:
1. **Bayes' theorem** on the newest measurement: `p(x_k | z_{1:k}, u_{1:k}) = η p(z_k | x_k, z_{1:k−1}, u_{1:k}) p(x_k | z_{1:k−1}, u_{1:k})`.
2. **Sensor Markov property:** `z_k` depends only on `x_k`, so the first factor is `p(z_k | x_k)`.
3. **Law of total probability** over `x_{k−1}`, plus the **motion Markov property** (`x_k` depends only on `x_{k−1}`, `u_k`), plus "`u_k` carries no information about `x_{k−1}`": `p(x_k | z_{1:k−1}, u_{1:k}) = ∫ p(x_k | x_{k−1}, u_k) bel(x_{k−1}) dx_{k−1}`.

The filter assumes a **static, known map**. A moving box breaks the Markov assumption unless it is tracked, so E11 uses static scenes. In a dynamic scene, beams hitting unmodelled movers fall to the mixture's junk term.

As a graphical model this is a **dynamic Bayesian network** (an HMM): `x_{k−1} → x_k → z_k`, with `u_k → x_k`. Draw it next to the D7 network: D7 is the inside of each `x_k → z_k` arrow.

**Show (c) 🟥 particle approximation (Monte Carlo localization).** Represent `bel(x_k) ≈ Σ_m w^[m] δ(x − x^[m])` with `M` particles. Each step:
1. **Predict:** sample `x^[m]_k ~ p(x_k | x^[m]_{k−1}, u_k)`.
2. **Weight:** `w^[m]_k ∝ w^[m]_{k−1} · p(z_k | x^[m]_k)`, then normalise. This is importance sampling: the proposal is the motion model, so the incremental weight is the likelihood. The weights **carry over** between steps and are reset to `1/M` only after a resample. Resetting every step throws away evidence; in a toy 1D check it gives posterior mean 0.50 instead of the exact 0.67.
3. **Resample** with the low-variance (systematic) scheme when `N_eff = 1/Σ_m (w^[m])² < M/2`.

The estimate is the weighted mean, with circular mean for θ. Caveat: with mirror aliasing the posterior can be multimodal and the weighted mean can fall *between* modes. Then also report the heaviest cluster.

**Show (d) 🟥 the likelihood and why naive models break.**
- Per beam we use D6. We add a small uniform "junk" component (Thrun's beam model), so one bad beam can't zero out a particle:
  ```
  p_mix(z_i | x) = (1 − ε)·p(z_i | x) + ε / R_max,     ε ≈ 0.05
  ```
- **Penalty without the mixture:** suppose a beam goes through glass and reads the wall behind, but the naive model expects the glass 1 m closer. With `σ_R = 1 cm`, that single beam costs `(1 / 0.01)² / 2 = 5000` nats of log-likelihood. It overwhelms every other beam, so the filter collapses onto whatever wrong pose "explains" it.
- **With the mixture**, the penalty per beam is bounded. Peak `log p ≈ 3.64`, floor `log(ε/R_max) ≈ −4.72`, so at most **≈ 8.35 nats** per beam. In closed form, `Δ_max ≈ ln((1 − ε) R_max / (ε σ √(2π)))`; the uniform takes over for residuals beyond about `σ √(2 Δ_max)` ≈ 4σ.
- `R_max` in the mixture is the **sensor's** range limit (5.6 m, the support of the uniform), not the target-dependent max range of D5.
- **The mixture only protects numeric readings.** For `z = ∅` the model gives `Pr(∅) = (1 − ε)(1 − P_D)`, which still → 0 where the model is sure of a return. So we add a small junk mass at ∅ (Thrun's "max-range" term): `Pr(∅) = (1 − ε)(1 − P_D) + ε_∅`, with `ε_∅ = 0.01`. This makes the total probability `1 + ε_∅` for every pose, so particle weights are unaffected; divide by it if you need a proper distribution.
- **In the code (E11):** the filter uses an inflated likelihood width `σ_lik = 5 cm`, not the true 1 cm. With M = 300 particles, a 1 cm likelihood is so sharp that one particle takes almost all the weight (**weight degeneracy**), and resampling then leaves only copies of it (**sample impoverishment**). With 5 cm the bounded per-beam gap is **≈ 6.75 nats** (7.66 at 2 cm, 6.05 at 10 cm), versus `(1/0.05)²/2 = 200` nats for a 1 m mismatch without the mixture. State this as an approximation. It's standard practice.
- **Tempering:** beams aren't truly independent, because neighbouring beams share surfaces and the D7 d-separation note applies. So use every k-th beam, or raise the likelihood to the power `1/k`. State this as an approximation.
- **The key argument (by inspection):**
  - Naive model + mixture: mirror-ghost and see-through-glass beams become "outliers", so their information is thrown away.
  - Naive model, no mixture: they actively *mislead* it.
  - Physics-aware model: it predicts the ghosts and the see-through, so those beams become *information*.
  - In D11's terms, the Fisher information sums over more usable beams, so the posterior is tighter.
  - A mirror can also create **pose aliasing**: the virtual room behind it (D8) can look like a real one, so the naive posterior can go multimodal.

**Show (e) 🟧 (optional) where scan matching goes wrong.** Point-to-point ICP aligns two point sets by repeatedly matching nearest points and solving for the best rigid transform with an SVD (Kabsch).

Careful: ghost points do **not** move "the wrong way" for static walls. `g = Ref(q)` depends only on the true point `q` and the mirror, so ghosts are static in the world. Scan-to-scan ICP sees a rigid, consistent "virtual room". The bias comes from:
1. matching a scan to the real **map**: ghosts have no counterpart and snap to the nearest real wall;
2. a **moving mirror**;
3. **moving objects seen in the mirror**, whose image velocity is `v' = v − 2(v·n̂)n̂`.

Heiden et al. compare against ICP, GICP and NDT. (Not implemented in our code: DECISIONS #8.)

**Used in:** E11; code `localization.motion`, `localization.likelihood` (naive vs. physics), `localization.particle_filter`, optional `localization.icp`.
**Course tie-in:** Bayes' theorem and Markov processes (L4); Bayesian networks / dynamic BN (L4); unicycle kinematics (L2); probabilistic measurement model (L3); estimation (D11/CRLB).

---

## D14 🟥 3D LiDAR: geometry, FOV, and why 2D is a special case

**Sensor:** 16 laser rings at elevations `ε_k ∈ {−15°, −13°, …, +15°}` (2° apart), spinning through 360° in azimuth `φ_j` (1° steps, so 5,760 rays per scan). It's mounted at height `h = 0.6 m` on the robot. This is Velodyne VLP-16-style geometry. We keep the **same power/noise model and calibration** as the 2D sensor so the two are comparable (a modelling choice; state it).

**Show (a) 🟥 beam directions with Euler angles (L5).** Body-frame direction `b_jk = [cos ε_k cos φ_j, cos ε_k sin φ_j, sin ε_k]ᵀ`. World frame: `u_jk = R_z(ψ) R_y(ϑ) R_x(φ_r) b_jk`, the yaw–pitch–roll sequence from L5, like the pinhole handout's pan/tilt rotations. For our planar robot, only yaw `ψ = θ_s` is nonzero, plus an optional fixed mount tilt `ϑ` to show orientation dependence. Sign convention: `R_y(ϑ) = [[cos ϑ, 0, sin ϑ],[0, 1, 0],[−sin ϑ, 0, cos ϑ]]`, so **positive ϑ tips the forward axis down** ("5° downward" = +5°).

**Show (b) 🟥 3D FOV and the blind zone.**
```
S_3D(x) = { q :  R_min ≤ ‖q − p‖ ≤ R_max,   elevation of (q − p) ∈ [ε_min, ε_max],   LOS(p, q) }
```
(Azimuth covers all of 360°, so it drops out.) The elevation must be measured in the **sensor frame**: `w = (R_z R_y R_x)ᵀ (q − p)`, `elevation = atan2(w_z, √(w_x² + w_y²))`. The world-frame elevation is only correct with zero pitch and roll.
- **Floor ring:** the lowest ring hits the floor at horizontal distance `r = h / tan 15° ≈ 2.24 m`. A 5° downward mount tilt pulls it in to ≈ 1.65 m, but **only straight ahead**. The lowest ring's world elevation goes from −20° ahead, to −18.5° at 45°, −14.9° to the side (r ≈ 2.25 m), −11.4° at 135°, and −10° behind (r = 0.6/tan 10° ≈ 3.40 m). E12 plots the exact curve.
- **Blind zone for low objects:** an object of height `z_top` is only hit once `r ≥ (h − z_top) / tan 15°`, with `r` the horizontal distance along that beam. For a 0.3 m box that's **1.12 m**. Rings −15° … −3° then cover it continuously out to ≈ 11.45 m. That coverage is *geometric*; detection still stops at the D5 max range (5.6 m for ρ = 0.9).
- **The 2D sensor at the same height never sees that box at all.** Its FOV is a single horizontal plane, a set of zero volume in 3D. That's a clean, provable FOV/LOS dependency.
- **Ring spacing:** at range `R` the gap between rings is about `R·Δε`, ≈ 17.5 cm at 5 m. Thin horizontal things (a table top) can fall between rings.

**Show (c) 🟥 ray–triangle intersection (Möller–Trumbore), the 3D analogue of D3(c).** Solve `o + t·d = (1 − u − v)·v₀ + u·v₁ + v·v₂` for `(t, u, v)` by Cramer's rule. With `e₁ = v₁ − v₀`, `e₂ = v₂ − v₀`, `P = d × e₂`, `det = e₁·P`, `T = o − v₀`, `Q = T × e₁`:
```
u = (T·P)/det,   v = (d·Q)/det,   t = (e₂·Q)/det
hit iff |det| > τ,  u ≥ 0,  v ≥ 0,  u + v ≤ 1,  t > 0
```
(`τ` is a small tolerance; the letters `u, v, P, T, Q` are local to this formula.) Hint: Cramer's rule writes each unknown as a ratio of scalar triple products. Embree (the ray engine we call through trimesh) does this for us. Our tests check Embree against this formula.

**Show (d) 🟥 the physics carries over unchanged.**
- **Reflection:** `u' = u − 2(u·n̂)n̂` holds in any dimension. The **ghost-point theorem** (D8) holds with "reflection across the mirror *plane*"; the proof uses only dot products.
- **Fresnel/Snell** (D9) and the **power equation** (D5) depend only on the incidence angle and the path length.
- **Incidence angle on a vertical wall:** in 3D the incidence angle picks up the elevation, `cos θ_inc = cos ε · cos θ_inc,horizontal`. This holds for vertical surfaces only. For the floor, `cos θ_inc = |sin ε|`, which is why a grazing floor (ρ = 0.5) is mostly below threshold: with no tilt even the steepest floor hit reaches only 0.84 × the threshold. With a 5° tilt the floor shows up only ahead of the robot: ±79° of azimuth on the −15° ring, ±54° on −13°, ±10° on −11°. That's why E12's floor-ring figure uses a white (ρ = 0.9) floor-only scene.

**Show (e) 🟥 2D is a slice of 3D (our consistency check, E13).** Build the 3D scene by extruding every 2D segment into a vertical wall (with mirrors and glass vertical too), and add a floor and ceiling.
- A horizontal ring (`ε = 0`) then gives **identical** paths, ranges and powers to the 2D sensor. Horizontal rays hit vertical surfaces with horizontal normals, so reflections stay horizontal. Note: the VLP-16 ring set (odd degrees) has **no** ε = 0 ring, so the check (E13, `test_horizontal_ring_equals_2d_scan`) uses a dedicated horizontal test ring.
- For a tilted ring hitting a wall, `R_3D = R_2D / cos ε` (≈ 1.035 × at 15°). This also holds through vertical-mirror bounces, until the ray reaches the floor or ceiling.
- **Power:** at the same horizontal distance, `P_3D = P_2D cos³ε` (one cos from the incidence angle, cos² from the longer path). So the horizontal reach is `R_max cos^{3/2} ε`, ≈ 0.95 R_max at 15°.
- This is how we validate the 3D backend against the 2D one.

**Used in:** E12, E13; code `backends.mesh3d`, `sensor3d`, `scenes3d`, tests in `tests/test_backends.py`.
**Course tie-in:** Euler angles and frames (L5); FOV and LOS (L2, L6); pinhole handout (3D rotations and projection).

---

## Checklist for the hand-off to code

When you finish a derivation, put the **final equation(s) + any constants** in a message or comment so the code can be checked against it. The code has unit tests for:
- D3(c) intersection, D8 ghost theorem, D9 Fresnel table values
- D5 max-range formula, D1/D6 noise std, D10 Lambert-W max range
- D13: particle filter converges in a plain matte room; resampling keeps the weights unbiased
- D14: Embree vs. Möller–Trumbore, the 2D-slice identity, floor-ring radius and blind-zone distance
