# Decisions made while building (please review)

I made each of these calls so I could keep going. None of them blocks anything. Reply with a number and "change it to …" for any you disagree with. Newest at the bottom.

| # | Decision | Why | Easy to change? |
|---|---|---|---|
| 1 | Mirrors and glass are **two-sided** thin surfaces (both faces reflect/transmit). | Real mirrors have a matte back, but in our scenes rays almost never reach the back. Keeps the model simple. | Yes: add a `one_sided` flag to `Material`. |
| 2 | **Specular gain** `K_s/K = 1/(2θ_w²)` ≈ 730 for θ_w = 1.5°. | Treats the specular lobe as a Gaussian normalised over solid angle, vs. the Lambertian 1/π lobe. Head-on glass then shows up strongly (like Heiden Fig. 1(4)), and only within ~±2°. | Yes: `config.SPEC_GAIN`. D5 in the derivations is updated to say this. |
| 3 | Default-room mirror angled at **45°** (centre (6.6, 4.2)), not facing the room centre. | With the 5.6 m Hokuyo range, a mirror facing the room sends reflections across the room. Total path ≈ 7 m > ~4.7 m max range, so **zero ghosts**. At 45° reflections hit the nearby top wall, and ghosts appear once the robot is within ~3 m. That's realistic, and a nice result in itself. | Yes: `default_room(mirror_angle_deg=…)`. |
| 4 | Detection is decided on the **chosen** return only. Choose by policy on noise-free power, then one Bernoulli draw with its P_D. | Keeps the sim and the particle-filter likelihood consistent (one P_D per beam, exactly D6). A real sensor might fall back to a weaker second return. | Moderate. |
| 5 | Head-on glass is the strongest return out to ~4.9° incidence, and above threshold to ~5.0°. | Follows from decision #2 with R_F ≈ 0.04. So glass shows up as a ~10-beam blip at 1° spacing. | Via `SPEC_GAIN` / `SPEC_LOBE_DEG`. |
| 6 | **GIF/MP4 files are git-ignored**; PNGs are tracked. GIFs render at 80 dpi. | A single 100-frame GIF was 13 MB. Media go to Canvas, not git, and `experiments.run_all` regenerates them. | Yes: remove the lines from `.gitignore`. |
