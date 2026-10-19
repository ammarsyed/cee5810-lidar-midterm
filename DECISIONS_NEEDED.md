# Decisions made while building (please review)

I made each of these calls so I could keep going. None of them blocks anything. Reply with a number and "change it to …" for any you disagree with. Newest at the bottom.

| # | Decision | Why | Easy to change? |
|---|---|---|---|
| 1 | Mirrors and glass are **two-sided** thin surfaces (both faces reflect/transmit). | Real mirrors have a matte back, but in our scenes rays almost never reach the back. Keeps the model simple. | Yes: add a `one_sided` flag to `Material`. |
| 2 | **Specular gain** `K_s/K = 1/(2θ_w²)` ≈ 730 for θ_w = 1.5°. | Treats the specular lobe as a Gaussian normalised over solid angle, vs. the Lambertian 1/π lobe. Head-on glass then shows up strongly (like Heiden Fig. 1(4)), and only within ~±2°. | Yes: `config.SPEC_GAIN`. D5 in the derivations is updated to say this. |
