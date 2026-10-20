"""Generate notebooks/lidar_walkthrough.ipynb (the notebook is a thin viewer over the package).

Run: uv run python notebooks/make_notebook.py
Then: uv run jupyter nbconvert --to html --execute notebooks/lidar_walkthrough.ipynb
"""

from pathlib import Path

import nbformat as nbf

md, code = nbf.v4.new_markdown_cell, nbf.v4.new_code_cell

cells = [
    md("""# LiDAR perception simulation — walkthrough

CEE 5810 Robot Perception, midterm project. A 2D (and 16-ring 3D) LiDAR in a room with matte walls, a **mirror**,
a **glass pane**, a **moving box** and **fog**. Then a particle filter uses the sensor model to localize a robot.

This notebook is only a **viewer**. Every figure comes from the `lidar_sim` package, an experiment's `run()` function or a `demo/` module,
the same code the command-line scripts use. Set `PERSON = "gal"` for Gal's run (his room, seeds
and parameters from `people/gal.toml`), and `FAST = True` for a quick pass.

Maths: `docs/02_DERIVATIONS.md` (D1–D14). Experiments: E1–E13."""),
    code("""import os, sys
from pathlib import Path
ROOT = Path.cwd() if (Path.cwd() / "lidar_sim").exists() else Path.cwd().parent
os.chdir(ROOT); sys.path.insert(0, str(ROOT))
%matplotlib inline
import matplotlib.pyplot as plt
from IPython.display import Markdown, display
from experiments.common import markdown_table

PERSON = "ammar"    # "ammar" or "gal": whose config (people/<name>.toml) to use
FAST = False        # True = fewer Monte Carlo samples / seeds
RESULTS = {}

def show(name, res):
    RESULTS[name] = res
    for fig in res.get("figs", {}).values():
        display(fig); plt.close(fig)
    for title, (header, rows) in res.get("tables", {}).items():
        display(Markdown(f"**{name} — {title}**\\n\\n" + markdown_table(header, rows)))"""),
    md("""## 1. The sensor and the scene

A Hokuyo-URG-04LX-like planar LiDAR has a 240° FOV, 241 beams, 1 cm range noise (σ_t ≈ 67 ps) and a 5.6 m max range on a white wall.
Every surface has a material: matte (Lambertian, reflectivity ρ), mirror (R_m = 0.95) or glass (n = 1.5)."""),
    code("""from experiments import params_table
display(Markdown(params_table.run(PERSON)["text"]))"""),
    md("""## 2. One scan, explained

- **Blue dots** are what the LiDAR reports.
- **Orange crosses** are **ghosts**: light that bounced off the mirror. The LiDAR assumes straight lines, so it places these points behind the mirror.
- **Aqua circles** are where that light really came from (D8).
- **Gray rays** are beams with no return: too dark, too far, or a grazing mirror/glass hit.
- The shaded region is the FOV with line of sight (D4); glass is transparent."""),
    code("""from demo import tour
show("scan", tour.snapshot(PERSON))"""),
    md("""## 3. Sensor state → measurements (Fig. 1: x → Z)

**E1:** move the sensor. The range to a flat wall follows the closed form R = (d − n·p)/(n·u) (D3).
Moving *parallel* to a wall leaves that wall's range unchanged.

**E2:** rotate the sensor. The FOV wedge, and with it what the sensor can see, rotates too (D4)."""),
    code("""from experiments import e1_sensor_position, e2_sensor_heading
show("E1", e1_sensor_position.run(PERSON, fast=FAST))
show("E2", e2_sensor_heading.run(PERSON, fast=FAST))"""),
    md("""## 4. Target state and features → measurements (Fig. 1: x_T, X → Z)

**E3:** a moving box enters and leaves a beam exactly when its geometry predicts (line of sight).

**E4:** reflectivity and orientation set the detection probability and the max range (D5, D6)."""),
    code("""from experiments import e3_moving_target, e4_reflectivity
show("E3", e3_moving_target.run(PERSON, fast=FAST))
show("E4", e4_reflectivity.run(PERSON, fast=FAST))"""),
    md("""## 5. Mirror and glass

**E5:** ghost points are the mirror image of the true hit point (D8). They lie on a "virtual room" behind the mirror.

**E6:** glass reflects only 4% head-on (Fresnel, D9). The LiDAR sees *through* it, except within about 5° of head-on."""),
    code("""from experiments import e5_mirror, e6_glass
show("E5", e5_mirror.run(PERSON, fast=FAST))
show("E6", e6_glass.run(PERSON, fast=FAST))"""),
    md("""## 6. Operating conditions → measurements (Fig. 1: Λ → Z)

**E7:** fog attenuates the signal as e^(−2αR) (Beer–Lambert). The max range follows R_max = W₀(αR₀)/α (D5, D10)."""),
    code("""from experiments import e7_fog
show("E7", e7_fog.run(PERSON, fast=FAST))"""),
    md("""## 7. The noise model, checked by Monte Carlo (E8)"""),
    code("""from experiments import e8_noise_mc
show("E8", e8_noise_mc.run(PERSON, fast=FAST))"""),
    md("""## 8. Bayesian network: learning a CPT from the simulator (D7, L4)

The table below is P(detected | material, fog, range bin), estimated by counting simulated shots.
That's "learning CPTs from data", with the simulator as the data source."""),
    code("""from experiments import e_bn_cpt
show("CPT", e_bn_cpt.run(PERSON, fast=FAST))"""),
    md("""## 9. Localization: the sensor model used for estimation (E11, D13)

Three particle filters get identical scans and odometry:
- **N1:** naive model (mirror and glass treated as opaque walls)
- **N2:** naive model + robust mixture
- **P:** our physics-aware model

In the glass corridor, along-corridor information mostly comes *through the glass*.
The notebook uses 3 seeds to stay fast; `experiments/e11_localization.py` runs 20."""),
    code("""from experiments import e11_localization
show("E11", e11_localization.run(PERSON, fast=True))"""),
    md("""## 10. 3D LiDAR (E12, E13, D14)

A 16-ring, VLP-16-style sensor with the same physics.
- **E12:** a planar LiDAR can't see the low box or the table top.
- **E13:** the 2D model is exactly the ε = 0 slice of the 3D one."""),
    code("""from experiments import e12_3d_fov, e13_2d3d_consistency
show("E12", e12_3d_fov.run(PERSON, fast=FAST))
show("E13", e13_2d3d_consistency.run(PERSON, fast=FAST))"""),
    md("""## 11. Play with it

Set `RUN_INTERACTIVE = True` and run the cell. It switches to the widget backend (ipympl) and opens the same slider demo as
`uv run python -m demo.interactive`. It's left off so the exported HTML stays static."""),
    code("""RUN_INTERACTIVE = False
if RUN_INTERACTIVE:
    %matplotlib widget
    from demo.interactive import launch
    fig, controls = launch(PERSON)"""),
    md("""## 12. Summary: theory vs simulation"""),
    code("""def find(name, table, label, col=1):
    for r in RESULTS[name]["tables"][table][1]:
        if str(r[0]).startswith(label):
            return r[col]
    return float("nan")

e1 = max(float(r[2]) for r in RESULTS["E1"]["tables"]["closed_form"][1])
e4 = max(float(r[3]) for r in RESULTS["E4"]["tables"]["max_range"][1][:4])
e7 = max(float(r[1]) for r in RESULTS["E7"]["tables"]["fog"][1][:3])
e11 = {r[0]: r[1] for r in RESULTS["E11"]["tables"]["glass_corridor_summary"][1][:3]}
rows = [
    ["E1", "range to a flat wall vs closed form (D3)", f"{e1:.1e} m"],
    ["E2", "FOV membership prediction vs simulated detection (D4)", find("E2", "summary", "agreement")],
    ["E4", "max range vs reflectivity (D5)", f"{e4:.1e} m"],
    ["E5", "ghost point = mirror image of true hit (D8)", f"{find('E5', 'ghost_theorem', 'max |reported'):.1e} m"],
    ["E6", "pane transmission vs Fresnel (D9)", f"{find('E6', 'glass', 'max |sim'):.1e}"],
    ["E7", "fog max range vs W0(αR0)/α (D10)", f"{e7:.1e} m"],
    ["E8", "range noise ~ N(R, σ_R²): KS p-value (D6)", f"{find('E8', 'noise', 'KS p-value'):.2f}"],
    ["E11", "corridor RMSE: naive+mixture vs physics-aware (D13)",
     f"{e11['N2 naive + mixture']:.3f} m vs {e11['P physics-aware']:.3f} m"],
    ["E13", "3D ε=0 ring vs 2D scan (D14)", f"{find('E13', 'consistency', 'max |R_3D(ε=0)'):.1e} m"],
]
display(Markdown(markdown_table(["exp.", "check", "result"], rows)))"""),
]

nb = nbf.v4.new_notebook()
nb["cells"] = cells
nb["metadata"]["kernelspec"] = {"name": "python3", "display_name": "Python 3", "language": "python"}
out = Path(__file__).with_name("lidar_walkthrough.ipynb")
nbf.write(nb, out)
print(out)
