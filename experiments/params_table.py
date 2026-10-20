"""Dump every model parameter and the scene layout to params_table.md (for the report's Procedure).

Run: uv run python -m experiments.params_table --person ammar|gal
"""

from pathlib import Path

from experiments.common import load_person, parser, resolve, scene_of
from lidar_sim import config
from lidar_sim.physics import max_range
from lidar_sim.sensor import LidarParams


def run(person="ammar", seed=None, fast=False):
    cfg = load_person(person)
    p = LidarParams()
    w = scene_of(cfg)
    lines = [f"# Simulation parameters — {cfg['name']} ({cfg['scene']} scene, base seed {cfg['seed']})", "",
             f"Experiment-specific settings (poses, reflectivities, distances, tilt, …) are in people/{cfg['key']}.toml.",
             "", "## Sensor (2D, Hokuyo URG-04LX-like)", "",
             "| parameter | symbol | value |", "|---|---|---|",
             f"| angular FOV | Φ | {p.fov_deg:.0f}° |", f"| beams | N | {p.n_beams} (Δβ = {p.fov_deg / (p.n_beams - 1):.1f}°) |",
             f"| min range | R_min | {p.r_min} m |", f"| timing jitter | σ_t | {p.sigma_t * 1e12:.1f} ps |",
             f"| range noise | σ_R = cσ_t/2 | {p.sigma_r * 100:.2f} cm |",
             f"| power noise | σ_P | {p.sigma_p_rel} P_th |",
             f"| calibration | K/P_th | {p.k_over_pth:.2f} m² (R_max = {config.R_MAX_REF} m at ρ = {config.RHO_REF}) |",
             f"| specular lobe width | θ_w | {p.spec_lobe_deg}° |", f"| specular gain | K_s/K | {p.spec_gain:.0f} |",
             f"| max bounces | B | {p.max_bounces} |", f"| return policy | — | {p.return_policy} |", "",
             "## Materials", "", "| material | parameters |", "|---|---|",
             f"| matte walls | ρ = {config.RHO_WALL} (dark 0.1, bright 0.9); R_max = {float(max_range(0.7)):.2f} / "
             f"{float(max_range(0.1)):.2f} / {float(max_range(0.9)):.2f} m |",
             f"| mirror | R_m = {config.R_MIRROR} |", f"| glass | n = {config.N_GLASS} (R_F = 0.04 head-on) |", "",
             "## 3D sensor (VLP-16-like geometry)", "", "| parameter | value |", "|---|---|",
             f"| rings | {config.N_RINGS}, {config.ELEV_MIN_DEG:.0f}° … {config.ELEV_MAX_DEG:.0f}° |",
             f"| azimuth step | {config.AZ_STEP_DEG}° |", f"| mounting height | {config.SENSOR_HEIGHT} m |", "",
             "## Scene surfaces", "", "| # | name | material | from | to |", "|---|---|---|---|---|"]
    for i, s in enumerate(w.at_time(0).segments):
        m = s.material
        desc = f"{m.kind}" + (f" ρ={m.rho}" if m.kind == "matte" else "")
        lines.append(f"| {i} | {s.name} | {desc} | ({s.a[0]:.2f}, {s.a[1]:.2f}) | ({s.b[0]:.2f}, {s.b[1]:.2f}) |")
    return {"text": "\n".join(lines) + "\n"}


def main():
    args = resolve(parser(__doc__).parse_args())
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    path = out / "params_table.md"
    path.write_text(run(args.person)["text"])
    print(path)


if __name__ == "__main__":
    main()
