# 2D LiDAR Simulator — CEE 5810 Robot Perception Midterm

A Python simulation of a 2D scanning LiDAR in a room with matte walls, a mirror, a glass pane, a moving box, and fog. It shows how the measurements depend on sensor pose, target state and features, and operating conditions (Fig. 1 of the assignment).

- Project plan: [plan/01_PROJECT_PLAN.md](plan/01_PROJECT_PLAN.md)
- Derivations (D1–D12): [plan/02_DERIVATIONS.md](plan/02_DERIVATIONS.md)
- Build stages: [plan/03_CODE_PLAN.md](plan/03_CODE_PLAN.md)
- What's done so far: [PROGRESS.md](PROGRESS.md)

## Setup

Requires [uv](https://docs.astral.sh/uv/). uv installs Python 3.12 and the dependencies (numpy, scipy, matplotlib, pillow, pytest) into `.venv/`.

```bash
uv sync
```

## Run

Run everything from this folder.

```bash
uv run pytest
```

Experiment scripts (added from Stage 6) run as modules, e.g. `uv run python -m experiments.e1_sensor_position`. Outputs go to `figures/ammar/` or `figures/partner/`.

## Layout

```
lidar_sim/     simulator package (geometry, materials, world, physics, raytrace, sensor, noise, viz)
experiments/   one script per experiment (E1–E10), generates report media
tests/         pytest checks against the analytic results in plan/02_DERIVATIONS.md
figures/       generated figures and animations, one folder per teammate
plan/          planning docs
```

The course PDFs in this folder are git-ignored, so the repo can be pushed without redistributing them.
