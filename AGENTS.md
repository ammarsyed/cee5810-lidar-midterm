# AGENTS.md — instructions for AI coding assistants

Project: physics-based 2D/3D LiDAR simulator + particle-filter localization for a CEE 5810 (Robot Perception) midterm. Two students share this code; each writes their **own** report with **distinct** media. Read [README.md](README.md) for the overview and repository map, and [docs/02_DERIVATIONS.md](docs/02_DERIVATIONS.md) for the math (D1–D14).

## Setup and commands
- Python 3.12 via **uv**. Always run things through uv from the repo root: `uv sync`, `uv run …`.
- Tests: `uv run pytest` (must stay green; ~20 s).
- Lint: `uvx ruff check .` (must pass; config in `pyproject.toml`).
- Regenerate all media for one person: `uv run python -m experiments.run_all --person ammar|partner` (~4.5 min; add `--fast --no-anim` for a quick check, but don't leave fast-mode figures in `figures/`).
- Demo tools: `uv run python -m demo.interactive`, `uv run python -m demo.board --open`, `uv run python -m demo.tour`.
- Notebook: edit `notebooks/make_notebook.py`, then `uv run python notebooks/make_notebook.py` (never hand-edit the .ipynb).

## Conventions
- `lidar_sim/` is the library; `experiments/` produce report results; `demo/` holds presentation tools.
- Every experiment module exposes `run(scene="default", seed=0, fast=False) -> {"figs": {...}, "tables": {name: (header, rows)}, "data": {...}}` with no file I/O, plus `main()` using `experiments.common.parser / resolve / finish`.
- Scenes are per person: `ammar` → `default`, `partner` → `variant_b`. Any new experiment needs presets for both in `experiments/common.PRESETS`, so the two reports get different figures.
- Physics lives in `lidar_sim/physics.py`; the tracer must call it rather than re-implementing formulas. Constants live in `lidar_sim/config.py`.
- Every new formula gets a test that compares the simulation with the closed form from `docs/02_DERIVATIONS.md`.
- Plotting: use `lidar_sim/viz.py` (palette + `set_style`); at most three categorical colours in scatter plots.
- Record modelling choices in [DECISIONS.md](DECISIONS.md) (plain English: what / tradeoff / recommendation) and progress in [PROGRESS.md](PROGRESS.md) (update "Status" and "What's left").

## Git rules (from the repo owner)
- Commits must show **only the human author**. Do **not** add `Co-Authored-By` or any AI attribution trailers to commit messages or PR descriptions.
- Never commit `course_materials/` (copyrighted PDFs), videos (`figures/**/*.gif|mp4`), the executed notebook HTML, or `.venv/`. `.gitignore` already covers these.
- Keep commit messages short and plain.
