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
