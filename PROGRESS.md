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

## 2026-10-19 — Derivations page + reconciliation ✅

**Done**
- Partner page published (private, needs sharing from its Share menu): https://claude.ai/artifact/7VtNLZAhL791QDsZSSYCva. It's built from `plan/02_DERIVATIONS.md`, with MathJax math and BN and ghost-point diagrams.
- The page session re-checked every number in D1–D11 (all headline values correct) and fixed 16 points. All 16 are ported into `plan/02_DERIVATIONS.md`:
  - D7 BN: missing R_i→z_i edge, grouped G_i = (R_i, θ_inc, T_path), corrected d-separation claims
  - D6: likelihood derivation from L3's joint
  - D5: mirror lobe uses R_m; target-specific R₀; principal branch W₀
  - D4: tilted-face beam-gap condition
  - D8: virtual-sensor caveats
  - D9: must/should split, polarisation assumption, glass range bias, "plastic sheet" correction
  - D10: R_max(α) table
  - D11: explicit ∂P_D/∂ρ, ∂P_D/∂α, dR_max/dα, box-face sensitivity, CRLB assumptions
  - D12: phasor (exact) vs. linear mixed-pixel model
  - Notation: 13 symbols + name clashes
- New values recomputed independently in this session before porting. `plan/03_CODE_PLAN.md`: mirror lobe uses R_m; Stage 3 tests check the D10 table and dR_max/dα.

**Next:** Stage 1 (unchanged).

## 2026-10-19 — Libraries + demo/notebook added to the plan ✅

**Done**
- New libraries:
  - core: `imageio-ffmpeg` (bundled ffmpeg, so MP4 export works; tested a matplotlib MP4 + GIF), `tqdm`
  - dev: `jupyterlab`, `ipympl` (+ `ipywidgets`)
- Resolves the Stage 0 "no MP4" note.
- `plan/03_CODE_PLAN.md`:
  - §2 is now a library table with the ones deliberately skipped
  - Stage 6 scripts must expose `run()` for reuse
  - new **Stage 7a** slider demo (`experiments/interactive.py`, `--record` → MP4)
  - new **Stage 7b** walkthrough notebook (`notebooks/lidar_walkthrough.ipynb`, nbconvert → HTML appendix)
  - stretch is now Stage 8, polish Stage 9
- `plan/01_PROJECT_PLAN.md` and README updated to match. Added `notebooks/`.

**Tests:** `uv run pytest` → all pass.

**Next:** Stage 1 (unchanged).
