# Lizard Robot MuJoCo + RFT guidance

This is the main handoff for humans and AI agents. It explains what is
authoritative, how to run the project, how to rebuild meshes, how to generate
the 3×3 video matrix, where outputs go, how to analyze them, and which tests
must pass.

## 1. Current authoritative state

- Conda environment: `lizard_rft`
- Original detailed CAD locomotion: `run.py` + `models/lizard.xml`
- Simplified topology-gated RFT locomotion: `lizard_sand.py` +
  `Lizard_Sand.xml`
- Active RFT meshes: `asset/*.STL`
- Read-only external-envelope reconstruction source:
  `models/mesh_sources/fusion_external_envelope/*.STL`
- RFT mesh recipe: `configs/rft_mesh_recipe.json`
- Force sites: 13,916, exactly one per active mesh triangle
- Sand surface: z = 0
- Emergency rigid floor: z = -0.25 m
- RFT raw convention: body-on-sand; the integration applies its negative to
  the robot
- IsaacLab: out of scope and untouched

The automated mesh checks are clean. The remaining research boundary is
semantic CAD review: confirm that the manually unified Fusion envelope and
outward normals are the intended physical sand-contact surface.

## 2. Start and validate

```powershell
conda activate lizard_rft
cd C:\path\to\Lizard_robot

python scripts/validate_project.py
```

Full behavior-changing validation:

```powershell
python scripts/validate_project.py --full
```

The validator checks active mesh topology/self-intersections, triangle/site
correspondence, MuJoCo model invariants, all unit tests, rigid locomotion, and
RFT locomotion.

## 3. Interactive locomotion

Original detailed CAD model on a rigid flat floor:

```powershell
python run.py
```

Simplified RFT model on sand:

```powershell
python lizard_sand.py --view --duration 10
```

Show every RFT triangle site:

```powershell
python lizard_sand.py --view --duration 10 --show-force-sites all
```

Show one body only:

```powershell
python lizard_sand.py --view --duration 10 --show-force-sites FR
```

Valid body names are `Mid Front FR FL Back HR HL Tail`.

## 4. Generate the 3×3 video matrix

Production command:

```powershell
python scripts/generate_video_matrix.py --duration 6 --fps 30
```

The generator simulates rigid ground once and RFT sand once, then replays
those states from three cameras:

- `top`: top-down;
- `side`: side view, best for penetration;
- `diag45`: 45-degree view.

It produces three visual scenarios:

- `rigid_original`: original detailed CAD on a rigid flat floor;
- `sand_simplified`: simplified topology-gated mesh on RFT sand, sites hidden;
- `sand_simplified_sites`: the exact same sand replay with all RFT sites
  visible.

This gives nine individual MP4 files plus
`videos/video_matrix_overview.mp4`. The overview has three scenario rows and
four columns: `Top | Side | 45° | Analysis`. Its first three columns are the
nine pure camera views; its final column contains exactly one panel per
scenario.

Every individual frame contains:

- robot center-of-mass trajectory and current COM marker;
- COM displacement in millimetres;
- maximum current sand penetration and active-triangle count for sand runs;
- an eight-row component contact diagram;
- a current-time cursor.

Short rendering smoke:

```powershell
python scripts/generate_video_matrix.py `
  --duration 0.25 --fps 10 `
  --width 640 --height 480 --panel-width 420
```

Compose an overview for an older compatible run:

```powershell
python scripts/compose_video_matrix.py outputs\video_matrix\run_...
```

Full details: `docs/VIDEO_MATRIX_GUIDE.md`.

## 5. Analyze a video-matrix run

The generator prints the run directory. Analyze it with:

```powershell
python scripts/analyze_video_matrix.py `
  outputs\video_matrix\run_YYYY-MM-DD_HHMMSS_microseconds
```

The analyzer:

- verifies every artifact hash and size from `matrix_manifest.json`;
- reports COM displacement, COM path length, and mean COM speed;
- reports per-component contact duty, event count, first contact, last release,
  mean/max contact-event duration;
- reports per-component maximum penetration and active-triangle counts for
  RFT sand;
- writes `analysis/derived_metrics.json`;
- writes `analysis/component_metrics.csv`.

File-by-file extraction examples and data definitions are in
`docs/RESULTS_ANALYSIS_GUIDE.md`.

## 6. Output policy and target folders

Raw generated data is local and intentionally ignored by Git:

```text
outputs/
├── data/                  # original rigid-floor runs
├── sand/                  # RFT runs
├── mesh_audit/            # mesh metrics and comparison plots
├── mesh_candidates/       # isolated mesh builds before promotion
└── video_matrix/          # 3×3 videos and their analysis
```

Compact evidence, configs, summaries, hashes, and selected screenshots belong
under `docs/regressions/`. Do not commit large raw NPZ or MP4 files.

## 7. Rebuild and promote RFT meshes

```powershell
python scripts/build_rft_mesh_candidates.py
python scripts/promote_rft_mesh_candidate.py `
  outputs\mesh_candidates\run_YYYY-MM-DD_HHMMSS_microseconds
python scripts/validate_project.py --full
```

The builder is isolated from `asset/`. Promotion refuses failed, incomplete,
recipe-mismatched, CLI-overridden, or hash-mismatched candidates.

Do not recreate the removed `remesh_even.py` or `remesh_clean.py` workflows.
They directly simplified overlapping CAD assembly exports and could create
non-watertight/self-intersecting active meshes.

## 8. Tests

```powershell
python -m unittest discover -s tests -v
```

Test files:

- `tests/test_rft_core.py`: force sign, zero force above sand, rotation;
- `tests/test_video_matrix.py`: contact intervals, component mapping, frame
  sampling, dashboard dimensions, penetration/contact metrics, COM metrics.

See `docs/TEST_AND_VALIDATION_GUIDE.md` for all validation levels and expected
outputs.

## 9. Required reading before behavioral changes

1. `AGENTS.md`
2. `GUIDANCE.md`
3. `docs/PROJECT_STATUS.md`
4. `docs/CHANGELOG.md`
5. `docs/DECISIONS.md`
6. `docs/PROVENANCE.md`
7. `docs/VALIDATION.md`
8. `docs/IMPLEMENTATION_RECORD_2026-07-27.md`

Live code, active configs, tests, and reproducible raw output take precedence
over historical prose.
