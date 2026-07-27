# Lizard Robot V1 — MuJoCo rigid floor + granular RFT

This repository is the active lizard-robot simulation. One isolated Conda
environment, `lizard_rft`, runs both the original rigid-floor MuJoCo model and
the topology-gated triangle-level 3D resistive-force-theory (RFT) model.
IsaacLab is unrelated and remains untouched.

## New: reproducible 3 × 3 locomotion video matrix

One command now creates nine synchronized locomotion videos:

| Scenario | Top | Side | 45-degree |
| --- | --- | --- | --- |
| Original detailed model on rigid ground | MP4 | MP4 | MP4 |
| Simplified model on RFT sand, sites hidden | MP4 | MP4 | MP4 |
| Simplified model on RFT sand, sites visible | MP4 | MP4 | MP4 |

Every video includes the mass-weighted center-of-mass trajectory, displacement,
and an eight-component binary contact timeline. Sand videos also show maximum
penetration and the active RFT-triangle count. The two sand presentations
replay the same recorded states, so showing the sites cannot change the
physics.

```powershell
conda activate lizard_rft
cd C:\Users\wxy22\Documents\Lizard_Robot_RFT\Lizard_robot-main

python scripts/generate_video_matrix.py --duration 6 --fps 30
python scripts/analyze_video_matrix.py outputs\video_matrix\run_...
```

Large videos and raw arrays remain under ignored `outputs/`. A compact,
hash-verified smoke run is tracked under
`docs/regressions/2026-07-27-video-matrix-smoke/`.

Read [GUIDANCE.md](GUIDANCE.md) first for the complete start, validation,
video, output, analysis, mesh-rebuild, and handoff workflow.

## Quick start

```powershell
conda activate lizard_rft
cd C:\Users\wxy22\Documents\Lizard_Robot_RFT\Lizard_robot-main

# Fast project validation
python scripts/validate_project.py

# Original detailed model on rigid ground
python run.py

# Latest simplified model with granular RFT
python lizard_sand.py --view --duration 10

# Show every RFT force point
python lizard_sand.py --view --duration 10 --show-force-sites all

# Show only one component's points
python lizard_sand.py --view --duration 10 --show-force-sites FR

# Headless run: saves results.npz, summary.json, and config.yaml
python lizard_sand.py --duration 10
```

MuJoCo is installed as a Python dependency inside `lizard_rft`; no separate
system installation is needed. The environment is reproducible from
`environment.yml`.

## Current validated baseline

- Eight active RFT meshes are closed, single-component, consistently oriented,
  and free of detected self-intersections.
- `Lizard_Sand.xml` contains 13,916 sequential force sites, exactly one per
  active triangle.
- Force sites are hidden in visual group 5 by default and can be displayed for
  one component or all components.
- The RFT surface is at z = 0; the rigid plane at z = -0.25 m is only an
  emergency catch floor.
- The upstream routine returns body-on-sand force; the integration applies the
  equal-and-opposite reaction to the robot before `mj_step`.
- Ten fast unit tests, rigid-floor smoke, granular smoke, topology gates, and
  the clean nine-video smoke run pass.

This is a stable numerical baseline, not a claim of physical calibration. The
external-envelope interpretation still needs confirmation against the
authoritative CAD, and `RFTCOEFF=3.75` must be calibrated for the real granular
material.

## Project map

```text
.
|-- GUIDANCE.md                    # complete human/agent start and handoff
|-- run.py                         # original rigid-floor runner
|-- lizard_sand.py                 # granular RFT runner
|-- sim_fxn_lib.py                 # vectorized 3D RFT calculations
|-- Lizard_Sand.xml                # RFT scene and generated force sites
|-- environment.yml                # canonical Conda environment
|-- configs/                       # gait and mesh-recipe configuration
|-- controllers/                   # deterministic CPG gait controller
|-- asset/                         # active topology-gated RFT surfaces
|-- models/                        # rigid model, source meshes, generated sites
|-- scripts/                       # build, validation, video, and analysis tools
|-- tests/                         # fast physics and reporting tests
|-- docs/                          # decisions, provenance, guides, evidence
|-- third_party/RFT-SiM/           # pinned upstream Git submodule
`-- outputs/                       # generated local runs; intentionally ignored
```

## RFT mesh rebuild

```powershell
python scripts/build_rft_mesh_candidates.py

# Promotion refuses failed, incomplete, recipe-mismatched, or hash-mismatched
# builds. Replace run_... with the directory printed by the builder.
python scripts/promote_rft_mesh_candidate.py outputs\mesh_candidates\run_...

python scripts/validate_project.py --full
```

The builder reads the tracked Fusion external-envelope sources and
`configs/rft_mesh_recipe.json`, writes only to ignored `outputs/`, and requires
closed, single-component, consistently oriented, zero-self-intersection
surfaces before promotion. The retired Open3D decimation scripts were removed
because they merged overlapping CAD subassemblies and could replace active
meshes without topology gates.

## Documentation

Start with:

1. [GUIDANCE.md](GUIDANCE.md)
2. [Implementation record](docs/IMPLEMENTATION_RECORD_2026-07-27.md)
3. [Video matrix guide](docs/VIDEO_MATRIX_GUIDE.md)
4. [Results analysis guide](docs/RESULTS_ANALYSIS_GUIDE.md)
5. [Test and validation guide](docs/TEST_AND_VALIDATION_GUIDE.md)
6. [Project status](docs/PROJECT_STATUS.md)
7. [Decisions](docs/DECISIONS.md)
8. [Provenance](docs/PROVENANCE.md)

The original rigid-floor working copy is preserved separately at
`C:\Users\wxy22\Documents\Lizard_Robot_MuJoCo`.
