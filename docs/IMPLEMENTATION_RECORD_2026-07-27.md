# Implementation record — 2026-07-27

## Scope and safety boundary

This record covers the consolidated MuJoCo + RFT cleanup, mesh correction,
validation, and 3×3 video/analysis workflow.

- In scope: `Lizard_Robot_RFT/Lizard_robot-main`
- Original compatibility source:
  `C:\Users\wxy22\Documents\Lizard_Robot_MuJoCo`
- Runtime environment: `lizard_rft`
- Explicitly out of scope: `isaaclab_50`
- IsaacLab was not activated, installed into, edited, renamed, or removed.

## A. Environment consolidation

### Change

- Standardized original MuJoCo and RFT work on `lizard_rft`.
- Added the exact environment to `environment.yml`.
- Added PyMeshLab 2025.7.post1 for reconstruction/remeshing.
- Removed the redundant old CentipedeTracking environment during the earlier
  workspace cleanup.

### Basis

The RFT environment is a superset of the original MuJoCo runtime. IsaacLab
has an unrelated and fragile dependency graph and must remain isolated.

### Result

Both original rigid-floor and RFT models run from one MuJoCo environment.

## B. RFT integration fixes

### Changes

- Sand surface moved to z=0.
- Rigid plane moved to z=-0.25 m as an emergency floor.
- Upstream body-on-sand force is negated at the MuJoCo integration boundary.
- Runtime orientation uses `data.xmat`.
- RFT forces are computed/applied before `mj_step`.
- Inactive smoothed triangle forces are cleared.
- Signed `foot_depth` is separated from nonnegative `sinkage`.
- Per-body force/site/triangle validation is exact.
- Force, power, active-triangle, submerged-triangle, penetration, and
  peak-site diagnostics are logged.

### Basis

The former rigid floor masked sand penetration; force sign/orientation/timing
errors produced nonphysical behavior; stale triangle buffers could apply
force above sand.

### Result

The current integration is finite, dissipative, and physically active. It is
a numerical baseline, not an experimental calibration.

## C. Original rigid-floor compatibility

### Changes

- Corrected short-run speed denominators.
- Unwrapped yaw for heading drift.
- Preserved YAML leg order.
- Rejected invalid/duplicate controller configuration.
- Added collision-resistant output timestamps and explicit renderer cleanup.

### Evidence

Original-repository local commit:
`6c848e90a0658fe8713f0dbca7876ed0465ac573`.

## D. Mesh root cause and correction

### Evidence

Original CAD assembly exports:

- 96,208 triangles total;
- seven bodies with 13 overlapping components each;
- approximately 7,600–8,500 self-intersecting pairs per affected body;
- internal servo/screw/housing surfaces mixed with the exterior.

Former Open3D active remesh:

- 8/8 non-watertight;
- 2,731 non-manifold edges total;
- 12–936 self-intersections per body;
- strongly clustered/coincident RFT centroids.

### Root cause

Vertex clustering and quadric decimation cannot infer the exterior union of
overlapping CAD solids. Vertex clustering additionally merged nearby surfaces
across thin/internal components.

### Changes

- Tracked the manually unified Fusion external envelope as read-only source.
- Added independent mesh auditing.
- Added deterministic Screened Poisson reconstruction.
- Welded STL facet vertices before component selection.
- Added isotropic remeshing and per-body recipe parameters.
- Forced single-thread Poisson after observing multi-thread hash/topology
  variation.
- Added hard topology, orientation, intersection, quality, and surface
  deviation gates.
- Separated candidate build from hash-checked active promotion.
- Removed the two unsafe direct-to-asset remesh scripts.

### Result

- 8/8 active meshes are one component, watertight, manifold, orientable, and
  positive-volume;
- zero boundary, non-manifold, inconsistent-winding, duplicate, degenerate,
  or self-intersecting faces;
- 13,916 triangles/sites;
- candidate-to-Fusion P95 surface deviation: 0.390–0.586 mm.

Exact evidence:
`docs/regressions/2026-07-27-rft-mesh-rebuild/`.

## E. 3×3 video and analysis workflow

### New files

- `scripts/generate_video_matrix.py`
- `scripts/analyze_video_matrix.py`
- `tests/test_video_matrix.py`
- `GUIDANCE.md`
- `docs/VIDEO_MATRIX_GUIDE.md`
- `docs/RESULTS_ANALYSIS_GUIDE.md`
- `docs/TEST_AND_VALIDATION_GUIDE.md`

### Generator behavior

The generator produces:

```text
3 scenarios × 3 cameras = 9 MP4 files
```

Scenarios:

1. original detailed CAD on rigid floor;
2. simplified RFT sand, sites hidden;
3. identical sand replay, sites visible.

Cameras:

1. top;
2. side;
3. 45-degree.

Every frame includes:

- mass-weighted robot COM trajectory;
- current COM and displacement;
- per-component contact 0/1 diagram;
- current time cursor;
- sand penetration and active-triangle count where applicable.

### Contact definitions

- rigid: MuJoCo geom contact with floor, assigned to the nearest named
  component ancestor;
- sand: at least one force-producing active RFT triangle.

### Generated data

- rigid and sand NPZ bundles;
- COM CSV;
- binary contact-timeline CSV;
- continuous contact-event CSV;
- sand penetration/active-triangle CSV;
- static contact diagrams;
- resolved gait config;
- artifact hash/size manifest;
- derived JSON and component-metrics CSV.

### Design basis

- Physics is simulated only once per medium.
- All camera views replay the same qpos states.
- Hidden and visible sand-site videos replay exactly the same RFT trajectory.
- Raw MP4/NPZ output remains ignored; compact evidence and hashes are tracked.

## F. Tests and results

### Unit tests

Command:

```powershell
python -m unittest discover -s tests -v
```

Result: 10/10 passed.

Coverage:

- 3 RFT core tests;
- 7 video/contact/COM/penetration tests.

### Fast project validation

Command:

```powershell
python scripts/validate_project.py
```

Result:

- active mesh audit passed;
- 13,916 sites matched 13,916 triangles;
- rigid smoke passed;
- RFT smoke passed;
- all tests passed.

### Full 6 s RFT baseline

- forward displacement: 0.2177366 m;
- lateral displacement: -0.1208181 m;
- peak total force: 86.6290 N;
- peak site force: 0.5997 N;
- active steps: 98.9167%;
- RFT power: -37.0232 to 0 W;
- positive-power steps: 0%;
- all 19 arrays finite.

### Video-matrix clean smoke

Implementation commit:
`ef27d2bfb82f2752ef341810aaf6300d9a094b0e`

Command:

```powershell
python scripts/generate_video_matrix.py `
  --duration 0.25 --fps 10 `
  --width 640 --height 480 --panel-width 420 `
  --output-dir outputs\video_matrix\release_smoke_ef27d2b
```

Result:

- `git.dirty=false`;
- nine MP4 files generated;
- all artifact hashes verified;
- analyzer completed;
- selected screenshots and contact diagrams visually inspected.

Tracked evidence:
`docs/regressions/2026-07-27-video-matrix-smoke/`.

## G. Output locations and recovery

Raw local targets:

- original runs: `outputs/data/`
- RFT runs: `outputs/sand/`
- mesh audit: `outputs/mesh_audit/`
- candidate meshes: `outputs/mesh_candidates/`
- video matrix: `outputs/video_matrix/`

Tracked evidence:

- mesh rebuild: `docs/regressions/2026-07-27-rft-mesh-rebuild/`
- 6 s pre-rebuild history: `docs/regressions/2026-07-27-rft-6s/`
- video smoke: `docs/regressions/2026-07-27-video-matrix-smoke/`

Large local output is recoverable/identifiable through exact commands, configs,
relative paths, byte sizes, and SHA-256 values in tracked manifests.

## H. Remaining research work

1. Compare the Fusion external envelope and normals against authoritative CAD.
2. Calibrate `RFTCOEFF=3.75` for the actual material.
3. Define locomotion objectives before optimization.
4. Investigate initial impact transients and lateral drift with experiments.
5. Validate the reconstructed hardware duty law against hardware logs/source.

## I. GitHub publication

- Publication repository: `https://github.com/wxy108/Lizard_robot`
- Branch: `agent/rft-video-guidance`
- Existing GitHub baseline: `79a2b905`
- Original-model compatibility commit: `6c848e9`
- Clean project-root RFT integration commit: `ed5edc6`
- Imported local documented snapshot: `48952e6`

The publication keeps the GitHub repository's existing root layout. It does
not introduce the local workspace's extra `Lizard_robot-main/` nesting or an
unrelated-history merge. All 119 regular tracked project files and the RFT-SiM
submodule pointer were verified against the local documented snapshot before
commit. Historical generated MP4 files were removed from Git tracking but
remain recoverable from earlier history. Raw future output belongs in ignored
`outputs/`; the reviewed canonical production set is separately tracked under
`docs/media/` according to decision D-006.

## J. Master video overview

- Added `scripts/compose_video_matrix.py`.
- Added automatic `videos/video_matrix_overview.mp4` generation.
- Layout: three scenario rows × `Top | Side | 45° | Analysis`.
- The first three columns contain the nine pure camera views; the last column
  contains exactly three scenario-level COM/contact/penetration panels.
- Composition consumes synchronized MP4 output and does not rerun physics.
- Added an overview-frame regression test; the suite at that stage was 11/11.

## K. Portable fresh-system deployment and published media

- Added Windows PowerShell and Linux/macOS/WSL setup scripts.
- Removed local absolute paths from all primary startup commands.
- Pinned every top-level Python dependency in `environment.yml`.
- Documented Git/Conda prerequisites, submodule recovery, manual setup,
  environment update, macOS `mjpython`, Linux EGL headless rendering, and
  clean-room reproduction.
- Published nine individual MP4 files, the master overview, an animated
  preview, a representative frame, and a complete SHA-256 table under
  `docs/media/video_matrix_production_6s/`.
- Kept raw NPZ/CSV/experimental runs in ignored `outputs/`.

## L. Rejected-mesh diagnostic evidence

- Added `scripts/generate_failed_mesh_videos.py`.
- Selected only three representative failures: raw CAD Back assembly, legacy
  vertex-clustered Back, and direct fixed-count Fusion FR.
- Preserved the 94,884-byte legacy Back STL with its source path and SHA-256;
  it is diagnostic evidence and cannot be an active asset.
- Rendered rejected/source-only and accepted centroids in the same body frame
  and camera, colored by normalized local nearest-centroid spacing.
- Put exact independent topology/distribution audits on-screen and in a
  machine-readable manifest.
- Generated from clean commit `5c90d91`: three 97-frame, 24 FPS, 1280×720
  MP4 files plus a preview and contact sheet.
- Published the bounded, hash-documented set under
  `docs/media/failed_mesh_diagnostics/`.
- Added two tests; current fast suite: 13/13.
- Did not run invalid meshes through locomotion/RFT physics.

## M. Canonical local workspace consolidation

- Designated `Lizard_Robot_MuJoCo` as the only active local Git root.
- Verified it was clean and identical to GitHub `main` at `0186850`.
- Moved the 894,219,200-byte former outer RFT workspace and the
  47,832,103-byte stale publication checkout to
  `Lizard_Robot_Archive/2026-07-27/`.
- Preserved both old Git repositories and all archive/output files; nothing
  was deleted.
- Detected that the canonical RFT-SiM submodule used a local object alternate
  pointing at the former outer workspace.
- Repacked 232 reachable objects into a 21.7 MiB canonical submodule pack,
  removed the alternate, ran `git fsck --full`, and confirmed HEAD remained
  `303283f`.
- Reran the rejected-mesh generator after relocation. It produced all three
  cases and five artifacts from clean commit `0186850`, proving that the
  tracked legacy Back STL is sufficient and the archive is not a runtime
  dependency.
