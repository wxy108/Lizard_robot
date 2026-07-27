# Changelog

All notable project changes are recorded here. Git is the authoritative
line-level record; this file explains intent and consequences.

## 2026-07-27 — portable deployment and directly viewable outputs

- Added `scripts/setup.ps1` for Windows and `scripts/setup.sh` for
  Linux/macOS/WSL clean-system deployment.
- Pinned all top-level Python packages in `environment.yml`.
- Rewrote `README.md` and `GUIDANCE.md` from fresh clone through installation,
  validation, viewing, simulation, generation, analysis, headless servers,
  macOS, troubleshooting, mesh rebuild, and clean-room reproduction.
- Added the canonical 10-video production set under
  `docs/media/video_matrix_production_6s/`.
- Added an animated 720×368 preview and direct links to every MP4 so results
  are visible without first installing the environment.
- Preserved raw/experimental outputs under ignored `outputs/`; only the
  bounded reviewed production media set is tracked.

## 2026-07-27 — 3×3 master overview with three analysis panels

- Added `scripts/compose_video_matrix.py`.
- The normal generator now emits nine individual view videos and one master
  overview automatically.
- The overview contains three scenario rows, three pure camera-view columns,
  and exactly one synchronized analysis panel per row.
- Added a frame-layout regression test, bringing the fast suite to 11/11.
- Generated and hash-verified the clean production 6 s matrix and overview;
  compact evidence is tracked under
  `docs/regressions/2026-07-27-video-matrix-production-6s/`.

## 2026-07-27 — reproducible locomotion video and analysis matrix

### Synchronized simulation and rendering

- Added `scripts/generate_video_matrix.py` to generate the requested 3 × 3
  combination of rigid/RFT scenarios and top/side/45-degree views.
- Recorded each physical scenario once and replayed its states for every
  camera. The site-hidden and site-visible sand presentations share exactly
  the same replay.
- Added a 45-degree diagnostic camera and made the non-colliding visual sand
  translucent so penetration is visible without changing dynamics.
- Added mass-weighted COM trajectory/displacement, RFT penetration and active
  triangle diagnostics, and eight-component binary contact timelines to every
  applicable video.

### Analysis, evidence, and handoff

- Added `scripts/analyze_video_matrix.py` to verify manifest sizes/hashes and
  derive COM-path, contact duty/event timing, penetration, and triangle
  activity metrics.
- Added seven video/reporting tests, bringing the fast suite to 10/10.
- Added `GUIDANCE.md`, focused video/results/test guides, and a complete dated
  implementation record.
- Added clean 0.25 s smoke evidence under
  `docs/regressions/2026-07-27-video-matrix-smoke/`; large MP4 and raw arrays
  remain ignored under `outputs/`.
- The smoke run generated and hash-verified all nine MP4 files. It verifies
  reporting behavior only and is not a gait-performance baseline.

## 2026-07-27 — topology-gated RFT mesh rebuild

### Root cause and source boundary

- Proved that seven of eight original CAD-export STLs contain 13 overlapping
  components, non-manifold edges, and roughly 7,600–8,500 self-intersecting
  triangle pairs. They are assemblies/internal detail, not one contact shell.
- Proved that the retired Open3D vertex-clustering pipeline merged nearby
  surfaces across those overlapping components. The active meshes it produced
  were non-watertight, had 2,731 non-manifold edges in total, and retained
  12–936 self-intersection pairs per body.
- Preserved the manually unified Fusion external-envelope meshes as tracked,
  read-only reconstruction input under
  `models/mesh_sources/fusion_external_envelope/`.

### Deterministic replacement pipeline

- Added a read-only mesh auditor for topology, self-intersection, triangle
  quality, RFT-centroid spacing, orientation, and signed volume.
- Added a deterministic candidate builder using MeshLab Screened Poisson
  reconstruction plus isotropic remeshing.
- Fixed STL facet welding before connected-component selection; without it,
  one valid reconstruction was incorrectly reduced to two triangles.
- Forced single-thread Poisson reconstruction after proving that multi-thread
  reductions changed output hashes and could create 0–2 marginal
  self-intersections between identical builds.
- Added `configs/rft_mesh_recipe.json`, per-body effective parameters, hard
  quality gates, SHA-256 manifests, and an isolated `outputs/` build boundary.
- Added an explicit promotion tool that refuses failed, incomplete,
  recipe-mismatched, or hash-mismatched candidates.
- Removed `remesh_even.py` and `remesh_clean.py`; both wrote directly to
  `asset/` without topology gates and operated on the overlapping CAD export.

### Active model and regression

- Replaced all eight active RFT STLs with one-component, watertight,
  consistently oriented, zero-self-intersection surfaces.
- Rebuilt `Lizard_Sand.xml` and `models/lizard_sites.xml` with 13,916
  sequential force sites, exactly one per active triangle.
- Changed project validation from a hard-coded site count to the live
  per-body triangle counts and added active-mesh topology/uniformity gates.
- Added PyMeshLab 2025.7.post1 to the tracked `lizard_rft` environment.
- Passed the complete 6 s RFT regression: 0.2177 m forward, -0.1208 m lateral,
  86.629 N peak total force, 0.600 N peak site force, 98.9% active steps, and
  zero positive-power steps.
- Tracked the accepted build manifest under
  `docs/regressions/2026-07-27-rft-mesh-rebuild/`.

## 2026-07-27 — tracked RFT baseline

### Physics and simulation

- Moved the RFT surface to z = 0 and the rigid emergency floor to z = -0.25 m.
- Applied the negative of the upstream body-on-sand RFT force to the robot.
- Replaced Euler-order-dependent orientation with MuJoCo `data.xmat`.
- Applied external forces before integration and cleared inactive smoothed
  triangle forces.
- Required exact per-body force-site and mesh-triangle correspondence.
- Split signed foot depth from nonnegative physical sinkage.
- Added force, power, peak force, and triangle-activity diagnostics.

### Visualization and models

- Assigned all generated force sites to hidden visual group 5.
- Added `--show-force-sites BODY` and a larger offscreen scene capacity.
- Restored the original CAD center after mesh simplification.
- Fixed project-root resolution in the site insertion/rebuild tools.
- Normalized active RFT mesh filename case to match MJCF and scripts on
  case-sensitive systems.

### Original rigid-floor compatibility

- Corrected speed metrics when a run ends before gait settle time.
- Unwrapped yaw for heading drift.
- Closed the renderer explicitly and used collision-resistant timestamps.
- Preserved YAML leg order and rejected duplicate/overlapping leg configs.
- Added positive-frequency and valid-duty-cycle checks.
- Committed these original-repository changes as
  `6c848e90a0658fe8713f0dbca7876ed0465ac573`.

### Tests and reproducibility

- Added three RFT core unit tests.
- Added `environment.yml` for the shared `lizard_rft` environment.
- Added `scripts/validate_project.py`.
- Preserved the canonical 6 s summary/config and raw SHA-256 under
  `docs/regressions/2026-07-27-rft-6s/`.

### Structure and provenance

- Classified files before cleanup in `CLEANUP_MANIFEST_2026-07-27.md`.
- Created RFT workspace baseline commit
  `02960805b35f1acc091b03b3ffefca278410f960`.
- Moved maintenance scripts under `scripts/`.
- Moved the upstream RFT-SiM checkout to `third_party/RFT-SiM` and pinned
  commit `303283fae075cae4101ee3af102a36a4a5775998` as a Git submodule.
- Removed one exact nested duplicate of that same clean upstream clone.
- Moved historical downloads and pre-cleanup artifacts to workspace
  `archive/`; no historical project was discarded.
- Made generated `outputs/` local-only and added status, decision,
  provenance, and validation handoff documents.
