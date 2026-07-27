# Validation record

Last run: 2026-07-27

## Environment

- Conda environment: `lizard_rft`
- Python: 3.11.15
- MuJoCo: 3.9.0
- NumPy: 1.26.4
- Open3D: 0.19.0
- PyMeshLab: 2025.7.post1
- Optuna: 4.9.0

The environment is reproducible from `environment.yml`. `isaaclab_50` was not
activated, modified, renamed, or used.

Fresh-system entry points:

- Windows: `scripts/setup.ps1`;
- Linux/macOS/WSL: `scripts/setup.sh`;
- manual: `conda env create --file environment.yml`.

All top-level project dependencies are version-pinned. Setup scripts use only
repository-relative paths and run the canonical validator after installation.

## Automated checks

Commands:

```powershell
python scripts/validate_project.py
python scripts/validate_project.py --full
```

Coverage:

- audits every active STL for connectivity, manifoldness, orientation,
  signed volume, duplicate/degenerate faces, exact self-intersections,
  triangle-area distribution, slivers, and RFT-centroid spacing;
- derives the expected force-site count from live per-body triangle counts;
- requires every sequential force-site name and visual group 5;
- loads `Lizard_Sand.xml` and checks sand/floor separation;
- runs all sixteen fast unit tests;
- runs the original rigid-floor smoke test;
- runs a short or full granular RFT integration.

The three physics-core tests verify:

1. a plate above sand receives zero RFT force;
2. the applied reaction opposes downward intrusion and dissipates power;
3. direct rotation matrices match the compatibility Euler path.

The eight reporting tests verify scenario/view expansion, component ancestor
mapping, rigid-contact aggregation, COM calculation, timeline drawing, contact
event extraction, analyzer metrics, and the 3×3-plus-three-panel overview
layout.

The two rejected-mesh video tests verify that all selected inputs are portable
repository paths and that synchronized centroid projection produces finite,
in-frame coordinates.

The three historical-video recovery tests verify Annex-B header parsing,
truncated `mdat` handling, exact complete-NAL retention, and the tracked
source's expected 111-unit/9,400-byte-tail structure.

## Locomotion video-matrix smoke

Clean source commit:
`ef27d2bfb82f2752ef341810aaf6300d9a094b0e`

Commands:

```powershell
python scripts/generate_video_matrix.py --duration 0.25 --fps 10 `
  --width 640 --height 480 --panel-width 420 `
  --output-dir outputs\video_matrix\release_smoke_ef27d2b
python scripts/analyze_video_matrix.py `
  outputs\video_matrix\release_smoke_ef27d2b
```

Observed:

- exactly nine MP4 files generated;
- all recorded artifact sizes and SHA-256 values verified;
- all three camera views generated for each of the three presentations;
- site-hidden and site-visible RFT outputs used the same state replay;
- COM, displacement, component contact, penetration, and active-triangle
  overlays were visually inspected in selected frames;
- static contact diagrams render continuous 0/1 blocks rather than sparse
  sampling lines.

Tracked evidence:
`docs/regressions/2026-07-27-video-matrix-smoke/`.

The 0.25 s run is intentionally a generator/reporting smoke test. It covers
initial settling only and must not be interpreted as locomotion performance.

## Production 6 s master overview

Source commit: `ff1dda2f954dc27e7d01499f5200497ad30c91f7`.

The production run generated nine individual videos and one combined overview.
The master is 181 frames at 30 FPS (6.033 s), 1680×858, and uses three scenario
rows × `Top | Side | 45° | Analysis`. The generator recorded `git.dirty=false`;
the analyzer verified all manifest artifacts and completed metric extraction.

Tracked evidence and exact hashes:
`docs/regressions/2026-07-27-video-matrix-production-6s/`.

The same ten production MP4 files are copied byte-for-byte to
`docs/media/video_matrix_production_6s/` for direct remote viewing. Their
individual hashes are recorded in that directory's `README.md`; the animated
GIF is a derived lightweight preview.

## Rejected-mesh diagnostic production

Source commit:
`5c90d9118fc2211f5989dc99f0b63c99ea4a522b`; generator recorded
`git.dirty=false`.

Command:

```powershell
python scripts\generate_failed_mesh_videos.py `
  --duration 4 --fps 24 --width 1280 --height 720 `
  --output-dir outputs\failed_mesh_videos\production_4s_5c90d91
```

Observed:

- three MP4 files;
- 97 frames per file at 24 FPS, 4.042 encoded seconds, 1280×720;
- H.264/yuv420p encoding;
- all first and midpoint frames decoded at the expected shape;
- exact source paths, full audits, artifact sizes, and hashes recorded;
- all five generated media artifacts match the ignored output byte-for-byte;
- the tracked JSON manifest is content-identical apart from enforced LF line
  endings.

Representative audit deltas:

| Case | Metric | Rejected/source-only | Accepted |
| --- | --- | ---: | ---: |
| raw CAD Back | components | 13 | 1 |
| raw CAD Back | self-X pairs | 8,456 | 0 |
| raw CAD Back | centroid-spacing P95/P05 | 39.12 | 4.32 |
| legacy clustered Back | self-X pairs | 1,055 | 0 |
| legacy clustered Back | centroid-spacing P95/P05 | 9.68 | 4.32 |
| fixed-count Fusion FR | self-X pairs | 6 | 0 |
| fixed-count Fusion FR | area P95/P05 | 112.78 | 33.28 |
| fixed-count Fusion FR | sliver fraction | 12.33% | 0.62% |

The videos were visually inspected through the contact sheet and animated
preview. They are geometry diagnostics only; no rejected mesh was simulated.
Tracked production copy:
`docs/media/failed_mesh_diagnostics/`.

## Historical invalid-RFT locomotion recovery

Source implementation commit:
`e238a80b0fa54d1021cd9ff29af42d15abb9000d`; generator recorded
`git.dirty=false`.

Command:

```powershell
python scripts\recover_legacy_rft_video.py `
  --output-dir outputs\legacy_rft_locomotion\production_e238a80
```

Observed:

- original archived source: 1,499,129 bytes, SHA-256
  `627F98081BBD6B988F1DF30973C9845E314C604D46D2CCBDC8DB0DBF8B3E54F1`;
- matching tracked SPS/PPS: 39 bytes, SHA-256
  `71612059CF519E13130C1F1D717AAB90BAC3B63597380F456430AFFCB961F8F4`;
- 111 complete H.264 NAL units retained and one 9,400-byte incomplete tail
  discarded;
- both MP4 files decode exactly 110 frames at 30 FPS, 3.667 seconds,
  1280×720, H.264/yuv420p;
- recovered MP4: 3,193,980 bytes, SHA-256
  `A2CE3648E185D14F02C07EBA9683F8DCAB300B800080B7CC6C3B827C1FDD8682`;
- zoomed MP4: 2,652,376 bytes, SHA-256
  `C380F225DF0B8C746782C45B64198B448E11100D5424070265982A9B45868E2F`;
- contact sheet: 474,779 bytes, SHA-256
  `CCDC29FD6684422B0795ACA9F479D8C23AEFC124F86D2308831C6AD7BF6B2AEB`;
- recovered and zoomed frame counts match;
- the six selected frames were visually inspected and clearly show clustered
  and sparse force-site regions over the moving whole robot;
- manifest records `resimulation=false`.

The zoom changes only camera presentation and adds an invalid-evidence label.
No MuJoCo/RFT state was regenerated. Tracked reviewed copy:
`docs/media/legacy_incorrect_rft_locomotion/`.

Post-change reporting compatibility smoke:

```powershell
python scripts\generate_video_matrix.py `
  --duration 0.25 --fps 10 --width 640 --height 480 --panel-width 420 `
  --output-dir outputs\video_matrix\legacy_media_reporting_smoke_e238a80
python scripts\analyze_video_matrix.py `
  outputs\video_matrix\legacy_media_reporting_smoke_e238a80
```

It generated all nine views plus the master overview; the analyzer verified
the manifest artifacts and completed metric extraction. This local smoke was
run while the curated-media documentation was being assembled, so it is
compatibility validation rather than a clean canonical evidence release.

## Mesh root-cause audit

Original assembly-export STLs:

- total: 96,208 triangles;
- only `Front.STL` was watertight;
- the other seven bodies each had 13 connected components, 9 non-manifold
  edges, and approximately 7,618–8,456 self-intersecting triangle pairs;
- area P95/P05 reached 1,185.

Former active Open3D-remeshed STLs:

- 8/8 were non-watertight;
- 2,731 non-manifold edges in total;
- 12–936 self-intersecting triangle pairs per body;
- area P95/P05: 74–293;
- approximately 18–20% of triangles had minimum angle below 10 degrees;
- several bodies had coincident triangle centroids.

Accepted active STLs:

- 8/8 are one connected component, watertight, manifold, orientable, and have
  positive signed volume;
- zero boundary, non-manifold, inconsistent-winding, duplicate, degenerate,
  or self-intersecting faces;
- area P95/P05: 7.49–33.28;
- centroid-neighbor P95/P05: 2.86–7.78;
- sliver fraction: 0.11–1.10%;
- candidate-to-Fusion-source P95 distance: 0.390–0.586 mm.

Exact metrics and hashes:
`docs/regressions/2026-07-27-rft-mesh-rebuild/build_manifest.json`.

## Current 13,916-site granular regression

- Local raw path:
  `outputs/sand/run_07_27_2026_161321_700369/results.npz`
- Raw SHA-256:
  `A3628E6536A1CE61582B97CDED9C1EC5EBF890519B48310B9F957D41F174DF70`
- Exact config:
  `docs/regressions/2026-07-27-rft-mesh-rebuild/config.yaml`
- Summary:
  `docs/regressions/2026-07-27-rft-mesh-rebuild/summary.json`

Observed results:

- duration: 6.0 s;
- forward displacement: 0.2177366 m;
- lateral displacement: -0.1208181 m;
- maximum foot sinkage: 5.46–6.66 mm;
- peak total RFT force: 86.6290 N;
- peak individual-triangle force: 0.5997 N;
- RFT-active steps: 98.9167%;
- RFT power range: -37.0232 to 0 W;
- positive RFT-power steps: 0%;
- all 19 saved arrays are finite;
- base z remained 0.01877–0.04000 m; the -0.25 m safety floor was not reached.

Locally measured MuJoCo XML load time:

- accepted 13,916-site model: 21.52 s;
- rejected 30,020-site uniform-fine prototype: 99.53 s.

## Original rigid-floor regression

- 0.2 s pre-settle run: 0 mm/s, not the former divide-by-near-zero result.
- 3.0 s run: 0.0596 m forward displacement; no fall.

Original-repository fix commit:
`6c848e90a0658fe8713f0dbca7876ed0465ac573`.

## Historical pre-rebuild granular baseline

The former 12,097-site run is preserved for history:

- local raw path: `outputs/sand/run_07_27_2026_151216/results.npz`;
- tracked evidence: `docs/regressions/2026-07-27-rft-6s/`;
- raw SHA-256:
  `ECE98E6C711EF23B5FC8090416E14E6E7A2A72D8A1AD36D8DC35DD7D3E80CAE6`.

It is not the current numerical acceptance target because the contact mesh
changed materially.

All granular results are numerical regressions, not claims of experimentally
calibrated force accuracy.
