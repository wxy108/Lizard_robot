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
- runs all ten fast unit tests;
- runs the original rigid-floor smoke test;
- runs a short or full granular RFT integration.

The three physics-core tests verify:

1. a plate above sand receives zero RFT force;
2. the applied reaction opposes downward intrusion and dissipates power;
3. direct rotation matrices match the compatibility Euler path.

The seven reporting tests verify scenario/view expansion, component ancestor
mapping, rigid-contact aggregation, COM calculation, timeline drawing, contact
event extraction, and analyzer metrics.

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
