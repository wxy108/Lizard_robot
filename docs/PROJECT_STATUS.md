# Project status

Last verified: 2026-07-27

## Git anchors

- Clean GitHub integration commit:
  `ed5edc6`
- Original rigid-floor compatibility fixes:
  `6c848e90a0658fe8713f0dbca7876ed0465ac573`
- Upstream RFT-SiM dependency:
  `303283fae075cae4101ee3af102a36a4a5775998`

The nested local RFT workspace retains its own source-history anchors:
baseline `0296080`, mesh rebuild `bddb2f7`, video implementation `ef27d2b`,
and imported documented source snapshot `48952e6`. They are provenance anchors,
not ancestors of the clean GitHub branch, because the local project originally
lived one directory below a separately initialized workspace repository.

Use `git log --oneline --decorate -10` at the repository root for publication
history.

## Current baseline

The active granular simulation runs in the single Conda environment
`lizard_rft`, which also runs the original rigid-floor MuJoCo model. The
`isaaclab_50` environment is independent and untouched.

The current RFT integration is topology-gated and regression-tested:

- reaction-force sign, orientation, force timing, stale smoothing, and
  site/triangle validation are corrected;
- the rigid floor no longer masks the RFT sand region;
- force sites are hidden by default and selectively visualizable;
- headless logs include force, power, sinkage, and triangle diagnostics;
- active RFT surfaces are reconstructed from a manually unified Fusion
  external envelope and pass automated topology/self-intersection gates;
- the candidate build is deterministic, hash-checked, and separate from the
  explicit promotion step;
- 13,916 force sites are generated from 13,916 active triangles;
- original short-run speed and yaw metrics are corrected;
- controller leg order and configuration validation are deterministic;
- the workspace, upstream dependency, changes, and regression evidence are
  under Git tracking.

The reporting workflow is now reproducible:

- a single command records the rigid and RFT simulations once and renders top,
  side, and 45-degree views;
- the site-hidden and site-visible sand videos replay identical states;
- all nine videos include COM trajectory/displacement and eight named
  component contact timelines;
- sand videos also report maximum penetration and active RFT triangles;
- CSV, NPZ, diagrams, a resolved gait config, and a SHA-256 artifact manifest
  are written with every run;
- a separate analyzer verifies every manifest hash before deriving
  contact-event, duty-cycle, penetration, and COM-path metrics.

## Verified measurements

Current 13,916-site 6 s RFT validation:
`python scripts/validate_project.py --full`

- forward displacement: 0.2177 m
- lateral displacement: -0.1208 m
- maximum foot sinkage: 5.46–6.66 mm
- peak total RFT force: 86.629 N
- peak triangle force: 0.600 N
- RFT-active steps: 98.9%
- positive RFT-power steps: 0%
- active meshes: 8/8 watertight and orientable, 0 self-intersections,
  0 boundary/non-manifold/inconsistent-winding edges
- candidate-to-Fusion-source P95 surface deviation: 0.390–0.586 mm
- MuJoCo XML load time measured locally: 21.52 s (30,020-site prototype:
  99.53 s)

These demonstrate a numerically stable and dissipative integration. They do
not prove physical calibration.

The video-matrix clean smoke run at commit `ef27d2b` generated exactly nine
MP4 files, passed artifact-hash verification, and is represented by compact
tracked evidence under
`docs/regressions/2026-07-27-video-matrix-smoke/`. Its 0.25 s duration covers
only initial settling and must not be used for locomotion-performance claims.

Fast automated tests: 10/10 passing.

The earlier 12,097-site regression remains tracked under
`docs/regressions/2026-07-27-rft-6s/` as a historical pre-mesh-rebuild
baseline; its numerical values are not the current acceptance target.

## Open research work

1. Confirm the manually unified Fusion external envelope, removed internal
   surfaces, and outward-normal convention against the authoritative CAD.
   Automated topology checks are now clean, but only CAD review can establish
   that the selected exterior is the intended physical contact surface.
2. Calibrate `RFTCOEFF=3.75` against the actual granular material and an
   intrusion/force measurement.
3. Investigate the initial 86.6 N impact transient with physical data.
4. Define sand-gait metrics before optimization; the current gait has
   substantial lateral drift over 6 s.
5. Validate the reconstructed hardware duty-cycle function against the
   missing original helper or hardware logs.
6. Generate the full 6 s, 30 fps video matrix after gait/CAD/calibration
   decisions are frozen; the tracked 0.25 s run validates reporting mechanics,
   not gait quality.

## Next safe step

The next engineering step is a side-by-side CAD surface review followed by a
controlled intrusion test. Do not begin a large gait search until the exterior
surface and coefficient are validated, because optimizer rankings would
otherwise be specific to an uncalibrated model.
