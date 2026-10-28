# Verified per-robot motion / remesh / contact-proxy comparison

Rendering source: clean `230cda2`. Publication helper: clean `d3db057`.
Native simulation sources remain the original `36659ca` batch and `5078fdc`
G1 fast failure, individually identified in the media index. Publication does
not relabel those simulation commits or overwrite their artifacts.

## Results

- Seven robots, eight rigid-above-sand motion pairs including G1 fast control.
- Seven rotating original/remeshed mesh and triangle-centroid reviews.
- Nineteen **independent named-component proxy enlargement PNGs**.
- All 15 movies decode completely: 1,423 frames total; all media hashes pass.
- Eight motion movies: 1280x944, 121 frames at 20 FPS; 6.05 s encoded including
  the endpoint, with actual source row times shown. An early sand stop leaves
  a no-data scene while the available rigid recording continues.
- Seven geometry movies: 1600x960, 65 frames at 16 FPS, 4.0625 s encoded.
- Sixteen motion input state files verified, none unavailable; original batch
  and qualified-bundle manifests reverified after generation. URI display
  replays preserve source NPZ bytes and execute no physics.
- 81/81 tests and canonical fast validation passed. The prescribed original
  nine-view plus overview video smoke and artifact analyzer also passed.

The primary outputs, CSVs, exact settings, snapshot hashes and original model
notices are in [the media gallery](../../media/robot_comparisons/README.md).
`verification.json`, `source_index.json` and LF-normalized logs provide the
compact audit; raw work stays in `outputs/robot_comparisons/`.

## Scientific interpretation

Source means the **selected contact proxy's original tessellation**, not raw
whole-body CAD. Remeshed geometry is the already-qualified saved surface.
The rest of the robot, its mass/inertia, native collision geometry, controller,
RFT force law and numerical environment remain unchanged. Matched timestep
does not imply identical native/RFT contact discretization (e.g. T1 native
corner spheres versus its declared sole-box RFT proxy).

Each proxy has shared bounds/camera and separate mesh/centroid panels. Colors
use local 3D nearest-neighbor spacing divided by that mesh's own median;
absolute median millimetres and actual counts are separate. Rear points are
faded but retained. Arrows are sampled outward normals, not force vectors.
The grey whole-model pose is location context; gold shows all selected RFT
points. Coarser, denser or more uniform sampling is not automatically more
accurate. Go1 spacing ratio improves while its area ratio increases; wheel
and caster spacing ratios can increase despite meeting existing gates.
These tradeoffs are reported, not tuned away or labelled universal gains.

Motion panels use mass-weighted COM displacement and conservative OR within
each contact time-pixel bin. Rigid native support overlap and sand centroid
penetration are distinct labelled quantities. Gray timeline regions mean no
recorded data. Raw continuous telemetry is unchanged and remains the source
for quantitative analysis. Geometry qualification is not sand-gait acceptance.

## Reproduce

Run from the project root in `lizard_rft`, with the verified catalog batch
already generated using the catalog guide:

```sh
conda run -n lizard_rft python scripts/generate_robot_comparisons.py --batch outputs/robot_catalog/clean_6s_36659ca --fast-run outputs/ground_rft/g1_clean_6s_5078fdc --output-dir outputs/robot_comparisons/NEW_RUN
conda run -n lizard_rft python scripts/generate_robot_comparisons.py --verify outputs/robot_comparisons/NEW_RUN
```

Omit `--fast-run` if that optional saved negative-control run is absent. It
adds a variant, not a robot. Generator never calls stepping or remeshing.
Mandatory legacy validation is separate and does run its existing short
0.2/0.25 s baseline smokes, not replacement comparison trajectories.

```sh
conda run -n lizard_rft python scripts/validate_project.py
conda run -n lizard_rft python scripts/generate_video_matrix.py --duration 0.25 --fps 10 --width 640 --height 480 --panel-width 420 --output-dir outputs/robot_comparisons/NEW_SMOKE
conda run -n lizard_rft python scripts/analyze_video_matrix.py outputs/robot_comparisons/NEW_SMOKE
```

Publication requires review and clean source, then the generator's `--publish`
mode. See D-022 and `docs/ROBOT_COMPARISONS_2026-10-28.md` for the complete
stepwise ledger. Windows native rendering was tested; no new environment,
material calibration, gait optimization or cross-platform runtime claim.
