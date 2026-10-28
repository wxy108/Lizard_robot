# Ground pipeline evidence — 2026-10-28

## Outcomes, not just inventory

| Check | Observed result |
| --- | --- |
| Unit tests at clean final runtime `f4c0360` | **49/49 passed** |
| Official Go1/G1 obs/action parity | 20 articulated states each, exactly zero differences |
| Original lizard full regression at clean `5078fdc` | Unchanged 217.7 mm forward, -120.8 mm lateral, 86.629/0.600 N peaks, no positive active power |
| Go1 command 0.4 m/s, six-second sand | Completed, dx 2.104190 m, dy +0.322239 m, depth 31.002 mm |
| G1 command 0.2 m/s, six-second sand | Completed, dx 1.860660 m, dy +0.002779 m, depth 30.311 mm |
| G1 command 0.4 m/s sand | **Failed at 3.765 s**, height/upright guard, not a numerical warning |
| Supporting rigid contact/force in all sand runs | Exactly zero |
| Go1 repeated before/after guards | All saved state/control/force/trace/activity arrays exactly equal |
| Three-pose binding | Position error 0; velocity error <=1.11e-16 m/s |
| Prepared geometry | Go1 1,248 / G1 972 triangles, closed outward shells, no detected self-intersections |
| Environment delta | Existing versions unchanged; only ORT 1.31.1, flatbuffers 25.12.19, protobuf 6.33.0 added |

Positive cases are **bounded runtime screens**, not calibrated sand or precise
tracking. Mean sand vx: Go1 0.35070 vs 0.4 m/s (-12.3%); G1 0.31011 vs 0.2
(+55.1%). G1 matched rigid already averages 0.29770 (+48.9%); the bias cannot
be solely attributed to RFT. Whole-window means include initial transient.
Feet-only proxy selection is not whole-CAD certification. Higher-command G1
remains a required negative example. No further search followed the single
planned lower-command diagnostic.

## Files / provenance layers

- `review.json`: raw artifact hashes/locations, clean simulation commits,
  resolved settings, model/source hashes, effective recipes/gates/metrics,
  native binding checks, nine video decode counts, tracking bias, contact
  duties, repeatability and environment delta. Reviewer was authored afterward:
  its own dirty flag is not a dirty simulation; its file SHA-256 is recorded.
- `controller_parity.json`: clean final source parity and package inventory.
- `verification_5078fdc.log`: original full lizard regression, 47 tests,
  Go1 and G1 0.4 m/s runs including the failure.
- `verification_f4c0360.log`: final 49 tests, parity, Go1 repeat and G1 slow.
- Three `*_diagnostics.png`: COM/root/depth, RFT vs rigid force, contact 0/1.
- `source_index.json`: source/config/test hashes and commits.
- `cli_checks.json`: intentional geometry-only/unsupported-file stops and
  verified prepared-bundle reuse; exit 2 is an expected safe-stop signal.
- Reviewed [videos, previews, original notices](../../media/ground_rft_pipeline/README.md).

## Exact raw runs and extraction

```text
outputs/ground_rft/go1_clean_6s_f4c0360/
outputs/ground_rft/g1_slow_clean_6s_f4c0360/
outputs/ground_rft/g1_clean_6s_5078fdc/          retained failure
outputs/ground_rft/go1_clean_6s_5078fdc/         guard-change control
outputs/ground_rft/review_clean_release_20261028_v2/
outputs/general_rft/ground_install_20261028_000238_266964/
outputs/general_rft/walking_controller_parity_20261028_003037_042310/
```

These identify local raw output, not additional deployment dependencies.
Fresh machines regenerate using the guide; codec/platform may change encoded
bytes. Curated MP4 bytes are directly included in the repo.

Each run retains bundle, import inventory, resolved config, pose checks and
three scenario folders. Scenario `summary.json` names `trace` columns; read
`trajectory.npz` with `numpy.load(..., allow_pickle=False)`. State/control/
force/activity rows share each pre-step time. Final qpos/qvel are separate.
`groups` names component columns. Independent review verifies hashes before
extracting data/decoding video. Run `scripts/review_ground_pipeline.py --runs
... --output-dir NEW_DIRECTORY`; compare/environment flags are in the
[guide](../../GROUND_RFT_GUIDE.md).

## Reasons and outstanding work

[Full staged/failure ledger](../../GROUND_RFT_PIPELINE_2026-10-28.md): reject
ambiguous CAD before remesh; independently preserve the original deviation
bound; cache quadrature outside dynamics; check ownership in articulated
poses; preserve controller conventions; detect fixed/mocap supports and pair
bypasses; stop falls; separate timestep, medium and command.

Next: review exposed whole-CAD surfaces, real URDF/non-legged coverage,
mesh/step sensitivity, G1 force variation/tracking, physical material/force
calibration. These are not counted as completed by this engineering release.
