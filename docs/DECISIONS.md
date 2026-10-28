# Decision log

## D-019 — bounded ground-pipeline success and failure media

Date: 2026-10-28. Extend D-006 with `docs/media/ground_rft_pipeline/`:
five MP4s (matched rigid/sand Go1 and G1 slow, plus G1 fast sand failure),
three preview sheets and original model/policy notices. Runtime `f4c0360`
is clean; retained fast failure is clean `5078fdc`. Verify raw manifests,
decode all frames, copy videos without transcoding and record hashes.
Caption-band previews do not change physics/raw media.

Reason: show actual walking integration, including failure missed by short
screening. Do not erase failure, relax thresholds, pad stopped trajectories,
or call commanded speed actual speed. Preserve all prior lizard/interface
media. Raw arrays remain ignored; compact provenance and analysis are tracked.

## D-018 — qualify prepared surfaces before ground-locomotion simulation

Date: 2026-10-28.

The next generic path is a staged, fail-closed import/selection/remesh/audit/
bundle/binding/controller/simulation pipeline. It does not auto-infer missing
dynamics or an exterior union from arbitrary CAD. Geometry-only files stop at
preparation until a physical body owner/model is supplied. Persistent bundles
must verify their artifacts and model/body mapping before runtime use. Source
visuals, collision shapes, masses and inertias are not remeshed in place.

Ground-locomotion evidence requires an independently working controller on
rigid ground, explicit control parameters/cadence, and actual sand support
telemetry. Rigid-trained policy transfer may fail and must remain visible.
The old Panda demo is interface smoke, not a locomotion validation. Preserve
it as historical evidence without presenting it as a ground robot baseline.
See `GROUND_RFT_PIPELINE_2026-10-28.md` for stages, budgets and stopping rules.

Reviewed explicit support pairs are removed using native MjSpec in a private
sand-model copy, with unchanged physical identity and self-contact pairs.
Mask-only conversion is insufficient for G1. No extreme negative margins or
global collision disabling may hide residual support.

## D-017 — publish two bounded generic integration demo videos

Date: 2026-10-27. Extends the bounded-media policy in D-006.

Track exactly two short Go1/Panda integration MP4s, their preview sheets and
upstream notices under `docs/media/general_rft_stage1/`. They were generated
from clean source `3be21df`, verified by artifact hashes/frame decoding and
visually inspected. Copy without transcoding; preserve all raw NPZ/state
data and original manifests under ignored `outputs/general_rft/`.

Reason: readers need to inspect actual cross-robot binding/force integration
before downloading optional model assets. Label these as selected-proxy,
hold/sine, two-second integration screens, not optimized locomotion, approved
hardware envelopes or material calibration. Retain the original lizard
production and legacy-invalid media unchanged. Exact hashes and sources are
in the media README and the dated regression evidence.

## D-016 — versioned generic bindings coexist with the frozen lizard

Date: 2026-10-27.

Add the independent `rft/` package and explicit robot surface recipes without
editing the legacy lizard runner, force library, models, accepted geometry or
gait. New bindings may use body-local proxy triangles or selected compiled
geom triangles/primitives; force sites are optional in this new path. The
legacy site count/group invariants and D-007 envelope gates remain unchanged
for the existing lizard. The new path permits declared multiple surfaces per
body, and never silently discards smaller physical components.

`legacy_lizard_v1` deliberately preserves the original velocity reference,
absolute-z depth factor, non-finite sanitization and alpha smoothing for
compatibility. `body_frame_v1` uses regular body-frame origin velocity,
relative plane depth and explicit invalid-force errors. A passing legacy
comparison does not adopt the corrected convention for the lizard. Record
corrected-convention differences separately.

Providers return private generalized-force buffers; they do not step, control
the robot, or clear other external forces. Per-body wrench aggregation must
match per-triangle generalized forces and is not combined with that path.

Selected open-source model demos may use clearly labelled collision proxies
and independent hold/sine controllers to screen import, binding, RFT and
rendering. Such screening is not an externally calibrated granular-contact
model or a newly validated walking gait. Preserve upstream model licenses and
pin all downloaded blobs; keep optional assets ignored and reproducible.

See `RFT_GENERALIZATION_PLAN.md` and the dated implementation worklog for
scope, exact conventions, tests, outputs and staged adoption criteria.

## D-015 — bundle the immutable RFT-SiM reference for public deployment

Date: 2026-10-08. Supersedes the submodule distribution choice in D-005.

Replace the `third_party/RFT-SiM` gitlink with the complete original tree at
`303283fae075cae4101ee3af102a36a4a5775998`. Preserve every upstream blob,
including the MIT License in its README, assets and two original example
MP4 files. The MP4 files are a bounded provenance exception to D-006; they
are not new experimental output or current lizard validation media.

Record commit, tree, original blob IDs, sizes and SHA-256 in
`third_party/RFT-SiM.snapshot.json`. Disable line-ending conversion for the
snapshot and verify its hashes during setup and canonical validation. Keep
first-party RFT integration outside the immutable snapshot.

Reason: the upstream URL currently returns `Repository not found`; ordinary
ZIP downloads never contained the old submodule contents, and recursive
clones/setup fail when the upstream is inaccessible. A locally preserved,
clean copy matches the pinned commit and passes Git object validation. The
upstream README grants MIT redistribution with the copyright/license notice
retained. Bundling this fixed tree makes public clones and ZIP downloads
self-contained without adding upstream Git history. No physics, mesh,
controller, environment pins or force conventions change.

Scope of evidence: the original crab examples are preserved, not newly
validated in `lizard_rft`. The verification record distinguishes fresh
GitHub source deployment in the existing environment from a new-environment
installation. See `docs/DEPLOYMENT_FIX_2026-10-08.md`.

## D-014 — invalid historical locomotion is visual evidence only

Date: 2026-07-27

Preserve the interrupted historical `legacy_sand.mp4` recording, its matching
H.264 SPS/PPS headers, a valid-container recovery, and an explicitly labelled
zoomed presentation. Recovery may decode complete historical frames and
discard an incomplete trailing frame. It must not rerun the rejected mesh,
infer missing motion, or attach force/performance claims to the video.

Reason: the recording uniquely shows the full moving robot with the former
uneven RFT force-site pattern. It is valuable root-cause evidence, but the
underlying non-watertight/self-intersecting mesh and unknown historical
integration state make it invalid as a physics or locomotion baseline.

## D-013 — master video is 3×3 views plus three row panels

Date: 2026-07-27

The master video contains three scenario rows. Its first three columns are
cropped pure Top, Side, and 45-degree camera frames. Its fourth column contains
one analysis dashboard per scenario, taken from the synchronized side-view
video. The compositor reads the nine generated MP4 files and never resimulates.

Reason: retaining the dashboard inside every 3×3 cell would repeat nine small
panels and make the robot views unreadable. One panel per row preserves all
requested COM/contact/penetration information while keeping comparisons clear.

## D-012 — reporting compares identical physical replays

Date: 2026-07-27

Simulate the rigid scenario once and the RFT scenario once. Render all camera
views from saved states. Render RFT sites hidden and visible from the same
saved RFT state sequence.

Reason: camera choice and diagnostic-site visibility are presentation
variables. Re-simulating each view would introduce avoidable numerical or
configuration differences and weaken visual comparisons.

## D-011 — component contact is a binary time series

Date: 2026-07-27

Report contact for the named whole components `Mid`, `Front`, `FR`, `FL`,
`Back`, `HR`, `HL`, and `Tail`. On rigid ground, a component is active if any
of its descendant geoms contacts the floor. In RFT sand, a component is active
if any of its triangles is active in the RFT calculation. Preserve raw counts
alongside the binary value.

Reason: the requested 0/1 diagram must show start/end timing at a stable,
human-readable component level. Counts retain diagnostic detail without
changing the meaning of the binary chart.

## D-010 — COM is mass-weighted across moving robot bodies

Date: 2026-07-27

Compute the displayed center of mass from MuJoCo body inertial positions
`data.xipos[1:]`, weighted by `model.body_mass[1:]`; exclude the world body.
Store the numeric trajectory in the run analysis arrays.

Reason: an average of geom or site positions would depend on mesh
discretization and would not be the robot's physical COM.

## D-001 — one MuJoCo environment, IsaacLab isolated

Date: 2026-07-27

Use `lizard_rft` for both the rigid-floor and RFT MuJoCo projects. It pins
MuJoCo 3.9.0, NumPy 1.26.4, and Open3D 0.19.0. `isaaclab_50` remains isolated
and is never modified by this project.

Reason: the RFT stack is a superset of the original MuJoCo runtime, while
IsaacLab has a more fragile and unrelated dependency graph.

## D-002 — sand surface and emergency floor are separate

Date: 2026-07-27

The RFT `sand_height` and non-colliding sand visualization are at z = 0. The
rigid contact floor is at z = -0.25 m and serves only as a numerical safety
catch.

Reason: a rigid floor at the sand surface prevented penetration and therefore
prevented the RFT model from becoming physically active.

## D-003 — explicit reaction-force sign

Date: 2026-07-27

`rft_3D_body_full_mat` retains the upstream body-on-sand convention.
`lizard_sand.py` negates that result before applying it to MuJoCo.

Reason: this preserves comparability with the upstream equations while making
the equal-and-opposite sand-on-body force explicit at the integration boundary.

## D-004 — MuJoCo rotation matrix is authoritative

Date: 2026-07-27

Runtime RFT transformations use `data.xmat`, the world-from-body matrix.
Euler angles remain only as a tested compatibility path.

Reason: the former roll/pitch/yaw versus yaw/pitch/roll mismatch corrupted
triangle positions, normals, and force directions.

## D-005 — upstream RFT-SiM is a pinned dependency

Date: 2026-07-27

Historical decision: the submodule distribution was superseded by D-015 on
2026-10-08. The upstream version and first-party integration boundary remain.

Keep upstream RFT-SiM as a Git submodule under `third_party/`, pinned to
`303283fae075cae4101ee3af102a36a4a5775998`. Keep local integration code in
this repository rather than modifying the upstream checkout.

Reason: a submodule records exact provenance without vendoring duplicate Git
history or blurring upstream and local behavior.

## D-006 — raw runs stay out of Git; curated evidence releases are bounded

Date: 2026-07-27

Keep new videos, optimizer databases, NPZ arrays, CSV tables, and raw runs
under ignored `outputs/`. Track compact regression summaries/configurations
and SHA-256 hashes under `docs/regressions/`.

The deliberate exceptions are explicitly reviewed, bounded evidence sets
under `docs/media/`:

- `video_matrix_production_6s/`: ten canonical locomotion MP4 files plus a
  preview, representative frame, and hashes;
- `failed_mesh_diagnostics/`: three geometry-only rejected/source comparison
  MP4 files plus a preview, contact sheet, exact audit manifest, and hashes.
- `legacy_incorrect_rft_locomotion/`: one recovered historical recording, one
  labelled zoomed copy, a contact sheet, and exact recovery/hashes. It is
  failure evidence only and carries no valid force or gait claim.
- `general_rft_stage1/`: two explicitly labelled, hash-verified cross-robot
  integration demos with previews and upstream notices (D-017); not a physics
  calibration or new gait-performance baseline.
- `ground_rft_pipeline/`: five ground-controller/RFT success/failure videos,
  three previews and notices (D-019); not calibrated physics or precise
  velocity-tracking certification.

Adding or replacing a set requires a clean source commit, verified generator
manifest, tests, documentation, and an explicit decision. Rejected meshes are
never promoted into physics merely to produce a video.

Reason: raw experiments grow without bound and obscure source changes, but a
new user must be able to see the canonical behavior immediately after cloning.
Small, purpose-specific releases balance direct inspection with repository
size and traceability while raw experiments remain reproducible and ignored.

## D-007 — RFT contact geometry is one external envelope

Date: 2026-07-27

Do not triangulate or decimate the raw assembly-export STLs directly for RFT.
Use the tracked, manually unified Fusion external envelope as reconstruction
input. The active RFT surface for each body must be one closed component with
positive signed volume, consistent outward winding, zero boundary or
non-manifold edges, and zero self-intersecting triangle pairs.

Reason: seven raw CAD exports contain 13 overlapping subassemblies and
thousands of intersections, including internal hardware faces that are not
physical sand-contact surfaces. Decimation preserves or merges those errors;
it cannot infer the intended exterior. The Fusion source is materially safer,
but authoritative CAD review is still required to confirm that its chosen
envelope is physically correct.

## D-008 — candidate build and active promotion are separate

Date: 2026-07-27

`scripts/build_rft_mesh_candidates.py` may write only under ignored
`outputs/`. It uses the tracked recipe, deterministic sampling, single-thread
Screened Poisson reconstruction, isotropic remeshing, and hard geometry gates.
Only `scripts/promote_rft_mesh_candidate.py` may promote a completed build
after checking all body results, the recipe hash, and every candidate hash.

Reason: mesh reconstruction involves parameter choices and can fail on one
body even when the other seven are valid. Separating build from promotion
prevents partial or failed experiments from silently replacing the runtime
model. Single-thread Poisson is required because multi-thread reductions were
observed to produce different hashes and marginal self-intersections.

## D-009 — force-site density balances geometry and XML cost

Date: 2026-07-27

Use the per-body edge lengths in `configs/rft_mesh_recipe.json`, not a single
triangle-count target. The accepted model has 13,916 force sites. Topology and
zero self-intersection are absolute gates; distribution thresholds are
bounded engineering criteria recorded in the recipe.

Reason: a uniform 3.75 mm prototype produced exceptionally even 30,020-point
surfaces but made MuJoCo XML compilation take 99.53 s locally. The accepted
per-body recipe keeps difficult geometry finer and simpler bodies coarser,
loads in 21.52 s, and remains close to the former 12,097-site runtime size.
Triangle area weighting makes unequal per-body triangle counts physically
valid; identical triangle counts are an upstream implementation limitation,
not a requirement of this vectorized integration.
