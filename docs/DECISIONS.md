# Decision log

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
