# General RFT implementation ledger — 2026-10-27

Current outcome: stage-one implementation, 29 tests, same-state regression,
full original lizard regression, video/reporting smoke, two real-model demos
and the lower-floor diagnosis are complete. Implementation source is `3be21df`;
the final evidence commit changes documentation/media only. Numerical legacy
files remain the `ccc6946` control. Open research/adoption limits are listed
at the end; the initial queue below is historical, not current pending work.

## Identity and scope

- Canonical root: `Lizard_Robot_MuJoCo`.
- Branch: `agent/general-rft-stage1`.
- Preserved baseline: `ccc694677461012d488cb5ad7d7e9db945b74a80`.
- Plan: `RFT_GENERALIZATION_PLAN.md`.
- Runtime: existing `lizard_rft`; no IsaacLab access or changes.
- Work authorized: save plan; implement stage-one generic binding/provider;
  same-state legacy comparison; selected open-source model installation and
  integration screening; diagnose the suspected extra ground in video.

## S0 — inventory and freeze (completed)

Purpose: ensure later differences can be attributed and original behavior
remains recoverable. Clean canonical checkout and public GitHub HEAD both
resolved to `ccc6946`. Created the implementation branch without editing
legacy code or assets.

Frozen working-file SHA-256:

| Path | SHA-256 |
| --- | --- |
| `lizard_sand.py` | `9BED2344A1271057257B2B6C1E34DCB78B77D9D15F1EA4446450F25B04AF6C9F` |
| `sim_fxn_lib.py` | `903EF7AB8763719053E1691B46D7B22D751C40C2A99DBD13483724D83FF52C90` |
| `Lizard_Sand.xml` | `63BD0C702F2E1C43C4C9F52710D1D80DBFFE4675A887CC01CB999A1B5F1B399D` |
| `models/lizard.xml` | `642E600528D448B9A367ABA30EF18806EACCC6AC11263AF8736524B4FB29B8EC` |
| `configs/gait.yaml` | `25054AC1513077C5407C13286BA49E1FC83A4BCAF2137798B3994BF79FB81958` |
| `environment.yml` | `C40A4C29E97A76E73FCA6DE4E88AE40506CDD8FE4FD5A2CFB6C9A62596266F25` |

## S1 — plan saved (completed)

Purpose: turn the agreed architecture into an inspectable scope, staged
acceptance and stopping conditions before implementation. Output:
`docs/RFT_GENERALIZATION_PLAN.md`. It distinguishes compatibility, physical
correctness, external-model screening and the floor audit.

## Infrastructure observation I-001

The first direct Python import in `lizard_rft` failed at MuJoCo `_specs` with
`DLL load failed ... An Application Control policy has blocked this file`.
NumPy 1.26.4 imported successfully. An ordinary `conda run` import is being
checked independently. No system security policy or binary trust metadata was
changed. Runtime tests must not be reported as passed until executed.

Follow-up: the ordinary `conda run -n lizard_rft python ...` import succeeded
with MuJoCo 3.9.0. Runtime checks use that environment launch path. The direct
launch failure is retained as an observation; its precise Windows DLL/trust
resolution cause has not been established. No security settings changed.

## S2 — generic binding/provider and known-answer checks (implemented)

Purpose: extract the reusable boundary while preserving the existing control.
Outputs: `rft/{binding,kernel,provider,records}.py`,
`configs/robots/lizard_rft.yaml`, `tests/test_generic_rft.py`.

The force law is copied without mathematical edits from the frozen module
into a NumPy-only module. Simulator-frame/depth differences are explicit
provider modes, not hidden changes to that law. Bindings are owned by named
bodies and can use multiple surfaces and unequal triangle counts without
sites. Results contain raw/smoothed force arrays, masks, positions, velocities
and a private generalized-force vector.

Command: `conda run --no-capture-output -n lizard_rft python -m unittest
discover -s tests -p test_generic_rft.py -v`.

Initial result: 11/11 passed. Covered offset-inertia point velocity against
the MuJoCo Jacobian, plane translation, unchanged external force/control/state,
release clearing, triangle/wrench generalized-force equivalence, primitive
closed winding, proxy units/translation and fixed-link URDF import.

## S3 — initial matched-state/floor smoke (completed)

Command: `conda run --no-capture-output -n lizard_rft python
scripts/validate_general_rft.py --duration 0.2 --output-dir
outputs/general_rft/stage1_smoke_20261027`.

Output: summary, matched raw/smoothed force arrays and generalized forces,
legacy state trajectory, floor-disabled trajectory, rigid-ground telemetry,
ground comparison plot and hashed manifest. Compatibility passed at declared
float64 tolerances. There were zero sand-run supporting rigid-ground steps;
floor-on/off qpos arrays were identical. This is only an initial-settling
smoke; the 6 s audit is a separate required record.

The physical checkout resolves to `X:/Projects/Lizard_Robot_MuJoCo` through
the documented `C:/Users/wxy22/Documents/Lizard_Robot_MuJoCo` junction. These
are two paths to the same canonical Git checkout, not independent copies.

## S4 — external assets and optional library installation (installed)

Selected Menagerie commit `f3b6013629c7b94f8ee93fd977b161fa8350f9bc`, models
`unitree_go1` and `franka_emika_panda`: 91 files, 48,615,728 bytes. The bounded
installer verified every original Git blob and preserved BSD-3-Clause / Apache
model licenses. Downloaded files stay ignored under `third_party/robot_assets`.
Lock and installer are tracked; effective per-file SHA-256 is in the generated
download manifest. Installation record:
`outputs/general_rft/robot_install_20261027_182907_772686/`.

Installed only optional pure-Python registry dependencies with `pip install
--no-deps -r requirements-robots.txt`: robot_descriptions 3.3.0, GitPython
3.2.1, gitdb 4.0.12 and smmap 5.0.3. Registry import and `pip check` passed.
The registry's automatic unbounded model-cloning path is not used; pinned
local asset configs are the tested integration route. Model runtime and video
screening are separate steps, not implied by package installation.

## Test correction T-001 — compiled-mesh fixture (resolved)

The additional source-versus-compiled mesh transform test initially failed:
224 triangle centroids were returned where the fixture expected four. The
fixture specified a mesh asset on a geom but omitted `type="mesh"`, so the
MuJoCo compiler fitted its default sphere primitive to the asset. The adapter
correctly followed the compiled geom type. Fixed the fixture to request a mesh;
the unchanged binding implementation then passed. No tolerance was widened.
The 12-test rerun passed. A further passive URDF force test was then added.

## S5 — full lizard same-state and floor control (completed)

Command: `conda run --no-capture-output -n lizard_rft python
scripts/validate_general_rft.py --duration 6 --output-dir
outputs/general_rft/stage1_full_6s_20261027`.

Outputs: 12,000-step baseline qpos/qvel/ctrl trajectory; the matched 12,000-step
floor-disabled control; original rigid-ground telemetry; 17 representative
forward-synchronized same-state fixtures with 13,916 triangles; raw force and
generalized-force arrays; summary, graph and hashed manifest.

Compatibility results: all 17 fixtures passed, zero mask disagreements,
maximum raw triangle-force error 5.551115123125783e-17 N, smoothed-force error
2.7755575615628914e-17 N, generalized-force error 8.881784197001252e-16.
Generalized-force components have DOF-dependent units (N / N m). The original
files' SHA-256 remained unchanged.

The explicitly corrected body-frame convention is DIFFERENT: maximum sampled
triangle-force change 0.18259551746171374 N and generalized component change
4.357166440537851, with matching states/history. At sand z=0 this primarily
exposes the legacy COM-velocity/body-origin lever-arm mixture; the independent
offset-inertia Jacobian test establishes the new point-velocity convention.
This change was not silently adopted in the original lizard runner and is not
proof of experimental force accuracy.

Ground answer for this tested gait/duration: `sand_visual` is a non-colliding
plane at z=0; the only colliding world plane is `floor` at z=-0.25 m. The sand
run recorded zero world-ground contact/support steps and 0 N ground support.
Minimum RFT triangle-centroid z was -0.00931106929157266 m, far above that
safety floor. Disabling the floor produced byte-identical qpos arrays across
all 12,000 steps (maximum difference 0). The original rigid scenario has its
intentional floor at z=0, 10,531 contact steps and a minimum soft-contact
distance -0.0005747163213678276 m (about 0.575 mm). No extra supporting layer
was found in the tested sand run; the safety floor is present but inactive.

The translucent sand geom, thin visual `sand_height` site and force-point
sphere radii can affect perceived depth. The contact telemetry and matched
floor-disabled control, not appearance, establish the conclusion. Historical
invalid-RFT recovered video remains visual evidence only; it cannot establish
ground or force behavior of the current model.

## S6 — first real external-model screening (completed)

Commands: `scripts/run_generic_robot.py --config
configs/robots/menagerie_go1.yaml --duration 2 --video --show-points` and the
equivalent `menagerie_panda.yaml` config, each under `conda run -n lizard_rft`.
Explicit output directories: `outputs/general_rft/go1_2s_20261027/` and
`outputs/general_rft/panda_2s_20261027/`.

Go1: four selected foot-sphere surfaces, 896 triangles, 3,999 RFT-active steps
of 4,000; all states finite. Panda: one selected compiled link7 mesh, 200
triangles, 2,778 active steps of 4,000; all states finite. No triangle force
sites were added to either model. Both produced 41-frame, 20 FPS videos
(2.05 encoded seconds, covering the two-second simulation and final frame).
First/middle/last frames decoded at 960x540, and manifest verification passed.

The Go1 example holds upstream home actuator commands. Panda adds a 0.05 rad,
0.5 Hz sine to one upstream actuator; its horizontal test sand surface is
explicitly 15 mm above the selected initial minimum centroid. These are
bounded integration screens with upstream collision proxies, not new walking
policies, whole-robot contact-envelope approvals or material calibration.

## Initial queue (historical; all stage-one outcomes recorded above/below)

1. Implement reusable surface bindings, provider, explicit conventions and
   focused frame/provider tests.
2. Generate representative legacy snapshots and compare the same states and
   histories. Preserve raw arrays and compact difference summaries.
3. Inventory effective ground geoms and actual supporting contacts; perform
   a bounded matched safety-floor control where warranted.
4. Pin/install selected open-source robot models and optional loading tools;
   validate real model loading, binding, RFT and supported control demos.
5. Run canonical/full validation and reporting smoke, record results and
   commit focused source/evidence changes. Do not publish new media as a
   canonical release without documenting its status.

## S7 — clean-source final acceptance (completed)

Implementation commit: `3be21dfc68218528b5d936868b91ff2fa33e2e1f`.
All of the following ran sequentially from that clean source:

1. `validate_general_rft.py --replay-run` against the frozen seventeen-state
   oracle: passed with the same machine-roundoff-level maximum errors.
2. `validate_project.py --full`: all 29 tests, upstream snapshot, eight active
   mesh gates, 13,916 sites, original rigid smoke and original 6 s RFT passed.
   Legacy displacement/force/power/sinkage values matched the recorded baseline.
3. `generate_video_matrix.py --duration 0.25 --fps 10 --width 640 --height 480
   --panel-width 420`: nine MP4 views plus overview, clean git flag.
4. `analyze_video_matrix.py`: all declared artifact sizes/hashes passed and
   derived metrics/CSV were produced.
5. Go1 and Panda demos were regenerated at clean source into
   `outputs/general_rft/{go1_clean_3be21df,panda_clean_3be21df}/`; counts and
   finite-state outcomes matched S6. Both manifests and first/middle/final
   frame decoding passed. Preview sheets were visually inspected.

Full console output: `outputs/general_rft/final_verification_3be21df.log`,
copied as compact `docs/regressions/2026-10-27-general-rft/final_verification.log`.
The 0.25 s reporting smoke is mechanics validation, not a gait-performance
result. No full production video set was regenerated or overwritten.

The new kernel's parsed AST matches the frozen force function exactly after
renaming the function; import of `rft.kernel` loads neither MuJoCo nor Open3D.
The worst legacy triangle-force roundoff was at fixture 12, global triangle
4378, component FR, z component. Corrected-convention maximum change was
fixture 13, triangle 4451, FR, z. Exact details are in `difference_details.json`.

Environment inventory comparison found zero changed pre-existing packages.
Only the four optional registry packages listed in S4 were added. `pip check`
passed. Numerical pins and the original `environment.yml` hash are unchanged.

## S8 — evidence and handoff (completed)

Purpose: let another person/agent reproduce each step and distinguish source,
configuration, fixtures, measured differences and media from interpretation.

- Architecture: `docs/RFT_GENERALIZATION_PLAN.md`.
- Module responsibilities and portable commands: `docs/GENERAL_RFT_GUIDE.md`.
- Compact results, raw-run paths, source/config SHA-256, per-artifact hashes,
  force differences, ground plot and same-pose rendering comparison:
  `docs/regressions/2026-10-27-general-rft/`.
- Exactly two reviewed clean-source MP4 demos plus previews/notices:
  `docs/media/general_rft_stage1/`, exception documented by D-017.
- Raw trajectories, force arrays, derived outputs and initial failures remain
  under ignored `outputs/general_rft/`; pinned original model assets remain
  ignored under `third_party/robot_assets/`. Nothing was silently discarded.
- Original RFT source, original model/gait, all accepted STLs, controller and
  environment file were compared with `ccc6946` and remain unchanged.

Commit sequence: `afa8236` records the agreed plan/freeze before code;
`3be21df` contains the tested binding/provider and tooling; the subsequent
focused documentation/media commit records this completed evidence. No
baseline-replacing merge is implied by preparing the implementation branch.

## Remaining adoption/research work (not claimed complete)

1. Authoritative contact-envelope/CAD review and material calibration remain
   necessary for new hardware; collision proxies are an explicit approximation.
2. Corrected body-frame forces are not adopted for legacy lizard locomotion.
   A future adoption must assess and record behavior changes independently.
3. Real external URDF/gait screening beyond the known-answer fixtures,
   automatic actuator/root synthesis, stock registry auto-cloning, persistent
   prepared-surface caching and arbitrary terrain/deformable support remain
   future work. The two downloaded real examples are MJCF.
4. Hold/sine examples are ready-to-run integration screens, not a universal
   walking-policy library or long-horizon stability proof.
