# Ground locomotion RFT pipeline: contract and implementation ledger

## Request, preserved controls, and completion criteria

The user's 2026-10-28 correction supersedes the stage-one demo selection:
Panda is interface smoke only, **not ground-locomotion generalization evidence**.
Build an auditable file-to-simulation pipeline, not a growing set of manually
decorated models. Preserve lizard `ccc6946`, the legacy mathematical law,
accepted STLs, controller and numerical environment pins. Use only `lizard_rft`;
IsaacLab is neither a dependency nor an installation target.

Baseline before this work: `4de8b1d`. Work stays on a separate Git branch.

## Pipeline and stage products

| Stage | Purpose | Output / stop rule |
| --- | --- | --- |
| Input | Identify MJCF, URDF or geometry-only input | Source inventory; unsupported/CAD conversion and missing dynamics reported, never invented |
| Import | Compile, resolve packages, name bodies/geoms, read SI coordinates | Effective inventory and asset hashes; no source rewrites |
| Select | Declare which external shell belongs to which rigid body | Explicit surface recipe; overlapping parts cannot be mistaken for an exterior union |
| Prepare | Tessellate primitives or load qualified exterior mesh; optional isotropic remesh | Body-local candidate meshes; never combine across joints, drop components or replace collision/inertia |
| Audit | Check closure, winding, degeneracy, intersections, size, area/spacing and reference deviation | Per-surface before/after report; failures block bundle adoption |
| Bundle | Save centers/normals/areas and mesh topology, ownership and provenance | Hash-checked portable prepared bundle, bound by names rather than persisted numeric IDs |
| Bind | Check owners, compiled model fingerprint and pose transforms | Provider-ready arrays, explicit convention; stale/tampered bundle rejected |
| Control | Validate named joints/actuators, observation order, gains, limits and cadence | Independent controller contract and readback; no implicit gait synthesis |
| Run | Rigid reference first; RFT mode disables only declared support geoms | Time-aligned states, commands, COM, RFT and rigid support, contact, penetration, termination reason |
| Review | Decode videos and inspect telemetry; distinguish stable motion from finite falling | Summary, plots, hashes and reproducible commands, including failed runs |

## Scope and evidence policy

"Arbitrary shape" means a morphology-independent framework with explicit
adapters and fail-closed validation, not a claim that arbitrary file formats
or defective CAD can be automatically assigned correct dynamics/exteriors.
Supported mesh input without a robot description is geometry-only preparation;
it cannot be promoted to locomotion without body/dynamics/controller metadata.
Closed qualified shells may be remeshed; ambiguous internal/overlapping/open
assemblies require a reviewed exterior or a separately justified reconstruction.
Topology pass does not certify the intended hardware shape. Collision proxies
remain explicitly approximations. Material coefficient 3.75 is uncalibrated.

Read official MuJoCo Playground Go1/G1 native sim2sim deployment, policy,
model, sensors and asset dependencies at a pinned commit. Bound downloads to
these two families and their notices, maximum 180 MB. No JAX/PyTorch training
stack, gamepad, ROS, hardware runtime or IsaacLab installation is necessary.
Use CPU ONNX inference with bounded threads. Record any added packages and
ensure pre-existing numerical package versions are unchanged.

Screen original controller on rigid ground before testing sand. A policy
trained on rigid ground is not assumed to walk on RFT. Separate physics mode,
integration timestep, mesh resolution and controller changes in records.
Report failures/collapse as failures, not "finite-state success". First screen
2 s, then 6 s representative runs; at most two targeted diagnosis changes per
robot unless there is new decision-relevant evidence. Do not retune a robot
or invent support merely to make a video pass. Longer validation/adoption is
distinct from these bounded demonstrations.

## Work ledger

- S0 / plan: read repository handoff contract, existing binding/provider,
  mesh audit/reconstruction and generic runner. Existing demo runner accepts
  selected raw surfaces without a persistent qualification bundle and uses
  hold/sine commands. This document records the new acceptance boundary
  before code changes.
- S1 / controller source review: official Playground native deployments expose
  Go1 50 Hz policy / 4 ms upstream simulation and G1 50 Hz / 2 ms simulation.
  Both use position targets with action scale 0.5, but observation order and
  G1 phase state differ. Exact pinned files, effective model gains and runtime
  checks will be recorded rather than inferring policy compatibility by size.

Implementation, failures, measured results and publication anchors are appended
below as they occur. Raw runs belong to `outputs/ground_rft/`; compact evidence
belongs to `docs/regressions/2026-10-28-ground-rft/`.

### S2 / pinned sources and environment

Installer verified 116 original blobs / 57,190,087 selected bytes from
Playground `12ad7f3842791ced39f27e5ac96d066bbe361653` and Menagerie
`f3b6013629c7b94f8ee93fd977b161fa8350f9bc`; existing Go1 assets reused.
Manifest/before-install inventory:
`outputs/general_rft/ground_install_20261028_000238_266964/`.
Initial ONNX-only `--no-deps` installation exposed missing flatbuffers and
protobuf via `pip check`. Optional requirements now explicitly pin ONNX
Runtime 1.31.1, flatbuffers 25.12.19, protobuf 6.33.0; final `pip check` passes.
No numerical pin or IsaacLab change. Before/after inventory retained.

### S3 / mesh qualification, remesh and persistent binding

`rft/geometry.py`: explicit units, exact STL welding, closed single-shell
topology/intersection/positive-volume gates, bounded isotropic remesh and
independent two-way sampled deviation. No arbitrary CAD Poisson reconstruction.
`rft/pipeline.py`: explicit ownership, same-body duplicate/intersection/
containment rejection, persistent verified bundle, recomputed quadrature,
compiled identity, and three-pose native position/Jacobian checks.
`rft/binding.py` only exposes its existing compiled geometry extraction;
its old surface API retains the same transformations/mathematics.

Observed failures/corrections:

- Native VFS flattens names: relative/basename aliases caused duplicate
  `hfield.png`. Keep one unambiguous basename as in upstream; a nested-include
  unit test covers this. No source XML rewrite was used.
- First Go1 remesh, 8 mm edge / 1 mm requested deviation, passed MeshLab's
  local construction check but failed independent reverse distance:
  **1.0414867 mm**, versus 0.9001426 mm in the forward direction. Refusal:
  `outputs/ground_rft/development_go1_bundle/rejected.json`. The original
  acceptance was NOT widened. Halved the internal construction allowance;
  independent two-way 1 mm gate stays unchanged. Initial failure kept source
  and metrics; persistence was then moved before final gates so future failed
  candidates are saved too.
- Resource guards bound triangle count before topology processing and
  area/edge-size estimated cost before generation. These are engineering
  guards, not proof of geometric or physical accuracy.

### S4 / controllers and support conversion

`controllers/playground_onnx.py` preserves official obs/action/phase but checks
named joint/actuator/sensor mappings. CPU inference is one intra-op and one
inter-op thread. Apache-2.0 notice retained. Standalone verifier extracts only
the reviewed, hash-checked original OnnxController AST, replacing Gamepad with
a fixed command and bounding threads. No upstream imports, viewer, training
or hardware code executes. Across 20 articulated states per family, original
and adapted observation/action arrays are **bit-identical**, including evolving
G1 phase. Development source parity evidence:
`outputs/general_rft/walking_controller_parity_20261028_001449_279156/`.

First G1 run stopped because two explicit foot/floor contact pairs bypass
collision masks: `outputs/ground_rft/g1_development_2s_v1/pipeline.json`.
Corrected with an exact reviewed `disable_support_pairs` list and private
native MjSpec deletion, sand only. Three unrelated self-contact pairs remain.
Compiled named physical identity, geometry, mass/inertia and servos must stay
equal. Raw name-buffer bytes are not physical identity: removing pair names
changes that buffer. Pair conversion is logged, never disguised by global
contact disable or extreme negative margins.

### S5 / development runtime screening

`rft/ground.py` and `scripts/run_ground_pipeline.py`: upstream rigid, matched
rigid, sand; independent identical controller initialization/command. States,
commands, forces and trace are aligned pre-step. Numerical warnings stop
immediately; sustained upright/height/depth violations stop with partial data.

| Robot / 2 s development run | Upstream rigid dx | Matched rigid dx | Sand dx | Max sand centroid depth |
| --- | ---: | ---: | ---: | ---: |
| Go1 `go1_development_2s_v2` | 0.671 m | 0.668 m | 0.647 m | 30.02 mm |
| G1 `g1_development_2s_v2` | 0.850 m | 0.840 m | 0.716 m | 33.02 mm |

Both used `[0.4,0,0]`, 50 Hz, zero rigid sand support. These are dirty-source
development screens, not final acceptance. Clean-source six-second evidence
follows. No policy retuning or material calibration was performed.

### S6 / tests before final validation

18 new known-answer tests plus 29 retained: area/volume, open/inward/multiple/
crossing/duplicate shells, invalid arrays, units, resource bounds, immutable
model identity, cache tampering/wrong model, owner/duplicate selection,
undeclared world floor, contact-pair bypass/removal, native upward force,
VFS nesting, obs/action/cadence. Source: `tests/test_ground_pipeline.py`.
Clean-source validation and exact outputs are appended after execution.

### S7 / clean six-second screen and bounded follow-up

Clean source `5078fdc83d418f329bd7d54d0272fb9aaa4fb8f6`: official controller
parity, all 47 tests, eight original mesh gates, unchanged 13,916 sites,
original rigid smoke and full six-second lizard regression passed. Baseline
again: dx 217.7 mm, dy -120.8 mm, peak total/site 86.629/0.600 N, no positive
active-power steps. Complete log:
`outputs/ground_rft/final_verification_5078fdc.log`.

Go1 `outputs/ground_rft/go1_clean_6s_5078fdc`: all three six-second scenarios
passed; dx upstream rigid 2.185 m / matched rigid 2.142 m / sand 2.104 m.
G1 `outputs/ground_rft/g1_clean_6s_5078fdc`: both rigid scenarios completed,
dx 2.965 / 2.936 m, but sand **failed at 3.765 s**, despite passing its 2 s
development screen. Root dropped to 0.264 m, upright cosine 0.415, maximum
centroid depth 43.98 mm, zero rigid support and zero positive smoothed power.
It was stopped by the existing 100 ms locomotion-envelope guard; not a
numerical-warning failure and not accepted as full sand locomotion. Failure
video, states, plots and manifests are preserved. No threshold was relaxed.

One controlled follow-up changes only command vx from 0.4 to 0.2 m/s:
`configs/robots/playground_g1_slow.yaml`. It is a diagnostic, not an adopted
replacement of the failed 0.4 m/s case. No other policy, gain, material, mesh,
dt or acceptance change. Stop this branch after that bounded six-second test,
regardless of whether a favorable result is found.

Read-only safety review also found the generic support inventory must include
**world-welded fixed children and mocap bodies**, not just geom_bodyid==0.
Corrected that guard and added nested-static/mocap floor tests. Require a
floating root for the two audited walking-policy adapters, and unique named
support pairs for removal. These stricter guards do not change the tested
Go1/G1 physical configuration; the six-second Go1 run is repeated after them.
The original lizard source/physics remains byte-identical and its full
regression above is retained, not rerun merely for stricter generic guards.
