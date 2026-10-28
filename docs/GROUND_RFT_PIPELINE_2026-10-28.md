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
