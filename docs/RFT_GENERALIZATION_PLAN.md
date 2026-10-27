# General RFT architecture and staged adoption

Approved scope: 2026-10-27. Implementation baseline:
`ccc694677461012d488cb5ad7d7e9db945b74a80`.

## Goal and fixed boundaries

Accept robots described by MJCF or supported URDF plus their assets, explicitly
select sand-contact surfaces, bind every surface to a compiled MuJoCo body,
and provide RFT generalized forces independently of the controller and step
orchestrator. The existing lizard is a preserved control, not physical ground
truth. Use only `lizard_rft`; IsaacLab is outside scope.

Do not remesh, retune, replace, or silently correct the legacy lizard while
extracting reusable interfaces. Keep `lizard_sand.py`, `sim_fxn_lib.py`, both
lizard MJCF models, accepted STLs and gait configuration unchanged. Legacy
sites remain valid. New bindings do not require sites.

## Architecture

```text
MJCF/URDF + assets -> ModelAdapter -> compiled model + name/body/geom mapping
                                           |
explicit RFT surface recipe -> SurfaceBuilder/Binding -> SurfaceBundle
                                                            |
independent controller -> Runner <----- RFTForceProvider <----+
                            |
                   MuJoCo step + recorder
```

### ModelAdapter

- Resolve model-relative assets and explicitly supplied URDF package roots.
- Preserve fixed links during URDF import unless an explicit mapping handles
  fusion. Record effective compiler options and source hashes.
- Resolve names to numeric IDs on each compiled model; never persist IDs as
  portable identifiers. Reject missing/ambiguous bindings.
- Treat root mode, initialization and actuators as model/controller inputs.
  Import success does not imply a complete locomotion controller exists.
- Use MuJoCo's parser/compiler, not regex rewrites of arbitrary MJCF.

### SurfaceBuilder and Binding

Separate visual geometry, rigid-collision geometry and RFT geometry. Inputs
may be explicit proxy meshes, selected compiled mesh geoms, or triangulated
primitive geoms. Discovery does not establish that a surface is physically
exposed; source selection must be inspectable and exclude duplicate/internal
surfaces. Do not combine surfaces across moving joints.

For every surface record:

- stable surface ID, body name and reporting group;
- source path/geom, source hash, units and source-to-body transform;
- body-local triangle centroids, outward normals and areas;
- triangle order, quality metrics, effective recipe and tool versions.

Support several surfaces on one body and several bodies in a report group.
Each triangle has exactly one physical body owner. Distinct physical shells
are not automatically dropped or bridged. Topology policies apply to declared
surfaces rather than assuming every robot has eight single-component bodies.

Keep qualified surfaces as-is. Use isotropic remeshing only when requested.
Use reconstruction only after the intended external envelope is established.
Record thresholds in physical units and reference-surface deviation. RFT
geometry preparation must not change model mass, inertia or collision shape.

MuJoCo compiler mesh centering/alignment and source mesh transforms must not
be applied twice. Compiled vertices and compiled geom transforms form one
coordinate route; source vertices and source transforms form another. Verify
bindings in multiple poses before force validation.

### RFTForceProvider

Own immutable binding arrays and per-instance smoothing state. Read model
state, compute triangle positions/velocities, call the force law and return
an independent `qfrc_rft` plus per-triangle diagnostics. Never call `mj_step`,
write control, clear another provider's applied force, or hide invalid output.

Use explicit versioned conventions:

- `legacy_lizard_v1`: preserve existing depth, body velocity reference,
  non-finite sanitization and smoothing behavior for matched-state comparison;
- `body_frame_v1`: consistent body-origin velocity and depth relative to the
  configured horizontal sand plane, with explicit invalid-force handling.

Do not label the second convention adopted for lizard merely because it is
more general. Measure and explain differences separately. World position,
normal, point velocity and force use consistent frames and SI at the adapter
boundary. Preserve historical centimetre conversions inside the existing
force law until independently replacing that law is justified.

First use per-triangle `mj_applyFT` into a private generalized-force buffer.
Body-level force/torque aggregation is an optional optimization that must pass
a generalized-force equivalence check. The two paths must never both apply
the same force. New sites/points are visualization, not hardware sensors.

### Controller and Runner

The controller supplies actuator commands independently of morphology and
RFT. Existing lizard gait control can be wrapped without changing it. The
runner refreshes state, updates control at the chosen rate, computes all force
providers each physics step, assembles their buffers, steps once and records
time-aligned data. Initially retain the supported integrator/timestep behavior
used by the lizard control. Do not claim arbitrary integrators are validated.

### Recorder

Store state source, time, body/surface/group order, pose, COM, RFT-active and
geometrically submerged counts, per-body force/power, triangle forces when
requested, rigid-contact geom pairs/forces and penetration. Include model,
mesh, config, source commit and artifact SHA-256 in a manifest. Foot-tip
markers are optional robot-specific diagnostics. Root pose is name/address
based rather than assuming `qpos[:3]`.

## Stages and acceptance

| Stage | Purpose | Deliverable | Acceptance |
| --- | --- | --- | --- |
| 0 | Preserve scope and control | This plan, frozen file hashes, step ledger | Original lizard files unchanged |
| 1 | Extract binding/provider | Generic package, lizard binding config, same-state comparison tool | Matched triangle positions, masks, forces and generalized forces within declared tolerances; unrelated force/control/state untouched |
| 2 | Validate binding mechanics | Offset/rotation/scale/multi-body tests, URDF import probe | Known-answer frame/velocity/force tests; no reliance on force sites |
| 3 | Screen external robots | Pinned open-source model sources, configs and reproducible demo commands | Compile, bind, finite active RFT, stepping and rendered output on selected models; report geometry approximation/controller limitations |
| Parallel audit | Explain apparent sinkage/floor | Effective plane inventory, contact telemetry and bounded floor-disabled comparison | Determine whether the safety floor or another rigid geom actually supports the sand run; separate original rigid-ground scenario |

Stage 3 demonstrates selected supported robot adapters. It is not a claim that
every upstream robot has an automatically generated walking gait or that its
granular contact surface/material is experimentally calibrated.

## Comparison conditions and stopping rules

Replay identical legacy-generated state snapshots; do not compare different
controllers, remeshed surfaces or independent trajectories to infer a binding
error. Preserve legacy history when testing the smoothing path. Compare:

1. triangle counts/order, positions, normals and active masks;
2. raw and smoothed triangle forces;
3. body totals and generalized forces;
4. changed-convention results separately from compatibility results.

Use float64, record absolute/relative tolerances, maximum differences, worst
sample/body and non-finite counts. Include above-plane, intrusion, rotation,
articulation, nonzero velocity and stale-buffer release cases. Freeze at least
one recorded 6 s legacy trajectory for representative matched-state samples.
An equivalence failure remains a failure until explained; do not widen
tolerances merely to pass. A baseline discrepancy is not itself proof that
the new convention is physically wrong.

For the floor question, first inspect compiled collision masks, world plane
height/orientation and actual contacts. Only then run one matched finite
duration floor-disabled control if useful. Do not remove the legacy safety
floor or alter the published model on the basis of video appearance alone.

Bound external downloads to selected pinned models; keep license/source
records and avoid changing existing numerical dependency pins. Record failed
imports/downloads/tests. System security controls are not disabled to execute
tests. Finish with measured results and explicit remaining blockers rather
than calling source inspection a runtime success.

## Working records

- Step-by-step implementation and failures:
  `docs/RFT_GENERALIZATION_WORKLOG_2026-10-27.md`.
- Compact verified summaries: `docs/regressions/2026-10-27-general-rft/`.
- Raw arrays/videos/logs: ignored `outputs/general_rft/`.
- Original baseline: Git commit `ccc6946`; do not replace its historical media.
