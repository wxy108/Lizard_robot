# Robot and format expansion: plan, acceptance and ledger

Request: broaden ground robots/formats and reduce manual deployment steps.
Start: `54273a6`; keep branch `agent/ground-rft-pipeline`, original lizard
`ccc6946`, existing Go1/G1 success/failure records, force law and numerical
pins unchanged. IsaacLab stays outside scope.

## Deliverables and evidence levels

- Robot catalog with per-robot pinned downloads, notices, recipes and controller
  contracts; install only requested robots. Initial targets: Go1, G1, Berkeley
  Humanoid, Booster T1, Apollo if its assets fit the budget, and TurtleBot3
  Burger/Waffle Pi (real URDF/Xacro differential-drive models).
- Single CLI to list, diagnose, install, prepare/run and batch known robots;
  unknown inputs produce inventory + explicit unreviewed recipe instead of
  inventing a physically valid robot. Cache hits still verify source/geometry.
- Real URDF/Xacro import, explicit recorded floating-base/velocity-actuator/
  support adapters, mesh path resolution; compiled MJB and a relocatable
  self-contained robot package with size/path/hash/version checks.
- Mesh formats STL/OBJ/PLY exercised as equivalent geometric inputs, not
  counted as independent robot or locomotion validations.
- Report separately: enumerated, downloaded, imported, surface-qualified,
  controller-parity checked, rigid-screened, sand-screened and adopted demo.
  Failing cases and previously missed short-screen failures remain visible.

## Criteria, budgets, stop conditions

"Easier" means fewer hand-authored startup steps for supported catalog robots,
clear actionable errors, bounded downloads and verified cache reuse. Do not
trade correctness for count. Native policy obs/action conventions must match
their pinned official code (including phase, head masking, action scale and
default pose); original hard-ground reference precedes sand. Wheel kinematics,
servo additions and caster approximation are explicit adaptations, not a claim
that URDF supplied a complete MuJoCo controller.

At most seven actual robot models in this initial batch, <=400 MB new selected
upstream assets, <=120 MB per additional model. CPU inference only. Screen 2 s,
then 6 s representative runs; at most one decision-relevant corrective attempt
per new robot. No large gait search, policy training, material/gain tuning to
hide a failed transfer, or unrelated environment upgrades. Existing Go1/G1
matched trajectories and original six-second lizard regression are controls.

Package round-trip compares compiled physical identity, initial state,
controller output and force at the same state. A package cannot silently
drop explicit floor pairs or remove unrelated self-contact. Untrusted archive
paths/symlinks, oversized expansion, missing hashes or mismatched MuJoCo
binary version must stop before model loading. Source/model edits remain in
private native specs or generated outputs, never vendored originals.

## Ledger

E0: reviewed handoff docs, live pipeline and official controller sources.
Current manual barriers: explicit YAML per model, only Go1/G1 controller
routing, URDF fixed root/no actuators, no Xacro expansion, no catalog-selective
installer, and no portable single-file package. T1 differs materially from
G1 (head obs masking, action scale 1.0); Apollo has command-dependent phase.
These differences will not be papered over by copying one adapter.

Append actual steps/results/failures below. Raw output:
`outputs/robot_catalog/`; compact evidence:
`docs/regressions/2026-10-28-robot-catalog/`.
