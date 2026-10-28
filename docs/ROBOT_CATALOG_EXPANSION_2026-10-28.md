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

### E1 — pinned catalog and environment isolation

`configs/robot_catalog.json`, `rft/catalog.py` and `scripts/robot_pipeline.py`
provide selective immutable Git-blob/SHA downloads, list/doctor/install/run/
batch/inspect/pack. Separate additive `catalog_manifest.json` leaves earlier
download manifests intact. Seven models selected 190,664,598 bytes: Playground
73 files, Menagerie 175, TurtleBot3 10. Install evidence:
`outputs/robot_catalog/install_20261028_023353_881277/`. Added only Xacro 2.1.1
to optional requirements and `lizard_rft`; pip check passed. No numerical pin
or IsaacLab change. Final package delta must be checked against saved inventory.

### E2 — formats and explicit native adapters

`rft/formats.py`, `rft/binding.py`, `rft/pipeline.py` resolve package meshes,
expand Xacro, identify URDF by XML root, preserve links and name collision
geoms. Recipe-only floating-base/velocity-servo/floor/keyframe additions have
effective readback, including native inferred mass where URDF omits inertia.
Ambiguous single-collision selections fail. Native MJB/MJZ and checked archives
load without modifying originals. `rft/onboarding.py` recognizes exact known
bytes; unknown or unsupported inputs stop with a draft, never invented gait.

### E3 — independent controllers, not one adapter for every policy

`controllers/catalog_controllers.py` and `configs/controllers/playground_profiles.json`
encode official Berkeley/T1/Apollo joint order, dimensions, scales, phase and
target poses. Source refs and recipe paths are in the catalog. Thirty states
per policy (zero/near-zero/moving commands) produced exact zero observation and
target difference: `controller_parity_20261028_024854_869341/` (development).
T1's upstream accidental physical qvel overwrite is deliberately removed;
Berkeley uses named home instead of invalid source index; Apollo stand vs
knees_bent distinction retained. Differential drive uses explicit geometry and
servo assumptions. No policy weights, gains or sand coefficients were retuned.

### E4 — verified portable models and repeat preparation

`rft/robot_package.py` saves separate exact compiled rigid/sand models, policy,
recipe, source/license/hash/version metadata; import checks inventory and
relocated files. Version locking prevents treating MJB as a universal ABI.
`scripts/run_ground_pipeline.py` now has verified content-addressed cache and
separate timing ledger; clones compiled models instead of repeatedly parsing
unchanged assets. `rft/records.py` writes canonical LF JSON bytes to avoid
Windows/Git hash changes. Provider/kernel/legacy physics remain unchanged.

### E5 — development screens and validation limits

Two-second runs (dirty source, not adopted release): Berkeley sand dx 0.279 m
and T1 0.354 m passed; Apollo completed but moved backward -0.545 m and failed
progress. Burger rigid dx 0.150/0.156 m but sand approximately 0.005 m failed;
Waffle Pi also failed sand progress. Retain these as wheel/proxy and policy
transfer limitations, not evidence that import failed. Wheel recipes require
at least 50% of commanded progress, so tiny creep is not called locomotion.
No tuning/retry was used to erase these failures. Six-second clean screens
follow; short passage is not sustained success.

Real Go1/G1/Burger binary package round-trip had identical physical fingerprints
and zero active triangle-force/generalized-force/control difference; Burger
expanded pure URDF matched Xacro. Native Go1 MJZ had identical active force/
control probe but quaternion differences up to 1.7021e-7 and size differences
4.9446e-8 m after text serialization. Preserve this residual: MJZ is not
bitwise physical identity or full-trajectory equivalence. Binary packages must
match exact fingerprints. Raw: `dev_format_validation/`.

### E6 — known-answer tests, reporting and handoff

Added 16 tests (65 total), including real Xacro/URDF adapters, MJB/MJZ,
archive/version/hash guards, STL/OBJ/PLY, canonical JSON, T1 read-only obs,
Apollo phase, wheel kinematics, cloning and extra dynamic actors. Initial
Xacro test incorrectly counted native inferred sphere mass as zero; corrected
the oracle to the declared base mass, without relaxing a tolerance. Fixed
native ZIP helper file-handle ResourceWarning by passing managed file objects.
Open3D's OBJ triangle-normal export warning is an expected format limitation;
normals are recomputed from faces. `scripts/review_robot_catalog.py` verifies
cached bundle/raw manifests, finite aligned telemetry and decoded frames,
supports previous Go1/G1 array comparisons, and preserves failures. Full user
entry point: `ROBOT_CATALOG_GUIDE.md`. Clean validation/adoption follows below.

### E7 — clean runtime 36659ca and final interface/presentation hardening

All 65 initial tests, actual native doctor for seven models, three 30-state
policy parity tests and real format/package probes passed on clean `36659ca`.
The policy verifier initially ignored the requested output argument and wrote
its default directory; results remain valid at
`controller_parity_20261028_031618_188059/`. Add explicit argparse handling and
repeat it. Full canonical lizard validator passed: 13,916 sites, dx 217.7 mm,
dy -120.8 mm, peak total/site 86.629/0.600 N, zero positive active power.

`clean_6s_36659ca/` completed all seven robots and 21 scenario videos. All
14 rigid scenarios passed. Go1/G1/T1 sand passed; Berkeley stopped 3.4975 s,
Apollo 2.325 s; Burger/Waffle completed but failed forward progress (~3 mm).
`review_36659ca/` verified all manifests/caches and 2,417 video frames.
Go1/G1 all seven saved arrays in all three scenarios are bitwise equal to
`f4c0360` runs. Environment delta: only Xacro 2.1.1 added, no package changed.

Final review found URDF collision boxes hid original visual meshes and black
backgrounds hid labels. Only the render scene is corrected; immutable saved
states replay to new directories (D-021). Also tighten input recognition:
copied identical scene XML must not silently substitute canonical neighboring
assets. Canonical verified paths alone auto-select catalog recipes; unknown
Xacro/native ZIP get explicit inspection/drafts. Add four tests (69 total),
preserve original recipe/effective-adapter provenance inside packages, and
record all remaining checks/media below. None changes force/integration/control.
