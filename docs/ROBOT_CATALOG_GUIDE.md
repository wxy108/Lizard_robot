# Multi-robot RFT catalog: install, run, inspect and move

This is the expanded entry point on branch `agent/ground-rft-pipeline`.
Original lizard and the earlier Go1/G1 evidence remain unchanged. A catalog
entry means an explicit reviewed recipe exists, **not** that its sand gait
passes. Results and failures: [implementation ledger](ROBOT_CATALOG_EXPANSION_2026-10-28.md).

## Fresh machine

Follow [core prerequisites](../GUIDANCE.md), then use this branch explicitly
(`main` deliberately retains the original lizard release):

```sh
git clone --branch agent/ground-rft-pipeline --single-branch https://github.com/wxy108/Lizard_robot.git
cd Lizard_robot
conda env create -f environment.yml
conda run -n lizard_rft python -m pip install --no-deps -r requirements-locomotion.txt
conda run -n lizard_rft python -m pip check
conda run -n lizard_rft python scripts/robot_pipeline.py doctor
conda run -n lizard_rft python scripts/robot_pipeline.py list
```

Use only `lizard_rft`; no IsaacLab, ROS, JAX, PyTorch, GPU policy training or
separate MuJoCo installer. Optional packages are CPU ONNX Runtime and Xacro;
the numerical environment pins remain those of `environment.yml`. On a
headless Linux host select an available renderer, e.g. `MUJOCO_GL=egl` or
`osmesa`; rendering still requires the corresponding system driver/library.
Local validation is Windows; this is not a claim of tests on every OS.

## One command per supported robot

```sh
conda run -n lizard_rft python scripts/robot_pipeline.py run t1 --duration 6 --video --output-dir outputs/my_t1
```

Missing requested source assets download automatically from pinned commits,
including original licenses and policy weights. Existing modified assets are
refused, not overwritten. All output directories must be new. To prefetch and
verify for offline runs:

```sh
conda run -n lizard_rft python scripts/robot_pipeline.py install go1 g1 berkeley t1 apollo turtlebot3_burger turtlebot3_waffle_pi
conda run -n lizard_rft python scripts/robot_pipeline.py doctor go1 g1 berkeley t1 apollo turtlebot3_burger turtlebot3_waffle_pi
conda run -n lizard_rft python scripts/robot_pipeline.py batch go1 g1 berkeley t1 apollo turtlebot3_burger turtlebot3_waffle_pi --duration 6 --video --output-dir outputs/my_catalog
```

Seven actual robot models; `g1_fast_failure` is the old 0.4 m/s negative
variant, not an eighth robot. Selection uses 190,664,598 upstream bytes for
all seven (includes reused assets); the whole upstream repositories are not
cloned. Catalog source paths, commits and per-robot configurations are in
`configs/robot_catalog.json`. Download counts are not validation counts.

The CLI exits nonzero if a batch member fails, but continues remaining
members and saves the failure. Exit 2 means an input needs a reviewed adapter.
Use `--prepare-only` to stop after geometry/binding checks. Without `--video`,
numeric evidence and diagrams still save; frame decoding requires videos.

## What the pipeline actually does

1. Verify pinned sources and parse MJCF/URDF/Xacro or a version-locked native
   model. URDF adapters explicitly add the floating base, wheel servos,
   floor/keyframe and stable link/collision names without editing originals.
2. Inventory effective bodies, geometry, mass/inertia, joints and actuator
   limits. Bind explicitly selected external collision proxies to body names.
   Ambiguous or missing owners stop; raw CAD assemblies are not auto-unioned.
3. Extract the selected body-local surface, isotropically remesh, then require
   closed/oriented/non-self-intersecting geometry, bounded deviation and
   acceptable distribution. This remesh changes only RFT quadrature surfaces,
   not visuals, native collisions, mass or inertia.
4. Save a hash-verified qualified bundle and check native point positions and
   velocities in three poses. Cache key includes physical fingerprint,
   configuration, builder code and numerical-library versions. A corrupt
   cache stops; clear or replace its exact directory only after review.
5. Validate the independent controller contract: joint order, observation
   dimensions, action scales, cadence and target pose. Run upstream rigid,
   matched-timestep rigid, then sand. Sand disables explicit floor pairs as
   well as masks, retains unrelated self-contact, and checks unintended rigid
   support/dynamic actors. No success is inferred from compilation alone.
6. Persist finite state/force/contact/COM/penetration telemetry, acceptance
   failures, full effective settings, diagrams and optional saved-state video.

Berkeley/T1/Apollo use their own pinned official policies, not copied G1
conventions. T1's accidental physical head-velocity overwrite is removed while
preserving exact observation/target outputs; Berkeley's invalid numeric reset
index is replaced by its named home key. T1 is tested on the explicit flat
scene. Apollo starts at `stand` while its policy target is `knees_bent`.

TurtleBot3 uses genuine ROBOTIS URDF/Xacro with declared mass/inertia and
package meshes, a standard differential-drive controller and explicit velocity
servos (`kv=0.2`, ±1 Nm, ±10 rad/s). Fixed caster boxes and smooth wheel
cylinders are source proxies, not calibrated rolling/tire models. Hard-ground
motion does not demonstrate sand traction. The sand coefficient remains the
uncalibrated inherited value 3.75 for every robot; it is not fitted to pass.

## File format coverage and safe limits

| Input | Current route | What still requires a decision |
| --- | --- | --- |
| MJCF `.xml` | Native parse, includes/assets, explicit recipe | Select contact surface and compatible controller |
| URDF `.urdf` or `<robot>` XML | Native conversion, stable link names | Explicit floating-root/actuator/support recipe |
| Xacro, including macros inside `.urdf` | Python Xacro + package roots | Mapping arguments and package directories |
| MuJoCo `.mjb` | Version-locked native binary | Exact MuJoCo version and recipe/controller |
| Native `.mjz` / native ZIP | Checked native archive | Requalify geometry; text serialization may round fields |
| `.rftpkg` / exported package ZIP | Verified MJB + recipe + policy + licenses | Same MuJoCo version and compatible pipeline/environment |
| STL/OBJ/PLY | Component geometry extraction/remesh | Body owner, scale, inertia/model and controller; not a whole robot |
| STEP/IGES/SDF/other formats | Inspection/refusal | Reviewed CAD/conversion adapter; no invented dynamics |

```sh
conda run -n lizard_rft python scripts/robot_pipeline.py inspect --input path/to/unknown.urdf --output-dir outputs/inspection
conda run -n lizard_rft python scripts/robot_pipeline.py run --input path/to/model.xml --output-dir outputs/input_run
```

Only unchanged input at its canonical installed catalog path selects its
recipe automatically: a copied scene may reference different adjacent meshes.
The filename or XML bytes alone never substitute canonical dependencies.
Unknown files produce an inventory and
`unreviewed_recipe.json` with `ready_for_simulation: false`, or an actionable
missing-adapter reason. Edit a reviewed recipe using `configs/robots/` and
[the lower-level guide](GROUND_RFT_GUIDE.md), then invoke
`scripts/run_ground_pipeline.py --config YOUR_RECIPE --duration 6 --video`.
Mesh-only files cannot supply a safe locomotion model/controller automatically.

## A single-file portable robot

```sh
conda run -n lizard_rft python scripts/robot_pipeline.py pack g1 --output outputs/g1.rftpkg
conda run -n lizard_rft python scripts/robot_pipeline.py run --input outputs/g1.rftpkg --duration 6 --video --output-dir outputs/relocated_g1
```

Move the package to another configured machine; original asset folders and
network are no longer needed. It contains separate rigid/sand MJB models so
G1/Apollo floor-pair conversion cannot be lost, recipe, policy when applicable,
hashes, source provenance and original licenses. Geometry is qualified by the
destination pipeline; the package is not an OS installer or a universal gait.
Check exact MuJoCo binary version. Archive paths, duplicates, symlinks, size,
inventory and hashes are checked before extraction/native loading; these are
defensive guards, not a complete security guarantee for arbitrary native files.

Binary packages require exact physical fingerprint equality. In the real Go1
native MJZ round-trip, geometry quaternion components changed by up to
1.70e-7 and geometry sizes by 4.94e-8 m from text serialization. Its active
same-state force/control probe matched, but this is **not** bitwise model or
full-trajectory equivalence. Use the binary package when exact identity matters.

## Read and reproduce results

Batch root: `batch.json`, `manifest.json`, one stdout log and directory per
robot. Each robot directory contains `pipeline.json`, effective inventory,
binding checks and `rigid_upstream/`, `rigid_matched/`, `sand/`. Geometry bundle
may live outside it in `outputs/robot_catalog/cache/`; its exact path is in
`pipeline.json`, and its own manifest must verify.

Each scenario: `summary.json` (status, reasons, durations, effective inputs),
`trajectory.npz` (qpos/qvel/control/generalized RFT and synchronized trace),
`diagnostics.png` (COM, penetration, 0/1 component contact), and
optional `locomotion.mp4`. Read `trace_columns`/component names from saved
metadata; do not infer force channels from array position or label commands as
actual speed. `failed_screen` plus `termination: completed` means the numeric
run finished but acceptance failed, not a valid sand gait.

```sh
conda run -n lizard_rft python -m unittest discover -s tests -v
conda run -n lizard_rft python scripts/verify_catalog_controllers.py --output-dir outputs/policy_parity
conda run -n lizard_rft python scripts/validate_catalog_formats.py --output-dir outputs/format_checks
conda run -n lizard_rft python scripts/review_robot_catalog.py --batch outputs/my_catalog --output-dir outputs/my_review
```

The reviewer verifies raw/cache/scenario manifests, finite aligned arrays and
every video frame; it derives speeds and preview sheets without rewriting raw
data. Positive force dissipation/support checks are necessary, not proof of
calibrated granular accuracy. See the tracked compact report for tested bounds,
failures, source commits and media. All full raw outputs stay ignored.

For visual-only corrections, replay into a new folder without running physics:

```sh
conda run -n lizard_rft python scripts/replay_ground_video.py --run outputs/my_catalog/turtlebot3_burger --config configs/robots/turtlebot3_burger.yaml --scenario sand --output-dir outputs/burger_replay
```

The replay verifies source/cache manifests, preserves the trajectory hash and
records simulation vs rendering commits. URDF bodies show original visuals
instead of overlapping opaque collision proxies; collision-only casters stay
visible. White caption bands keep metadata legible on black source backgrounds.
This is presentation, not a new physics experiment. Xacro expansion is tested
on explicit package roots/arguments; ROS/ament package discovery is not supplied.
