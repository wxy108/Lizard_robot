# Audited ground locomotion pipeline

Current ground runner scope is one floating robot and an explicitly reviewed
horizontal support scene. Moving dynamic external actors, multiple robots,
arbitrary terrain and whole-body burial need separate scene/contact adapters;
the static/mocap support guard is not validation of those scenarios.

This is the current generic entry point. Historical hold/sine integration
screens, especially Panda, are **not walking-policy evidence**. The new
examples use official Go1 quadruped and G1 biped walking policies with CPU
ONNX inference; no IsaacLab, ROS, JAX, PyTorch, training or hardware access.

## Install and run from a fresh clone

This feature is on **`agent/ground-rft-pipeline`**, not the preserved `main`.
Clone the correct branch first; do not assume a default-branch ZIP has it:

```bash
git clone --branch agent/ground-rft-pipeline https://github.com/wxy108/Lizard_robot.git
cd Lizard_robot
```

For an existing clone, preserve uncommitted work, fetch origin and switch to
this branch before running commands. Do not reset/overwrite local work.

Use the normal project `environment.yml` and `scripts/setup.ps1` or
`scripts/setup.sh` first (see `../GUIDANCE.md`). Then from the repository root:

```powershell
conda run --no-capture-output -n lizard_rft python scripts/install_ground_robots.py
conda run --no-capture-output -n lizard_rft python -m pip install --no-deps -r requirements-locomotion.txt
conda run --no-capture-output -n lizard_rft python -m pip check
conda run --no-capture-output -n lizard_rft python scripts/verify_walking_controllers.py
conda run --no-capture-output -n lizard_rft python scripts/run_ground_pipeline.py --config configs/robots/playground_go1.yaml --duration 6 --video
conda run --no-capture-output -n lizard_rft python scripts/run_ground_pipeline.py --config configs/robots/playground_g1_slow.yaml --duration 6 --video
```

These commands are also valid in POSIX shells. Linux headless rendering may
require `MUJOCO_GL=egl`, and an appropriate installed driver. Current native
runtime evidence is Windows, not an independent Linux/macOS validation.
Only optional CPU inference packages are added to `lizard_rft`; numerical
pins and every IsaacLab environment stay unchanged. Downloads are 116 pinned
original files, 57,190,087 selected bytes, with reused Go1 assets not fetched
again. Source lock: `../configs/ground_robot_sources.json`. Original notices
are retained; modified controller conventions retain Apache-2.0 notice in
`../controllers/NOTICE.playground`.

Use the screened **0.2 m/s G1 recipe** above for the bounded demo. Original
`playground_g1.yaml` requests 0.4 m/s and failed in sand at 3.765 s, despite
passing its two-second screen; it remains a reproducible negative case.
No policy, mesh, material or acceptance threshold was retuned. Go1 at 0.4
and G1 at 0.2 completed six seconds with zero rigid sand support. Completion
is not accurate tracking: mean sand vx 0.351 vs 0.4 m/s (Go1), 0.310 vs 0.2
(G1). G1 already overshot on matched rigid ground (0.298 m/s).
See [reviewed videos](media/ground_rft_pipeline/README.md) and
[measured evidence](regressions/2026-10-28-ground-rft/README.md).

## What one pipeline command actually does

1. Compile the original model and retain a body/geom/joint/actuator inventory,
   effective PD gains, damping, armatures, limits and source file hashes.
2. Select explicitly named exterior/proxy geoms or body-local mesh sources.
   Resolve compiled mesh centering/scaling exactly once; primitives become
   closed tessellations. Geometry sources declare metres or millimetres and
   a source-to-body transform. No automatic dynamics/actuator synthesis.
3. Reject open, inward, inconsistent, degenerate, duplicate, intersecting or
   multiple-component shells. Overlapping or contained shells on one owner
   need an explicitly reviewed exterior union. No largest-component shortcut.
4. Perform bounded isotropic remeshing in a candidate directory. Preserve
   sharp features, keep a construction margin, and independently check both
   reference-to-candidate and candidate-to-reference sampled distance against
   the **original** physical tolerance. This is not an exact Hausdorff proof.
5. Audit topology, outward normals, area/centroid-spacing ratios and slivers
   through edge-aspect limits. Enforce per-surface triangle budget before
   topology processing and an area/edge-size estimate before generating a
   potentially excessive remesh. Save failed candidate/diagnosis, not a pass.
6. Save prepared topology and immutable quadrature arrays, ownership, recipe,
   metrics, version/commit/source hashes. Reload via artifact hashes and
   compiled model identity; recompute centers/normals/areas from topology.
7. Check three poses against native `mj_local2Global` positions and point
   Jacobian velocities before simulation.
8. Run official walking controller on rigid ground at upstream timestep,
   then rigid and sand at the same smaller coupling timestep. This separates
   timestep effects from the change in contact model. Initialization and
   command match; observations and actions evolve independently with physics.
9. In sand mode remove exactly the reviewed explicit support contact pairs
   via a private native `MjSpec`, disable declared world support masks, and
   reject undeclared world collision support, including fixed child bodies
   and mocap platforms. Preserve self-contact pairs,
   geometry, dynamics, servo definitions and original on-disk XML.
10. Record synchronized pre-step state/command/force/contact, COM, depth,
    root height/uprightness, numerical warnings and termination reason.
    Render saved states only; orange centroid dots are not force sensors.

## Input formats and safe stopping

```powershell
conda run --no-capture-output -n lizard_rft python scripts/run_ground_pipeline.py --input path/to/robot.xml
conda run --no-capture-output -n lizard_rft python scripts/run_ground_pipeline.py --input path/to/robot.urdf
conda run --no-capture-output -n lizard_rft python scripts/run_ground_pipeline.py --input path/to/shell.stl --units mm
```

`--input` inspects and reports missing metadata; exit code 2 means an
intentional safe stop, not a completed simulation. MJCF `.xml`/`.mjcf` and
URDF are dynamics inputs. STL/OBJ/PLY are geometry-only; without a model,
body ownership, units and controller contract they must not become a walking
robot. STEP/IGES/USD, proprietary CAD and missing package mappings require an
explicit conversion/adapter. The pipeline does not silently guess these.

For a new robot, copy an example recipe and explicitly provide `model`,
optional `asset_dirs` or URDF `package_roots`, `surfaces`, `remesh`, initial
state, root body, support policy and an audited controller adapter. Each
surface references either a named geom or `{mesh, units, transform_to_body}`
and exactly one body. Multiple separate exterior shells may have the same
body owner if they do not intersect/contain each other; joints are never
bridged. Body reporting groups are independent of numeric model IDs.

`rft/geometry.py`, `rft/pipeline.py` and `rft/provider.py` are morphology
independent. The **walking controllers are intentionally not universal**:
`controllers/playground_onnx.py` contains two reviewed policy contracts, and
`rft/ground.py` currently routes these two families. Another robot needs its
own verified observation/action/controller adapter, not a renamed Go1 policy.
No generic module can infer a correct walking strategy from shape alone.

Prepare without simulation or reuse an existing verified bundle:

```powershell
conda run --no-capture-output -n lizard_rft python scripts/run_ground_pipeline.py --config configs/robots/playground_go1.yaml --prepare-only --output-dir outputs/ground_rft/go1_prepared
conda run --no-capture-output -n lizard_rft python scripts/run_ground_pipeline.py --config configs/robots/playground_go1.yaml --bundle-dir outputs/ground_rft/go1_prepared/bundle --duration 6
```

Output destinations must not already exist. Config or compiled-identity
changes require rebuilding/review, rather than silently reusing stale data.
Hashes detect stale/accidental modification; they are not a signature against
an adversary rewriting the manifest too. Source files remain read-only.

## Controller contracts and parameters

Both native deployments use 50 Hz and `target = default + 0.5 * action`.
Go1 observation is `[local linear velocity, gyro, gravity, joint offset,
joint velocity, previous action, command]` (48 floats).
G1 uses `[local linear velocity, gyro, gravity, command, joint offset,
joint velocity, previous action, cos(phase), sin(phase)]` (103 floats).
G1 phase starts `[0, pi]` and advances at 1.5 Hz. Commands in the tested
recipes are `[vx=0.4 m/s, vy=0, yaw_rate=0]` for Go1 and 0.2 m/s for the
screened G1 demo; its original 0.4 m/s recipe is a retained failure.

Joint names, actuator transmission order, gear, sensor dimensions, position
servo gains, ONNX input/output signatures and exact timestep divisibility are
checked. CPU inference uses one intra-op and one inter-op thread. Original
Go1 simulation is 4 ms; G1 is 2 ms. Matched rigid/sand runs use 0.5 ms, with
commands updated every 40 physics steps. Like the source callback, the first
policy call happens on the last pre-step state of the first control interval.
Actuator commands are retained between calls. Native actuator limiting remains
in MuJoCo, not replaced with undocumented controller clipping.

Original source, downloaded policies and models:

- [Official native Go1 controller](https://github.com/google-deepmind/mujoco_playground/blob/12ad7f3842791ced39f27e5ac96d066bbe361653/mujoco_playground/experimental/sim2sim/play_go1_joystick.py)
- [Official native G1 controller](https://github.com/google-deepmind/mujoco_playground/blob/12ad7f3842791ced39f27e5ac96d066bbe361653/mujoco_playground/experimental/sim2sim/play_g1_joystick.py)
- [Pinned Menagerie assets](https://github.com/google-deepmind/mujoco_menagerie/tree/f3b6013629c7b94f8ee93fd977b161fa8350f9bc)
- [MuJoCo native model editing](https://mujoco.readthedocs.io/en/3.9.0/python.html#model-editing)

## Products, extraction and interpretation

```text
outputs/ground_rft/run_.../
  pipeline.json                ordered stage status and failures
  resolved_config.json         exact requested settings
  inventory.json               compiled ownership, gains, damping, limits
  binding_checks.json          independent multi-pose position/velocity checks
  bundle/
    bundle.json + manifest.json
    <surface>/source.ply + prepared.ply + surface.npz
  rigid_upstream/ | rigid_matched/ | sand/
    summary.json + manifest.json
    trajectory.npz
    diagnostics.png
    locomotion.mp4              only with --video
  results.json + manifest.json
```

`trajectory.npz`: `trace` columns are named by `summary.json:trace_columns`;
`qpos`, `qvel`, `ctrl`, `qfrc_rft`, `active`, `rigid_active` share each pre-step
time. `groups` names component columns. `final_qpos`/`final_qvel` are stored
separately, avoiding the former generic demo's mixed pre/post alignment.
Contact is 1 when a component produces RFT force or touches native rigid
support; submerged-but-inactive is not the same as active force contact.
MP4 frames replay the saved states. `diagnostics.png` gives COM displacement,
root height, maximum centroid depth, RFT/rigid vertical force and 0/1 contact.

Use `rft.pipeline.verify_artifacts(run_directory)` to verify a run, then
`numpy.load(..., allow_pickle=False)` to extract arrays. Summaries deliberately
separate `passed_screen`, `failed_screen`, `rigid_reference_failed`,
`sand_transfer_failed`, `rejected`, and missing-metadata stops. The model is
not accepted merely because state arrays stayed finite. Height/uprightness/
depth violations sustained for 100 ms terminate with partial evidence; a
MuJoCo numerical warning terminates immediately, before its automatic state
reset can masquerade as a successful run.

Independent review decodes all frames, verifies immutable raw artifacts,
and derives tracking bias/contact duty without changing the source run:

```powershell
conda run --no-capture-output -n lizard_rft python scripts/review_ground_pipeline.py --runs outputs/ground_rft/run_... --output-dir outputs/ground_rft/review_...
```

Optional `--compare-runs OLD NEW` checks exact saved-array repeatability;
`--environment-before INVENTORY.json` reports package changes/additions.

## Scientific and coverage boundary

The examples select only original **foot collision proxies**, not an audited
whole-robot CAD exterior. Remesh deviation is measured against their initial
tessellations, not authoritative CAD or the exact analytic primitive. They
are useful pipeline/controller-transfer evidence within the tested upright
motion envelope, not proof of arbitrary falling/burrowing contact accuracy.
Coefficient 3.75 remains an uncalibrated material assumption. Smoothed RFT
power is recorded, not silently assumed passive for every robot/state.
Six seconds is a bounded screen, not long-horizon stability, mesh-resolution
convergence, real hardware performance or a universal deployment guarantee.

Implementation/failure ledger and measured results:
`GROUND_RFT_PIPELINE_2026-10-28.md` and
`regressions/2026-10-28-ground-rft/`.
