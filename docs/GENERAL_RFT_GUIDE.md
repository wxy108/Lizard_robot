# General RFT stage one: use, boundaries and reproduction

Start with `RFT_GENERALIZATION_PLAN.md` for architecture and
`RFT_GENERALIZATION_WORKLOG_2026-10-27.md` for the dated implementation,
failures, exact outputs and measured differences. The preserved baseline is
Git commit `ccc6946`; the original lizard entry points/assets are unchanged.

## What is implemented

| Component | File | Responsibility |
| --- | --- | --- |
| Model import and binding | `rft/binding.py` | MJCF/URDF load, explicit units/poses, named owners, compiled mesh/primitive geometry |
| Numerical law | `rft/kernel.py` | NumPy-only frozen triangle law, original license in `rft/NOTICE` |
| Force provider | `rft/provider.py` | Independent per-instance history, versioned frames/depth, triangle forces and private generalized forces |
| Controller adapter | `controllers/generic_controller.py` | Independent actuator-command hold/sine, no RFT imports |
| Example runner | `scripts/run_generic_robot.py` | Resolve config, initialize, call controller/provider, step and record/render |
| Compatibility/floor audit | `scripts/validate_general_rft.py` | Same-state oracle comparison, bounded legacy trajectories, contact inventory, floor-disabled control |
| Output review | `scripts/review_general_rft_outputs.py` | Verify hashes, decode video frames and render saved-state floor evidence |

The generic provider requires forward-synchronized `mjData`. It does not step
or mutate `ctrl`, `qpos`, `qvel`, `qfrc_applied` or another external force.
Its returned buffer is assembled by the caller. It supports unequal triangle
counts, arbitrary named bodies and multiple surfaces per body; no force sites
are required. Model-specific controller selection remains outside RFT.

This stage does not automatically infer a physically exposed CAD envelope,
run generic reconstruction for every new robot, calibrate material, synthesize
a walking policy, or validate arbitrary terrains/deformable bodies. Existing
lizard reconstruction remains available through its original gated workflow.

## Environment

Use only `lizard_rft`. In this workstation session direct Python initially
encountered a Windows DLL/Application-Control error, while the normal Conda
launch succeeded; commands below use that verified launch path. No security
settings or IsaacLab environment were changed.

Existing numerical pins remain in `environment.yml`. Optional registry tools
are isolated in `requirements-robots.txt` and installed without dependency
upgrades:

```powershell
conda run --no-capture-output -n lizard_rft python -m pip install --no-deps -r requirements-robots.txt
conda run --no-capture-output -n lizard_rft python -m pip check
```

The registry is optional: the demo route uses explicit pinned local assets,
not the registry's automatic Git clone/cache behavior.

## Install and run the two tested open-source model examples

From the repository root:

```powershell
conda run --no-capture-output -n lizard_rft python scripts/install_robot_examples.py
conda run --no-capture-output -n lizard_rft python scripts/run_generic_robot.py --config configs/robots/menagerie_go1.yaml --duration 2 --video --show-points
conda run --no-capture-output -n lizard_rft python scripts/run_generic_robot.py --config configs/robots/menagerie_panda.yaml --duration 2 --video --show-points
```

The installer fetches only 91 files / 48,615,728 bytes for the selected models
at the commit in `configs/robot_library_lock.json`, validates every Git blob,
retains model licenses, and writes a SHA-256 download manifest under ignored
`third_party/robot_assets/mujoco_menagerie/`. Rerunning verifies and reuses
identical local files; a changed existing asset is refused rather than
overwritten. Source model files are not edited.

- Go1: original foot-sphere collision proxies, four surfaces / 896 triangles;
  holds upstream home actuator commands while RFT supports the floating robot.
- Panda: selected link7 mesh / 200 triangles; fixed-base arm, one actuator
  receives a 0.05 rad, 0.5 Hz sine. The diagnostic horizontal sand plane is
  15 mm above that surface's initial lowest centroid, explicitly configured.

Both are **integration demos**, not optimized walking policies or approved
whole-robot contact envelopes. Only selected parts receive RFT. The unchanged
upstream collision model can be an approximation of the hardware exterior.
Two seconds establishes a bounded loading/force/stepping/rendering screen,
not long-horizon stability or experimental accuracy.

The reviewed clean-source MP4s and previews are directly available under
[`docs/media/general_rft_stage1/`](media/general_rft_stage1/README.md), so
readers can inspect the actual output before installing optional assets.

Every run prints its fresh `outputs/general_rft/<run>/` directory containing
`trajectory.npz`, `summary.json`, `manifest.json`, and, with `--video`,
`robot_rft.mp4`. New output directories are never overwritten. Review with:

```powershell
conda run --no-capture-output -n lizard_rft python scripts/review_general_rft_outputs.py outputs/general_rft/<run>
```

The review verifies recorded hashes and decodes the first/middle/last frames;
it adds `video_contact_sheet.png` and `review.json`, then updates the manifest.
Forty-one frames at 20 FPS cover the two-second run including its final frame
(2.05 encoded seconds). Point visibility changes rendering only.

## Configure another robot

Make a robot config under `configs/robots/`. Its `model` and mesh paths are
relative to the config file. Identify selected contact surfaces explicitly:

```yaml
schema_version: 1
model: ../../third_party/robot_assets/my_robot/model.xml
initial_keyframe: home
convention: body_frame_v1
coefficient: 3.75
alpha: 0.5
application: body_wrench
sand_height: 0.0
controller: {type: hold_keyframe}
surfaces:
  - id: left_foot_contact
    body: left_foot_link
    mesh: rft/left_foot.stl
    units: mm
    transform_to_body:
      pos_m: [0, 0, 0]
      quat_wxyz: [1, 0, 0, 0]
```

Alternatively select a named compiled geom with
`{id: left_foot_contact, geom: left_foot_geom, body: left_foot_link}`. A missing
name or a geom on the wrong body fails immediately. Explicit
`select_collision_geoms` supports selected named bodies / geom groups for
discovery screens, but does not certify union/exposure or remove overlaps.
Do not select visual and collision copies of the same surface together.

The tested configs use robot-only MJCF, not an upstream scene with a rigid
ground plane. The provider intentionally preserves all original rigid-contact
settings; arbitrary scene/terrain conversion and automatic ground filtering
are not implemented in stage one. Review any added scene ground explicitly
before treating that robot as an RFT-only sand experiment.

The model uses SI conventions. Proxy `units` must explicitly be `m` or `mm`;
their transform is applied before centroid/normal/area calculation. Compiled
mesh vertices use compiled geom poses as the matching coordinate route;
source asset scale/centering is not reapplied. A moving-joint hierarchy is
preserved by binding each triangle to its own physical body. Reporting groups
do not merge physics owners.

URDF is supported through the importer, with fixed root retained and fixed
links/visuals preserved. Supply `package_roots` mappings for `package://`
assets. There is no automatic actuator or floating-base synthesis. A known-
answer passive URDF test exercises real RFT generalized forces without any
actuators or sites. The two real downloaded examples above are MJCF; a
production external URDF robot/gait has not yet been validated.

## Conventions and regression

`legacy_lizard_v1` preserves the original velocity/depth/sanitization/smoothing
contract for matching the frozen lizard. `body_frame_v1` uses the regular
body-origin point velocity and depth relative to the plane; it is the explicit
choice in the new example configs. At z=0 the sampled difference in forces
can still be substantial because the legacy linear velocity refers to COM
while its lever arm uses the body origin. Do not silently switch the legacy
runner and expect the old trajectory to remain unchanged.

```powershell
conda run --no-capture-output -n lizard_rft python -m unittest discover -s tests -v
conda run --no-capture-output -n lizard_rft python scripts/validate_general_rft.py --duration 6
conda run --no-capture-output -n lizard_rft python scripts/validate_project.py --full
```

The new audit uses one legacy trajectory and representative same-state
fixtures; BOTH force evaluators receive forward-synchronized copies and the
same preceding legacy smoothing history. It checks positions, exact masks,
raw/smoothed forces and generalized forces. Corrected-convention results are
recorded separately, not adopted as a new lizard baseline.

A final-source replay can check frozen arrays without another trajectory:

```powershell
conda run --no-capture-output -n lizard_rft python scripts/validate_general_rft.py --replay-run outputs/general_rft/stage1_full_6s_20261027
```

This reconstructs active smoothing history algebraically from recorded raw
and smoothed oracle arrays. It is a regression replay, not an independent
physical oracle or a newly simulated gait.

## Ground/sinkage result

The tested six-second sand run had zero rigid-ground support. Its safety
floor is at z=-0.25 m; the lowest RFT centroid reached -9.311 mm. Disabling
that floor produced exactly the same 12,000 qpos states. The original rigid-
ground scenario has its intentional floor at z=0 and about 0.575 mm maximum
soft-contact penetration. These are different scenarios in the video matrix.

The translucent sand geom and thin `sand_height` visualization site can make
the lower plane/background visible. They are not extra supporting ground.
`same_pose_floor_visibility.png` compares the SAME recorded pose with only
the safety-floor visibility changed. The original safety floor and published
videos remain unchanged. Exact telemetry and the plot are in the dated
regression evidence and raw output paths in the worklog.

## Upstream sources

- Menagerie: https://github.com/google-deepmind/mujoco_menagerie
  (commit fixed by the lock file; individual model licenses apply).
- Optional registry: https://github.com/robot-descriptions/robot_descriptions.py
  (`robot_descriptions==3.3.0`; stock automatic cache/clone route not validated here).
- MuJoCo mesh frames: https://mujoco.readthedocs.io/en/stable/XMLreference.html#asset-mesh
- Cartesian force projection: https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-applyft
