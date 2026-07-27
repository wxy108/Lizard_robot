# Human and agent handoff contract

This file applies to every change in this repository.

## Required read order

Before editing code or models, read:

1. `GUIDANCE.md`
2. `docs/IMPLEMENTATION_RECORD_2026-07-27.md`
3. `docs/PROJECT_STATUS.md`
4. `docs/CHANGELOG.md`
5. `docs/DECISIONS.md`
6. `docs/PROVENANCE.md`
7. the relevant source and raw output on disk

Historical reports are context, not truth. Live code, current configuration,
tests, and reproducible raw output take priority.

## Safety boundary

- Use only the `lizard_rft` Conda environment for this project.
- Do not install into, edit, rename, remove, or activate `isaaclab_50`.
- IsaacLab is not a dependency and is outside project scope.
- Do not treat `archive/` or `outputs/` as active source.
- Do not edit `third_party/RFT-SiM` in place. Pin a new upstream commit or
  document a deliberate fork.

## Canonical validation

From this directory:

```powershell
conda activate lizard_rft
python scripts/validate_project.py
```

For a behavior-changing RFT change, also run:

```powershell
python scripts/validate_project.py --full
```

Record material numerical changes in `docs/VALIDATION.md`; keep raw large
outputs under `outputs/` and record their relative path plus SHA-256 hash. For
reporting changes, also run the video smoke command in
`docs/TEST_AND_VALIDATION_GUIDE.md` and verify the generated manifest.
Only explicitly reviewed, hash-documented media sets under `docs/media/` may
be tracked as video; do not add arbitrary experimental MP4 files. Files under
`reference/rejected_meshes/` are diagnostic evidence only and must never be
used as active MuJoCo/RFT assets.

## Change tracking protocol

Every behavior or model change must include:

1. source/config changes;
2. a test or explicit reproducible validation command;
3. a dated entry in `docs/CHANGELOG.md`;
4. an update to `docs/PROJECT_STATUS.md` if the current state or next work
   changes;
5. a rationale in `docs/DECISIONS.md` when a convention, coordinate system,
   force sign, model boundary, contact definition, or research assumption
   changes;
6. one focused Git commit whose message explains the intent.

Do not label a gait, coefficient, mesh, or optimizer result as "best" without
an explicit metric, raw result path, configuration, and comparison set.

## Non-negotiable model invariants

- World axes: +X forward, +Y left, +Z up.
- RFT surface: `sand_height` at z = 0.
- Emergency rigid floor: z = -0.25 m.
- RFT raw force convention: body-on-sand; MuJoCo receives its negative.
- Rotation: use MuJoCo world-from-body `data.xmat`.
- Force application occurs before `mj_step`.
- Inactive triangle smoothing buffers are zeroed.
- Force sites are sequential per body, equal triangle count, visual group 5.
- `foot_depth` is signed; `sinkage = max(foot_depth, 0)`.
- Site-hidden and site-visible sand videos must replay the same recorded
  trajectory; visualization state must not alter simulation state.
- Video-derived comparisons must identify the manifest, exact config, raw
  output directory, and artifact hashes.

If any invariant must change, update the decision log and regression evidence
in the same commit.
