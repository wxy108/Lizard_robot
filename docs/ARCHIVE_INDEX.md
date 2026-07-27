# Archive index

Inventory date: 2026-07-27

The workspace archive is intentionally ignored by Git because it contains
large historical downloads and local tool artifacts. This tracked index
records what was moved, why, and how to recover active dependencies.

| Archive directory | Files | Bytes | Contents and status |
| --- | ---: | ---: | --- |
| `active-root-legacy-2026-07-27` | 20 | 2,319,131 | Pre-cleanup README/debug report, backup MJCF, MuJoCo log/marker, IDE settings, and unused `meshes_rft` variant. Reference only. |
| `failed-download-__Lizard_robot-main` | 4 | 652 | Cloud/download error placeholders. Reference only. |
| `reference-simplified-model` | 511 | 330,620,468 | Intact three-level historical project download with additional mesh variants and duplicated upstream clones. Not imported by the active runtime. |

Active code does not reference these archive paths. Do not copy files back
into the core project without a dependency check and a documented reason.

## Removed exact duplicate

The former
`Lizard_robot-main/models/RFT-SiM/RFT-SiM`
was removed rather than archived. Before removal it was verified as a clean
clone with:

- origin `https://github.com/Crab-Lab-CWRU/RFT-SiM.git`;
- branch `main`;
- HEAD `303283fae075cae4101ee3af102a36a4a5775998`;
- no tracked or untracked changes.

The retained checkout is pinned as
`Lizard_robot-main/third_party/RFT-SiM` at the same commit. It currently
contains 187 files totaling 74,702,487 bytes including its Git metadata.

## Generated output

`Lizard_robot-main/outputs/` remains local and ignored. Raw RFT regressions
are recoverable/identifiable through tracked configs, summaries, relative
paths, sizes, and SHA-256 values under `docs/regressions/`. The current mesh
baseline is `2026-07-27-rft-mesh-rebuild`; `2026-07-27-rft-6s` is the
historical pre-rebuild baseline.

The current reporting smoke baseline is
`docs/regressions/2026-07-27-video-matrix-smoke/`. It tracks compact manifests,
tables, diagrams, selected frames, and hashes. Its nine MP4 files and raw
analysis arrays remain local at
`outputs/video_matrix/release_smoke_ef27d2b/`.

The reviewed 6 s production MP4 set is the sole media exception to the
ignored-output policy. It is tracked under
`docs/media/video_matrix_production_6s/` for immediate viewing after clone.
Raw arrays and new experimental videos remain ignored under `outputs/`.
