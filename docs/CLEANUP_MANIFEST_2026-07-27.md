# Cleanup manifest — 2026-07-27

This is the pre-move audit record for the RFT workspace. It exists so later
humans and agents can distinguish active code from upstream dependencies,
historical downloads, and generated data.

## Scope and safety boundary

- In scope: `Lizard_Robot_RFT` and the two verified fixes already made in
  `Lizard_Robot_MuJoCo`.
- Explicitly out of scope: every IsaacLab project, file, Conda environment,
  and cache. No IsaacLab path is moved, edited, installed into, or removed.
- Historical material is archived rather than deleted unless it is a byte-for-
  byte redundant Git clone that can be recovered from the recorded upstream.

## Classification before cleanup

| Classification | Current path | Action and reason |
| --- | --- | --- |
| `KEEP_CORE` | `Lizard_robot-main/{lizard_sand.py,sim_fxn_lib.py,Lizard_Sand.xml,run.py}` | Keep at active project root; these are runtime entry points and model integration code. |
| `KEEP_CORE` | `Lizard_robot-main/{asset,models,configs,controllers,tests}` | Keep; active meshes, MJCF, configuration, controller, and regression tests. |
| `KEEP_LIVE_TOOL` | `remesh_even.py`, `rebuild_sites.py`, `models/meshes_count.py` | Move under `scripts/`; update root discovery and commands. |
| `KEEP_CORE` | `V1TailAssemblyURDF`, `V1TailAssemblyUSD` | Keep in place as CAD/export provenance. Moving them would break the conversion script and ROS package paths. |
| `REFERENCE_HISTORY` | `Lizard_Sand_backup.xml`, `MUJOCO_LOG.TXT`, file `3.9.0`, `.idea/` | Move out of the active attention path into the workspace archive. |
| `REFERENCE_HISTORY` | `models/meshes_rft/` | Archive; only the old triangle-count helper references it, while the current RFT model uses `asset/*.STL`. |
| `KEEP_LIVE_TOOL` | `models/RFT-SiM/` outer Git checkout | Move to `third_party/RFT-SiM` and register as a Git submodule at upstream commit `303283fae075cae4101ee3af102a36a4a5775998`. |
| `QUARANTINE_STALE_DANGEROUS` | `models/RFT-SiM/RFT-SiM/` | Remove after exact-state check: it is a second clean clone of the same URL and same commit nested inside the first checkout. Recoverable from `https://github.com/Crab-Lab-CWRU/RFT-SiM.git`. |
| `ARCHIVE_GENERATED_OUTPUT` | `outputs/` | Keep locally but Git-ignore. Copy the canonical 6 s regression summary/config plus the raw file hash into `docs/regressions/`. |
| `REFERENCE_HISTORY` | sibling `Lizard_robot-simplified-model-main/` | Move intact to `Lizard_Robot_RFT/archive/`; it is a three-level historical download with its own meshes and duplicated upstream clones, not the active RFT runtime. |
| `REFERENCE_HISTORY` | sibling `__Lizard_robot-main/` | Move intact to `Lizard_Robot_RFT/archive/`; it is a failed-download artifact. |

## Verified dependency facts

- The active runtime does not import code from `models/RFT-SiM`; the upstream
  checkout is retained for provenance and comparison.
- Both nested RFT-SiM repositories report branch `main`, clean tracked files,
  origin `https://github.com/Crab-Lab-CWRU/RFT-SiM.git`, and HEAD
  `303283fae075cae4101ee3af102a36a4a5775998`. The outer checkout reports only
  its nested duplicate as untracked.
- `models/meshes_rft` is referenced only by the legacy
  `models/meshes_count.py`; the active `Lizard_Sand.xml` meshes resolve under
  `asset/`.
- `scripts/convert_urdf.py` requires the current `V1TailAssemblyURDF` path, so
  CAD/export folders remain where they are.

## Expected post-cleanup layout

```text
Lizard_Robot_RFT/
├── README.md
├── archive/                       # preserved historical downloads/artifacts
└── Lizard_robot-main/             # active Git repository
    ├── AGENTS.md
    ├── README.md
    ├── docs/                      # status, provenance, validation, decisions
    ├── scripts/                   # maintenance and validation tools
    ├── third_party/RFT-SiM/       # pinned upstream Git submodule
    ├── outputs/                   # local generated runs, ignored by Git
    └── ...                        # runtime code, models, meshes, CAD sources
```

## Recovery

- All moves stay on the same volume and preserve file content.
- Archived files remain under `Lizard_Robot_RFT/archive`.
- The only planned removal is the exact redundant nested RFT-SiM clone. Its
  upstream URL and commit are recorded above and in `docs/PROVENANCE.md`.

## Execution result

Completed on 2026-07-27:

- all `KEEP_CORE` and `KEEP_LIVE_TOOL` items were retained;
- maintenance tools were moved under `scripts/` and made independent of the
  caller's working directory;
- historical downloads/artifacts were moved intact to `archive/`;
- generated output remained local and is now ignored by Git;
- the exact nested upstream duplicate was removed after the checks above;
- the retained upstream checkout was registered as a submodule;
- the audited workspace baseline was committed as
  `02960805b35f1acc091b03b3ffefca278410f960`.

See `ARCHIVE_INDEX.md` for the post-move inventory.
