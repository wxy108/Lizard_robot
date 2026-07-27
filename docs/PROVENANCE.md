# Source and data provenance

## Local projects

### Canonical-workspace consolidation

On 2026-07-27, the original workstation was reduced to one active Git root:
`Lizard_Robot_MuJoCo`, connected to
`https://github.com/wxy108/Lizard_robot.git`.

The former outer integration repository and temporary publication checkout
were moved intact to:

`Lizard_Robot_Archive/2026-07-27/{Lizard_Robot_RFT,Lizard_Robot_GitHub_Publish}`.

The former outer repository had no remote and HEAD `c5fee1a368bb`; the
temporary publication checkout was at stale HEAD `9ef92cddc3f4`. They are
provenance only.

The canonical RFT-SiM submodule object database previously referenced the
outer workspace through a local Git alternate. Reachable objects were repacked
locally, the alternate was removed, `git fsck --full` passed, and submodule
HEAD remained `303283fae075cae4101ee3af102a36a4a5775998`. Fresh GitHub clones
were never dependent on this workstation-only alternate.

### Original rigid-floor MuJoCo project

- Path: `C:\Users\wxy22\Documents\Lizard_Robot_MuJoCo`
- Remote: `https://github.com/wxy108/Lizard_robot.git`
- Imported baseline: `79a2b905a46a9d139c5fd6bd5ed85a6b9dd25d30`
- Validated compatibility-fix commit:
  `6c848e90a0658fe8713f0dbca7876ed0465ac573`

The active RFT project carries matching `run.py` and
`controllers/gait_controller.py` behavior so one environment can run both.

### RFT integration

Before 2026-07-27, `Lizard_robot-main` had no top-level Git history. The first
workspace commit therefore records the best recoverable current baseline,
not a reconstruction of unknown earlier authorship. The debug report and
cleanup manifest state which changes were made during this audit.

- Workspace baseline commit:
  `02960805b35f1acc091b03b3ffefca278410f960`
- Topology-gated mesh rebuild commit: `bddb2f7`
- Video generator implementation commit:
  `ef27d2bfb82f2752ef341810aaf6300d9a094b0e`
- Imported documented local source snapshot: `48952e6`
- Publication target: `https://github.com/wxy108/Lizard_robot`

### Clean GitHub publication

The GitHub repository already used the project itself as its root, whereas the
audited local RFT history used `Lizard_robot-main/` beneath a workspace root.
To avoid publishing a nested project or unrelated-history merge, the final
tracked project snapshot was overlaid byte-for-byte onto the existing GitHub
history.

- Publication branch: `agent/rft-video-guidance`
- Existing GitHub base: `79a2b905a46a9d139c5fd6bd5ed85a6b9dd25d30`
- Original compatibility commit: `6c848e90a0658fe8713f0dbca7876ed0465ac573`
- Clean RFT integration commit: `ed5edc6`
- Imported local source snapshot: `48952e6`

The import compared SHA-256 for all 119 regular tracked project files and the
pinned submodule pointer. All matched the documented local source before the
publication commit. The former tracked `outputs/videos/*.mp4` files were
removed from the publication branch because generated outputs are now ignored
and reproducible under `outputs/`; their earlier Git history remains
recoverable.

## Upstream RFT-SiM

- Repository: `https://github.com/Crab-Lab-CWRU/RFT-SiM.git`
- Branch observed: `main`
- Pinned commit: `303283fae075cae4101ee3af102a36a4a5775998`
- Local path: `third_party/RFT-SiM`

The local vectorized implementation in `sim_fxn_lib.py` follows the upstream
body-on-sand force convention. The integration applies the negative reaction
force to the robot; see decision D-003.

During cleanup, two nested local clones were verified to have the same remote,
branch, commit, and clean tracked state. The redundant inner clone was removed.

## Model assets

- `V1TailAssemblyURDF/` and `V1TailAssemblyUSD/` preserve CAD/export sources.
- `models/meshes/*.STL` are assembly-export body meshes used by the rigid
  model. They contain overlapping/internal CAD components and are not valid
  direct RFT remeshing inputs.
- `models/mesh_sources/fusion_external_envelope/*.STL` are byte-for-byte
  copies of the manually unified Fusion meshes formerly stored under
  `Lizard_Robot_Archive/2026-07-27/Lizard_Robot_RFT/archive/reference-simplified-model/`.
  They are tracked, read-only reconstruction inputs in the original body
  coordinate frames.
- `asset/*.STL` are active, topology-gated RFT surfaces reconstructed from the
  Fusion external envelope.
- `models/lizard_sites.xml` and the force sites inside `Lizard_Sand.xml` are
  generated from `asset/*.STL`.

The accepted reconstruction uses:

- PyMeshLab 2025.7.post1 Screened Poisson reconstruction;
- PyMeshLab isotropic explicit remeshing;
- Open3D 0.19.0 for deterministic surface sampling, orientation, distance
  queries, and independent topology checks;
- recipe `configs/rft_mesh_recipe.json`, SHA-256
  `01CA141C8736C82E6A50D7F3FEF4B63C5BB4C232A199E9247B95573F1D69F2AC`;
- accepted build manifest
  `docs/regressions/2026-07-27-rft-mesh-rebuild/build_manifest.json`,
  SHA-256
  `1E328830B5473CABDF6C2C4A43DAF332361026A9A2E15E696AE01699EBFAEEC2`.

The manifest records source and active candidate hashes, tool versions,
effective parameters, topology metrics, distribution metrics, and
candidate-to-source deviations for every body.

Primary tool references:

- PyMeshLab package: `https://pypi.org/project/pymeshlab/`
- PyMeshLab filter reference:
  `https://pymeshlab.readthedocs.io/en/latest/filter_list.html`

Historical mesh variants and pre-cleanup artifacts are preserved outside the
active repository under
`Lizard_Robot_Archive/2026-07-27/Lizard_Robot_RFT/archive/`. Their post-move
file counts and sizes are recorded in `docs/ARCHIVE_INDEX.md`.

## Video and analysis artifacts

The video generator is first-party project code in
`scripts/generate_video_matrix.py`; the independent manifest verifier and
metric extractor is `scripts/analyze_video_matrix.py`. A run records:

- the source commit and dirty-state flag;
- the resolved gait configuration;
- scenario timing and camera/presentation choices;
- byte size and SHA-256 for every emitted artifact;
- raw COM, penetration, triangle activity, component contact, and contact
  count arrays.

The clean smoke run used commit `ef27d2b`, produced exactly nine MP4 files,
and is represented by tracked manifest, derived tables, diagrams, selected
frames, and hashes under
`docs/regressions/2026-07-27-video-matrix-smoke/`. The full MP4 and NPZ files
remain local under `outputs/video_matrix/release_smoke_ef27d2b/` according to
decision D-006.

### Curated production media

The clean production output generated from source commit `ff1dda2` is copied
without transcoding into `docs/media/video_matrix_production_6s/`: nine
individual view MP4 files and one master overview MP4. Their byte hashes match
the original ignored `outputs/video_matrix/production_6s_main_ff1dda2/`
artifacts. A 720×368 animated GIF is a derived README preview; a representative
full-resolution PNG is copied from the tracked regression evidence.

The media directory's `README.md` records every byte size and SHA-256. Raw
NPZ/CSV data remains represented by
`docs/regressions/2026-07-27-video-matrix-production-6s/`.

### Rejected-mesh diagnostic media

`scripts/generate_failed_mesh_videos.py` is first-party, geometry-only
diagnostic code. Its three production comparisons were generated from clean
commit `5c90d9118fc2211f5989dc99f0b63c99ea4a522b` and copied without
transcoding into `docs/media/failed_mesh_diagnostics/`.

Inputs:

- raw assembly Back: tracked `models/meshes/Back.STL`;
- legacy vertex-clustered Back: tracked diagnostic copy
  `reference/rejected_meshes/legacy_vertex_cluster/Back.STL`, copied
  byte-for-byte from
  `Lizard_Robot_Archive/2026-07-27/Lizard_Robot_RFT/archive/active-root-legacy-2026-07-27/meshes_rft/Back.stl`;
- fixed-count Fusion FR source:
  `models/mesh_sources/fusion_external_envelope/FR.STL`;
- accepted comparisons: `asset/Back.STL` and `asset/FR.STL`.

The legacy reference SHA-256 is
`D94F85292A81E0ED183EA88D72D207D74AF44E366B93200C0F2FF7E2FF622460`.
The generated manifest contains repository-relative paths, full independent
mesh audits, render parameters, artifact byte sizes, and SHA-256 values. No
rejected/source-only mesh enters the RFT solver.

### Historical invalid-RFT locomotion media

The archived outer workspace contained
`Lizard_robot-main/outputs/videos/legacy_sand.mp4`, the earlier full-robot
locomotion recording with visibly uneven force sites. The recording process
was interrupted before writing an MP4 `moov` atom:

- source bytes: 1,499,129;
- source SHA-256:
  `627F98081BBD6B988F1DF30973C9845E314C604D46D2CCBDC8DB0DBF8B3E54F1`;
- available `mdat` payload: 1,499,081 bytes;
- complete H.264 NAL units: 111;
- incomplete trailing data: 9,400 bytes.

The matching 39-byte Annex-B SPS/PPS sequence was extracted from the archived
same-batch `optimized.mp4` recording and preserved as
`reference/rejected_media/legacy_sand_h264_sps_pps.bin`, SHA-256
`71612059CF519E13130C1F1D717AAB90BAC3B63597380F456430AFFCB961F8F4`.

`scripts/recover_legacy_rft_video.py` restores only those complete historical
frames. The playable recovery is losslessly re-encoded from the decoded
historical frames into a valid MP4 container. The labelled zoom and contact
sheet are derived presentation copies. No MuJoCo state is resimulated, no
missing frames are invented, and none of these files is a valid physics
baseline.

## Granular coefficient

The current coefficient is `RFTCOEFF=3.75`, inherited from the upstream
Quikrete play-sand example. It is an uncalibrated research assumption for the
local experimental material, not a verified material constant.
