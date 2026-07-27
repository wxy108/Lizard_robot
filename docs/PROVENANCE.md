# Source and data provenance

## Local projects

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
- Publication target: `https://github.com/wxy108/Lizard_robot`

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
  `Lizard_Robot_RFT/archive/reference-simplified-model/`. They are tracked,
  read-only reconstruction inputs in the original body coordinate frames.
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
active repository under `Lizard_Robot_RFT/archive/`. Their post-move file
counts and sizes are recorded in `docs/ARCHIVE_INDEX.md`.

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

## Granular coefficient

The current coefficient is `RFTCOEFF=3.75`, inherited from the upstream
Quikrete play-sand example. It is an uncalibrated research assumption for the
local experimental material, not a verified material constant.
