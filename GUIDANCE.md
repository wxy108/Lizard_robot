# Lizard Robot MuJoCo + RFT: complete deployment and operation guide

This is the authoritative start-to-finish handbook for a new human,
workstation, compute server, CI runner, or AI agent. It assumes no local paths,
no pre-existing Python environment, and no separate MuJoCo installation.

## 0. What this repository contains

Two locomotion workflows share one controller and one Conda environment:

| Workflow | Entry point | Geometry | Medium |
| --- | --- | --- | --- |
| Original | `run.py` | detailed CAD export | rigid flat floor |
| Granular RFT | `lizard_sand.py` | simplified external envelope | force-based sand at z = 0 |

The repository also contains:

- topology-gated active RFT surfaces in `asset/`;
- 13,916 triangle-centroid force sites;
- deterministic candidate build and hash-checked mesh promotion;
- nine-view video generation and a 3×3-plus-three-panel master compositor;
- raw-output analysis and artifact-hash verification;
- checked-in production MP4 files for immediate inspection;
- compact regression manifests and numerical evidence.

The canonical environment is `lizard_rft`. IsaacLab is outside project scope.

## 1. See the output before installing anything

Open these files directly on GitHub:

1. [animated master preview](docs/media/video_matrix_production_6s/video_matrix_overview_preview.gif);
2. [full master MP4](docs/media/video_matrix_production_6s/video_matrix_overview.mp4);
3. [all nine individual videos](docs/media/video_matrix_production_6s/README.md);
4. [production metrics and hashes](docs/regressions/2026-07-27-video-matrix-production-6s/MANIFEST.md).

Master layout:

```text
                           ANALYSIS
Original rigid   TOP | SIDE | 45° | COM + contact
RFT hidden       TOP | SIDE | 45° | COM + contact + penetration
RFT sites        TOP | SIDE | 45° | COM + contact + penetration
```

The two RFT rows replay the same physical trajectory. Force-site visibility is
only a presentation difference.

## 2. Fresh-system prerequisites

Required:

- 64-bit Windows, Linux, macOS, or WSL;
- Git;
- a working Conda distribution;
- enough network access to clone GitHub repositories and download Python
  wheels;
- approximately 4 GB free disk space for the environment and repository;
- OpenGL-capable graphics for interactive windows.

Recommended Conda distribution:
[Miniforge](https://github.com/conda-forge/miniforge).

Official Conda environment-file reference:
[conda environment creation](https://docs.conda.io/projects/conda/en/latest/commands/env/create.html).

Official MuJoCo Python installation reference:
[MuJoCo Python bindings](https://mujoco.readthedocs.io/en/stable/python.html).
The `mujoco` Python package includes the native MuJoCo library. Do not download
or configure another MuJoCo installation for this project.

### Confirm prerequisites

```bash
git --version
conda --version
```

If `conda` is not found:

- Windows: open “Miniforge Prompt” or “Anaconda PowerShell Prompt”;
- Linux/macOS: initialize the installed Conda executable for the shell, close
  and reopen the terminal, or source its `conda.sh`.

## 3. Clone correctly

Preferred:

```bash
git clone --recurse-submodules https://github.com/wxy108/Lizard_robot.git
cd Lizard_robot
```

If the repository was cloned without submodules:

```bash
git submodule update --init --recursive
```

Expected upstream submodule:

```text
third_party/RFT-SiM
commit 303283fae075cae4101ee3af102a36a4a5775998
```

The local integration does not modify the upstream checkout.

## 4. One-command deployment

### Windows PowerShell

From the repository root:

```powershell
powershell -ExecutionPolicy Bypass -File scripts\setup.ps1
conda activate lizard_rft
```

Options:

```powershell
# Synchronize an environment that already exists
powershell -ExecutionPolicy Bypass -File scripts\setup.ps1 -Update

# Install/import-check only; skip the project validator
powershell -ExecutionPolicy Bypass -File scripts\setup.ps1 -SkipValidation
```

### Linux, macOS, and WSL

```bash
bash scripts/setup.sh
conda activate lizard_rft
```

Options:

```bash
bash scripts/setup.sh --update
bash scripts/setup.sh --skip-validation
```

Both setup scripts:

1. initialize the pinned Git submodule;
2. create `lizard_rft` from `environment.yml` if missing;
3. optionally update an existing environment;
4. verify the major imports and versions;
5. run `scripts/validate_project.py`.

They do not install, activate, update, rename, or remove any IsaacLab
environment.

## 5. Manual deployment

Use this when automation scripts cannot be executed:

```bash
git submodule update --init --recursive
conda env create --file environment.yml
conda activate lizard_rft
python scripts/validate_project.py
```

To update an existing environment:

```bash
conda env update --name lizard_rft --file environment.yml --prune
conda activate lizard_rft
python scripts/validate_project.py
```

Installed top-level versions are pinned in `environment.yml`:

```text
Python 3.11
MuJoCo 3.9.0
NumPy 1.26.4
SciPy 1.17.1
PyYAML 6.0.3
Open3D 0.19.0
PyMeshLab 2025.7.post1
OpenCV 4.11.0.86
Matplotlib 3.11.1
imageio 2.37.4
imageio-ffmpeg 0.6.0
Optuna 4.9.0
```

Verify imports manually:

```bash
python -c "import mujoco, numpy, open3d, pymeshlab, cv2, imageio; print(mujoco.__version__, numpy.__version__, open3d.__version__)"
```

## 6. Expected first validation

Run:

```bash
python scripts/validate_project.py
```

Acceptance:

- 11/11 unit tests;
- eight active STLs are one-component, watertight, manifold, consistently
  oriented, positive-volume, and free of detected self-intersections;
- 13,916 force sites exactly match 13,916 active triangles;
- force sites are sequential and assigned to visual group 5;
- RFT sand surface is z = 0;
- emergency rigid floor is z = -0.25 m;
- original rigid-floor 0.2 s smoke completes;
- granular RFT 0.2 s smoke completes;
- active RFT power is non-positive.

Full behavior-changing validation:

```bash
python scripts/validate_project.py --full
```

This adds the complete 6 s RFT regression and is required after physics, mesh,
controller, or model changes.

## 7. Platform-specific rendering

### Windows

Normal interactive and headless commands use `python`.

### Linux desktop

Normal interactive commands use the available X11/GLX display.

### Linux server without a display

MuJoCo supports EGL for accelerated headless rendering and OSMesa for software
rendering. Start with:

```bash
export MUJOCO_GL=egl
python scripts/validate_project.py
python scripts/generate_video_matrix.py --duration 6 --fps 30
```

The setup shell detects Linux without `$DISPLAY` and sets `MUJOCO_GL=egl` for
its validation subprocess. The variable must be set again in a later shell
before rendering.

If EGL is unavailable, install the system's appropriate Mesa/EGL or GPU driver
packages. Do not change Python dependencies first; confirm the OpenGL backend.

### macOS

Headless commands use `python`. MuJoCo passive-viewer commands may require the
`mjpython` launcher installed with the MuJoCo wheel:

```bash
mjpython run.py
mjpython lizard_sand.py --view --duration 10
```

## 8. Run the original rigid-floor model

Interactive:

```bash
python run.py
```

Headless without saving:

```bash
python run.py --headless --duration 6 --no-save
```

Headless with output:

```bash
python run.py --headless --duration 6
```

Generated data is written below `outputs/data/`.

## 9. Run the latest RFT model

Interactive:

```bash
python lizard_sand.py --view --duration 10
```

All force sites visible:

```bash
python lizard_sand.py --view --duration 10 --show-force-sites all
```

One component visible:

```bash
python lizard_sand.py --view --duration 10 --show-force-sites FR
```

Valid names:

```text
Mid Front FR FL Back HR HL Tail
```

Headless with output:

```bash
python lizard_sand.py --duration 10
```

Headless without saving:

```bash
python lizard_sand.py --duration 0.2 --no-save
```

Generated data is written below `outputs/sand/`.

## 10. Generate the complete video result

Production:

```bash
python scripts/generate_video_matrix.py --duration 6 --fps 30
```

Explicit dimensions and destination:

```bash
python scripts/generate_video_matrix.py \
  --duration 6 --fps 30 \
  --width 960 --height 540 --panel-width 480 \
  --output-dir outputs/video_matrix/my_production_run
```

PowerShell line continuation uses a backtick instead of `\`:

```powershell
python scripts\generate_video_matrix.py `
  --duration 6 --fps 30 `
  --width 960 --height 540 --panel-width 480 `
  --output-dir outputs\video_matrix\my_production_run
```

The output directory must not exist. This is an intentional overwrite guard.

The generator runs only two physical simulations:

1. original rigid model;
2. simplified RFT model.

All cameras replay saved qpos states. RFT sites hidden/visible share the same
RFT state sequence.

### Video output layout

```text
outputs/video_matrix/run_.../
|-- matrix_manifest.json
|-- resolved_gait_config.yaml
|-- videos/
|   |-- rigid_original/{top,side,diag45}.mp4
|   |-- sand_simplified/{top,side,diag45}.mp4
|   |-- sand_simplified_sites/{top,side,diag45}.mp4
|   `-- video_matrix_overview.mp4
`-- analysis/
    |-- rigid_original.npz
    |-- sand_rft.npz
    |-- rigid_original_com.csv
    |-- sand_rft_com.csv
    |-- rigid_original_contact_timeline.csv
    |-- sand_rft_contact_timeline.csv
    |-- sand_rft_penetration.csv
    |-- contact_events.csv
    |-- rigid_original_contact_diagram.png
    `-- sand_rft_contact_diagram.png
```

Short pipeline smoke:

```bash
python scripts/generate_video_matrix.py \
  --duration 0.25 --fps 10 \
  --width 640 --height 480 --panel-width 420
```

This smoke is intentionally under one second and is not a locomotion result.

### Compose an older compatible run

```bash
python scripts/compose_video_matrix.py outputs/video_matrix/run_...
```

The compositor reads the nine MP4s, crops the repeated dashboards from the
3×3 view cells, and preserves one synchronized dashboard per scenario row. It
does not rerun physics.

## 11. Analyze and verify results

```bash
python scripts/analyze_video_matrix.py outputs/video_matrix/run_...
```

The analyzer first verifies artifact byte sizes and SHA-256 values from
`matrix_manifest.json`, then writes:

```text
analysis/derived_metrics.json
analysis/component_metrics.csv
```

Metrics:

- COM displacement vector;
- COM path length;
- mean COM speed;
- contact duty per component;
- contact event count;
- first contact and last release;
- mean and maximum event duration;
- maximum RFT penetration per component;
- active triangle statistics.

Detailed schemas and extraction examples:
`docs/RESULTS_ANALYSIS_GUIDE.md`.

## 12. Understand the visual diagnostics

### Center of mass

COM is mass-weighted from MuJoCo body inertial positions:

```text
data.xipos[1:] weighted by model.body_mass[1:]
```

The world body is excluded.

### Rigid contact

A named component is active when any geom belonging to that component or its
unnamed child body contacts the rigid floor.

### RFT contact

A named component is active when:

```text
active_triangles > 0
```

Being geometrically below z = 0 is not sufficient; the RFT triangle must be
force-producing.

### Penetration

For each component:

```text
max_penetration = max(0, sand_z - minimum component site z)
```

### Site visibility

RFT sites are MuJoCo visual group 5. Hiding them does not disable their force
application points.

## 13. Output and Git policy

Two output classes are deliberate:

### Checked-in curated media

`docs/media/video_matrix_production_6s/` contains the 10 canonical MP4 files,
an animated preview, a representative frame, and hashes. These files let a
new user see results immediately after cloning.

### Reproducible local raw output

`outputs/` contains new NPZ, CSV, optimizer, audit, candidate-mesh, and video
runs. It is ignored by Git because experiments can be large and numerous.

Promote a new canonical video set only when:

1. source Git state is clean;
2. the production command is recorded;
3. the analyzer verifies every manifest artifact;
4. relevant tests pass;
5. old curated media is replaced intentionally;
6. the new manifest, hashes, and reason are documented.

## 14. Rebuild and promote RFT meshes

Do not simplify the raw CAD assembly exports directly.

Candidate build:

```bash
python scripts/build_rft_mesh_candidates.py
```

Promotion:

```bash
python scripts/promote_rft_mesh_candidate.py \
  outputs/mesh_candidates/run_...
```

Required after promotion:

```bash
python scripts/validate_project.py --full
python scripts/generate_video_matrix.py --duration 6 --fps 30
```

The builder:

- reads `models/mesh_sources/fusion_external_envelope/*.STL`;
- reads `configs/rft_mesh_recipe.json`;
- writes only below ignored `outputs/mesh_candidates/`;
- uses deterministic single-thread Screened Poisson reconstruction;
- applies isotropic remeshing;
- audits topology, orientation, intersections, distribution, and deviation;
- records source, recipe, candidate, and tool hashes.

The promoter refuses partial, failed, recipe-mismatched, overridden, or
hash-mismatched builds.

## 15. Clean-room reproduction checklist

On a computer that has never run this project:

```bash
git clone --recurse-submodules https://github.com/wxy108/Lizard_robot.git
cd Lizard_robot
conda env create --file environment.yml
conda activate lizard_rft
python scripts/validate_project.py
python scripts/generate_video_matrix.py \
  --duration 6 --fps 30 \
  --output-dir outputs/video_matrix/reproduction_6s
python scripts/analyze_video_matrix.py \
  outputs/video_matrix/reproduction_6s
```

Then compare:

- configuration and body order;
- nine individual video paths;
- overview layout and frame count;
- metric trends;
- artifact hashes only when source commit, platform, codec, and dependency
  versions are identical.

H.264 byte hashes can differ across FFmpeg builds even when decoded frames and
physics data match. NPZ/config/metric comparisons are stronger cross-platform
physics checks than encoded-video bytes alone.

## 16. Troubleshooting

### `conda` is not recognized

Open a Conda/Miniforge-specific terminal or initialize Conda for the shell.
Confirm `conda --version` before running setup.

### Environment already exists

Reuse it:

```bash
conda activate lizard_rft
```

Or synchronize it:

```bash
conda env update --name lizard_rft --file environment.yml --prune
```

### Git submodule initialization fails

Confirm a complete Git installation and network access to GitHub. Manual
fallback:

```bash
git clone https://github.com/Crab-Lab-CWRU/RFT-SiM.git third_party/RFT-SiM
git -C third_party/RFT-SiM checkout 303283fae075cae4101ee3af102a36a4a5775998
```

### MuJoCo import fails

Confirm the correct environment:

```bash
conda activate lizard_rft
python -c "import mujoco; print(mujoco.__version__)"
```

Do not install a separate MuJoCo application. Recreate or update the Conda
environment instead.

### Interactive viewer fails on macOS

Use:

```bash
mjpython run.py
```

or:

```bash
mjpython lizard_sand.py --view --duration 10
```

### Rendering fails on a headless Linux machine

Try:

```bash
export MUJOCO_GL=egl
python scripts/generate_video_matrix.py --duration 0.25 --fps 10 \
  --width 640 --height 480 --panel-width 420
```

If EGL initialization fails, confirm host GPU/Mesa drivers. MuJoCo also
supports OSMesa software rendering, but it requires compatible system
libraries.

### Video appears to be one second

You generated the 0.25 s smoke command. Use:

```bash
python scripts/generate_video_matrix.py --duration 6 --fps 30
```

### Video codec error

Verify:

```bash
python -c "import imageio_ffmpeg; print(imageio_ffmpeg.get_ffmpeg_exe())"
```

Then synchronize `environment.yml`.

### XML load is slow

`Lizard_Sand.xml` contains 13,916 sites. First compilation can take tens of
seconds. This is expected; the accepted model is much faster than the rejected
30,020-site prototype.

### RFT sites are not visible

Use:

```bash
python lizard_sand.py --view --show-force-sites all
```

### No RFT force is produced

Confirm that `sand_height` is z = 0 and that the rigid emergency floor remains
z = -0.25 m. A rigid plane at the sand surface prevents penetration and masks
RFT.

## 17. Scientific boundary

Automated checks establish that the current active surfaces are closed,
manifold, consistently oriented, and free of detected self-intersections.
They do not establish that a manually unified external envelope is the exact
intended hardware contact surface.

Required research work:

1. compare the Fusion envelope and outward normals with authoritative CAD;
2. calibrate `RFTCOEFF=3.75` using the real granular material;
3. investigate initial impact transients;
4. define locomotion objectives before large gait searches;
5. compare the reconstructed duty law with original hardware logs/source.

Do not describe numerical regression values as experimentally calibrated.

## 18. Required reading before changes

1. `AGENTS.md`
2. `GUIDANCE.md`
3. `docs/PROJECT_STATUS.md`
4. `docs/CHANGELOG.md`
5. `docs/DECISIONS.md`
6. `docs/PROVENANCE.md`
7. `docs/VALIDATION.md`
8. `docs/IMPLEMENTATION_RECORD_2026-07-27.md`

Live source, configs, tests, checked manifests, and reproducible raw output
take priority over historical prose.
