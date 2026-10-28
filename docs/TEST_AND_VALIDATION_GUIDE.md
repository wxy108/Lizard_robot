# Test and validation guide

## Level 0: fresh-system deployment

Windows:

```powershell
powershell -ExecutionPolicy Bypass -File scripts\setup.ps1
```

Linux/macOS/WSL:

```bash
bash scripts/setup.sh
```

Acceptance:

- all 159 bundled upstream snapshot files match recorded hashes;
- `lizard_rft` is created or reused;
- all pinned top-level imports succeed;
- Level 1 and Level 2 below pass.

## Level 1: unit tests

```powershell
python -m unittest discover -s tests -v
```

### `tests/test_rft_core.py`

- no force above sand;
- upward/dissipative reaction during downward intrusion;
- direct MuJoCo rotation matrix matches the legacy Euler compatibility path.

### `tests/test_generic_rft.py`

Thirteen known-answer checks cover named ownership/multiple surfaces without
sites, invalid/immutable geometry, body-origin point velocity versus Jacobian,
relative sand-plane translation, intrusion reaction, untouched external force
and control, smoothing release/reset, triangle/wrench equivalence, primitive
closed winding, original-source versus compiled mesh transform/scale,
proxy units/pose, fixed-link URDF preservation and passive URDF RFT forces.
The preserved binding/provider stage brought the suite to 29 tests. Same-state commands
and open-source model demos are in `GENERAL_RFT_GUIDE.md`.

### `tests/test_ground_pipeline.py`

Twenty tests add known area/volume, topology/winding/multiple/intersection/
duplicate rejection, units, remesh budgets, verified bundle/model identity,
ownership, fixed-child/mocap support, explicit contact pairs and retained
self-pairs, native force sign, virtual assets and controller contracts.
This ground-pipeline stage contributed to the original 49 tests. Standalone official-policy parity and native
three-scenario six-second screens are in `GROUND_RFT_GUIDE.md`; they require
optional pinned assets/inference packages and are not unit-test downloads.

### Catalog formats and comparison reporting

`tests/test_catalog_formats.py` adds 20 format/package/import/safety checks;
`tests/test_robot_comparisons.py` adds 12 mesh-quadrature/spacing/camera,
frame/timeline/layout and no-step/no-remesh reporting checks. Current suite:
**81 tests** with optional Xacro installed; missing optional Xacro causes its
explicitly marked tests to skip. No robot-asset download occurs in unit tests.

New per-robot media readback:

```powershell
python scripts/generate_robot_comparisons.py --verify docs/media/robot_comparisons
```

It verifies all published hashes, every movie frame, every component zoom,
stop semantics and available source state hashes. Current D-022 release:
seven robots, 15 movies, 1,423 frames and 19 independent component enlargements.

### `tests/test_video_matrix.py`

- binary contact intervals start/end correctly;
- a contact still active at run end is closed at `last_time + dt`;
- unnamed child geoms map to the nearest named component;
- video sampling contains the first and final state;
- dashboard output dimensions and content are valid;
- contact duty, penetration, and active-triangle metrics are correct;
- COM displacement/path metrics are correct.

### `tests/test_failed_mesh_videos.py`

- the three selected rejected/accepted inputs exist inside the repository;
- the rotating centroid projection returns finite coordinates inside the
  requested frame.

### `tests/test_legacy_rft_video_recovery.py`

- Annex-B SPS/PPS headers split correctly;
- truncated MP4 `mdat` parsing keeps complete length-prefixed NAL units only;
- the tracked source yields exactly 111 complete units and a 9,400-byte
  incomplete tail.

## Level 2: full fast project check

```powershell
python scripts/validate_project.py
```

This adds:

- active STL topology, orientation, self-intersection, and distribution gates;
- exact active triangle/force-site correspondence;
- sand surface and emergency-floor invariants;
- original rigid-floor 0.2 s smoke;
- RFT sand 0.2 s smoke.

## Level 3: video rendering smoke

```powershell
python scripts/generate_video_matrix.py `
  --duration 0.25 --fps 10 `
  --width 640 --height 480 --panel-width 420
```

Acceptance:

- process exits zero;
- exactly nine individual MP4 files and one overview MP4 exist;
- both NPZ files, all CSV files, diagrams, config, and manifest exist;
- `git.dirty` is false for canonical evidence;
- visible-site sand videos show the triangle sites;
- hidden-site sand videos do not;
- all eight contact rows fit in every video;
- the overview contains three scenario rows, three pure-view columns, and
  exactly one analysis panel per row;
- side view shows the translucent sand surface/penetration;
- COM trail and time cursor update.

Verify and derive metrics:

```powershell
python scripts/analyze_video_matrix.py outputs\video_matrix\run_...
```

### Rejected-mesh diagnostic smoke

```powershell
python scripts\generate_failed_mesh_videos.py `
  --duration 0.25 --fps 4 --width 960 --height 540
```

Acceptance:

- three MP4 files, one contact sheet, one GIF, and one manifest exist;
- every case audits both the rejected/source-only and accepted input;
- manifest paths are repository-relative;
- all artifact hashes match;
- rejected meshes are not loaded into MuJoCo or the RFT solver.

### Historical invalid-RFT video recovery

```powershell
python scripts\recover_legacy_rft_video.py
```

Acceptance:

- recovery reports `git.dirty=false` for canonical evidence;
- source and header SHA-256 values match the tracked references;
- exactly 110 frames decode from both MP4 files at 30 FPS and 1280×720;
- recovered and zoomed frame counts match;
- the contact sheet contains six frames;
- the zoom is explicitly labelled as invalid historical evidence;
- `resimulation=false` is recorded in `manifest.json`.

## Level 4: behavior-changing regression

```powershell
python scripts/validate_project.py --full
```

This runs the complete 6 s RFT integration. Record material changes in
`docs/VALIDATION.md` and place raw outputs under `outputs/`.

## Level 5: mesh rebuild

```powershell
python scripts/build_rft_mesh_candidates.py
```

All eight candidates must pass. Never promote a partial run. Promotion and a
full validation are separate required steps.
