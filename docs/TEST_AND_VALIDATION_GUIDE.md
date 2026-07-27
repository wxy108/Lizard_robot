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

- pinned submodule is initialized;
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
