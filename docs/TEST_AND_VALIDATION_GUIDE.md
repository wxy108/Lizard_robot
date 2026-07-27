# Test and validation guide

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
- exactly nine MP4 files exist;
- both NPZ files, all CSV files, diagrams, config, and manifest exist;
- `git.dirty` is false for canonical evidence;
- visible-site sand videos show the triangle sites;
- hidden-site sand videos do not;
- all eight contact rows fit in every video;
- side view shows the translucent sand surface/penetration;
- COM trail and time cursor update.

Verify and derive metrics:

```powershell
python scripts/analyze_video_matrix.py outputs\video_matrix\run_...
```

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
