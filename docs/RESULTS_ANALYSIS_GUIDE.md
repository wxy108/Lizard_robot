# Results extraction and analysis

## Recommended entry point

```powershell
python scripts/analyze_video_matrix.py outputs\video_matrix\run_...
```

Hash verification is on by default. To inspect manually modified output:

```powershell
python scripts/analyze_video_matrix.py `
  outputs\video_matrix\run_... --skip-hash-verification
```

Do not use the skip option for canonical evidence.

## CSV files

### `*_com.csv`

Columns:

- `time_s`
- `com_x_m`, `com_y_m`, `com_z_m`
- `base_x_m`, `base_y_m`, `base_z_m`

COM is the robot's mass-weighted center of mass. Base position is the free
root joint position. They are not interchangeable.

### `*_contact_timeline.csv`

Columns:

- `time_s`
- `Mid`, `Front`, `FR`, `FL`, `Back`, `HR`, `HL`, `Tail`

Every component column is integer 0 or 1.

### `contact_events.csv`

Each row is one continuous contact interval:

- scenario;
- component;
- zero-based event index;
- start time;
- end time;
- duration.

### `sand_rft_penetration.csv`

For each component:

- maximum RFT-site penetration in millimetres;
- active RFT-triangle count.

## NPZ files

Load with:

```python
import numpy as np

data = np.load(
    "outputs/video_matrix/run_.../analysis/sand_rft.npz"
)
print(data.files)
```

Shared fields:

- `time`: simulation time;
- `com`: mass-weighted robot COM, shape `(steps, 3)`;
- `base_pos`: free-root position;
- `contact`: component binary contact, shape `(steps, 8)`;
- `body_order`: component labels;
- `render_steps`: indices used to render video frames.

Sand-only fields:

- `submerged_triangles`;
- `active_triangles`;
- `max_penetration_m`;
- `sand_surface_z`.

## Example: contact duty

```python
import numpy as np

data = np.load("outputs/video_matrix/run_.../analysis/sand_rft.npz")
for name, duty in zip(data["body_order"], data["contact"].mean(axis=0)):
    print(name, float(duty))
```

## Example: COM displacement and path

```python
import numpy as np

data = np.load("outputs/video_matrix/run_.../analysis/sand_rft.npz")
com = data["com"]
displacement = com[-1] - com[0]
path_length = np.linalg.norm(np.diff(com, axis=0), axis=1).sum()
print("displacement [m]", displacement)
print("path length [m]", float(path_length))
```

## Example: maximum penetration

```python
import numpy as np

data = np.load("outputs/video_matrix/run_.../analysis/sand_rft.npz")
maximum_mm = 1000 * data["max_penetration_m"].max(axis=0)
for name, value in zip(data["body_order"], maximum_mm):
    print(name, float(value))
```

## Interpretation cautions

- Rigid and sand contact use different physical definitions; compare timing
  patterns, not raw contact-force equivalence.
- The visible-site and hidden-site sand videos share one numerical replay.
- Force predictions remain uncalibrated until `RFTCOEFF` is fitted to the
  actual material.
- A clean mesh topology does not replace authoritative CAD surface review.
- Short smoke runs include settling transients and are not locomotion
  performance measurements.

## Canonical target directories

- local raw results: `outputs/`
- compact tracked evidence: `docs/regressions/`
- current video smoke evidence:
  `docs/regressions/2026-07-27-video-matrix-smoke/`
