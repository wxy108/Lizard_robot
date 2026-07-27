# 3×3 locomotion video matrix

## Purpose

The matrix provides directly comparable evidence for:

1. original detailed CAD locomotion on a rigid flat floor;
2. simplified external-envelope locomotion on RFT sand without visible force
   sites;
3. the exact same sand replay with all RFT triangle sites visible.

Each scenario is rendered from top, side, and 45-degree cameras, giving nine
videos.

## Generate

```powershell
conda activate lizard_rft
cd C:\Users\wxy22\Documents\Lizard_Robot_RFT\Lizard_robot-main
python scripts/generate_video_matrix.py --duration 6 --fps 30
```

Optional explicit target:

```powershell
python scripts/generate_video_matrix.py `
  --duration 6 --fps 30 `
  --width 960 --height 540 --panel-width 480 `
  --output-dir outputs\video_matrix\my_run
```

The output directory must not already exist. This prevents accidental
overwrites.

## Why only two simulations are run

The rigid model is simulated once. The RFT sand model is simulated once.
Saved qpos states are then replayed from three cameras. The hidden-site and
visible-site sand videos reuse the same sand state sequence.

Therefore differences between the two sand categories are visual only, not
different physics, random seeds, or integration histories.

## Camera matrix

| Output name | MuJoCo camera | Interpretation |
| --- | --- | --- |
| `top.mp4` | `track_top` | X–Y displacement and heading |
| `side.mp4` | `track_side` | vertical motion and penetration |
| `diag45.mp4` | `diag` | combined posture and displacement |

## Scenario matrix

| Folder | Geometry | Medium | RFT sites |
| --- | --- | --- | --- |
| `rigid_original` | original detailed CAD export | rigid z=0 floor | none |
| `sand_simplified` | topology-gated external envelope | RFT sand z=0 | hidden |
| `sand_simplified_sites` | same simplified envelope/replay | RFT sand z=0 | all visible |

## Frame overlays

### Center of mass

COM is a mass-weighted average of MuJoCo body inertial positions
`data.xipos[1:]`, excluding the world body.

- side view plots X–Z;
- top and 45-degree views plot X–Y;
- full trajectory is gray;
- elapsed trajectory is cyan;
- current COM is red;
- displacement is shown in millimetres.

### Sand penetration

For every component and step:

```text
max_penetration = max(0, sand_z - minimum component RFT-site z)
```

The panel displays the maximum across components and the current total active
RFT-triangle count. The sand visual plane is translucent so below-surface
geometry and force sites remain visible.

### Component contact diagram

Rows are always:

```text
Mid, Front, FR, FL, Back, HR, HL, Tail
```

For rigid ground, contact is 1 when a geom belonging to that component (or its
unnamed child body) has a MuJoCo contact with the rigid floor.

For RFT sand, contact is 1 when the component has at least one active RFT
triangle:

```text
active_triangles > 0
```

This is intentionally stricter than merely being below the sand surface:
submerged but non-leading/non-force-producing triangles are not marked as
active contact.

## Output tree

```text
outputs/video_matrix/run_.../
├── matrix_manifest.json
├── resolved_gait_config.yaml
├── videos/
│   ├── rigid_original/{top,side,diag45}.mp4
│   ├── sand_simplified/{top,side,diag45}.mp4
│   └── sand_simplified_sites/{top,side,diag45}.mp4
└── analysis/
    ├── rigid_original.npz
    ├── sand_rft.npz
    ├── rigid_original_com.csv
    ├── sand_rft_com.csv
    ├── rigid_original_contact_timeline.csv
    ├── sand_rft_contact_timeline.csv
    ├── sand_rft_penetration.csv
    ├── contact_events.csv
    ├── rigid_original_contact_diagram.png
    └── sand_rft_contact_diagram.png
```

After running the analyzer, the analysis folder also contains:

```text
derived_metrics.json
component_metrics.csv
```

## Manifest guarantees

`matrix_manifest.json` records:

- generator command;
- Git commit and dirty state;
- duration, FPS, dimensions, cameras, body order;
- contact definitions;
- scenario summaries;
- all nine relative video paths;
- every generated artifact's byte size and SHA-256.

Use a clean Git tree for canonical experiments.
