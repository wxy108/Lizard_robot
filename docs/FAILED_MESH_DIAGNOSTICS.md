# Failed RFT mesh diagnostic videos

## Purpose

These videos preserve the mesh failures that motivated the current
topology-gated reconstruction pipeline. They answer a narrow question:

> What was wrong with the former RFT point distributions and surfaces?

They do not simulate locomotion. Running known self-intersecting,
non-watertight meshes through the solver would create visually persuasive but
invalid force results. The generator therefore performs a read-only audit and
renders triangle-centroid point clouds beside the accepted mesh.

## Two complementary evidence sets

The project now preserves both forms of historical evidence:

1. [three controlled geometry comparisons](media/failed_mesh_diagnostics/README.md)
   isolate topology and point-spacing defects beside the accepted surface;
2. [the recovered whole-robot locomotion recording](media/legacy_incorrect_rft_locomotion/README.md)
   shows how the old uneven white force-site spheres looked on the moving
   lizard.

The second item is the former `legacy_sand.mp4`, not a newly simulated demo.
Its recording ended before the MP4 index was written, so the project preserved
the exact payload and recovered 110 complete frames. The
[zoomed labelled MP4](media/legacy_incorrect_rft_locomotion/legacy_incorrect_rft_locomotion_zoomed.mp4)
is the clearest view; the
[full-frame recovery](media/legacy_incorrect_rft_locomotion/legacy_incorrect_rft_locomotion_recovered.mp4)
keeps the original camera framing.

Use that video only to observe point distribution. The rejected mesh and
unknown historical integration state invalidate apparent penetration,
contact, displacement, force, and gait behavior.

## How to read the videos

- One dot = one triangle centroid = one RFT force site in the active pipeline.
- Blue/purple = locally dense centroid spacing.
- Green/yellow = near the median.
- Orange/red = locally sparse.
- Both sides use the same camera angle, body frame, center, and scale.
- The red card reports the rejected/source-only mesh.
- The green card reports the current active mesh.

The color value is:

```text
log2(nearest-centroid distance / median nearest-centroid distance)
```

It is clipped to the range 1/4× through 4× before applying the color map.
This makes both point clusters and holes visible without changing point
positions.

## Selected cases

Only three representative cases are included to keep the comparison bounded.

| Case | Body | Historical problem | Rejected/source-only | Current accepted |
| --- | --- | --- | ---: | ---: |
| A | Back | raw CAD assembly treated as one contact shell | 14,592 points, 13 components, 8,456 self-X pairs | 2,132 points, 1 component, 0 self-X |
| B | Back | vertex clustering applied to overlapping solids | 1,896 points, 13 components, 1,055 self-X pairs | 2,132 points, 1 component, 0 self-X |
| C | FR | same 1,500-face budget forced onto every body | 1,500 points, 6 self-X pairs, 12.33% slivers | 1,126 points, 0 self-X, 0.62% slivers |

### Case A — raw CAD assembly

The Back assembly export includes exterior skin, servos, screws, housings, and
other internal surfaces. Those surfaces are individually meaningful for CAD
but are not one granular contact envelope.

Measured change:

- connected components: 13 → 1;
- self-intersecting pairs: 8,456 → 0;
- triangle-area P95/P05: 1,185.36 → 14.04;
- centroid-spacing P95/P05: 39.12 → 4.32;
- sliver fraction below 10°: 30.46% → 0.23%.

This is the clearest example of the visibly uneven RFT-point distribution.

### Case B — legacy vertex clustering

The old remesher reduced triangle count but could not infer the union of
overlapping solids. It retained all 13 components and many intersections.
Decimation changed the symptom without fixing the contact-surface definition.

Measured change:

- connected components: 13 → 1;
- self-intersecting pairs: 1,055 → 0;
- triangle-area P95/P05: 63.18 → 14.04;
- centroid-spacing P95/P05: 9.68 → 4.32;
- sliver fraction below 10°: 12.08% → 0.23%.

The preserved reference mesh is
`reference/rejected_meshes/legacy_vertex_cluster/Back.STL`. Its hash and
archived source are recorded in the adjacent reference README.

### Case C — direct fixed-count Fusion source

The manually unified Fusion envelope is a much better reconstruction source,
but the direct source triangulation is not the active mesh. Forcing exactly
1,500 faces onto bodies with different size and curvature produces unequal
area distribution, and the FR source still contains six intersections.

Measured change:

- self-intersecting pairs: 6 → 0;
- triangle-area P95/P05: 112.78 → 33.28;
- centroid-spacing P95/P05: 8.23 → 7.78;
- sliver fraction below 10°: 12.33% → 0.62%.

The centroid-spacing improvement in FR is modest; the strong improvements are
topology, sliver removal, and triangle-area distribution. The accepted mesh is
not claimed to be perfectly uniform. It passes the documented project gates
and still requires review against authoritative CAD.

## Reproduce

Activate `lizard_rft`, then run:

```bash
python scripts/generate_failed_mesh_videos.py
```

Explicit production command:

```bash
python scripts/generate_failed_mesh_videos.py \
  --duration 4 --fps 24 --width 1280 --height 720 \
  --output-dir outputs/failed_mesh_videos/my_run
```

The destination must not exist. The generator writes:

```text
outputs/failed_mesh_videos/my_run/
|-- 01_raw_cad_assembly_vs_accepted.mp4
|-- 02_legacy_vertex_cluster_vs_accepted.mp4
|-- 03_fixed_count_fusion_vs_accepted.mp4
|-- failed_mesh_diagnostics_contact_sheet.png
|-- failed_mesh_diagnostics_preview.gif
`-- manifest.json
```

The manifest records:

- source commit and dirty state;
- exact input paths;
- full independent audit metrics for both sides;
- render parameters;
- artifact byte sizes and SHA-256 values.

The reviewed production copy is
`docs/media/failed_mesh_diagnostics/`.

Recover the historical full-robot recording separately:

```bash
python scripts/recover_legacy_rft_video.py
```

This command performs media recovery only; it never loads the rejected mesh
into MuJoCo or the RFT solver.

## Scientific boundary

The videos demonstrate geometric defects only. They do not prove that the
accepted mesh is the authoritative CAD envelope, nor do they validate sand
coefficients, gait quality, contact pressure, or hardware behavior.

Never use the files under `reference/rejected_meshes/` as active assets.
`Lizard_Sand.xml` must continue to resolve its RFT meshes from `asset/`.
