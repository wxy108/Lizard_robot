# Topology-gated RFT mesh rebuild

Date: 2026-07-27

## Accepted artifacts

- Recipe: `configs/rft_mesh_recipe.json`
- Recipe SHA-256:
  `01CA141C8736C82E6A50D7F3FEF4B63C5BB4C232A199E9247B95573F1D69F2AC`
- Full build evidence: `build_manifest.json`
- Build-manifest SHA-256:
  `1E328830B5473CABDF6C2C4A43DAF332361026A9A2E15E696AE01699EBFAEEC2`
- Local raw 6 s run:
  `outputs/sand/run_07_27_2026_161321_700369/results.npz`
- Raw-result SHA-256:
  `A3628E6536A1CE61582B97CDED9C1EC5EBF890519B48310B9F957D41F174DF70`
- Tracked summary SHA-256:
  `63E5728DE1DC5A680626A21743ECE3920FE4716DEA7641304001656A44743E9E`
- Tracked config SHA-256:
  `100014AD456F3BBE817BCFC103694E3437C10C90D7A003D05BC41764C5A97DF7`

## Mesh acceptance

Every row is one connected component, watertight, edge/vertex manifold,
orientable with positive signed volume, and has zero boundary edges,
non-manifold edges, inconsistent-winding edges, duplicate/degenerate faces,
and self-intersecting triangle pairs.

| Body | Triangles | Area P95/P05 | Centroid-NN P95/P05 | Slivers <10° | Candidate→source P95 |
| --- | ---: | ---: | ---: | ---: | ---: |
| Mid | 1,816 | 9.53 | 3.71 | 0.11% | 0.525 mm |
| Front | 2,278 | 7.49 | 2.86 | 0.26% | 0.563 mm |
| FR | 1,126 | 33.28 | 7.78 | 0.62% | 0.575 mm |
| FL | 1,360 | 12.06 | 4.63 | 0.22% | 0.546 mm |
| Back | 2,132 | 14.04 | 4.32 | 0.23% | 0.586 mm |
| HR | 2,638 | 10.76 | 4.46 | 0.11% | 0.390 mm |
| HL | 1,186 | 26.74 | 6.83 | 1.10% | 0.538 mm |
| Tail | 1,380 | 11.59 | 3.90 | 0.14% | 0.541 mm |
| **Total** | **13,916** |  |  |  |  |

The complete per-body source hashes, candidate hashes, bounding boxes, areas,
angles, topology counts, signed volumes, and bidirectional surface-deviation
statistics are in `build_manifest.json`.

## Runtime acceptance

Command:

```powershell
python scripts/validate_project.py --full
```

Observed:

- 3/3 RFT core tests passed;
- original rigid-floor 0.2 s smoke test passed;
- active force-site count exactly matched 13,916 mesh triangles;
- 6 s RFT run completed with 0.2177 m forward and -0.1208 m lateral
  displacement;
- peak total/site force: 86.629/0.600 N;
- RFT-active steps: 98.9%;
- RFT power range: -37.023 to 0 W, with zero positive-power steps;
- per-foot maximum sinkage: 5.46–6.66 mm;
- all 19 saved arrays finite;
- base z stayed at 0.01877–0.04000 m, above the -0.25 m safety floor.

Locally measured MuJoCo XML load time was 21.52 s. A rejected uniform
3.75 mm/30,020-site prototype took 99.53 s to load.

## Reproduction

```powershell
conda activate lizard_rft
python scripts/build_rft_mesh_candidates.py
python scripts/promote_rft_mesh_candidate.py outputs\mesh_candidates\run_...
python scripts/validate_project.py --full
```

The builder must print `PASS` for all eight bodies. Promotion verifies the
recipe hash and all candidate hashes before changing `asset/`.
