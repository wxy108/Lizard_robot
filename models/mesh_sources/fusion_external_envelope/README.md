# Fusion external-envelope mesh source

Status: tracked, read-only reconstruction source; not an active MuJoCo asset.

These eight STL files were copied byte-for-byte on 2026-07-27 from:

`Lizard_Robot_RFT/archive/reference-simplified-model/Lizard_robot-main/Lizard_robot-main/asset`

They are manually remeshed Fusion 360 external-envelope candidates that were
translated back into the original CAD body frames. They are materially safer
than `models/meshes/*.STL` as an RFT starting point:

- one connected component per body instead of 13 overlapping CAD components;
- zero boundary and non-manifold edges;
- zero inconsistent-winding edges;
- 0–14 self-intersecting triangle pairs instead of roughly 7,600–8,500.

They are not used directly because six bodies still contain a small number of
self-intersections and triangle density is not sufficiently uniform.
`scripts/build_rft_mesh_candidates.py` treats these files as read-only input,
reconstructs a closed surface, applies isotropic remeshing, and writes gated
candidates under ignored `outputs/`. The exact accepted recipe is
`configs/rft_mesh_recipe.json`; the accepted manifest is tracked under
`docs/regressions/2026-07-27-rft-mesh-rebuild/`.

Do not point `Lizard_Sand.xml` at this folder directly.
