# Rejected mesh references

This directory contains only the smallest historical artifacts needed to
reproduce documented failure diagnostics. Files here are never active MuJoCo
or RFT assets.

The normal runtime resolves meshes from `asset/`. Maintenance code must not
point `Lizard_Sand.xml` at this directory.

## `legacy_vertex_cluster/Back.STL`

- status: rejected historical output;
- purpose: reproduce the former vertex-clustering failure video;
- original workspace location:
  `Lizard_Robot_RFT/archive/active-root-legacy-2026-07-27/meshes_rft/Back.stl`;
- copied byte-for-byte on 2026-07-27;
- bytes: 94,884;
- SHA-256:
  `D94F85292A81E0ED183EA88D72D207D74AF44E366B93200C0F2FF7E2FF622460`.

Independent audit:

- 1,896 triangles / centroid force points;
- 13 connected components;
- not watertight;
- 9 non-manifold edges;
- 1,055 self-intersecting triangle pairs;
- triangle-area P95/P05 = 63.18;
- centroid-spacing P95/P05 = 9.68;
- 12.08% of triangles have a minimum angle below 10 degrees.

This artifact is preserved as evidence of why vertex clustering was rejected,
not as a candidate for reuse.
