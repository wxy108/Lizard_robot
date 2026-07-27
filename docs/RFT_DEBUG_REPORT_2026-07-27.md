# Lizard MuJoCo + RFT debug report — 2026-07-27

## Scope

This audit covered the original rigid-floor MuJoCo runner and the latest
triangle-level RFT integration. Both were verified in the shared `lizard_rft`
Conda environment. IsaacLab was explicitly excluded and untouched.

## Corrected defects

| Problem | Consequence | Correction |
| --- | --- | --- |
| Rigid floor at z = 0 while RFT surface was z = -0.05 | Rigid contact prevented RFT penetration | Sand/RFT surface at z = 0; emergency floor at z = -0.25 |
| Missing reaction-force minus sign | Downward acceleration, >90 kN force, numerical failure | Apply equal-and-opposite sand-on-body force |
| Roll/pitch/yaw passed to a yaw/pitch/roll routine | Wrong triangle positions, normals, and force directions | Use MuJoCo `data.xmat` |
| RFT computed after `mj_step` | One-step force delay | Compute/apply before integration |
| Smoothed force persisted for inactive faces | Nonphysical force above sand | Zero inactive smoothing entries |
| Missing/misaligned sites could silently use wrong IDs | Force applied at wrong points | Exact names and per-body site/triangle equality |
| Negative clearance labeled “sinkage” | Misleading saved data | Signed `foot_depth`; clipped nonnegative `sinkage` |
| No force/activity diagnostics | Stable-looking run could have inactive RFT | Log force, power, peaks, submerged/active triangles |
| >10,000 visible sites exceeded viewer capacity | Warnings and missing visuals | Hidden group 5; selective site display |
| Remeshing shifted CAD centers | Mesh/joint/inertia misalignment | Translate simplified mesh back to original center |
| Maintenance scripts used wrong working directory | Rebuilds failed outside one directory | Resolve from script/project path |
| Original short runs divided by near-zero walking time | Huge false speed | Measure only post-settle samples; otherwise zero |
| Endpoint yaw was wrapped | False 360-degree drift | Unwrap yaw |
| Leg order came from a set | Nondeterministic controller ordering | Preserve YAML order and validate membership |
| Second-resolution run names collided | Outputs could overwrite | Microsecond timestamps |

## Regression evidence

### Rigid floor

- 0.2 s pre-settle run: 0 mm/s.
- 3.0 s run: 0.0596 m forward; no fall.

### RFT core

Three tests verify zero force above sand, dissipative upward reaction during
downward intrusion, and rotation-matrix consistency.

### Full 6 s RFT integration

- forward: 0.2423 m
- lateral: -0.0808 m
- maximum tilt: 2.409 degrees
- maximum foot sinkage: 5.35–6.62 mm
- peak total/site force: 80.027/13.974 N
- active steps: 99.008%
- positive-power steps: 0%
- all saved arrays finite
- emergency floor never reached

See `docs/VALIDATION.md` for the exact config, result path, and hash.

## P0 mesh defect resolution

The original assembly-export meshes were not valid single contact shells:
seven bodies contained 13 overlapping components and roughly 7,600–8,500
self-intersecting triangle pairs. The former vertex-clustering remesher merged
nearby surfaces across those components, leaving every active mesh
non-watertight and retaining 12–936 self-intersection pairs per body.

The replacement pipeline uses the manually unified Fusion external envelope,
Screened Poisson reconstruction, isotropic remeshing, deterministic
single-thread execution, and independent hard gates. The accepted 13,916-site
model has:

- one connected component per body;
- 8/8 watertight and consistently oriented surfaces;
- zero boundary/non-manifold/inconsistent-winding edges;
- zero duplicate, degenerate, or self-intersecting faces;
- candidate-to-source P95 deviations of 0.390–0.586 mm.

The current 6 s regression completed with 0.2177 m forward displacement,
-0.1208 m lateral displacement, 86.629 N peak total force, 0.600 N peak site
force, 98.9% active steps, and zero positive-power steps.

See `docs/regressions/2026-07-27-rft-mesh-rebuild/` for the recipe-linked
manifest, exact hashes, summary, and configuration.

## Remaining research risks

1. The automated topology defect is resolved, but the selected external
   envelope and normals still require side-by-side review against the
   authoritative CAD.
2. The coefficient 3.75 requires calibration for the actual sand.
3. The rigid-floor gait has about 8 cm lateral drift on sand.
4. The initial 80 N impact transient requires physical validation.
5. The hardware duty function is reconstructed because its original helper
   was not supplied.
