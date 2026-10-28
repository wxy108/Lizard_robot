# Per-robot motion, remesh and contact-proxy comparisons

## Request and fixed boundary

Produce one vertically paired motion video per ground robot (rigid above,
sand below), plus independent before/after remesh and RFT-centroid views,
including enlarged contact proxies. Start at `5219db9` on
`agent/ground-rft-pipeline`. Preserve all recorded trajectories, native model
geometry/dynamics, controllers, force code, environment and original lizard.
No physics simulation, policy tuning, remesh rebuild or package installation.
Use only `lizard_rft`; IsaacLab remains outside scope.

## Deliverable and acceptance

- Seven catalog robots: Go1, G1 slow, Berkeley, T1, Apollo, Burger and Waffle
  Pi. Add one motion-only G1 fast negative control when its saved run verifies.
- Top = matched-timestep rigid reference; bottom = recorded sand. Same output
  time axis, explicit physical row time, COM/progress, penetration and binary
  component contact. After an early stop, replace only the unavailable row
  with a labelled no-data card; never loop, stretch or freeze robot motion.
- One geometry video and poster per robot: whole-model contact location,
  enlarged original proxy mesh, remeshed mesh, sampled outward normals and
  before/after triangle-centroid clouds. Use common views/scales and an
  explicitly defined spacing/median color map. Report actual per-surface
  counts, 3D distribution ratios and saved bidirectional deviation.
- A separate enlarged PNG for **every selected surface**, even when several
  symmetric components share a representative geometry-video row. Distinct
  wheel/caster types must not be omitted. All proxies remain bound to their
  recorded named bodies and initial keyframe pose.
- New robots use already-valid collision primitives/proxies. Their coarse
  tessellation is not the invalid lizard CAD assembly from the reference
  screenshot. Label this as source discretization versus qualified remesh,
  not rejected CAD versus repaired whole robot; original visuals are unchanged.
- Verify source-run/bundle/scenario manifests, config/model identity, all
  input quadrature, output hashes and every video frame. Record source,
  simulation vs rendering provenance, exact commands and zero physics steps.
- Add known-answer reporting tests, run the canonical fast validator and
  prescribed legacy video smoke/analyzer; full six-second physics revalidation
  is not required for presentation-only changes.

## Bounded publication

At most 15 new MP4s: eight motion comparisons (including the G1 variant) and
seven geometry comparisons, plus posters/zoom PNGs, compact metrics and model
notices under `docs/media/robot_comparisons/`. Full working files stay under
`outputs/robot_comparisons/`. Preserve earlier media/manifests. Review first,
middle and last frames and all failure-stop transitions before publication.
Do not extend into new robot searches or experiments. Done means every
selected robot has a playable paired video and meaningful, traceable proxy
comparison; scientific failures remain failures.

## Ledger

C0: read repository contracts/current implementation and existing raw outputs.
The verified bundles already contain exact `source.ply`, `surface.npz`, and
before/after metrics: reuse them rather than rerunning remeshing. Motion rows
reuse native MP4/state records; Waffle Pi may need visual-only replay to remove
opaque collision overlays. Whole-model context is a display-only native pose.
Append actual changes, checks, output locations and remaining limits below.
