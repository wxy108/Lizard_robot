# Seven-robot catalog evidence

Clean physics source `36659ca`; final CLI/render/package source in `source_index.json`.

| Robot | Rigid dx upstream/matched (m) | Sand duration / dx (m) | Sand result |
| --- | --- | --- | --- |
| go1 | 2.185 / 2.142 | 6.0000 s / 2.104190 | passed_screen: completed |
| g1 | 1.796 / 1.786 | 6.0000 s / 1.860660 | passed_screen: completed |
| berkeley | 0.921 / 0.932 | 3.4975 s / 0.824411 | failed_screen: outside_declared_locomotion_envelope_for_100ms |
| t1 | 1.143 / 1.150 | 6.0000 s / 1.162309 | passed_screen: completed |
| apollo | 1.033 / 0.971 | 2.3250 s / -0.878359 | failed_screen: outside_declared_locomotion_envelope_for_100ms |
| turtlebot3_burger | 0.463 / 0.469 | 6.0000 s / 0.003136 | failed_screen: completed |
| turtlebot3_waffle_pi | 0.464 / 0.469 | 6.0000 s / 0.002525 | failed_screen: completed |

All 14 rigid scenarios pass; 3/7 sand screens pass. Failed transfers remain evidence. All 21 raw videos (2,417 frames) and all raw/cache manifests were verified. Go1/G1 saved arrays equal the earlier f4c0360 runs exactly. Only Xacro 2.1.1 was added; no existing package changed.

69 tests pass. Full original lizard six-second regression passed. Three new official policies match 30-state observation/action probes exactly; T1 physical-qvel mutation is intentionally removed. Version-locked binary packages preserve model fingerprints and active force/control probes; native MJZ serialization retains the documented small numeric residual.

`review.json` includes original summaries, source/settings, hashes, binding gates, contacts, speed biases and baseline array comparisons. `package_review.json` checks actual relocated six-second G1 simulation; `format_comparison.json` separates bitwise fingerprints from force probes. `*_render.json` records visual-only replay (zero physics steps). Logs are LF-normalized copies; original/copy hashes are in `publication.json`.

Raw: `outputs/robot_catalog/clean_6s_36659ca/`; keep these local. Reproduce using the catalog guide and reviewer/curator scripts, not workstation-specific absolute evidence paths. Component contact, COM and penetration diagrams are included here; [reviewed videos](../../media/robot_catalog/README.md). Short screens and failures are not material calibration or universal locomotion guarantees.
