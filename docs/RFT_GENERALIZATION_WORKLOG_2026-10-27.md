# General RFT implementation ledger — 2026-10-27

## Identity and scope

- Canonical root: `Lizard_Robot_MuJoCo`.
- Branch: `agent/general-rft-stage1`.
- Preserved baseline: `ccc694677461012d488cb5ad7d7e9db945b74a80`.
- Plan: `RFT_GENERALIZATION_PLAN.md`.
- Runtime: existing `lizard_rft`; no IsaacLab access or changes.
- Work authorized: save plan; implement stage-one generic binding/provider;
  same-state legacy comparison; selected open-source model installation and
  integration screening; diagnose the suspected extra ground in video.

## S0 — inventory and freeze (completed)

Purpose: ensure later differences can be attributed and original behavior
remains recoverable. Clean canonical checkout and public GitHub HEAD both
resolved to `ccc6946`. Created the implementation branch without editing
legacy code or assets.

Frozen working-file SHA-256:

| Path | SHA-256 |
| --- | --- |
| `lizard_sand.py` | `9BED2344A1271057257B2B6C1E34DCB78B77D9D15F1EA4446450F25B04AF6C9F` |
| `sim_fxn_lib.py` | `903EF7AB8763719053E1691B46D7B22D751C40C2A99DBD13483724D83FF52C90` |
| `Lizard_Sand.xml` | `63BD0C702F2E1C43C4C9F52710D1D80DBFFE4675A887CC01CB999A1B5F1B399D` |
| `models/lizard.xml` | `642E600528D448B9A367ABA30EF18806EACCC6AC11263AF8736524B4FB29B8EC` |
| `configs/gait.yaml` | `25054AC1513077C5407C13286BA49E1FC83A4BCAF2137798B3994BF79FB81958` |
| `environment.yml` | `C40A4C29E97A76E73FCA6DE4E88AE40506CDD8FE4FD5A2CFB6C9A62596266F25` |

## S1 — plan saved (completed)

Purpose: turn the agreed architecture into an inspectable scope, staged
acceptance and stopping conditions before implementation. Output:
`docs/RFT_GENERALIZATION_PLAN.md`. It distinguishes compatibility, physical
correctness, external-model screening and the floor audit.

## Infrastructure observation I-001

The first direct Python import in `lizard_rft` failed at MuJoCo `_specs` with
`DLL load failed ... An Application Control policy has blocked this file`.
NumPy 1.26.4 imported successfully. An ordinary `conda run` import is being
checked independently. No system security policy or binary trust metadata was
changed. Runtime tests must not be reported as passed until executed.

## Pending steps

1. Implement reusable surface bindings, provider, explicit conventions and
   focused frame/provider tests.
2. Generate representative legacy snapshots and compare the same states and
   histories. Preserve raw arrays and compact difference summaries.
3. Inventory effective ground geoms and actual supporting contacts; perform
   a bounded matched safety-floor control where warranted.
4. Pin/install selected open-source robot models and optional loading tools;
   validate real model loading, binding, RFT and supported control demos.
5. Run canonical/full validation and reporting smoke, record results and
   commit focused source/evidence changes. Do not publish new media as a
   canonical release without documenting its status.
