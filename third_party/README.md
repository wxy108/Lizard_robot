# Bundled RFT-SiM reference

`RFT-SiM/` contains the complete, unmodified upstream tree at commit
`303283fae075cae4101ee3af102a36a4a5775998`, including its example assets,
notebooks and two example videos. This is a regular tracked directory, so
ordinary Git clones and GitHub ZIP downloads include it. Git submodule
initialization and access to the upstream repository are no longer required.

Original repository: <https://github.com/Crab-Lab-CWRU/RFT-SiM>.
Copyright (c) 2026 Ryan Walker Brown. The original MIT License is preserved
verbatim in `RFT-SiM/README.md`. This distribution is a fixed reference
snapshot, not a claim of upstream ownership or new upstream development.

The original upstream example targets Python 3.12.10 / MuJoCo 3.3.8. The
current lizard integration uses the root-level `sim_fxn_lib.py` and the
`lizard_rft` environment in `environment.yml`. Bundling the reference does
not establish that the upstream crab examples run in the lizard environment.

`RFT-SiM.snapshot.json` records the original commit and tree, Git blob IDs,
byte sizes and SHA-256 for all 159 files. To verify a clone or extracted ZIP:

```bash
python scripts/verify_rft_snapshot.py
```

The verifier needs only the Python standard library. Git attributes disable
line-ending conversion for this directory to preserve upstream blob bytes.
Do not edit the snapshot; make lizard integration changes at the project
root. See decision D-015 and `docs/DEPLOYMENT_FIX_2026-10-08.md`.
