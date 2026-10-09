# GitHub download and deployment repair — 2026-10-08

## Cause and historical evidence

At original publication, the parent Git repository stored
`third_party/RFT-SiM` as a mode-160000 gitlink at upstream commit
`303283fae075cae4101ee3af102a36a4a5775998`, with the URL
`https://github.com/Crab-Lab-CWRU/RFT-SiM.git` in `.gitmodules`.
The workstation had the expanded contents, but the parent repository did not
contain those files. GitHub ZIP downloads excluded the submodule contents.

On 2026-10-08, `git ls-remote` against that upstream returned
`Repository not found`. This does not prove whether the repository was made
private, deleted or renamed, nor whether it was publicly accessible at the
original publication date. The immediate deployment failure is established:
both setup scripts required submodule initialization before environment reuse
or creation, so new users without upstream access stopped at step one.

The retained local repository was clean at the exact pinned commit and passed
`git fsck --full`. Its README contains the MIT License and
`Copyright (c) 2026 Ryan Walker Brown`.

## Changes and rationale

- Exported the complete upstream tree
  `9e1c444e12c92fccbe5339ce900bd2b6e4db5e8d`: 159 files, 18,144,059 bytes.
- Matched every exported file to its original upstream Git blob. Git export
  was run with `core.autocrlf=false`; `.gitattributes` sets `-text` for the
  snapshot so all platforms preserve the same bytes.
- Replaced the gitlink and `.gitmodules` with regular tracked files. Retained
  the complete original README/license, assets, notebooks and two example
  MP4s. No upstream source was edited.
- Added `third_party/RFT-SiM.snapshot.json` (commit/tree/blob IDs, file sizes
  and SHA-256) and `scripts/verify_rft_snapshot.py` (standard-library integrity
  checking, including missing, changed and unexpected files).
- Updated PowerShell/POSIX setup to use the bundled snapshot and verify it.
  Setup and canonical validation now work on extracted ZIPs without Git.
- Updated README, guidance, agent contract, provenance, current status,
  workspace layout and decision D-015. Historical July records remain intact.
- Model code, controller, active geometry, RFT conventions and environment
  version pins are unchanged. The upstream crab examples are preserved for
  reference; running them in the lizard environment is not an acceptance claim.

The original nested Git metadata is preserved locally at:

```text
C:\Users\wxy22\Documents\Lizard_Robot_Archive\2026-10-08\RFT-SiM-git-303283f-49478e5f
```

That archive is recovery/provenance only. It is not used by setup or runtime.

## Verification plan and current evidence

Local preparation passed: all 159 exported files match upstream blob IDs and
the SHA-256 manifest; archived Git object integrity passed; both setup scripts
passed syntax checks; `git diff --check` passed.

Publication acceptance requires new source copies obtained exclusively from
GitHub: ordinary HTTPS clone and GitHub ZIP, each containing the full snapshot.
Run the normal Windows setup and project validator from a fresh clone, then
verify and import-check ZIP deployment. Do not copy files or assets from the
canonical checkout or use a local Git alternate. Use only the existing
`lizard_rft` environment, without updating its packages.

This verifies remote source completeness and runtime compatibility on this
Windows workstation. It is not a newly created environment or a Linux/macOS
runtime test. POSIX setup is syntax-checked here.

GitHub verification results and cleanup will be recorded here after the
implementation commit is published. Temporary clones, downloaded ZIPs,
export archives, caches and test logs will be deleted; this compact record and
the preserved original Git metadata remain for traceability.
