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

## GitHub-only verification — completed

Tested implementation commit:
`dcfdf597bd09ed09e5721149e0b6224d2e2b460b`.

Local preparation passed: all 159 exported files match upstream blob IDs and
the SHA-256 manifest; archived Git object integrity passed; both setup scripts
passed syntax checks; `git diff --check` passed. The published vendored subtree
hash exactly equals the original upstream tree hash.

Two independent source acquisitions were made after publishing:

```powershell
git -c credential.helper= -c core.autocrlf=true clone --depth 1 --branch main https://github.com/wxy108/Lizard_robot.git clone
Invoke-WebRequest -Uri https://codeload.github.com/wxy108/Lizard_robot/zip/dcfdf597bd09ed09e5721149e0b6224d2e2b460b -OutFile github-source.zip
Expand-Archive -LiteralPath github-source.zip -DestinationPath zip
```

Both requests were public/anonymous. No source, config or asset was copied
from the canonical checkout. The clone origin was the HTTPS GitHub URL, and
`.git/objects/info/alternates` was absent. The ZIP had no `.git` metadata.
Every one of its **331 files** matched the cloned commit's Git blob bytes.

From **each** downloaded project root, separately:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/setup.ps1
```

Both full setup runs completed with exit 0:

| Check | HTTPS clone | ZIP extraction |
| --- | --- | --- |
| Environment reuse and runtime imports | PASS | PASS |
| 159 upstream snapshot files / SHA-256 | PASS | PASS |
| All 16 existing unit tests | PASS | PASS |
| Eight active mesh topology, orientation, intersection and distribution gates | PASS | PASS |
| 13,916 force sites, sequential names, visual group 5 | PASS | PASS |
| Sand z = 0, emergency floor z = -0.25 m | PASS | PASS |
| Original rigid model, 0.2 s, no-save | PASS; no fall | PASS; no fall |
| RFT model, 0.2 s / 400 steps, no-save | PASS | PASS |

Both RFT short runs reported:

```text
peak total/site force: 86.629 / 0.600 N
RFT power range: -37.023 to 0.000 W
positive active steps: 0.00%
All project validation checks passed.
Deployment complete.
```

ZIP setup with `-SkipValidation` additionally passed its imports/hash checks.
A temporary changed README was rejected with size and SHA-256 errors (exit 1).
A temporarily missing upstream `.gitignore` was rejected as missing (exit 1).
Both files were restored and the original ZIP snapshot reverified successfully.

One initial concurrently launched clone setup failed inside Conda activation:
`__conda_tmp_27365.txt` was in use, then not found. The parallel ZIP import
check passed. A subsequent sequential clone setup and a separate sequential
full ZIP setup both passed. This is recorded as an infrastructure failure
during concurrent setup, not a project-model failure. No source or environment
package changes were made for the retry; these results do not establish that
all concurrent Conda setup is supported.

Environment reused: `lizard_rft`, Python 3.11.15, MuJoCo 3.9.0, NumPy 1.26.4,
Open3D 0.19.0. No environment was recreated or updated. IsaacLab was not
activated or modified. This is fresh **GitHub source** deployment on Windows,
not a new-environment installation or a Linux/macOS runtime test. POSIX setup
passed syntax checking. The 0.2 s runs check packaging and initial integration,
not gait quality or calibrated physics. No 6 s baseline was regenerated.

Snapshot manifest SHA-256:
`44bf09dae38a91a3b346c686aabbb13b1e5a4759c8abd284653755770629f048`.

## Evidence identifiers and cleanup

The temporary artifacts below were removed from the testing location after
verification, at the user's request. Their identifiers and the essential
results above remain; raw logs are not a tracked evidence release.

| Temporary artifact | Bytes | SHA-256 |
| --- | ---: | --- |
| `clone-setup.log` | 1,160 | `568ded7388c0e12cb0435300c292b3f2b19fa5fa1293f1bca7d9a34f1533a47a` |
| `clone-setup-retry.log` | 5,404 | `5b86b1b988bec567dd9ff1062ce24d4b87bf8b8f7bddc806740a672c3af4dc6e` |
| `zip-setup.log` | 553 | `b227b607255fa32dde3de0d41c63f0e87d5dc8f3db8a4a5165c0e157a17a29cd` |
| `zip-setup-full.log` | 5,404 | `5b86b1b988bec567dd9ff1062ce24d4b87bf8b8f7bddc806740a672c3af4dc6e` |
| `snapshot-corruption-test.log` | 111 | `056953a5d462b496c1884f46c98d12b913039aa05c0859ec1cf3f2bba3250d0e` |
| `snapshot-missing-file-test.log` | 73 | `c32309fafa67f16e226f90fd106c61365811db69b8da516496c488f66d3885ad` |
| `github-source.zip` | 60,776,961 | `6e054a39850d20e08b73d39b5785b24248b7a98d2a8b10473e82e55e86cb14c1` |

The complete dedicated temporary directory was:

```text
C:\tmp\lizard-github-validation-20261008-49478e5f
```

It held 727 files totaling 322,452,459 bytes: downloaded clone and ZIP,
extracted ZIP project, upstream export ZIPs, Python caches and test logs.
The target and absence of linked directories were checked before cleanup.
Permanent recursive deletion was rejected by the tool policy; the safer
Windows **Send to Recycle Bin** operation succeeded. `Test-Path` for the
original temporary directory returned `False`. The files are recoverable
from the Recycle Bin and were not permanently erased.

The active canonical checkout and original upstream metadata archive remain.
The final follow-up commit changes only the README and deployment/validation/
status documentation; the tested runtime code, scripts, assets, environment
pins and bundled snapshot remain exactly as in the tested implementation
commit. Use `git log` to identify that documentation commit without a
self-referential commit hash.
