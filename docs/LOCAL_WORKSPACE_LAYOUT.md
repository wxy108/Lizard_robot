# Canonical local workspace and GitHub layout

## One active repository

The only active repository on the original workstation is:

```text
C:\Users\wxy22\Documents\Lizard_Robot_MuJoCo
```

Its remote is:

```text
https://github.com/wxy108/Lizard_robot.git
```

The local clone-directory name is not part of the Git tree. GitHub therefore
shows `README.md`, `asset/`, `docs/`, and the other tracked entries at the
repository root rather than inside a `Lizard_Robot_MuJoCo/` wrapper.

## What must match

Tracked content on local `main` must match GitHub `main`. Verify with:

```bash
git status --short
git rev-parse HEAD
git rev-parse origin/main
git diff --name-status HEAD origin/main
```

Acceptance:

- status is empty;
- both commits are identical;
- the diff is empty.

Git does not continuously synchronize independent clones or repositories.
Every extra clone has its own branch, HEAD, index, ignored output, and remote
state.

## Expected local-only entries

The physical local directory is intentionally not byte-for-byte identical to
the GitHub web view:

- `.git/` is local Git metadata;
- `outputs/` contains ignored generated data;
- `__pycache__/` contains ignored Python caches;
- `third_party/RFT-SiM/` is expanded locally, while GitHub shows the pinned
  submodule pointer at commit
  `303283fae075cae4101ee3af102a36a4a5775998`.

These differences do not represent missing tracked project files.

## Archived former workspaces

On 2026-07-27, the following local directories were moved intact to:

```text
C:\Users\wxy22\Documents\Lizard_Robot_Archive\2026-07-27\
```

| Archived directory | Former role | Final status |
| --- | --- | --- |
| `Lizard_Robot_RFT/` | outer integration workspace containing a nested old project and historical archive | provenance/recovery only |
| `Lizard_Robot_GitHub_Publish/` | temporary publication checkout at stale commit `9ef92cddc3f4` | provenance/recovery only |

The former outer RFT repository had no GitHub remote and was at local commit
`c5fee1a368bb`. Neither archived directory is an active clone, runtime source,
or output destination.

The canonical local RFT-SiM submodule formerly used a Git object alternate
pointing into the outer RFT workspace. During consolidation, all reachable
objects were repacked into the canonical submodule's own object store, the
alternate was removed, `git fsck --full` passed, and the pinned submodule HEAD
remained unchanged. The canonical repository is now locally self-contained.

## Rejected-mesh diagnostics remain portable

The only historical rejected STL needed by the diagnostic generator is
tracked inside the canonical repository:

```text
reference/rejected_meshes/legacy_vertex_cluster/Back.STL
```

Therefore:

```bash
python scripts/generate_failed_mesh_videos.py
```

does not read from the local archive. A post-archive smoke run generated all
three comparisons from clean commit `0186850`, with three cases and five
hash-recorded media artifacts.

## Non-negotiable rule

Run, edit, validate, commit, and generate output only from the Git root that
contains the `origin` remote above. Do not copy changes manually between the
archive and the canonical repository.
