"""Verify the bundled upstream RFT-SiM bytes without Git or third-party packages."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]


def verify_snapshot() -> None:
    manifest_path = ROOT / "third_party" / "RFT-SiM.snapshot.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    snapshot = ROOT / "third_party" / "RFT-SiM"
    failures = []
    files = manifest["files"]
    actual = {
        path.relative_to(snapshot).as_posix()
        for path in snapshot.rglob("*")
        if path.is_file()
        and ".git" not in path.relative_to(snapshot).parts
        and "__pycache__" not in path.relative_to(snapshot).parts
        and path.suffix != ".pyc"
    }
    for extra in sorted(actual - files.keys()):
        failures.append(f"Unexpected snapshot file: {extra}")
    for name, expected in files.items():
        path = snapshot / name
        if not path.is_file():
            failures.append(f"Missing snapshot file: {name}")
            continue
        content = path.read_bytes()
        if len(content) != expected["bytes"]:
            failures.append(f"Snapshot size mismatch: {name}")
        if hashlib.sha256(content).hexdigest() != expected["sha256"]:
            failures.append(f"Snapshot SHA-256 mismatch: {name}")
    if failures:
        raise ValueError("\n".join(failures))
    total = sum(item["bytes"] for item in files.values())
    if len(files) != manifest["file_count"] or total != manifest["total_bytes"]:
        raise ValueError("Snapshot manifest totals do not match its file entries")
    print(
        f"RFT-SiM snapshot verified: {len(files)} files, {total:,} bytes, "
        f"upstream commit {manifest['upstream_commit']}"
    )


if __name__ == "__main__":
    try:
        verify_snapshot()
    except (OSError, ValueError, KeyError) as error:
        print(f"RFT-SiM snapshot verification failed: {error}", file=sys.stderr)
        sys.exit(1)
