"""Bounded, pinned official locomotion model/policy downloader; no code execution."""
from concurrent.futures import ThreadPoolExecutor
import hashlib
import importlib.metadata
import json
from pathlib import Path
import sys
import urllib.parse

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.install_robot_examples import fetch
from rft.records import artifact_manifest, git_state, new_run, sha256, write_json


def main():
    lock_path = ROOT / "configs/ground_robot_sources.json"
    lock = json.loads(lock_path.read_text())
    run = new_run(prefix="ground_install")
    write_json(run / "environment_before.json", {
        d.metadata["Name"]: d.version for d in importlib.metadata.distributions()})
    sources = []
    total = 0
    for spec in lock["sources"]:
        tree = json.loads(fetch(f"https://api.github.com/repos/{spec['repository']}/git/trees/{spec['commit']}?recursive=1"))
        if tree.get("truncated"):
            raise ValueError("Truncated upstream tree")
        entries = [e for e in tree["tree"] if e["type"] == "blob" and
                   (e["path"] in spec["files"] or any(e["path"].startswith(p) for p in spec["prefixes"]))]
        paths = {e["path"] for e in entries}
        if not set(spec["files"]) <= paths or spec["license"] not in paths:
            raise ValueError("Required pinned source/license missing")
        total += sum(e["size"] for e in entries)
        if total > lock["maximum_download_bytes"]:
            raise ValueError("Download budget exceeded")
        sources.append((spec, entries))
    for spec, entries in sources:
        destination = (ROOT / spec["destination"]).resolve()
        if not destination.is_relative_to((ROOT / "third_party/robot_assets").resolve()):
            raise ValueError("Destination escapes asset directory")

        def install(entry):
            target = (destination / entry["path"]).resolve()
            if not target.is_relative_to(destination):
                raise ValueError("Unsafe upstream path")
            existing = target.is_file()
            content = target.read_bytes() if existing else fetch(
                f"https://raw.githubusercontent.com/{spec['repository']}/{spec['commit']}/{urllib.parse.quote(entry['path'])}")
            blob = hashlib.sha1(f"blob {len(content)}\0".encode() + content).hexdigest()
            if len(content) != entry["size"] or blob != entry["sha"]:
                raise ValueError(f"Pinned asset mismatch (not overwritten): {target}")
            if not existing:
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(content)
            return {"path": entry["path"], "bytes": len(content), "git_blob": blob,
                    "sha256": hashlib.sha256(content).hexdigest()}

        with ThreadPoolExecutor(max_workers=4) as pool:
            records = list(pool.map(install, entries))
        result = {**spec, "files": records, "source": git_state(), "lock_sha256": sha256(lock_path)}
        write_json(destination / "ground_download_manifest.json", result)
        write_json(run / (spec["repository"].split("/")[-1] + ".json"), result)
        print(f"Verified {len(records)} files: {spec['repository']} @ {spec['commit']}", flush=True)
    write_json(run / "manifest.json", {"artifacts": artifact_manifest(run), "selected_bytes": total})
    print(f"Selected bytes: {total}; installation record: {run}")


if __name__ == "__main__":
    main()
