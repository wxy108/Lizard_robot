"""Download only pinned selected upstream models, verifying original Git blobs.

No numerical package installation, environment activation, source rewrite or
unbounded repository clone. Optional registry installation is a separate
explicit pip command in the generalization guide.
"""

import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import importlib.metadata
import json
from pathlib import Path
import sys
import urllib.request

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from rft.records import new_run,sha256,write_json


def fetch(url):
    request=urllib.request.Request(url,headers={"User-Agent":"Lizard-RFT-Pinned-Models"})
    with urllib.request.urlopen(request,timeout=60) as response:
        return response.read()


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--lock",type=Path,default=ROOT/"configs/robot_library_lock.json")
    args=parser.parse_args()
    lock=json.loads(args.lock.read_text(encoding="utf-8"))
    destination=(ROOT/lock["destination"]).resolve()
    if not destination.is_relative_to((ROOT/"third_party/robot_assets").resolve()):
        raise ValueError("External model destination must be under third_party/robot_assets")
    tree=json.loads(fetch(f"https://api.github.com/repos/{lock['repository']}/git/trees/{lock['commit']}?recursive=1"))
    if tree.get("truncated"):
        raise ValueError("Upstream tree enumeration is truncated")
    files=[entry for entry in tree["tree"] if entry["type"]=="blob"
           and entry["path"].split("/")[0] in lock["models"]]
    total=sum(entry["size"] for entry in files)
    if not files or total>lock["maximum_download_bytes"]:
        raise ValueError(f"Unexpected external download size: {total}")
    for model in lock["models"]:
        if not any(entry["path"]==f"{model}/LICENSE" for entry in files):
            raise ValueError(f"License not found for selected model {model}")
    run=new_run(prefix="robot_install")
    write_json(run/"environment_inventory.json",{
        dist.metadata["Name"]:dist.version for dist in importlib.metadata.distributions()
    })
    def install(entry):
        path=(destination/entry["path"]).resolve()
        if not path.is_relative_to(destination):
            raise ValueError("Upstream file escapes its destination")
        content=path.read_bytes() if path.is_file() else None
        def matches(value):
            return value is not None and len(value)==entry["size"] and hashlib.sha1(
                f"blob {len(value)}\0".encode()+value).hexdigest()==entry["sha"]
        if not matches(content):
            if content is not None:
                raise ValueError(f"Existing upstream asset differs; preserve before replacement: {path}")
            content=fetch(f"https://raw.githubusercontent.com/{lock['repository']}/{lock['commit']}/{urllib.parse.quote(entry['path'])}")
            if not matches(content):
                raise ValueError(f"Upstream Git blob mismatch: {entry['path']}")
            path.parent.mkdir(parents=True,exist_ok=True)
            path.write_bytes(content)
        return {"path":entry["path"],"bytes":len(content),"git_blob":entry["sha"],
                "sha256":hashlib.sha256(content).hexdigest()}
    with ThreadPoolExecutor(max_workers=4) as pool:
        records=list(pool.map(install,files))
    manifest={"repository":lock["repository"],"commit":lock["commit"],
              "models":lock["models"],"file_count":len(records),"total_bytes":total,
              "lock_sha256":sha256(args.lock),"files":records}
    destination.mkdir(parents=True,exist_ok=True)
    write_json(destination/"download_manifest.json",manifest)
    write_json(run/"install_summary.json",manifest)
    print(f"Verified {len(records)} upstream files, {total:,} bytes at {destination}")
    print(f"Install record: {run}")


if __name__=="__main__":
    main()
