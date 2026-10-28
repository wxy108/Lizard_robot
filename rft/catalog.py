"""Declarative source catalog, bounded selective installation and preflight."""
from concurrent.futures import ThreadPoolExecutor
from functools import lru_cache
import hashlib
import importlib.metadata
import json
from pathlib import Path
import urllib.parse
import urllib.request

from .records import ROOT, artifact_manifest, git_state, new_run, sha256, write_json

CATALOG = ROOT/"configs/robot_catalog.json"


def catalog():
    return json.loads(CATALOG.read_text(encoding="utf-8"))


def entry(robot):
    items = catalog()["robots"]
    if robot not in items:
        raise ValueError(f"Unknown robot {robot}; choose: {', '.join(items)}")
    return items[robot]


def config_path(robot):
    return ROOT/entry(robot)["config"]


def selection(robot):
    item = entry(robot)
    if "playground_family" in item:
        family,policy = item["playground_family"],item["policy"]
        return {"playground": ["LICENSE",f"mujoco_playground/_src/locomotion/{family}/",
                 f"mujoco_playground/experimental/sim2sim/play_{policy}_joystick.py",
                 f"mujoco_playground/experimental/sim2sim/onnx/{policy}_policy.onnx"],
                "menagerie":[item["menagerie_model"]+"/"]}
    return {"turtlebot3":["LICENSE","turtlebot3_description/urdf/"+item["urdf"],
             "turtlebot3_description/urdf/common_properties.urdf",
             "turtlebot3_description/meshes/bases/"+item["base_mesh"],
             "turtlebot3_description/meshes/wheels/","turtlebot3_description/meshes/sensors/lds.stl",
             "turtlebot3_node/src/diff_drive_controller.cpp"]}


def fetch(url):
    request = urllib.request.Request(url,headers={"User-Agent":"Lizard-RFT-Selective-Catalog"})
    with urllib.request.urlopen(request,timeout=45) as response:
        return response.read()


@lru_cache(maxsize=8)
def tree(repo,commit):
    data = json.loads(fetch(f"https://api.github.com/repos/{repo}/git/trees/{commit}?recursive=1"))
    if data.get("truncated"):
        raise ValueError("Truncated pinned source tree")
    return data["tree"]


def install(robots):
    definitions = catalog()
    selected = {}
    for robot in robots:
        for key,paths in selection(robot).items():
            selected.setdefault(key,set()).update(paths)
    jobs,total = [],0
    for key,paths in selected.items():
        source = definitions["sources"][key]
        entries = [e for e in tree(source["repository"],source["commit"]) if e["type"] == "blob"
                   and any(e["path"].startswith(p) if p.endswith("/") else e["path"] == p for p in paths)]
        for target in paths:
            if not any(e["path"].startswith(target) if target.endswith("/") else e["path"] == target for e in entries):
                raise ValueError(f"Pinned source missing: {key}/{target}")
        if not any(e["path"].endswith("LICENSE") for e in entries):
            raise ValueError("Selected source has no license")
        total += sum(e["size"] for e in entries)
        jobs.append((key,source,entries))
    if total > definitions["maximum_selected_bytes"]:
        raise ValueError("Selected download budget exceeded")
    run = new_run(ROOT/"outputs/robot_catalog"/("install_"+__import__("datetime").datetime.now().strftime("%Y%m%d_%H%M%S_%f")))
    write_json(run/"environment_before.json",{d.metadata["Name"]:d.version for d in importlib.metadata.distributions()})
    for key,source,entries in jobs:
        destination = (ROOT/source["destination"]).resolve()
        if not destination.is_relative_to((ROOT/"third_party/robot_assets").resolve()):
            raise ValueError("Unsafe asset destination")
        def one(e):
            target = (destination/e["path"]).resolve()
            if not target.is_relative_to(destination):
                raise ValueError("Unsafe upstream file path")
            content = target.read_bytes() if target.exists() else fetch(
                f"https://raw.githubusercontent.com/{source['repository']}/{source['commit']}/{urllib.parse.quote(e['path'])}")
            if len(content) != e["size"] or hashlib.sha1(f"blob {len(content)}\0".encode()+content).hexdigest() != e["sha"]:
                raise ValueError(f"Modified/mismatched asset is not overwritten: {target}")
            if not target.exists():
                target.parent.mkdir(parents=True,exist_ok=True)
                target.write_bytes(content)
            return {"path":e["path"],"sha256":hashlib.sha256(content).hexdigest(),"bytes":len(content),"git_blob":e["sha"]}
        with ThreadPoolExecutor(max_workers=4) as workers:
            records = list(workers.map(one,entries))
        # Additive separate manifest: never replace the preserved earlier one.
        manifest_path = destination/"catalog_manifest.json"
        previous = json.loads(manifest_path.read_text()) if manifest_path.exists() else {"files":[],"commit":source["commit"]}
        if previous["commit"] != source["commit"]:
            raise ValueError("Different commit already owns this asset destination")
        merged = {f["path"]:f for f in previous["files"]}
        merged.update({f["path"]:f for f in records})
        write_json(manifest_path,{**source,"files":list(merged.values())})
        write_json(run/(key+".json"),{**source,"files":records})
        print(f"Verified {len(records)} files: {key}",flush=True)
    write_json(run/"manifest.json",{"source":git_state(),"robots":list(robots),"selected_bytes":total,"artifacts":artifact_manifest(run)})
    print(f"Catalog installation: {run}; selected bytes {total}",flush=True)
    return run


def verify_installed(robot):
    definitions = catalog()
    checked = 0
    for key,paths in selection(robot).items():
        base = ROOT/definitions["sources"][key]["destination"]
        manifest = base/"catalog_manifest.json"
        if not manifest.is_file():
            raise FileNotFoundError(f"Run robot_pipeline.py install {robot}")
        data = json.loads(manifest.read_text())
        if data["commit"] != definitions["sources"][key]["commit"]:
            raise ValueError("Installed source commit is stale")
        for target in paths:
            records = [f for f in data["files"] if
                       (f["path"].startswith(target) if target.endswith("/") else f["path"] == target)]
            if not records:
                raise FileNotFoundError(f"Missing catalog source selection: {target}")
            for record in records:
                path = (base/record["path"]).resolve()
                if not path.is_relative_to(base.resolve()) or not path.is_file() or sha256(path) != record["sha256"]:
                    raise ValueError(f"Installed asset changed/missing: {path}")
                checked += 1
    return checked
