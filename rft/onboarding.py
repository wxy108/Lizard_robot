"""Inspect unknown inputs; propose a clearly unreviewed recipe, never invent dynamics."""
import json
from pathlib import Path

import mujoco

from .catalog import catalog,config_path
from .records import ROOT,new_run,sha256,write_json,artifact_manifest


def recognize_input(path):
    path = Path(path).resolve()
    digest = sha256(path)
    import yaml
    seen = set()
    for robot,entry in catalog()["robots"].items():
        if entry["model_id"] in seen: continue
        seen.add(entry["model_id"])
        recipe = config_path(robot)
        if not recipe.exists(): continue
        config = yaml.safe_load(recipe.read_text())
        source = (recipe.parent/config["model"]).resolve()
        # Copied scene XML can reference different adjacent meshes: matching
        # bytes alone must not silently substitute canonical dependencies.
        if source.is_file() and source == path and sha256(source) == digest:
            return robot
    return None


def inspect_input(path,output=None):
    from .pipeline import inventory
    from .binding import load_model
    path = Path(path).resolve()
    if not path.is_file(): raise FileNotFoundError(path)
    output = new_run(output or ROOT/"outputs/robot_catalog"/("inspect_"+__import__("datetime").datetime.now().strftime("%Y%m%d_%H%M%S_%f")))
    report = {"input":str(path),"sha256":sha256(path),"status":"needs_review"}
    suffix = path.suffix.lower()
    try:
        if suffix in (".xml",".mjcf",".urdf",".xacro",".mjb",".mjz",".zip"):
            if suffix == ".mjb": model = mujoco.MjModel.from_binary_path(str(path))
            elif suffix in (".mjz",".zip"):
                from .formats import check_archive
                check_archive(path)
                with path.open("rb") as stream:
                    model = mujoco.MjSpec.from_zip(stream).compile()
            else: model = load_model(path)
            write_json(output/"inventory.json",inventory(model))
            candidates = [{"id":f"surface_{gid}","body":model.body(int(model.geom_bodyid[gid])).name,
                           "geom":model.geom(gid).name,"reviewed":False} for gid in range(model.ngeom)
                          if model.geom_bodyid[gid] > 0 and (model.geom_contype[gid] or model.geom_conaffinity[gid])]
            write_json(output/"unreviewed_recipe.json",{"ready_for_simulation":False,"model":str(path),
                       "candidate_surfaces":candidates,"missing":["reviewed exterior selection","units/coverage",
                       "initialization/root mode","controller contract","ground supports","acceptance criteria"]})
        elif suffix in (".stl",".obj",".ply"):
            report.update(status="needs_metadata",missing=["declared units","robot dynamics and body owner","controller"])
        else:
            report.update(status="unsupported_input",missing=["explicit format converter or package entry point"])
    except Exception as error:
        report.update(status="needs_adapter",error=str(error),next_action="Resolve packages/Xacro/assets or supply an explicit import recipe")
    write_json(output/"inspection.json",report)
    write_json(output/"manifest.json",{"artifacts":artifact_manifest(output)})
    print(f"Safe stop {report['status']}: {output}",flush=True)
    return output
