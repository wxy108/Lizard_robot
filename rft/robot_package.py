"""Relocatable compiled robot package with hashes and explicit rigid/sand models."""
from pathlib import Path
import json
import zipfile

import mujoco
import yaml

from .catalog import catalog,config_path,entry,selection,verify_installed
from .formats import check_archive
from .pipeline import fingerprint,import_robot
from .records import ROOT,artifact_manifest,git_state,new_run,sha256,write_json


def export_package(robot,output):
    verify_installed(robot)
    output = Path(output).resolve()
    if output.exists():
        raise FileExistsError("Package destination exists; refuse overwrite")
    staging = new_run(ROOT/"outputs/robot_catalog"/("pack_"+__import__("datetime").datetime.now().strftime("%Y%m%d_%H%M%S_%f")))
    rigid,config,sources = import_robot(config_path(robot))
    sand,sand_config,_ = import_robot(config_path(robot),medium="sand")
    if fingerprint(rigid) != fingerprint(sand):
        raise ValueError("Packaged rigid/sand physical identities differ")
    mujoco.mj_saveModel(rigid,str(staging/"rigid.mjb"))
    mujoco.mj_saveModel(sand,str(staging/"sand.mjb"))
    recipe = {k:v for k,v in config.items() if k not in ("asset_dirs","package_roots","model_adapter","effective_model_adapter")}
    recipe.update(model="rigid.mjb",sand_model="sand.mjb",compiled_mujoco_version=mujoco.__version__)
    if sand_config.get("effective_support_conversion"):
        recipe["effective_support_conversion"] = sand_config["effective_support_conversion"]
    if config["controller"].get("policy"):
        original = (config_path(robot).parent/config["controller"]["policy"]).resolve()
        (staging/"policy.onnx").write_bytes(original.read_bytes())
        recipe["controller"] = dict(recipe["controller"],policy="policy.onnx")
    write_json(staging/"recipe.json",recipe)
    (staging/"source_recipe.yaml").write_bytes(config_path(robot).read_bytes())
    definitions = catalog()
    notices = staging/"notices"; notices.mkdir()
    for source_name in selection(robot):
        base = ROOT/definitions["sources"][source_name]["destination"]
        license_path = base/(entry(robot)["menagerie_model"]+"/LICENSE" if source_name == "menagerie" else "LICENSE")
        (notices/(source_name+"_LICENSE.txt")).write_bytes(license_path.read_bytes())
    (notices/"controller_NOTICE.txt").write_bytes((ROOT/"controllers/NOTICE.playground").read_bytes())
    write_json(staging/"robot_package.json",{"schema_version":1,"robot_id":robot,"source":git_state(),
               "mujoco_version":mujoco.__version__,"physical_fingerprint":fingerprint(rigid),
               "original_source_files":sources,"source_recipe_sha256":sha256(config_path(robot)),
               "original_effective_adapter":config.get("effective_model_adapter"),
               "recipe":"recipe.json","artifacts":artifact_manifest(staging)})
    output.parent.mkdir(parents=True,exist_ok=True)
    with zipfile.ZipFile(output,"x",compression=zipfile.ZIP_DEFLATED) as archive:
        for path in sorted(staging.rglob("*")):
            if path.is_file(): archive.write(path,path.relative_to(staging).as_posix())
    print(f"Robot package: {output}; sha256={sha256(output)}",flush=True)
    return output


def import_package(path,destination=None):
    path = Path(path).resolve()
    members = check_archive(path)
    with zipfile.ZipFile(path) as archive:
        if "robot_package.json" not in members:
            raise ValueError("Not a robot package; native MJZ or source archives require an explicit entry adapter")
        metadata = json.loads(archive.read("robot_package.json"))
        if metadata.get("schema_version") != 1 or metadata.get("mujoco_version") != mujoco.__version__:
            raise ValueError("Robot package schema/MuJoCo binary version mismatch")
        expected = metadata["artifacts"]
        if set(members) != set(expected)|{"robot_package.json"} or metadata["recipe"] not in expected:
            raise ValueError("Package file inventory/recipe is not fully verified")
        import hashlib
        for name,info in expected.items():
            content = archive.read(name)
            if len(content) != info["bytes"] or hashlib.sha256(content).hexdigest() != info["sha256"]:
                raise ValueError(f"Package artifact mismatch: {name}")
        directory = Path(destination) if destination else ROOT/"outputs/robot_catalog/imports"/sha256(path)
        directory = directory.resolve()
        if directory.exists():
            # Do not blindly trust the extraction cache.
            for name,info in expected.items():
                target = (directory/name).resolve()
                if not target.is_relative_to(directory) or not target.is_file() or sha256(target) != info["sha256"]:
                    raise ValueError("Existing extracted package cache is stale/tampered")
        else:
            directory.mkdir(parents=True,exist_ok=False)
            for name in members:
                target = (directory/name).resolve()
                if not target.is_relative_to(directory):
                    raise ValueError("Unsafe package path")
                target.parent.mkdir(parents=True,exist_ok=True)
                target.write_bytes(archive.read(name))
    recipe = directory/metadata["recipe"]
    config = json.loads(recipe.read_text())
    # The package cannot resurrect source workstation paths or arbitrary files.
    if config.get("asset_dirs") or config.get("package_roots") or config.get("model_adapter"):
        raise ValueError("Compiled package recipe must be self-contained")
    for reference in (config["model"],config.get("sand_model"),config.get("controller",{}).get("policy")):
        if reference and reference not in expected:
            raise ValueError("Package recipe references an unverified external file")
    model,_,_ = import_robot(recipe)
    if fingerprint(model) != metadata["physical_fingerprint"]:
        raise ValueError("Packaged model physical identity mismatch")
    return recipe
