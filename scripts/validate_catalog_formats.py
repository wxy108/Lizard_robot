"""Real-model same-state force/control checks across source, URDF, MJB and archives."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
import mujoco
import numpy as np
import yaml

from controllers.catalog_controllers import make_controller
from rft.catalog import config_path,verify_installed
from rft.pipeline import build_bundle,load_bundle,import_robot,fingerprint
from rft.provider import RFTForceProvider
from rft.records import new_run,write_json,artifact_manifest,git_state,sha256
from rft.robot_package import export_package,import_package


def compare(original,other,config,other_config,surfaces,source_recipe,other_recipe):
    # Same initialization, nonzero articulated velocity and intrusion. Separate
    # provider instances guarantee equal zero smoothing histories.
    a,b = mujoco.MjData(original),mujoco.MjData(other)
    mujoco.mj_resetDataKeyframe(original,a,original.key(config["initial_keyframe"]).id)
    a.qpos[2] -= .03
    a.qvel[:] = np.random.default_rng(2834).normal(0,.1,original.nv)
    b.qpos[:],b.qvel[:] = a.qpos,a.qvel
    mujoco.mj_forward(original,a); mujoco.mj_forward(other,b)
    ca,_ = make_controller(original,a,config["controller"],source_recipe.parent)
    cb,_ = make_controller(other,b,other_config["controller"],other_recipe.parent)
    ua,ub = ca.update(original,a),cb.update(other,b)
    fa = RFTForceProvider(original,surfaces,application="body_wrench").compute(a)
    fb = RFTForceProvider(other,surfaces,application="body_wrench").compute(b)
    force = max(float(np.max(np.abs(x.forces-y.forces))) for x,y in zip(fa.surfaces,fb.surfaces))
    generalized = float(np.max(np.abs(fa.qfrc-fb.qfrc)))
    command = float(np.max(np.abs(ua-ub)))
    active = sum(int(x.active.sum()) for x in fa.surfaces)
    if not active or force > 1e-10 or generalized > 1e-10 or command > 1e-7:
        raise AssertionError(f"Format comparison failed: active={active}, F={force}, q={generalized}, ctrl={command}")
    differences = {}
    for field in ("body_parentid","body_pos","body_quat","body_mass","body_inertia","body_ipos","body_iquat",
                  "jnt_type","jnt_bodyid","jnt_pos","jnt_axis","jnt_range","dof_damping","dof_armature",
                  "geom_bodyid","geom_type","geom_dataid","geom_pos","geom_quat","geom_size",
                  "mesh_vert","mesh_face","actuator_trnid","actuator_gear","actuator_gainprm","actuator_biasprm"):
        x,y = getattr(original,field),getattr(other,field)
        if not np.array_equal(x,y):
            differences[field] = {"shapes":[list(x.shape),list(y.shape)],
                "max_absolute_difference":float(np.max(np.abs(x-y))) if x.shape==y.shape else None}
    return {"status":"passed_same_state_probe","active_triangles":active,"max_triangle_force_difference_N":force,
            "max_generalized_component_difference":generalized,"max_command_difference":command,
            "numeric_field_differences":differences,
            "physical_fingerprint_equal":fingerprint(original)==fingerprint(other)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir",type=Path,required=True)
    args = parser.parse_args(); output = new_run(args.output_dir)
    results = {}
    for robot in ("go1","g1","turtlebot3_burger"):
        verify_installed(robot)
        source_recipe = config_path(robot)
        model,config,files = import_robot(source_recipe)
        folder = output/robot; folder.mkdir()
        build_bundle(model,config,source_recipe,folder/"bundle",files)
        surfaces,_ = load_bundle(model,folder/"bundle")
        package = export_package(robot,folder/(robot+".rftpkg"))
        packed_recipe = import_package(package,folder/"relocated")
        packed,packed_config,packed_sources = import_robot(packed_recipe)
        if fingerprint(model) != fingerprint(packed):
            raise AssertionError("MJB package changed compiled physical identity")
        results[robot] = {"mjb_package":compare(model,packed,config,packed_config,surfaces,source_recipe,packed_recipe),
                          "package_sha256":sha256(package),"loaded_package_sources":packed_sources}
        sand,_,_ = import_robot(packed_recipe,medium="sand")
        support = {sand.geom(name).id for name in config["support_geoms"]}
        if any(a in support or b in support for a,b in zip(sand.pair_geom1,sand.pair_geom2)):
            raise AssertionError("Package resurrected explicit rigid floor pairs")
        if robot == "go1":
            assets = {}
            for raw in config["asset_dirs"]:
                for path in (source_recipe.parent/raw).resolve().rglob("*"):
                    if path.is_file(): assets[path.name] = path.read_bytes()
            spec = mujoco.MjSpec.from_file(str((source_recipe.parent/config["model"]).resolve()),assets=assets)
            archive = folder/"native.mjz"
            with archive.open("wb") as stream: spec.to_zip(stream)
            recipe = dict(config,model=str(archive))
            recipe.pop("asset_dirs")
            write_json(folder/"native_recipe.json",recipe)
            other,other_config,_ = import_robot(folder/"native_recipe.json")
            results[robot]["native_mjz"] = compare(model,other,config,other_config,surfaces,source_recipe,source_recipe)
        if robot == "turtlebot3_burger":
            import xacro
            expanded = folder/"expanded.urdf"
            expanded.write_bytes(xacro.process_file(str((source_recipe.parent/config["model"]).resolve()),mappings={"namespace":""}).toxml().encode())
            recipe = dict(config,model=str(expanded),package_roots={k:str((source_recipe.parent/v).resolve()) for k,v in config["package_roots"].items()})
            write_json(folder/"pure_urdf_recipe.json",recipe)
            other,other_config,_ = import_robot(folder/"pure_urdf_recipe.json")
            results[robot]["expanded_pure_urdf"] = compare(model,other,config,other_config,surfaces,source_recipe,source_recipe)
    write_json(output/"format_comparison.json",{"source":git_state(),"results":results})
    write_json(output/"manifest.json",{"artifacts":artifact_manifest(output)})
    print(json.dumps(results,indent=2)); print(output)


if __name__ == "__main__":
    main()
