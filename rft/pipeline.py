"""Portable, verified prepared bundles separating qualification from simulation."""
import hashlib
import importlib.metadata
import json
from pathlib import Path

import mujoco
import numpy as np
import yaml

from .binding import Surface, bind_surfaces, load_model, mesh_from_geom, rotation
from .geometry import GeometryRejected, make_mesh, mesh_arrays, prepare, read_mesh
from .records import artifact_manifest, git_state, sha256, write_json


def fingerprint(model):
    """Compiled geometry, dynamics and actuator identity; not dt/contact-mode overrides."""
    identities = {key:[getattr(model,key)(i).name for i in range(count)] for key,count in
                  (("body",model.nbody),("joint",model.njnt),("geom",model.ngeom),("actuator",model.nu),("site",model.nsite))}
    digest = hashlib.sha256(json.dumps(identities,sort_keys=True).encode())
    for key in ("body_parentid", "body_pos", "body_quat", "body_mass", "body_inertia", "body_ipos",
                "body_iquat", "jnt_type", "jnt_bodyid", "jnt_pos", "jnt_axis", "jnt_range",
                "dof_damping", "dof_armature", "geom_bodyid", "geom_type", "geom_dataid", "geom_pos",
                "geom_quat", "geom_size", "mesh_vert", "mesh_face", "actuator_trnid", "actuator_gear",
                "actuator_gainprm", "actuator_biasprm"):
        array = np.asarray(getattr(model, key))
        digest.update(key.encode()+str(array.shape).encode()+str(array.dtype).encode()+array.tobytes())
    return digest.hexdigest()


def inventory(model):
    return {"nq": model.nq, "nv": model.nv, "nu": model.nu,
            "bodies": [{"name": model.body(i).name, "parent": int(model.body_parentid[i]),
                        "mass_kg": float(model.body_mass[i])} for i in range(model.nbody)],
            "geoms": [{"name": model.geom(i).name, "index": i,
                       "body": model.body(int(model.geom_bodyid[i])).name,
                       "type": int(model.geom_type[i]), "contype": int(model.geom_contype[i]),
                       "conaffinity": int(model.geom_conaffinity[i]), "pos_body_m": model.geom_pos[i].tolist()}
                      for i in range(model.ngeom)],
            "joints":[{"name":model.joint(i).name,"type":int(model.jnt_type[i]),
                       "qpos_address":int(model.jnt_qposadr[i]),"dof_address":int(model.jnt_dofadr[i]),
                       "range":model.jnt_range[i].tolist(),
                       "damping":float(model.dof_damping[model.jnt_dofadr[i]]),
                       "armature":float(model.dof_armature[model.jnt_dofadr[i]]),
                       "frictionloss":float(model.dof_frictionloss[model.jnt_dofadr[i]])}
                      for i in range(model.njnt)],
            "actuators": [{"name": model.actuator(i).name, "joint": model.joint(int(model.actuator_trnid[i,0])).name
                           if model.actuator_trntype[i] == mujoco.mjtTrn.mjTRN_JOINT else None,
                           "gain": model.actuator_gainprm[i].tolist(), "bias": model.actuator_biasprm[i].tolist(),
                           "ctrlrange": model.actuator_ctrlrange[i].tolist(),
                           "forcerange": model.actuator_forcerange[i].tolist()}
                          for i in range(model.nu)], "compiled_fingerprint": fingerprint(model)}


def convert_support_spec(spec, config):
    """Remove only explicitly reviewed support pairs in a private native spec."""
    supports = set(config["support_geoms"])
    declared = set(config.get("disable_support_pairs",[]))
    pairs = [p for p in spec.pairs if p.geomname1 in supports or p.geomname2 in supports]
    if any(not p.name for p in pairs) or len({p.name for p in pairs}) != len(pairs):
        raise ValueError("Support pair conversion requires unique named pairs for review")
    actual = {p.name for p in pairs}
    if actual != declared:
        raise ValueError(f"Support pair conversion requires exact reviewed names: actual={sorted(actual)}, declared={sorted(declared)}")
    for pair in pairs:
        spec.delete(pair)
    return sorted(actual)


def import_robot(config_path, *, medium=None):
    path = Path(config_path).resolve()
    config = yaml.safe_load(path.read_text(encoding="utf-8"))
    model_path = (path.parent/config["model"]).resolve()
    if model_path.suffix.lower() not in (".xml", ".mjcf", ".urdf"):
        raise ValueError("Dynamics input requires MJCF/URDF; a mesh alone needs explicit body/dynamics metadata")
    assets, files = {}, {model_path: sha256(model_path)}
    for raw_dir in config.get("asset_dirs", []):
        directory = (path.parent/raw_dir).resolve()
        if not directory.is_dir():
            raise FileNotFoundError(f"Missing asset directory: {directory}; run the pinned installer")
        for item in sorted(directory.rglob("*")):
            if not item.is_file():
                continue
            content = item.read_bytes()
            # MuJoCo's native virtual filesystem flattens filenames. Supplying
            # both basename and relative alias creates duplicate VFS entries.
            for key in (item.name,):
                if key in assets and assets[key] != content:
                    raise ValueError(f"Ambiguous virtual asset name: {key}")
                assets[key] = content
            files[item] = sha256(item)
    if assets and model_path.suffix.lower() == ".urdf":
        raise ValueError("URDF uses package_roots, not the MJCF virtual-asset adapter")
    model = (mujoco.MjModel.from_xml_path(str(model_path), assets=assets) if assets else
             load_model(model_path, package_roots={k:(path.parent/v).resolve()
                         for k,v in config.get("package_roots", {}).items()}))
    if medium == "sand" and model.npair:
        if model_path.suffix.lower() == ".urdf":
            raise ValueError("URDF explicit-pair conversion needs a separate reviewed adapter")
        reference = fingerprint(model)
        spec = mujoco.MjSpec.from_file(str(model_path),assets=assets)
        removed = convert_support_spec(spec,config)
        model = spec.compile()
        if fingerprint(model) != reference:
            raise ValueError("Support conversion changed geometry/dynamics/controller identity")
        config["effective_support_conversion"] = {"removed_explicit_pairs":removed,"retained_pairs":[p.name for p in spec.pairs]}
    # The compiled fingerprint covers local implicit dependencies too. Explicit
    # asset directories additionally retain exact file-level source provenance.
    sources = [{"path": str(p), "sha256": value} for p,value in files.items()]
    return model, config, sources


def select_meshes(model, config, config_path):
    selected, seen_geoms, seen_ids = [], set(), set()
    for spec in config.get("surfaces", []):
        sid, owner = spec["id"], spec["body"]
        if sid in seen_ids or not sid.replace("_", "").replace("-", "").isalnum():
            raise ValueError("Surface IDs must be unique filename-safe alphanumeric identifiers")
        seen_ids.add(sid)
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, owner)
        if bid <= 0:
            raise ValueError(f"Invalid body owner: {owner}")
        if "geom" in spec:
            gid = mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,spec["geom"])
            if gid < 0 or int(model.geom_bodyid[gid]) != bid:
                raise ValueError(f"Geom owner mismatch: {sid}")
            if gid in seen_geoms:
                raise ValueError("Duplicate physical geom selection would double RFT")
            seen_geoms.add(gid)
            mesh = make_mesh(*mesh_from_geom(model,gid,segments=spec.get("segments",24)))
            source = {"geom":spec["geom"],"compiled_body_frame": True}
        else:
            filename = (Path(config_path).parent/spec["mesh"]).resolve()
            mesh = read_mesh(filename, spec.get("units"))
            transform = spec.get("transform_to_body", {})
            v, f = mesh_arrays(mesh)
            pos = np.asarray(transform.get("pos_m", [0,0,0]),dtype=float)
            if pos.shape != (3,) or not np.all(np.isfinite(pos)):
                raise ValueError("Invalid source-to-body translation")
            mesh = make_mesh(v @ rotation(transform.get("quat_wxyz",[1,0,0,0])).T + pos,f)
            source = {"mesh": str(filename), "sha256":sha256(filename),"units":spec["units"],"transform":transform}
        selected.append((spec, mesh, source))
    if not selected:
        raise ValueError("No reviewed surfaces: inspect inventory and provide explicit body/exterior selection")
    # Prohibit duplicate/intersecting/contained shells within one rigid owner.
    # Across moving bodies, their changing relative pose is a runtime issue.
    for i,(a,ma,_) in enumerate(selected):
        for b,mb,_ in selected[i+1:]:
            if a["body"] != b["body"]:
                continue
            if ma.is_intersecting(mb):
                raise ValueError("Intersecting same-body shells need a reviewed exterior union")
            for outer,inner in ((ma,mb),(mb,ma)):
                import open3d as o3d
                scene = o3d.t.geometry.RaycastingScene()
                scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(outer))
                if np.any(scene.compute_signed_distance(o3d.core.Tensor(
                    np.asarray(inner.vertices).astype(np.float32))).numpy() < -1e-7):
                    raise ValueError("Contained same-body shell would integrate internal faces")
    return selected


def build_bundle(model, config, config_path, directory, sources=()):
    directory = Path(directory)
    directory.mkdir(parents=True,exist_ok=False)
    reports = []
    try:
        for spec, mesh, source in select_meshes(model,config,config_path):
            recipe = spec.get("remesh",config.get("remesh",{}))
            prepared, report = prepare(mesh,recipe,directory/spec["id"])
            vertices, faces = mesh_arrays(prepared)
            surface = Surface.from_mesh(spec["id"],spec["body"],vertices,faces,group=spec.get("group",spec["body"]))
            np.savez_compressed(directory/spec["id"]/"surface.npz",vertices=vertices,faces=faces,
                                centers=surface.centers,normals=surface.normals,areas=surface.areas)
            reports.append({"id":spec["id"],"body":spec["body"],"group":surface.group,
                            "arrays":f"{spec['id']}/surface.npz","source":source,**report})
        metadata = {"schema_version":1,"status":"qualified","source":git_state(),
                    "model_fingerprint":fingerprint(model),"config_sha256":sha256(config_path),
                    "sources":list(sources),"surfaces":reports,
                    "versions":{p:importlib.metadata.version(p) for p in ("mujoco","numpy","open3d","pymeshlab")}}
        write_json(directory/"bundle.json",metadata)
        write_json(directory/"manifest.json",{"artifacts":artifact_manifest(directory)})
        return metadata
    except Exception as exc:
        failure = {"status":"rejected","stage":"prepare","error":str(exc),"completed_surfaces":reports,
                   "exception":type(exc).__name__}
        if isinstance(exc,GeometryRejected):
            failure["reasons"],failure["metrics"] = exc.reasons,exc.metrics
        write_json(directory/"rejected.json",failure)
        raise


def verify_artifacts(directory):
    directory = Path(directory).resolve()
    manifest = json.loads((directory/"manifest.json").read_text())
    for name,info in manifest["artifacts"].items():
        path = (directory/name).resolve()
        if not path.is_relative_to(directory) or not path.is_file():
            raise ValueError("Missing or unsafe bundle artifact")
        if path.stat().st_size != info["bytes"] or sha256(path) != info["sha256"]:
            raise ValueError(f"Artifact mismatch: {name}")
    return manifest


def load_bundle(model, directory):
    directory = Path(directory).resolve()
    verified = verify_artifacts(directory)["artifacts"]
    if "bundle.json" not in verified:
        raise ValueError("Bundle metadata is not verified")
    metadata = json.loads((directory/"bundle.json").read_text())
    if metadata.get("schema_version") != 1 or metadata.get("status") != "qualified":
        raise ValueError("Unqualified/unsupported bundle")
    if metadata["model_fingerprint"] != fingerprint(model):
        raise ValueError("Bundle belongs to a different compiled model")
    surfaces = []
    for spec in metadata["surfaces"]:
        if spec["arrays"] not in verified:
            raise ValueError("Unverified surface arrays")
        with np.load(directory/spec["arrays"],allow_pickle=False) as arrays:
            # Recompute the quadrature from topology rather than trusting cached fields.
            surface = Surface.from_mesh(spec["id"],spec["body"],arrays["vertices"],arrays["faces"],
                                        group=spec["group"],source="qualified_bundle")
            for key in ("centers","normals","areas"):
                if not np.array_equal(getattr(surface,key),arrays[key]):
                    raise ValueError("Cached quadrature disagrees with prepared triangles")
            surfaces.append(surface)
    bind_surfaces(model,surfaces)
    return tuple(surfaces),metadata


def verify_binding_poses(model,surfaces,initial_keyframe):
    """Independent MuJoCo local2Global + point Jacobian checks in three poses."""
    from .provider import RFTForceProvider
    provider = RFTForceProvider(model,surfaces,alpha=1,application="body_wrench")
    rng = np.random.default_rng(2810)
    worst_position = worst_velocity = 0.0
    checked = 0
    for pose in range(3):
        data = mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model,data,model.key(initial_keyframe).id)
        for j in range(model.njnt):
            if model.jnt_type[j] in (mujoco.mjtJoint.mjJNT_HINGE,mujoco.mjtJoint.mjJNT_SLIDE):
                data.qpos[model.jnt_qposadr[j]] += pose*.015
        data.qvel[:] = rng.normal(0,.15,model.nv)
        mujoco.mj_forward(model,data)
        result = provider.compute(data)
        for binding,diagnostic in zip(provider.bindings,result.surfaces):
            for index in np.unique([0,len(binding.surface.areas)//2,len(binding.surface.areas)-1]):
                expected_position,orientation = np.zeros(3),np.zeros(9)
                mujoco.mj_local2Global(data,expected_position,orientation,binding.surface.centers[index],
                                      np.array([1.,0.,0.,0.]),binding.body_id,0)
                linear,angular = np.zeros((3,model.nv)),np.zeros((3,model.nv))
                mujoco.mj_jac(model,data,linear,angular,expected_position,binding.body_id)
                worst_position = max(worst_position,float(np.max(np.abs(expected_position-diagnostic.positions_world[index]))))
                worst_velocity = max(worst_velocity,float(np.max(np.abs(linear@data.qvel-diagnostic.point_velocities_world[index]))))
                checked += 1
    if worst_position > 1e-11 or worst_velocity > 1e-10:
        raise AssertionError("Prepared binding does not match native pose/point velocity reference")
    return {"poses":3,"points_checked":checked,"max_position_difference_m":worst_position,
            "max_point_velocity_difference_m_s":worst_velocity,"status":"passed"}
