"""Explicit URDF/Xacro/native-archive/binary adapters; no upstream file edits."""
from pathlib import Path
import xml.etree.ElementTree as ET
import zipfile

import mujoco
import numpy as np

from .records import sha256


def check_archive(path, *, max_bytes=400000000, max_members=1500):
    """Validate before native archive loading or extraction."""
    with zipfile.ZipFile(path) as archive:
        entries = archive.infolist()
        if len(entries) > max_members or sum(e.file_size for e in entries) > max_bytes:
            raise ValueError("Archive expansion budget exceeded")
        seen = set()
        for item in entries:
            name = item.filename.replace("\\","/")
            parts = name.split("/")
            if name.startswith("/") or any(p in ("..",".") or ":" in p or p.rstrip(" .") != p for p in parts):
                raise ValueError("Unsafe archive member path")
            if ((item.external_attr >> 16) & 0o170000) == 0o120000:
                raise ValueError("Archive symlinks are not allowed")
            if name.casefold() in seen:
                raise ValueError("Duplicate/case-ambiguous archive member")
            seen.add(name.casefold())
        return [e.filename for e in entries if not e.is_dir()]


def urdf_spec(path, package_roots, mappings):
    path = Path(path).resolve()
    content = path.read_text(encoding="utf-8")
    macro = path.suffix.lower() == ".xacro" or "${" in content or "<xacro:" in content
    includes = []
    if macro:
        import xacro
        content = xacro.process_file(str(path),mappings=mappings).toxml()
        includes = [Path(p).resolve() for p in getattr(xacro,"all_includes",[]) if Path(p).is_file()]
    root = ET.fromstring(content)
    if root.tag != "robot":
        raise ValueError("URDF adapter requires a robot root element")
    files = {path,*includes}
    for element in root.iter():
        if "filename" not in element.attrib:
            continue
        filename = element.attrib["filename"]
        if filename.startswith("package://"):
            package,_,relative = filename[len("package://"):].partition("/")
            if package not in package_roots:
                raise ValueError(f"Missing URDF package root: {package}")
            base = Path(package_roots[package]).resolve()
            target = (base/relative).resolve()
            if not target.is_relative_to(base):
                raise ValueError("URDF package reference escapes declared package")
        else:
            target = (path.parent/filename).resolve()
        if not target.is_file():
            raise FileNotFoundError(target)
        element.set("filename",str(target))
        files.add(target)
    for link in root.findall("link"):
        for kind in ("collision","visual"):
            for index,element in enumerate(link.findall(kind)):
                element.set("name",f"{link.attrib['name']}__{kind}_{index}")
    extension = root.find("mujoco")
    if extension is None: extension = ET.SubElement(root,"mujoco")
    compiler = extension.find("compiler")
    if compiler is None: compiler = ET.SubElement(extension,"compiler")
    compiler.set("fusestatic","false"); compiler.set("discardvisual","false")
    spec = mujoco.MjSpec.from_string(ET.tostring(root,encoding="unicode"))
    missing_inertia = [link.attrib["name"] for link in root.findall("link") if link.find("inertial") is None]
    return spec,{"source_format":"xacro" if macro else "urdf","named_geoms":True,
                 "links_without_declared_inertia":missing_inertia,
                 "package_roots":{k:str(v) for k,v in package_roots.items()}},files


def compile_adapter(config,path,config_path,medium):
    """Return adapted model, effective adapter record and exact source files."""
    path,base = Path(path),Path(config_path).parent
    adapter = config.get("model_adapter",{})
    suffix = path.suffix.lower()
    files = {path}
    if suffix == ".mjb":
        if config.get("compiled_mujoco_version") != mujoco.__version__:
            raise ValueError("MJB requires matching compiled_mujoco_version metadata")
        chosen = path
        if medium == "sand" and config.get("sand_model"):
            chosen = (base/config["sand_model"]).resolve(); files.add(chosen)
        model = mujoco.MjModel.from_binary_path(str(chosen))
        return model,{"source_format":"mjb","mujoco_version":mujoco.__version__},files
    if suffix in (".mjz",".zip"):
        check_archive(path)
        with path.open("rb") as stream:
            spec = mujoco.MjSpec.from_zip(stream)
        report = {"source_format":"native_mjz"}
    else:
        roots = {key:(base/value).resolve() for key,value in config.get("package_roots",{}).items()}
        spec,report,files = urdf_spec(path,roots,adapter.get("xacro_mappings",{}))
    if adapter.get("floating_body"):
        body = spec.body(adapter["floating_body"])
        if body is None:
            raise ValueError("Declared floating body is missing")
        body.add_freejoint(name=adapter.get("floating_joint","catalog_freejoint"))
        body.pos = np.asarray(adapter.get("initial_pos_m",[0,0,.002]))
        report["floating_body"] = adapter["floating_body"]
    for joint in adapter.get("velocity_joints",[]):
        # This opt-in servo is a simulation actuator model, not supplied URDF.
        name = joint["joint"]
        if spec.joint(name) is None:
            raise ValueError(f"Velocity servo joint not found: {name}")
        gain = float(joint["kv"])
        if not np.isfinite(gain) or gain <= 0:
            raise ValueError("Velocity gain must be positive")
        actuator = spec.add_actuator(name=joint.get("name",name+"_velocity"),target=name,
            trntype=mujoco.mjtTrn.mjTRN_JOINT,gaintype=mujoco.mjtGain.mjGAIN_FIXED,
            biastype=mujoco.mjtBias.mjBIAS_AFFINE)
        actuator.gainprm[0] = gain; actuator.biasprm[2] = -gain
        actuator.ctrllimited = 1; actuator.ctrlrange = joint["ctrlrange"]
        actuator.forcelimited = 1; actuator.forcerange = joint["forcerange"]
    if adapter.get("add_floor"):
        if spec.geom("floor") is not None:
            raise ValueError("Adapter refuses to add a duplicate floor")
        spec.worldbody.add_geom(name="floor",type=mujoco.mjtGeom.mjGEOM_PLANE,size=[5,5,.01],
                                rgba=[.55,.55,.55,1],friction=[1,.005,.0001])
        spec.worldbody.add_light(pos=[0,0,3],dir=[0,0,-1])
    if adapter.get("add_keyframe"):
        spec.add_key(name=adapter["add_keyframe"])
    report["requested_adapter"] = adapter
    model = spec.compile()
    if medium == "sand" and model.npair:
        from .pipeline import convert_support_spec,fingerprint
        identity = fingerprint(model)
        removed = convert_support_spec(spec,config)
        model = spec.compile()
        if fingerprint(model) != identity:
            raise ValueError("Archive support-pair conversion changed physical identity")
        report["removed_explicit_pairs"] = removed
    report["model_mass_kg"] = float(model.body_mass.sum())
    report["native_inferred_masses_kg"] = {name:float(model.body_mass[model.body(name).id])
        for name in report.get("links_without_declared_inertia",[]) if model.body_mass[model.body(name).id] > 0}
    return model,report,files
