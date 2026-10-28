"""Resolve surface geometry into an explicitly owned body's local SI frame."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import yaml


def file_hash(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rotation(quaternion) -> np.ndarray:
    quaternion = np.asarray(quaternion, dtype=float)
    if quaternion.shape != (4,) or not np.all(np.isfinite(quaternion)):
        raise ValueError("Quaternion must be four finite wxyz components")
    if not np.isclose(np.linalg.norm(quaternion), 1, atol=1e-8):
        raise ValueError("Quaternion must have unit norm")
    result = np.empty(9)
    mujoco.mju_quat2Mat(result, quaternion)
    return result.reshape(3, 3)


def load_model(path, *, package_roots=None):
    """Load MJCF or URDF without modifying source files or physical parameters.

    URDF keeps fixed bodies and visual geoms. It retains URDF's fixed root;
    floating-root conversion/actuator synthesis are not silently invented.
    """
    path = Path(path).resolve()
    if path.suffix.lower() != ".urdf":
        return mujoco.MjModel.from_xml_path(str(path))
    roots = {name: Path(root).resolve() for name, root in (package_roots or {}).items()}
    xml = ET.parse(path).getroot()
    for element in xml.iter():
        if "filename" not in element.attrib:
            continue
        name = element.attrib["filename"]
        if name.startswith("package://"):
            package, _, relative = name[len("package://"):].partition("/")
            if package not in roots:
                raise ValueError(f"URDF package root is missing: {package}")
            resolved = roots[package] / relative
        else:
            resolved = path.parent / name
        if not resolved.is_file():
            raise FileNotFoundError(resolved)
        element.set("filename", str(resolved.resolve()))
    extension = xml.find("mujoco")
    if extension is None:
        extension = ET.SubElement(xml, "mujoco")
    compiler = extension.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(extension, "compiler")
    compiler.set("fusestatic", "false")
    compiler.set("discardvisual", "false")
    return mujoco.MjModel.from_xml_string(ET.tostring(xml, encoding="unicode"))


@dataclass(frozen=True)
class Surface:
    surface_id: str
    body_name: str
    centers: np.ndarray
    normals: np.ndarray
    areas: np.ndarray
    group: str = ""
    source: str = "explicit_arrays"

    def __post_init__(self):
        if not self.surface_id or not self.body_name:
            raise ValueError("Surface ID and body name are required")
        arrays = {}
        for key in ("centers", "normals", "areas"):
            array = np.array(getattr(self, key), dtype=np.float64, copy=True)
            if not np.all(np.isfinite(array)):
                raise ValueError(f"{self.surface_id}: {key} contains non-finite values")
            arrays[key] = array
        count = len(arrays["areas"])
        if count == 0 or arrays["areas"].shape != (count,):
            raise ValueError("Surface must contain a nonempty 1D area vector")
        if any(arrays[key].shape != (count, 3) for key in ("centers", "normals")):
            raise ValueError("Centroids and normals must have shape (triangles, 3)")
        if np.any(arrays["areas"] <= 0):
            raise ValueError("Triangle areas must be positive")
        if not np.allclose(np.linalg.norm(arrays["normals"], axis=1), 1, atol=1e-8):
            raise ValueError("Triangle normals must be unit vectors")
        for key, array in arrays.items():
            array.setflags(write=False)
            object.__setattr__(self, key, array)

    @classmethod
    def from_mesh(cls, surface_id, body_name, vertices, faces, **metadata):
        vertices = np.asarray(vertices, dtype=float)
        faces = np.asarray(faces, dtype=int)
        if vertices.ndim != 2 or vertices.shape[1] != 3:
            raise ValueError("Mesh vertices must have shape (vertices, 3)")
        if faces.ndim != 2 or faces.shape[1] != 3 or not len(faces):
            raise ValueError("Mesh faces must have shape (triangles, 3)")
        if np.any(faces < 0) or np.any(faces >= len(vertices)):
            raise ValueError("Mesh face index is out of bounds")
        triangles = vertices[faces]
        cross = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
        lengths = np.linalg.norm(cross, axis=1)
        if np.any(lengths <= 1e-16):
            raise ValueError("Mesh contains degenerate triangles")
        return cls(surface_id, body_name, triangles.mean(axis=1),
                   cross / lengths[:, None], lengths / 2, **metadata)


@dataclass(frozen=True)
class BoundSurface:
    surface: Surface
    body_id: int


def bind_surfaces(model, surfaces):
    result = []
    seen = set()
    for surface in surfaces:
        if surface.surface_id in seen:
            raise ValueError(f"Duplicate surface ID: {surface.surface_id}")
        seen.add(surface.surface_id)
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, surface.body_name)
        if body_id <= 0:
            raise ValueError(f"RFT owner must be a named robot body: {surface.body_name}")
        result.append(BoundSurface(surface, body_id))
    if not result:
        raise ValueError("No RFT surfaces were selected")
    return tuple(result)


def _revolution(profile, segments=16):
    vertices = [[0.0, 0.0, profile[0][1]]]
    for radius, z in profile[1:-1]:
        for index in range(segments):
            angle = 2 * np.pi * index / segments
            vertices.append([radius * np.cos(angle), radius * np.sin(angle), z])
    top = len(vertices)
    vertices.append([0.0, 0.0, profile[-1][1]])
    rings = len(profile) - 2
    faces = []
    for j in range(segments):
        k = (j + 1) % segments
        faces.append([0, 1 + k, 1 + j])
        for ring in range(rings - 1):
            a, b = 1 + ring * segments + j, 1 + ring * segments + k
            c, d = a + segments, b + segments
            faces.extend([[a, b, c], [b, d, c]])
        faces.append([top, 1 + (rings - 1) * segments + j,
                      1 + (rings - 1) * segments + k])
    return np.asarray(vertices), np.asarray(faces)


def primitive_mesh(geom_type, size, segments=16):
    """Closed outward tessellations; input sizes follow MuJoCo conventions."""
    if segments < 8:
        raise ValueError("Primitive tessellation needs at least eight segments")
    size = np.asarray(size)
    if geom_type == mujoco.mjtGeom.mjGEOM_BOX:
        vertices = np.asarray([[-1,-1,-1],[1,-1,-1],[1,1,-1],[-1,1,-1],
                               [-1,-1,1],[1,-1,1],[1,1,1],[-1,1,1]]) * size
        faces = np.asarray([[0,2,1],[0,3,2],[4,5,6],[4,6,7],[0,1,5],[0,5,4],
                            [1,2,6],[1,6,5],[2,3,7],[2,7,6],[3,0,4],[3,4,7]])
        return vertices, faces
    if geom_type in (mujoco.mjtGeom.mjGEOM_SPHERE, mujoco.mjtGeom.mjGEOM_ELLIPSOID):
        angles = np.linspace(-np.pi/2, np.pi/2, 9)
        profile = [(float(np.cos(a)), float(np.sin(a))) for a in angles]
        vertices, faces = _revolution(profile, segments)
        scale = size[:3] if geom_type == mujoco.mjtGeom.mjGEOM_ELLIPSOID else size[0]
        return vertices * scale, faces
    if geom_type == mujoco.mjtGeom.mjGEOM_CAPSULE:
        radius, half = size[:2]
        bottom = [(radius*np.cos(a), -half+radius*np.sin(a))
                  for a in np.linspace(-np.pi/2, 0, 5)]
        top = [(radius*np.cos(a), half+radius*np.sin(a))
               for a in np.linspace(0, np.pi/2, 5)]
        return _revolution(bottom + top, segments)
    if geom_type == mujoco.mjtGeom.mjGEOM_CYLINDER:
        radius, half = size[:2]
        return _revolution([(0,-half),(radius,-half),(radius,half),(0,half)], segments)
    raise ValueError(f"Unsupported robot RFT geom type: {geom_type}")


def mesh_from_geom(model, geom_id, *, segments=16):
    """Return compiled geometry in its owning body frame, without changing it."""
    if not 0 <= geom_id < model.ngeom:
        raise ValueError(f"Invalid geom ID: {geom_id}")
    body_id = int(model.geom_bodyid[geom_id])
    body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
    if body_id == 0 or body_name is None:
        raise ValueError("RFT geoms must belong to named robot bodies")
    if model.geom_type[geom_id] == mujoco.mjtGeom.mjGEOM_MESH:
        mesh_id = int(model.geom_dataid[geom_id])
        va, fa = int(model.mesh_vertadr[mesh_id]), int(model.mesh_faceadr[mesh_id])
        vertices = np.asarray(model.mesh_vert[va:va+model.mesh_vertnum[mesh_id]], dtype=float)
        faces = np.asarray(model.mesh_face[fa:fa+model.mesh_facenum[mesh_id]], dtype=int)
        # Compiled vertices already include asset scale and recentering. Use
        # compiled geom pose as its matching route; do not reapply mesh_pos.
    else:
        vertices, faces = primitive_mesh(model.geom_type[geom_id], model.geom_size[geom_id], segments)
    vertices = vertices @ rotation(model.geom_quat[geom_id]).T + model.geom_pos[geom_id]
    return vertices, faces


def surface_from_geom(model, geom_id, *, surface_id=None, group="", segments=16):
    vertices, faces = mesh_from_geom(model, geom_id, segments=segments)
    body_id = int(model.geom_bodyid[geom_id])
    body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
    geom_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
    return Surface.from_mesh(surface_id or f"{body_name}/geom_{geom_id}", body_name,
                             vertices, faces, group=group or body_name,
                             source=f"compiled_geom:{geom_name or geom_id}")


def load_surfaces(model, config_path):
    """Explicit proxy/geom recipes; collision discovery must be requested."""
    path = Path(config_path).resolve()
    config = yaml.safe_load(path.read_text(encoding="utf-8"))
    surfaces = []
    for spec in config.get("surfaces", []):
        if "geom" in spec:
            gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, spec["geom"])
            surface = surface_from_geom(model, gid, surface_id=spec["id"], group=spec.get("group", ""))
            if "body" in spec and surface.body_name != spec["body"]:
                raise ValueError("Selected geom belongs to a different body")
        else:
            import open3d as o3d

            mesh_path = (path.parent / spec["mesh"]).resolve()
            units = {"m": 1.0, "mm": 0.001}
            if spec.get("units") not in units:
                raise ValueError("Proxy mesh units must explicitly be 'm' or 'mm'")
            mesh = o3d.io.read_triangle_mesh(str(mesh_path))
            vertices = np.asarray(mesh.vertices) * units[spec["units"]]
            transform = spec.get("transform_to_body", {})
            vertices = vertices @ rotation(transform.get("quat_wxyz", [1,0,0,0])).T
            vertices += np.asarray(transform.get("pos_m", [0,0,0]), dtype=float)
            surface = Surface.from_mesh(spec["id"], spec["body"], vertices,
                                         np.asarray(mesh.triangles),
                                         group=spec.get("group", spec["body"]),
                                         source=f"{mesh_path.name}:sha256:{file_hash(mesh_path)}")
        surfaces.append(surface)
    selection = config.get("select_collision_geoms")
    if selection is not None:
        bodies = selection.get("bodies")
        group = selection.get("geom_group")
        excluded = set(selection.get("exclude_bodies", []))
        for gid in range(model.ngeom):
            body_id = int(model.geom_bodyid[gid])
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
            if body_id <= 0 or name is None or name in excluded:
                continue
            if bodies is not None and name not in bodies:
                continue
            if group is not None and model.geom_group[gid] != group:
                continue
            if not (model.geom_contype[gid] or model.geom_conaffinity[gid]):
                continue
            surfaces.append(surface_from_geom(model, gid, segments=selection.get("segments", 16)))
    # Validate ownership and duplicate IDs before returning any bindings.
    bind_surfaces(model, surfaces)
    return tuple(surfaces), config
