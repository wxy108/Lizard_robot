"""Closed-shell qualification and bounded isotropic remeshing, never CAD union."""
from pathlib import Path

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree


class GeometryRejected(ValueError):
    def __init__(self, reasons, metrics):
        self.reasons, self.metrics = reasons, metrics
        super().__init__("; ".join(reasons))


def mesh_arrays(mesh):
    return np.asarray(mesh.vertices).copy(), np.asarray(mesh.triangles).copy()


def make_mesh(vertices, faces):
    vertices, faces = np.asarray(vertices), np.asarray(faces)
    if vertices.ndim != 2 or vertices.shape[1] != 3 or not np.all(np.isfinite(vertices)):
        raise ValueError("Mesh needs finite XYZ vertices")
    if faces.ndim != 2 or faces.shape[1] != 3 or not len(faces) or faces.dtype.kind not in "iu":
        raise ValueError("Mesh needs nonempty integer triangle indices")
    if np.any(faces < 0) or np.any(faces >= len(vertices)):
        raise ValueError("Mesh face indices exceed vertex bounds")
    return o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(np.asarray(vertices)),
                                    o3d.utility.Vector3iVector(np.asarray(faces, dtype=np.int32)))


def read_mesh(path, units):
    if units not in ("m", "mm"):
        raise ValueError("Geometry-only source must declare units m or mm")
    if Path(path).suffix.lower() not in (".stl", ".obj", ".ply"):
        raise ValueError("Supported triangle formats: STL, OBJ, PLY; CAD needs an explicit converter")
    mesh = o3d.io.read_triangle_mesh(str(path), enable_post_processing=False)
    if mesh.is_empty() or not len(mesh.triangles):
        raise ValueError("Input contains no triangle surface")
    mesh.vertices = o3d.utility.Vector3dVector(np.asarray(mesh.vertices) * (0.001 if units == "mm" else 1))
    # Exact STL facet welding is representation normalization, not hole repair.
    mesh.remove_duplicated_vertices()
    mesh.remove_unreferenced_vertices()
    return mesh


def audit(mesh):
    vertices, faces = mesh_arrays(mesh)
    if not len(faces) or not np.all(np.isfinite(vertices)):
        raise ValueError("Empty or non-finite mesh")
    points = vertices[faces]
    cross = np.cross(points[:, 1]-points[:, 0], points[:, 2]-points[:, 0])
    areas = np.linalg.norm(cross, axis=1)/2
    edges = np.linalg.norm(points-np.roll(points, 1, axis=1), axis=2)
    canonical = np.sort(faces, axis=1)
    duplicate = len(faces)-len(np.unique(canonical, axis=0))
    directed = np.concatenate([faces[:, [0,1]], faces[:, [1,2]], faces[:, [2,0]]])
    keys, inverse, counts = np.unique(np.sort(directed, axis=1), axis=0, return_inverse=True, return_counts=True)
    orientation = np.bincount(inverse, weights=np.where(directed[:,0] < directed[:,1], 1, -1))
    centers = points.mean(axis=1)
    nearest = (cKDTree(centers).query(centers, k=2, workers=1)[0][:,1]
               if len(centers) > 1 else np.ones(1))
    labels, component_faces, _ = mesh.cluster_connected_triangles()
    labels = np.asarray(labels)
    # Evaluate each shell independently; signed volumes of oppositely wound
    # components must not cancel and a small inner shell must not be dropped.
    volumes = [float(np.einsum('ij,ij->i', points[labels == c,0],
               np.cross(points[labels == c,1],points[labels == c,2])).sum()/6)
               for c in range(len(component_faces))]
    return {
        "vertices": len(vertices), "triangles": len(faces),
        "bbox_extent_m": np.ptp(vertices,axis=0).tolist(),
        "boundary_edges": int(np.count_nonzero(counts == 1)),
        "nonmanifold_edges": int(np.count_nonzero(counts > 2)),
        "inconsistent_winding_edges": int(np.count_nonzero((counts == 2) & (orientation != 0))),
        "duplicate_faces": int(duplicate), "degenerate_faces": int(np.count_nonzero(areas <= 1e-16)),
        "vertex_manifold": bool(mesh.is_vertex_manifold()),
        "self_intersection_pairs": len(mesh.get_self_intersecting_triangles()),
        "components": len(component_faces), "component_signed_volumes_m3": volumes,
        "area_m2": float(areas.sum()),
        "area_p95_p05": float(np.percentile(areas,95)/max(np.percentile(areas,5),1e-30)),
        "centroid_spacing_p95_p05": float(np.percentile(nearest,95)/max(np.percentile(nearest,5),1e-30)),
        "edge_aspect_max": float(np.max(edges.max(axis=1)/np.maximum(edges.min(axis=1),1e-30))),
    }


def qualify(mesh, limits, *, final=True):
    if len(mesh.triangles) > limits.get("max_triangles",20000):
        raise GeometryRejected(["Triangle budget exceeded before native topology audit"],{"triangles":len(mesh.triangles)})
    metrics = audit(mesh)
    failures = [f"{k}={metrics[k]}" for k in
                ("boundary_edges", "nonmanifold_edges", "inconsistent_winding_edges", "duplicate_faces",
                 "degenerate_faces", "self_intersection_pairs") if metrics[k]]
    if not metrics["vertex_manifold"]:
        failures.append("nonmanifold vertex")
    if metrics["components"] != 1:
        failures.append("Multiple shells: select each intended exterior separately; no automatic union")
    if any(v <= 0 for v in metrics["component_signed_volumes_m3"]):
        failures.append("Shell winding must be outward with positive volume")
    if metrics["triangles"] > limits.get("max_triangles", 20000):
        failures.append("Triangle budget exceeded")
    if final:
        for key, default in (("area_p95_p05", 15), ("centroid_spacing_p95_p05", 10), ("edge_aspect_max", 12)):
            if metrics[key] > limits.get(key, default):
                failures.append(f"{key} exceeds declared engineering limit")
    if failures:
        raise GeometryRejected(failures, metrics)
    return metrics


def deviation(candidate, reference):
    """Bidirectional deterministic vertex+centroid samples, not exact Hausdorff."""
    def distances(source, target):
        v, f = mesh_arrays(source)
        points = np.vstack([v, v[f].mean(axis=1)]).astype(np.float32)
        scene = o3d.t.geometry.RaycastingScene()
        scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(target))
        d = scene.compute_distance(o3d.core.Tensor(points)).numpy()
        return {"p95_m": float(np.percentile(d,95)), "max_sampled_m": float(d.max())}
    return {"candidate_to_reference": distances(candidate, reference),
            "reference_to_candidate": distances(reference, candidate)}


def prepare(mesh, recipe, directory):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=False)
    # Reject ambiguous input BEFORE attempting an algorithm that might hide it.
    before = qualify(mesh, recipe.get("limits", {}), final=False)
    original = directory / "source.ply"
    o3d.io.write_triangle_mesh(str(original), mesh, write_ascii=False)
    method = recipe.get("method", "isotropic")
    if method == "isotropic":
        import pymeshlab
        edge = float(recipe["target_edge_m"])
        tolerance = float(recipe["max_deviation_m"])
        iterations = int(recipe.get("iterations", 5))
        if not 1e-5 <= edge <= 1 or not 0 < tolerance <= edge or not 1 <= iterations <= 20:
            raise ValueError("Remesh edge/deviation must be finite, positive physical lengths")
        estimate = 2*before["area_m2"]/(np.sqrt(3)/4*edge*edge)
        if estimate > recipe.get("limits",{}).get("max_triangles",20000):
            raise ValueError("Estimated remesh size exceeds triangle budget; choose a justified edge length")
        ms = pymeshlab.MeshSet()
        ms.add_mesh(pymeshlab.Mesh(*mesh_arrays(mesh)))
        ms.meshing_isotropic_explicit_remeshing(iterations=iterations,
            adaptive=False, targetlen=pymeshlab.PureValue(edge), featuredeg=30,
            # MeshLab's local constraint is not a symmetric global bound.
            # Keep a construction margin; the independent two-way audit below
            # still enforces the original requested tolerance without widening.
            checksurfdist=True, maxsurfdist=pymeshlab.PureValue(tolerance/2),
            splitflag=True, collapseflag=True, swapflag=True, smoothflag=True, reprojectflag=True)
        prepared = make_mesh(ms.current_mesh().vertex_matrix(), ms.current_mesh().face_matrix())
    elif method == "preserve":
        prepared = make_mesh(*mesh_arrays(mesh))
    else:
        raise ValueError("Allowed methods are isotropic and preserve; reconstruction requires reviewed exterior")
    # Retain failed candidates too, so a rejection can be investigated.
    o3d.io.write_triangle_mesh(str(directory / "prepared.ply"), prepared, write_ascii=False)
    after = qualify(prepared, recipe.get("limits", {}))
    errors = deviation(prepared, mesh)
    bound = float(recipe.get("max_deviation_m", 1e-7))
    if any(x["max_sampled_m"] > bound + 1e-7 for x in errors.values()):
        raise GeometryRejected(["Bidirectional sampled deviation exceeds declared bound"], errors)
    return prepared, {"before": before, "after": after, "sampled_deviation": errors,
                      "method": method, "recipe": recipe,
                      "construction_deviation_margin":0.5 if method == "isotropic" else None,
                      "effective_limits":{"area_p95_p05":15,"centroid_spacing_p95_p05":10,
                                          "edge_aspect_max":12,"max_triangles":20000,**recipe.get("limits",{})}}
