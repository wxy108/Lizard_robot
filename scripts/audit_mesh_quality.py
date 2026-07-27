"""Audit STL topology, triangle quality, and RFT-centroid uniformity.

This script is read-only: it never rewrites a mesh. Results are emitted as
JSON and CSV so candidate mesh pipelines can be compared before any active
asset is replaced.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import defaultdict
from datetime import datetime
from pathlib import Path

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree


ROOT = Path(__file__).resolve().parents[1]
BODIES = ("Mid", "Front", "FR", "FL", "Back", "HR", "HL", "Tail")


def percentile(values: np.ndarray, q: float) -> float:
    if values.size == 0:
        return float("nan")
    return float(np.percentile(values, q))


def safe_ratio(numerator: float, denominator: float) -> float:
    if not np.isfinite(denominator) or denominator <= 0:
        return float("inf")
    return float(numerator / denominator)


def coefficient_of_variation(values: np.ndarray) -> float:
    if values.size == 0:
        return float("nan")
    mean = float(np.mean(values))
    return safe_ratio(float(np.std(values)), mean)


def locate_mesh(mesh_dir: Path, body: str) -> Path:
    aliases = {
        body.lower(),
        f"{body.lower()}_edited",
        f"{body.lower()}-edited",
    }
    candidates = [
        path
        for path in mesh_dir.iterdir()
        if path.is_file()
        and path.suffix.lower() == ".stl"
        and path.stem.lower() in aliases
    ]
    if len(candidates) != 1:
        raise FileNotFoundError(
            f"Expected exactly one STL for {body} in {mesh_dir}; "
            f"found {[path.name for path in candidates]}"
        )
    return candidates[0]


def welded_topology(
    vertices: np.ndarray,
    triangles: np.ndarray,
    tolerance: float,
) -> tuple[np.ndarray, np.ndarray, int]:
    """Merge coincident STL facet vertices for meaningful edge topology."""
    keys = np.rint(vertices / tolerance).astype(np.int64)
    _, first, inverse = np.unique(
        keys, axis=0, return_index=True, return_inverse=True
    )
    welded_vertices = vertices[first]
    welded_triangles = inverse[triangles]

    repeated_vertex = (
        (welded_triangles[:, 0] == welded_triangles[:, 1])
        | (welded_triangles[:, 1] == welded_triangles[:, 2])
        | (welded_triangles[:, 2] == welded_triangles[:, 0])
    )
    collapsed = int(np.count_nonzero(repeated_vertex))
    welded_triangles = welded_triangles[~repeated_vertex]
    return welded_vertices, welded_triangles, collapsed


def edge_topology(
    triangles: np.ndarray,
) -> tuple[int, int, int, int]:
    edge_occurrences: dict[tuple[int, int], list[tuple[int, int]]] = defaultdict(
        list
    )
    for triangle in triangles:
        directed = (
            (int(triangle[0]), int(triangle[1])),
            (int(triangle[1]), int(triangle[2])),
            (int(triangle[2]), int(triangle[0])),
        )
        for start, end in directed:
            edge_occurrences[tuple(sorted((start, end)))].append((start, end))

    boundary = 0
    nonmanifold = 0
    inconsistent_winding = 0
    shared = 0
    for occurrences in edge_occurrences.values():
        if len(occurrences) == 1:
            boundary += 1
        elif len(occurrences) > 2:
            nonmanifold += 1
        else:
            shared += 1
            first, second = occurrences
            if first == second:
                inconsistent_winding += 1
    return boundary, nonmanifold, shared, inconsistent_winding


def signed_volume(vertices: np.ndarray, triangles: np.ndarray) -> float:
    points = vertices[triangles]
    return float(
        np.einsum(
            "ij,ij->i",
            points[:, 0],
            np.cross(points[:, 1], points[:, 2]),
        ).sum()
        / 6.0
    )


def triangle_geometry(
    vertices: np.ndarray,
    triangles: np.ndarray,
) -> dict[str, np.ndarray]:
    points = vertices[triangles]
    edges = np.stack(
        (
            np.linalg.norm(points[:, 1] - points[:, 0], axis=1),
            np.linalg.norm(points[:, 2] - points[:, 1], axis=1),
            np.linalg.norm(points[:, 0] - points[:, 2], axis=1),
        ),
        axis=1,
    )
    cross = np.cross(points[:, 1] - points[:, 0], points[:, 2] - points[:, 0])
    areas = 0.5 * np.linalg.norm(cross, axis=1)
    centroids = np.mean(points, axis=1)

    shortest = np.min(edges, axis=1)
    longest = np.max(edges, axis=1)
    edge_aspect = np.divide(
        longest,
        shortest,
        out=np.full_like(longest, np.inf),
        where=shortest > 0,
    )

    a, b, c = edges[:, 0], edges[:, 1], edges[:, 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        angle_a = np.arccos(
            np.clip((a * a + c * c - b * b) / (2.0 * a * c), -1.0, 1.0)
        )
        angle_b = np.arccos(
            np.clip((a * a + b * b - c * c) / (2.0 * a * b), -1.0, 1.0)
        )
        angle_c = math.pi - angle_a - angle_b
    min_angles_deg = np.degrees(
        np.nanmin(np.stack((angle_a, angle_b, angle_c), axis=1), axis=1)
    )

    if len(centroids) > 1:
        nearest = cKDTree(centroids).query(centroids, k=2, workers=-1)[0][:, 1]
    else:
        nearest = np.asarray([], dtype=float)

    return {
        "areas": areas,
        "centroids": centroids,
        "edge_aspect": edge_aspect,
        "min_angles_deg": min_angles_deg,
        "centroid_nearest": nearest,
    }


def audit_mesh(path: Path, mesh_set: str, body: str, check_self: bool) -> dict:
    mesh = o3d.io.read_triangle_mesh(str(path), enable_post_processing=False)
    vertices = np.asarray(mesh.vertices, dtype=float)
    triangles = np.asarray(mesh.triangles, dtype=np.int64)
    if len(vertices) == 0 or len(triangles) == 0:
        raise ValueError(f"Empty mesh: {path}")

    bbox_min = np.min(vertices, axis=0)
    bbox_max = np.max(vertices, axis=0)
    bbox_extent = bbox_max - bbox_min
    bbox_diagonal = float(np.linalg.norm(bbox_extent))
    weld_tolerance = max(bbox_diagonal * 1e-7, 1e-12)
    welded_vertices, welded_triangles, collapsed = welded_topology(
        vertices, triangles, weld_tolerance
    )
    canonical_triangles = np.sort(welded_triangles, axis=1)
    _, canonical_counts = np.unique(
        canonical_triangles, axis=0, return_counts=True
    )
    duplicate_welded_triangles = int(
        np.sum(np.maximum(canonical_counts - 1, 0))
    )

    geometry = triangle_geometry(vertices, triangles)
    areas = geometry["areas"]
    nonzero_areas = areas[areas > bbox_diagonal * bbox_diagonal * 1e-14]
    edge_aspect = geometry["edge_aspect"]
    min_angles = geometry["min_angles_deg"]
    nearest = geometry["centroid_nearest"]
    coincident_centroid_fraction = (
        float(np.mean(nearest <= weld_tolerance)) if nearest.size else 0.0
    )

    boundary, nonmanifold, shared, inconsistent = edge_topology(
        welded_triangles
    )

    welded_mesh = o3d.geometry.TriangleMesh(
        o3d.utility.Vector3dVector(welded_vertices),
        o3d.utility.Vector3iVector(welded_triangles),
    )
    welded_mesh.remove_duplicated_triangles()
    welded_mesh.remove_degenerate_triangles()
    welded_mesh.remove_unreferenced_vertices()

    cluster_ids, cluster_counts, _ = welded_mesh.cluster_connected_triangles()
    component_count = len(cluster_counts)
    largest_component_fraction = (
        max(cluster_counts) / len(welded_mesh.triangles)
        if len(welded_mesh.triangles)
        else 0.0
    )

    self_intersections = None
    if check_self:
        self_intersections = len(welded_mesh.get_self_intersecting_triangles())

    area_p05 = percentile(nonzero_areas, 5)
    area_p95 = percentile(nonzero_areas, 95)
    nearest_p05 = percentile(nearest, 5)
    nearest_p95 = percentile(nearest, 95)

    return {
        "mesh_set": mesh_set,
        "body": body,
        "path": str(path),
        "vertices_raw": int(len(vertices)),
        "vertices_welded": int(len(welded_vertices)),
        "triangles": int(len(triangles)),
        "triangles_collapsed_by_weld": collapsed,
        "duplicate_welded_triangles": duplicate_welded_triangles,
        "coincident_centroid_fraction": coincident_centroid_fraction,
        "center_x_m": float(np.mean(vertices[:, 0])),
        "center_y_m": float(np.mean(vertices[:, 1])),
        "center_z_m": float(np.mean(vertices[:, 2])),
        "bbox_x_m": float(bbox_extent[0]),
        "bbox_y_m": float(bbox_extent[1]),
        "bbox_z_m": float(bbox_extent[2]),
        "surface_area_m2": float(np.sum(areas)),
        "degenerate_triangles": int(len(areas) - len(nonzero_areas)),
        "area_min_m2": float(np.min(nonzero_areas)) if len(nonzero_areas) else 0.0,
        "area_p05_m2": area_p05,
        "area_median_m2": percentile(nonzero_areas, 50),
        "area_p95_m2": area_p95,
        "area_max_m2": float(np.max(nonzero_areas)) if len(nonzero_areas) else 0.0,
        "area_cv": coefficient_of_variation(nonzero_areas),
        "area_p95_p05": safe_ratio(area_p95, area_p05),
        "edge_aspect_p95": percentile(edge_aspect, 95),
        "edge_aspect_max": float(np.max(edge_aspect)),
        "min_angle_p05_deg": percentile(min_angles, 5),
        "sliver_fraction_lt_10deg": float(np.mean(min_angles < 10.0)),
        "centroid_nn_p05_m": nearest_p05,
        "centroid_nn_median_m": percentile(nearest, 50),
        "centroid_nn_p95_m": nearest_p95,
        "centroid_nn_cv": coefficient_of_variation(nearest),
        "centroid_nn_p95_p05": safe_ratio(nearest_p95, nearest_p05),
        "boundary_edges": boundary,
        "nonmanifold_edges": nonmanifold,
        "shared_edges": shared,
        "inconsistent_winding_edges": inconsistent,
        "connected_components": component_count,
        "largest_component_fraction": float(largest_component_fraction),
        "watertight": bool(welded_mesh.is_watertight()),
        "edge_manifold_with_boundary": bool(
            welded_mesh.is_edge_manifold(allow_boundary_edges=True)
        ),
        "vertex_manifold": bool(welded_mesh.is_vertex_manifold()),
        "orientable": bool(welded_mesh.is_orientable()),
        "signed_volume_m3": signed_volume(
            np.asarray(welded_mesh.vertices),
            np.asarray(welded_mesh.triangles),
        ),
        "self_intersecting_triangle_pairs": self_intersections,
    }


def parse_mesh_sets(values: list[str]) -> dict[str, Path]:
    if not values:
        return {
            "source_cad_export": ROOT / "models" / "meshes",
            "active_rft": ROOT / "asset",
        }
    result = {}
    for value in values:
        if "=" not in value:
            raise ValueError("--mesh-set must have the form NAME=PATH")
        name, raw_path = value.split("=", 1)
        path = Path(raw_path).expanduser().resolve()
        if not path.is_dir():
            raise NotADirectoryError(path)
        result[name] = path
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--mesh-set",
        action="append",
        default=[],
        metavar="NAME=PATH",
        help="repeat for every mesh directory to compare",
    )
    parser.add_argument(
        "--skip-self-intersections",
        action="store_true",
        help="skip the slower exact triangle-intersection check",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="default: outputs/mesh_audit/run_<timestamp>",
    )
    args = parser.parse_args()

    mesh_sets = parse_mesh_sets(args.mesh_set)
    output_dir = args.output_dir
    if output_dir is None:
        output_dir = (
            ROOT
            / "outputs"
            / "mesh_audit"
            / datetime.now().strftime("run_%Y-%m-%d_%H%M%S_%f")
        )
    output_dir = output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=False)

    rows = []
    for set_name, mesh_dir in mesh_sets.items():
        for body in BODIES:
            path = locate_mesh(mesh_dir, body)
            print(f"Auditing {set_name}/{body}: {path.name}", flush=True)
            rows.append(
                audit_mesh(
                    path,
                    set_name,
                    body,
                    check_self=not args.skip_self_intersections,
                )
            )

    json_path = output_dir / "mesh_quality.json"
    csv_path = output_dir / "mesh_quality.csv"
    with json_path.open("w", encoding="utf-8") as handle:
        json.dump(rows, handle, indent=2, allow_nan=True)
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    print(f"\nWrote {json_path}")
    print(f"Wrote {csv_path}")


if __name__ == "__main__":
    main()
