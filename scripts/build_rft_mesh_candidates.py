"""Build isolated, reproducible, topology-gated RFT mesh candidates.

Pipeline:
  1. load the manually unified Fusion external envelope;
  2. make winding consistent and outward;
  3. sample the surface deterministically;
  4. reconstruct one closed surface with MeshLab Screened Poisson;
  5. use MeshLab isotropic remeshing to regularize edge length/aspect ratio;
  6. reject candidates that fail topology, orientation, uniformity, or
     reference-surface-deviation gates.

The script refuses to write into the active ``asset`` directory.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
import sys
from datetime import datetime
from pathlib import Path

import numpy as np
import open3d as o3d
import pymeshlab

from audit_mesh_quality import BODIES, audit_mesh


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_INPUT = ROOT / "models" / "mesh_sources" / "fusion_external_envelope"
DEFAULT_RECIPE = ROOT / "configs" / "rft_mesh_recipe.json"
ACTIVE_ASSET = (ROOT / "asset").resolve()


def portable_path(path: Path) -> str:
    resolved = path.resolve()
    try:
        return resolved.relative_to(ROOT).as_posix()
    except ValueError:
        return str(resolved)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest().upper()


def signed_volume(mesh: o3d.geometry.TriangleMesh) -> float:
    vertices = np.asarray(mesh.vertices)
    triangles = np.asarray(mesh.triangles)
    points = vertices[triangles]
    return float(
        np.einsum(
            "ij,ij->i",
            points[:, 0],
            np.cross(points[:, 1], points[:, 2]),
        ).sum()
        / 6.0
    )


def clean_and_orient(mesh: o3d.geometry.TriangleMesh) -> o3d.geometry.TriangleMesh:
    mesh.remove_duplicated_vertices()
    mesh.remove_duplicated_triangles()
    mesh.remove_degenerate_triangles()
    mesh.remove_unreferenced_vertices()
    if not mesh.orient_triangles():
        raise ValueError("Mesh cannot be oriented consistently")
    if signed_volume(mesh) < 0:
        triangles = np.asarray(mesh.triangles).copy()
        triangles[:, [1, 2]] = triangles[:, [2, 1]]
        mesh.triangles = o3d.utility.Vector3iVector(triangles)
    mesh.compute_triangle_normals()
    mesh.compute_vertex_normals()
    return mesh


def largest_component(
    mesh: o3d.geometry.TriangleMesh,
) -> o3d.geometry.TriangleMesh:
    labels, counts, _ = mesh.cluster_connected_triangles()
    labels_array = np.asarray(labels)
    counts_array = np.asarray(counts)
    if len(counts_array) > 1:
        mesh.remove_triangles_by_mask(
            labels_array != int(np.argmax(counts_array))
        )
        mesh.remove_unreferenced_vertices()
    return mesh


def mesh_points(mesh: o3d.geometry.TriangleMesh) -> np.ndarray:
    vertices = np.asarray(mesh.vertices)
    triangles = np.asarray(mesh.triangles)
    centroids = vertices[triangles].mean(axis=1)
    return np.vstack((vertices, centroids)).astype(np.float32)


def point_to_surface_distances(
    points: np.ndarray,
    surface: o3d.geometry.TriangleMesh,
) -> np.ndarray:
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(surface))
    return scene.compute_distance(o3d.core.Tensor(points)).numpy()


def surface_deviation(
    candidate: o3d.geometry.TriangleMesh,
    reference: o3d.geometry.TriangleMesh,
) -> dict[str, float]:
    candidate_to_reference = point_to_surface_distances(
        mesh_points(candidate), reference
    )
    reference_to_candidate = point_to_surface_distances(
        mesh_points(reference), candidate
    )
    return {
        "candidate_to_reference_p95_m": float(
            np.percentile(candidate_to_reference, 95)
        ),
        "candidate_to_reference_max_m": float(
            np.max(candidate_to_reference)
        ),
        "reference_to_candidate_p95_m": float(
            np.percentile(reference_to_candidate, 95)
        ),
        "reference_to_candidate_max_m": float(
            np.max(reference_to_candidate)
        ),
    }


def reconstruct_poisson(
    reference: o3d.geometry.TriangleMesh,
    point_cloud_path: Path,
    output_path: Path,
    sample_points: int,
    depth: int,
    scale: float,
    seed: int,
) -> o3d.geometry.TriangleMesh:
    o3d.utility.random.seed(seed)
    points = reference.sample_points_poisson_disk(
        number_of_points=sample_points,
        init_factor=5,
        use_triangle_normal=True,
    )
    if not o3d.io.write_point_cloud(
        str(point_cloud_path), points, write_ascii=False
    ):
        raise OSError(f"Failed to write point cloud: {point_cloud_path}")
    mesh_set = pymeshlab.MeshSet()
    mesh_set.load_new_mesh(str(point_cloud_path))
    mesh_set.generate_surface_reconstruction_screened_poisson(
        depth=depth,
        fulldepth=5,
        scale=scale,
        samplespernode=1.5,
        pointweight=4.0,
        iters=8,
        confidence=False,
        preclean=True,
        # Poisson uses reduction order internally; a single worker keeps the
        # generated topology/hash reproducible across subset and full builds.
        threads=1,
    )
    mesh_set.save_current_mesh(str(output_path))
    reconstructed = o3d.io.read_triangle_mesh(
        str(output_path), enable_post_processing=False
    )
    # STL stores independent facet vertices. Weld identical coordinates before
    # connected-component analysis, otherwise every facet can be mistaken for
    # a separate component and "largest" collapses the reconstruction to one
    # or two triangles.
    reconstructed = clean_and_orient(reconstructed)
    reconstructed = largest_component(reconstructed)
    reconstructed = clean_and_orient(reconstructed)
    o3d.io.write_triangle_mesh(str(output_path), reconstructed)
    return reconstructed


def isotropic_remesh(
    source_path: Path,
    output_path: Path,
    target_edge_m: float,
    iterations: int,
    feature_angle_deg: float,
    max_surface_distance_m: float,
) -> None:
    mesh_set = pymeshlab.MeshSet()
    mesh_set.load_new_mesh(str(source_path))
    mesh_set.meshing_isotropic_explicit_remeshing(
        iterations=iterations,
        adaptive=False,
        selectedonly=False,
        targetlen=pymeshlab.PureValue(target_edge_m),
        featuredeg=feature_angle_deg,
        checksurfdist=True,
        maxsurfdist=pymeshlab.PureValue(max_surface_distance_m),
        splitflag=True,
        collapseflag=True,
        swapflag=True,
        smoothflag=True,
        reprojectflag=True,
    )
    mesh_set.save_current_mesh(str(output_path))


def evaluate_gates(
    metrics: dict,
    deviation: dict[str, float],
    max_area_ratio: float,
    max_sliver_fraction: float,
    max_centroid_ratio: float,
    max_surface_p95_m: float,
    max_surface_max_m: float,
) -> list[str]:
    failures = []
    required_true = (
        "watertight",
        "edge_manifold_with_boundary",
        "vertex_manifold",
        "orientable",
    )
    for field in required_true:
        if not metrics[field]:
            failures.append(f"{field}=false")
    required_zero = (
        "boundary_edges",
        "nonmanifold_edges",
        "inconsistent_winding_edges",
        "degenerate_triangles",
        "duplicate_welded_triangles",
        "self_intersecting_triangle_pairs",
    )
    for field in required_zero:
        if metrics[field] != 0:
            failures.append(f"{field}={metrics[field]}")
    if metrics["connected_components"] != 1:
        failures.append(
            f"connected_components={metrics['connected_components']}"
        )
    if metrics["signed_volume_m3"] <= 0:
        failures.append(f"signed_volume_m3={metrics['signed_volume_m3']}")
    if metrics["area_p95_p05"] > max_area_ratio:
        failures.append(f"area_p95_p05={metrics['area_p95_p05']:.3f}")
    if metrics["sliver_fraction_lt_10deg"] > max_sliver_fraction:
        failures.append(
            "sliver_fraction_lt_10deg="
            f"{metrics['sliver_fraction_lt_10deg']:.4f}"
        )
    if metrics["centroid_nn_p95_p05"] > max_centroid_ratio:
        failures.append(
            f"centroid_nn_p95_p05={metrics['centroid_nn_p95_p05']:.3f}"
        )
    if deviation["candidate_to_reference_p95_m"] > max_surface_p95_m:
        failures.append(
            "candidate_to_reference_p95_m="
            f"{deviation['candidate_to_reference_p95_m']:.6f}"
        )
    if deviation["candidate_to_reference_max_m"] > max_surface_max_m:
        failures.append(
            "candidate_to_reference_max_m="
            f"{deviation['candidate_to_reference_max_m']:.6f}"
        )
    return failures


def parse_args() -> argparse.Namespace:
    recipe_parser = argparse.ArgumentParser(add_help=False)
    recipe_parser.add_argument("--recipe", type=Path, default=DEFAULT_RECIPE)
    recipe_args, _ = recipe_parser.parse_known_args()
    with recipe_args.recipe.resolve().open(encoding="utf-8") as handle:
        recipe = json.load(handle)

    parser = argparse.ArgumentParser()
    parser.add_argument("--recipe", type=Path, default=recipe_args.recipe)
    parser.add_argument(
        "--input-dir",
        type=Path,
        default=ROOT / recipe.get("input_dir", str(DEFAULT_INPUT)),
    )
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument(
        "--target-edge-mm",
        type=float,
        default=None,
        help="override the recipe with one edge length for all selected bodies",
    )
    parser.add_argument(
        "--sample-points", type=int, default=recipe["sample_points"]
    )
    parser.add_argument(
        "--poisson-depth", type=int, default=recipe["poisson_depth"]
    )
    parser.add_argument(
        "--poisson-scale", type=float, default=recipe["poisson_scale"]
    )
    parser.add_argument("--seed", type=int, default=recipe["seed"])
    parser.add_argument("--iterations", type=int, default=recipe["iterations"])
    parser.add_argument(
        "--feature-angle-deg",
        type=float,
        default=recipe["feature_angle_deg"],
    )
    parser.add_argument(
        "--max-remesh-deviation-mm",
        type=float,
        default=recipe["max_remesh_deviation_mm"],
    )
    gates = recipe["gates"]
    parser.add_argument(
        "--gate-area-ratio", type=float, default=gates["area_p95_p05"]
    )
    parser.add_argument(
        "--gate-sliver-fraction",
        type=float,
        default=gates["sliver_fraction_lt_10deg"],
    )
    parser.add_argument(
        "--gate-centroid-ratio",
        type=float,
        default=gates["centroid_nn_p95_p05"],
    )
    parser.add_argument(
        "--gate-surface-p95-mm",
        type=float,
        default=gates["candidate_to_reference_p95_mm"],
    )
    parser.add_argument(
        "--gate-surface-max-mm",
        type=float,
        default=gates["candidate_to_reference_max_mm"],
    )
    parser.add_argument(
        "--bodies",
        nargs="+",
        choices=BODIES,
        default=BODIES,
        help="Build a subset for parameter studies; default builds all bodies.",
    )
    args = parser.parse_args()
    edge_map = recipe["target_edge_mm_by_body"]
    missing = [body for body in BODIES if body not in edge_map]
    if missing:
        raise ValueError(f"Recipe is missing target edge lengths for {missing}")
    args.target_edge_mm_by_body = {
        body: (
            args.target_edge_mm
            if args.target_edge_mm is not None
            else float(edge_map[body])
        )
        for body in BODIES
    }
    args.body_overrides = recipe.get("body_overrides", {})
    return args


def main() -> None:
    args = parse_args()
    if any(value <= 0 for value in args.target_edge_mm_by_body.values()):
        raise ValueError("Every target edge length must be positive")
    input_dir = args.input_dir.resolve()
    if not input_dir.is_dir():
        raise NotADirectoryError(input_dir)
    output_dir = args.output_dir
    if output_dir is None:
        output_dir = (
            ROOT
            / "outputs"
            / "mesh_candidates"
            / datetime.now().strftime("run_%Y-%m-%d_%H%M%S_%f")
        )
    output_dir = output_dir.resolve()
    if output_dir == ACTIVE_ASSET or ACTIVE_ASSET in output_dir.parents:
        raise ValueError("Refusing to write candidate output under active asset/")
    output_dir.mkdir(parents=True, exist_ok=False)
    poisson_dir = output_dir / "poisson_intermediate"
    point_dir = output_dir / "sampled_points"
    candidate_dir = output_dir / "candidate"
    poisson_dir.mkdir()
    point_dir.mkdir()
    candidate_dir.mkdir()

    build_config = {
        "input_dir": portable_path(input_dir),
        "recipe": portable_path(args.recipe),
        "recipe_sha256": sha256(args.recipe.resolve()),
        "target_edge_mm_by_body": args.target_edge_mm_by_body,
        "body_overrides": args.body_overrides,
        "sample_points": args.sample_points,
        "poisson_depth": args.poisson_depth,
        "poisson_scale": args.poisson_scale,
        "seed": args.seed,
        "bodies": args.bodies,
        "iterations": args.iterations,
        "feature_angle_deg": args.feature_angle_deg,
        "max_remesh_deviation_mm": args.max_remesh_deviation_mm,
        "gates": {
            "area_p95_p05": args.gate_area_ratio,
            "sliver_fraction_lt_10deg": args.gate_sliver_fraction,
            "centroid_nn_p95_p05": args.gate_centroid_ratio,
            "candidate_to_reference_p95_mm": args.gate_surface_p95_mm,
            "candidate_to_reference_max_mm": args.gate_surface_max_mm,
        },
        "pymeshlab_version": importlib.metadata.version("pymeshlab"),
        "open3d_version": o3d.__version__,
        "python_version": sys.version.split()[0],
    }

    results = []
    all_passed = True
    for index, body in enumerate(args.bodies):
        input_path = input_dir / f"{body}.STL"
        if not input_path.is_file():
            raise FileNotFoundError(input_path)
        print(f"\n[{index + 1}/{len(args.bodies)}] {body}", flush=True)
        reference = clean_and_orient(
            o3d.io.read_triangle_mesh(
                str(input_path), enable_post_processing=False
            )
        )
        poisson_path = poisson_dir / f"{body}.STL"
        reconstruct_poisson(
            reference,
            point_cloud_path=point_dir / f"{body}.ply",
            output_path=poisson_path,
            sample_points=args.sample_points,
            depth=args.poisson_depth,
            scale=args.poisson_scale,
            # Keep each body's result identical when it is built alone or as
            # part of the full set.
            seed=args.seed + BODIES.index(body),
        )

        candidate_path = candidate_dir / f"{body}.STL"
        body_options = args.body_overrides.get(body, {})
        isotropic_remesh(
            poisson_path,
            candidate_path,
            target_edge_m=args.target_edge_mm_by_body[body] / 1000.0,
            iterations=int(body_options.get("iterations", args.iterations)),
            feature_angle_deg=float(
                body_options.get(
                    "feature_angle_deg", args.feature_angle_deg
                )
            ),
            max_surface_distance_m=float(
                body_options.get(
                    "max_remesh_deviation_mm",
                    args.max_remesh_deviation_mm,
                )
            )
            / 1000.0,
        )
        candidate = clean_and_orient(
            o3d.io.read_triangle_mesh(
                str(candidate_path), enable_post_processing=False
            )
        )
        o3d.io.write_triangle_mesh(str(candidate_path), candidate)

        metrics = audit_mesh(
            candidate_path,
            mesh_set="candidate",
            body=body,
            check_self=True,
        )
        metrics["path"] = portable_path(candidate_path)
        deviation = surface_deviation(candidate, reference)
        failures = evaluate_gates(
            metrics,
            deviation,
            max_area_ratio=args.gate_area_ratio,
            max_sliver_fraction=args.gate_sliver_fraction,
            max_centroid_ratio=args.gate_centroid_ratio,
            max_surface_p95_m=args.gate_surface_p95_mm / 1000.0,
            max_surface_max_m=args.gate_surface_max_mm / 1000.0,
        )
        passed = not failures
        all_passed &= passed
        result = {
            "body": body,
            "passed": passed,
            "failures": failures,
            "input_sha256": sha256(input_path),
            "candidate_sha256": sha256(candidate_path),
            "metrics": metrics,
            "surface_deviation": deviation,
        }
        results.append(result)
        print(
            f"  triangles={metrics['triangles']} "
            f"watertight={metrics['watertight']} "
            f"intersections={metrics['self_intersecting_triangle_pairs']} "
            f"area_ratio={metrics['area_p95_p05']:.2f} "
            f"slivers={100 * metrics['sliver_fraction_lt_10deg']:.2f}% "
            f"surface_p95={1000 * deviation['candidate_to_reference_p95_m']:.3f} mm"
        )
        print("  PASS" if passed else f"  FAIL: {', '.join(failures)}")

    manifest = {
        "all_passed": all_passed,
        "config": build_config,
        "results": results,
    }
    manifest_path = output_dir / "build_manifest.json"
    with manifest_path.open("w", encoding="utf-8") as handle:
        json.dump(manifest, handle, indent=2)
    print(f"\nManifest: {manifest_path}")
    if not all_passed:
        raise SystemExit("One or more mesh candidates failed quality gates")


if __name__ == "__main__":
    main()
