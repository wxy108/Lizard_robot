"""Plot triangle centroids exactly as RFT force-point distributions."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d

from audit_mesh_quality import locate_mesh


ROOT = Path(__file__).resolve().parents[1]
PROJECTIONS = (("x", "y", 0, 1), ("x", "z", 0, 2), ("y", "z", 1, 2))


def parse_mesh_sets(values: list[str]) -> dict[str, Path]:
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


def centroid_data(path: Path) -> tuple[np.ndarray, np.ndarray]:
    mesh = o3d.io.read_triangle_mesh(str(path), enable_post_processing=False)
    vertices = np.asarray(mesh.vertices, dtype=float)
    triangles = np.asarray(mesh.triangles, dtype=np.int64)
    points = vertices[triangles]
    centroids = np.mean(points, axis=1)
    areas = 0.5 * np.linalg.norm(
        np.cross(points[:, 1] - points[:, 0], points[:, 2] - points[:, 0]),
        axis=1,
    )
    return centroids, areas


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--body", required=True)
    parser.add_argument(
        "--mesh-set",
        action="append",
        required=True,
        metavar="NAME=PATH",
    )
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    mesh_sets = parse_mesh_sets(args.mesh_set)
    data = {
        name: centroid_data(locate_mesh(path, args.body))
        for name, path in mesh_sets.items()
    }
    all_log_areas = np.concatenate(
        [np.log10(np.maximum(areas, np.finfo(float).tiny)) for _, areas in data.values()]
    )
    color_min, color_max = np.percentile(all_log_areas, (2, 98))

    figure, axes = plt.subplots(
        len(mesh_sets),
        len(PROJECTIONS),
        figsize=(13, 3.8 * len(mesh_sets)),
        constrained_layout=True,
        squeeze=False,
    )
    scatter = None
    for row, (set_name, (centroids, areas)) in enumerate(data.items()):
        log_areas = np.log10(np.maximum(areas, np.finfo(float).tiny))
        area_p05, area_p95 = np.percentile(areas, (5, 95))
        for column, (axis_a, axis_b, index_a, index_b) in enumerate(PROJECTIONS):
            axis = axes[row, column]
            scatter = axis.scatter(
                centroids[:, index_a] * 1000.0,
                centroids[:, index_b] * 1000.0,
                c=log_areas,
                cmap="turbo",
                vmin=color_min,
                vmax=color_max,
                s=5 if len(centroids) > 3000 else 9,
                alpha=0.82,
                linewidths=0,
            )
            axis.set_aspect("equal", adjustable="box")
            axis.set_xlabel(f"{axis_a} (mm)")
            axis.set_ylabel(f"{axis_b} (mm)")
            axis.grid(alpha=0.15)
            if column == 0:
                ratio = area_p95 / area_p05 if area_p05 > 0 else float("inf")
                axis.set_title(
                    f"{set_name}: {len(centroids)} RFT points, "
                    f"area P95/P05={ratio:.1f}"
                )
            else:
                axis.set_title(f"{axis_a.upper()}{axis_b.upper()} projection")

    if scatter is not None:
        figure.colorbar(
            scatter,
            ax=axes,
            label="log10 triangle area (m²)",
            shrink=0.82,
        )
    figure.suptitle(
        f"{args.body}: each dot is one triangle centroid / RFT force point",
        fontsize=15,
    )
    args.output = args.output.resolve()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=180)
    print(args.output)


if __name__ == "__main__":
    main()
