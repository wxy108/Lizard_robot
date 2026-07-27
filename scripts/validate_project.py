"""Fast reproducibility and model-integrity checks for the active project."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

import mujoco
import numpy as np
import open3d

from audit_mesh_quality import BODIES, audit_mesh


ROOT = Path(__file__).resolve().parents[1]
FORCE_SITE_GROUP = 5
MESH_RECIPE = ROOT / "configs" / "rft_mesh_recipe.json"
with MESH_RECIPE.open(encoding="utf-8") as handle:
    MESH_GATES = json.load(handle)["gates"]
MAX_AREA_P95_P05 = MESH_GATES["area_p95_p05"]
MAX_SLIVER_FRACTION = MESH_GATES["sliver_fraction_lt_10deg"]
MAX_CENTROID_P95_P05 = MESH_GATES["centroid_nn_p95_p05"]


def run(command: list[str]) -> None:
    print("\n>", " ".join(command))
    subprocess.run(command, cwd=ROOT, check=True)


def check_active_meshes() -> dict[str, int]:
    triangle_counts = {}
    for body in BODIES:
        path = ROOT / "asset" / f"{body}.STL"
        metrics = audit_mesh(
            path, mesh_set="active_rft", body=body, check_self=True
        )
        failures = []
        for field in (
            "watertight",
            "edge_manifold_with_boundary",
            "vertex_manifold",
            "orientable",
        ):
            if not metrics[field]:
                failures.append(f"{field}=false")
        for field in (
            "boundary_edges",
            "nonmanifold_edges",
            "inconsistent_winding_edges",
            "degenerate_triangles",
            "duplicate_welded_triangles",
            "self_intersecting_triangle_pairs",
        ):
            if metrics[field] != 0:
                failures.append(f"{field}={metrics[field]}")
        if metrics["connected_components"] != 1:
            failures.append(
                f"connected_components={metrics['connected_components']}"
            )
        if metrics["signed_volume_m3"] <= 0:
            failures.append(
                f"signed_volume_m3={metrics['signed_volume_m3']}"
            )
        if metrics["area_p95_p05"] > MAX_AREA_P95_P05:
            failures.append(
                f"area_p95_p05={metrics['area_p95_p05']:.3f}"
            )
        if metrics["sliver_fraction_lt_10deg"] > MAX_SLIVER_FRACTION:
            failures.append(
                "sliver_fraction_lt_10deg="
                f"{metrics['sliver_fraction_lt_10deg']:.4f}"
            )
        if metrics["centroid_nn_p95_p05"] > MAX_CENTROID_P95_P05:
            failures.append(
                "centroid_nn_p95_p05="
                f"{metrics['centroid_nn_p95_p05']:.3f}"
            )
        if failures:
            raise AssertionError(
                f"Active RFT mesh {body} failed: {', '.join(failures)}"
            )
        triangle_counts[body] = metrics["triangles"]
        print(
            f"Mesh {body}: {metrics['triangles']} triangles, "
            f"area ratio={metrics['area_p95_p05']:.2f}, "
            f"centroid ratio={metrics['centroid_nn_p95_p05']:.2f}"
        )
    return triangle_counts


def check_model(triangle_counts: dict[str, int]) -> None:
    model = mujoco.MjModel.from_xml_path(str(ROOT / "Lizard_Sand.xml"))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    sand_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "sand_height")
    floor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    if sand_id < 0 or floor_id < 0:
        raise AssertionError("Missing sand_height site or emergency floor geom")

    sand_z = float(data.site_xpos[sand_id, 2])
    floor_z = float(model.geom_pos[floor_id, 2])
    if not np.isclose(sand_z, 0.0, atol=1e-12):
        raise AssertionError(f"sand_height must be z=0, got {sand_z}")
    if not np.isclose(floor_z, -0.25, atol=1e-12):
        raise AssertionError(f"emergency floor must be z=-0.25, got {floor_z}")

    force_ids = []
    for site_id in range(model.nsite):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, site_id)
        if name and name.startswith("force_"):
            force_ids.append(site_id)
    expected_force_sites = sum(triangle_counts.values())
    if len(force_ids) != expected_force_sites:
        raise AssertionError(
            f"Expected {expected_force_sites} force sites from active meshes, "
            f"got {len(force_ids)}"
        )
    for body, count in triangle_counts.items():
        for index in range(count):
            name = f"force_{body}_site_{index}"
            if (
                mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name)
                < 0
            ):
                raise AssertionError(f"Missing sequential force site {name}")
    groups = model.site_group[np.asarray(force_ids, dtype=int)]
    if not np.all(groups == FORCE_SITE_GROUP):
        raise AssertionError("Every force site must be in hidden visual group 5")

    print(
        f"Model integrity: sand z={sand_z:g}, floor z={floor_z:g}, "
        f"force sites={len(force_ids)} (matches active triangles; "
        f"group {FORCE_SITE_GROUP})"
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--full",
        action="store_true",
        help="run a 6 s RFT regression smoke test instead of 0.2 s",
    )
    args = parser.parse_args()

    print(f"Python {sys.version.split()[0]}")
    print(f"MuJoCo {mujoco.__version__}")
    print(f"NumPy {np.__version__}")
    print(f"Open3D {open3d.__version__}")

    triangle_counts = check_active_meshes()
    check_model(triangle_counts)
    run([sys.executable, "-m", "unittest", "discover", "-s", "tests", "-v"])
    run(
        [
            sys.executable,
            "run.py",
            "--headless",
            "--duration",
            "0.2",
            "--no-save",
        ]
    )
    run(
        [
            sys.executable,
            "lizard_sand.py",
            "--duration",
            "6.0" if args.full else "0.2",
            "--no-save",
        ]
    )
    print("\nAll project validation checks passed.")


if __name__ == "__main__":
    main()
