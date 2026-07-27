"""Promote one fully gated RFT mesh build into the active model.

The candidate builder never writes ``asset/``. This separate, explicit step
checks the manifest and file hashes, replaces all eight active STL files, and
regenerates the one-force-site-per-triangle MJCF content.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import subprocess
import sys
from pathlib import Path

from audit_mesh_quality import BODIES


ROOT = Path(__file__).resolve().parents[1]
ACTIVE_ASSET = ROOT / "asset"
RECIPE = ROOT / "configs" / "rft_mesh_recipe.json"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest().upper()


def run(command: list[str]) -> None:
    print(">", " ".join(command), flush=True)
    subprocess.run(command, cwd=ROOT, check=True)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "run_dir",
        type=Path,
        help="candidate run directory containing build_manifest.json",
    )
    args = parser.parse_args()

    run_dir = args.run_dir.resolve()
    candidate_dir = run_dir / "candidate"
    manifest_path = run_dir / "build_manifest.json"
    with manifest_path.open(encoding="utf-8") as handle:
        manifest = json.load(handle)

    if not manifest.get("all_passed"):
        raise ValueError("Refusing to promote: manifest all_passed is false")
    with RECIPE.open(encoding="utf-8") as handle:
        recipe = json.load(handle)
    config = manifest["config"]
    if config["recipe_sha256"] != sha256(RECIPE):
        raise ValueError(
            "Refusing to promote: current recipe hash differs from manifest"
        )
    expected_config = {
        "input_dir": Path(recipe["input_dir"]).as_posix(),
        "recipe": RECIPE.relative_to(ROOT).as_posix(),
        "target_edge_mm_by_body": recipe["target_edge_mm_by_body"],
        "body_overrides": recipe.get("body_overrides", {}),
        "sample_points": recipe["sample_points"],
        "poisson_depth": recipe["poisson_depth"],
        "poisson_scale": recipe["poisson_scale"],
        "seed": recipe["seed"],
        "bodies": list(BODIES),
        "iterations": recipe["iterations"],
        "feature_angle_deg": recipe["feature_angle_deg"],
        "max_remesh_deviation_mm": recipe["max_remesh_deviation_mm"],
        "gates": recipe["gates"],
    }
    for field, expected in expected_config.items():
        if config.get(field) != expected:
            raise ValueError(
                f"Refusing to promote: manifest {field!r} does not match "
                "the tracked recipe"
            )

    results = {result["body"]: result for result in manifest["results"]}
    if set(results) != set(BODIES):
        raise ValueError("Refusing to promote: manifest must contain all bodies")
    for body in BODIES:
        result = results[body]
        path = candidate_dir / f"{body}.STL"
        if not result["passed"]:
            raise ValueError(f"Refusing to promote: {body} did not pass")
        if sha256(path) != result["candidate_sha256"]:
            raise ValueError(f"Refusing to promote: {body} hash mismatch")

    for body in BODIES:
        source = candidate_dir / f"{body}.STL"
        destination = ACTIVE_ASSET / source.name
        shutil.copy2(source, destination)
        print(f"Promoted {body}: {sha256(destination)}")

    run([sys.executable, "models/generate_sites.py"])
    run([sys.executable, "scripts/rebuild_sites.py"])
    print(
        "Promotion complete. Run scripts/validate_project.py --full "
        "before committing."
    )


if __name__ == "__main__":
    main()
