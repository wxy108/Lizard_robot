"""Report triangle counts for the active RFT meshes or another mesh folder."""

import argparse
from pathlib import Path

import open3d as o3d


ROOT = Path(__file__).resolve().parents[1]
BODIES = ("Mid", "Front", "FR", "FL", "Back", "HR", "HL", "Tail")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "mesh_dir",
        nargs="?",
        type=Path,
        default=ROOT / "asset",
        help="mesh directory (default: active asset folder)",
    )
    args = parser.parse_args()
    mesh_dir = args.mesh_dir.resolve()

    print(f"Triangle counts in {mesh_dir}:")
    print("=" * 60)
    missing = []
    for name in BODIES:
        path = mesh_dir / f"{name}.STL"
        if not path.is_file():
            missing.append(str(path))
            print(f"  {name:<6}: FILE NOT FOUND")
            continue
        mesh = o3d.io.read_triangle_mesh(str(path))
        print(f"  {name:<6}: {len(mesh.triangles)} triangles")
    print("=" * 60)
    if missing:
        raise FileNotFoundError("Missing meshes:\n" + "\n".join(missing))


if __name__ == "__main__":
    main()
