"""Compare source and active-mesh centers from any working directory."""

from pathlib import Path

import open3d as o3d


ROOT = Path(__file__).resolve().parents[1]
BODIES = ("Mid", "Front", "FR", "FL", "Back", "HR", "HL", "Tail")

print(f"{'Body':<8} {'Original center':<35} {'Remeshed center':<35} {'Shift norm'}")
print("=" * 105)
for body in BODIES:
    original_path = ROOT / "models" / "meshes" / f"{body}.STL"
    remeshed_path = ROOT / "asset" / f"{body}.STL"
    original = o3d.io.read_triangle_mesh(str(original_path))
    remeshed = o3d.io.read_triangle_mesh(str(remeshed_path))
    original_center = original.get_center()
    remeshed_center = remeshed.get_center()
    shift_norm = float(
        ((remeshed_center - original_center) ** 2).sum() ** 0.5
    )
    print(
        f"{body:<8} "
        f"[{original_center[0]:.4f} {original_center[1]:.4f} "
        f"{original_center[2]:.4f}]      "
        f"[{remeshed_center[0]:.4f} {remeshed_center[1]:.4f} "
        f"{remeshed_center[2]:.4f}]      "
        f"{shift_norm:.6f} m"
    )
