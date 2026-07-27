import open3d as o3d
import os

# Folder where this script lives (models/)
HERE = os.path.dirname(os.path.abspath(__file__))

meshes = ['Mid', 'Front', 'FR', 'FL', 'Back', 'HR', 'HL', 'Tail']
mesh_dir = os.path.join(HERE, '..', 'asset')
output_file = os.path.join(HERE, 'lizard_sites.xml')

print(f"Looking for meshes in: {mesh_dir}")

with open(output_file, 'w') as f:
    for name in meshes:
        path = os.path.join(mesh_dir, f'{name}.STL')
        mesh = o3d.io.read_triangle_mesh(path)
        n = len(mesh.triangles)
        if n == 0:
            raise ValueError(f"No triangles found in {path}")
        print(f"  {name}: {n} triangles")

        f.write(f'\n<!-- ===== Sites for {name} ({n} triangles) ===== -->\n')
        for s in range(n):
            # Group 5 is hidden by default by MuJoCo. lizard_sand.py can move
            # selected sites to a visible group for debugging.
            f.write(
                f'<site name="force_{name}_site_{s}" pos="0 0 0.000001" '
                'size="0.002" type="sphere" rgba="1 1 1 1" group="5"/>\n'
            )

print(f"\nDone! Written to {output_file}")
