#!/usr/bin/env python3
"""Reproduce the URDF -> MJCF conversion for the Lizard V1 robot.

The checked-in models/lizard.xml was created from
V1TailAssemblyURDF/urdf/V1TailAssemblyURDF.urdf via this script, then
hand-finished (floor, actuators, sensors, cameras, keyframe, materials).
Run this only if the CAD/URDF changes; diff the dump against
models/lizard.xml and port the body-tree changes manually.

Usage:  python scripts/convert_urdf.py
Output: models/lizard_body_dump.xml  (raw converted body tree)
"""

from pathlib import Path

import mujoco

ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / "V1TailAssemblyURDF" / "urdf" / "V1TailAssemblyURDF.urdf"
OUT = ROOT / "models" / "lizard_body_dump.xml"


def main() -> None:
    text = URDF.read_text(encoding="utf-8")
    # meshes live next to models/, strip ROS package prefix
    text = text.replace("package://V1TailAssemblyURDF/meshes/", "meshes/")
    # MuJoCo URDF extensions: keep visual geoms, don't fuse static bodies
    ext = (
        '\n  <mujoco>\n'
        '    <compiler meshdir="." balanceinertia="true" '
        'discardvisual="false" fusestatic="false"/>\n'
        '  </mujoco>'
    )
    text = text.replace('name="V1TailAssemblyURDF">', 'name="V1TailAssemblyURDF">' + ext, 1)

    tmp = ROOT / "models" / "_tmp_robot.urdf"
    # write next to models/meshes so relative mesh paths resolve
    tmp.write_text(text, encoding="utf-8")
    try:
        model = mujoco.MjModel.from_xml_path(str(tmp))
        mujoco.mj_saveLastXML(str(OUT), model)
        print(f"saved {OUT}")
        print(f"nq={model.nq} nv={model.nv} nbody={model.nbody} "
              f"mass={model.body_mass.sum():.4f} kg")
    finally:
        tmp.unlink(missing_ok=True)


if __name__ == "__main__":
    main()
