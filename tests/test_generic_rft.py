"""Known-answer binding/provider checks independent of lizard anatomy."""

import tempfile
from pathlib import Path
import unittest

import mujoco
import numpy as np

from rft import RFTForceProvider, Surface, bind_surfaces, load_model, load_surfaces
from rft.binding import primitive_mesh, surface_from_geom


PROBE = """<mujoco><option gravity="0 0 0"/>
<worldbody><body name="probe" pos="0 0 -0.02"><freejoint/>
<inertial pos="0.023 0.01 0" mass="0.1" diaginertia="0.001 0.002 0.003"/>
<geom name="offset" type="box" pos="0.04 0.01 0" euler="0 0 35" size="0.02 0.01 0.008"/>
<body name="finger" pos="0.06 0 0"><joint name="hinge" axis="0 1 0"/>
<geom name="tip" type="sphere" size="0.01" pos="0.03 0 0"/></body>
</body></worldbody><actuator><motor joint="hinge"/></actuator></mujoco>"""


def plate(owner="probe", ident="patch", shift=0):
    return Surface(ident, owner, np.array([[shift, 0, 0.0]]),
                   np.array([[0.0, 0.0, -1.0]]), np.array([1e-4]))


class GenericRftTests(unittest.TestCase):
    def setUp(self):
        self.model = mujoco.MjModel.from_xml_string(PROBE)
        self.data = mujoco.MjData(self.model)
        self.data.qvel[2] = -0.1
        mujoco.mj_forward(self.model, self.data)

    def test_named_owner_and_multiple_surfaces_do_not_need_sites(self):
        self.assertEqual(self.model.nsite, 0)
        surfaces = [plate(), plate(ident="second", shift=0.01), plate("finger", "third")]
        provider = RFTForceProvider(self.model, surfaces, alpha=1)
        result = provider.compute(self.data)
        self.assertEqual(len(result.surfaces), 3)
        self.assertGreater(np.linalg.norm(result.qfrc), 0)
        with self.assertRaises(ValueError):
            bind_surfaces(self.model, [plate("missing")])
        with self.assertRaises(ValueError):
            bind_surfaces(self.model, [plate(), plate()])

    def test_surface_validation_rejects_invalid_geometry(self):
        for areas, normals in [(np.array([-1.0]), np.array([[0,0,-1.0]])),
                               (np.array([1e-4]), np.array([[0,0,-2.0]]))]:
            with self.assertRaises(ValueError):
                Surface("bad", "probe", np.zeros((1,3)), normals, areas)
        with self.assertRaises(ValueError):
            Surface.from_mesh("bad", "probe", np.zeros((3,3)), [[0,1,2]])
        surface = plate()
        with self.assertRaises(ValueError):
            surface.centers[0,0] = 1

    def test_point_velocity_matches_jacobian_with_offset_inertia(self):
        self.data.qvel[:6] = [0.1, 0.2, -0.1, 1.2, -0.6, 0.8]
        self.data.qpos[3:7] = [np.cos(0.2), 0, np.sin(0.2), 0]
        mujoco.mj_forward(self.model, self.data)
        surface = plate(shift=0.03)
        result = RFTForceProvider(self.model, [surface]).compute(self.data).surfaces[0]
        jacobian = np.zeros((3, self.model.nv))
        angular = np.zeros_like(jacobian)
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "probe")
        mujoco.mj_jac(self.model, self.data, jacobian, angular, result.positions_world[0], bid)
        np.testing.assert_allclose(result.point_velocities_world[0], jacobian @ self.data.qvel,
                                   rtol=1e-12, atol=1e-12)
        legacy = RFTForceProvider(self.model, [surface], convention="legacy_lizard_v1").compute(self.data)
        self.assertGreater(np.linalg.norm(legacy.surfaces[0].point_velocities_world
                                         - result.point_velocities_world), 1e-5)

    def test_plane_translation_preserves_force_and_generalized_force(self):
        first = RFTForceProvider(self.model, [plate()], sand_height=0, alpha=1).compute(self.data)
        self.data.qpos[2] += 0.37
        mujoco.mj_forward(self.model, self.data)
        second = RFTForceProvider(self.model, [plate()], sand_height=0.37, alpha=1).compute(self.data)
        np.testing.assert_allclose(first.surfaces[0].forces, second.surfaces[0].forces, atol=1e-12)
        np.testing.assert_allclose(first.qfrc, second.qfrc, atol=1e-12)

    def test_intrusion_reaction_is_upward_and_dissipative(self):
        result = RFTForceProvider(self.model, [plate()], alpha=1).compute(self.data)
        force = result.surfaces[0].forces[0]
        self.assertGreater(force[2], 0)
        self.assertLess(np.dot(force, result.surfaces[0].point_velocities_world[0]), 0)

    def test_existing_force_control_and_state_are_untouched(self):
        self.data.qfrc_applied[:] = np.arange(self.model.nv) + 2
        self.data.ctrl[:] = 0.33
        old = {key: np.array(getattr(self.data, key)).copy()
               for key in ("qpos", "qvel", "ctrl", "qfrc_applied", "xfrc_applied")}
        result = RFTForceProvider(self.model, [plate()]).compute(self.data)
        self.assertGreater(np.linalg.norm(result.qfrc), 0)
        for key, value in old.items():
            np.testing.assert_array_equal(value, getattr(self.data, key))

    def test_smoothing_release_and_reset_do_not_retain_force(self):
        provider = RFTForceProvider(self.model, [plate()])
        self.assertGreater(np.linalg.norm(provider.compute(self.data).qfrc), 0)
        self.data.qpos[2] = 0.2
        mujoco.mj_forward(self.model, self.data)
        np.testing.assert_array_equal(provider.compute(self.data).qfrc, 0)
        np.testing.assert_array_equal(provider.history["patch"], 0)
        provider.reset()
        with self.assertRaises(ValueError):
            provider.reset({"wrong": np.zeros((1,3))})

    def test_body_wrench_and_triangle_application_agree(self):
        surfaces = [plate(shift=0.02), plate("finger", "finger_patch", shift=0.03)]
        first = RFTForceProvider(self.model, surfaces, alpha=1, application="triangles").compute(self.data)
        second = RFTForceProvider(self.model, surfaces, alpha=1, application="body_wrench").compute(self.data)
        np.testing.assert_allclose(first.qfrc, second.qfrc, rtol=1e-12, atol=1e-12)

    def test_compiled_geom_pose_and_closed_primitive_winding(self):
        gid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "offset")
        surface = surface_from_geom(self.model, gid)
        np.testing.assert_allclose(surface.centers.mean(axis=0), [0.04,0.01,0], atol=1e-14)
        for kind, size in [(mujoco.mjtGeom.mjGEOM_BOX, [0.02,0.01,0.008]),
                           (mujoco.mjtGeom.mjGEOM_SPHERE, [0.02,0,0]),
                           (mujoco.mjtGeom.mjGEOM_CAPSULE, [0.02,0.04,0]),
                           (mujoco.mjtGeom.mjGEOM_CYLINDER, [0.02,0.04,0]),
                           (mujoco.mjtGeom.mjGEOM_ELLIPSOID, [0.02,0.03,0.04])]:
            vertices, faces = primitive_mesh(kind, size)
            triangles = vertices[faces]
            volume = np.einsum('ij,ij->i', triangles[:,0], np.cross(triangles[:,1],triangles[:,2])).sum()/6
            self.assertGreater(volume, 0)
            edges = np.sort(np.concatenate([faces[:,[0,1]],faces[:,[1,2]],faces[:,[2,0]]]),axis=1)
            _, counts = np.unique(edges,axis=0,return_counts=True)
            np.testing.assert_array_equal(counts, 2)

    def test_urdf_fixed_link_mapping_and_missing_package(self):
        xml = """<robot name="tiny"><link name="base"/>
<link name="tool"><inertial><mass value="0.1"/><inertia ixx="0.001" iyy="0.001" izz="0.001" ixy="0" ixz="0" iyz="0"/></inertial>
<collision><geometry><box size="0.02 0.03 0.04"/></geometry></collision></link>
<joint name="fixed" type="fixed"><parent link="base"/><child link="tool"/><origin xyz="0 0 0.1"/></joint></robot>"""
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/"tiny.urdf"
            path.write_text(xml,encoding="utf-8")
            model = load_model(path)
            self.assertGreater(mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,"tool"), 0)
            path.write_text(xml.replace('<box size="0.02 0.03 0.04"/>',
                            '<mesh filename="package://missing/tool.stl"/>'),encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "package root"):
                load_model(path)

    def test_compiled_mesh_matches_source_transform_without_double_scale(self):
        vertices=np.array([[.1,.2,.3],[.16,.2,.3],[.1,.27,.3],[.1,.2,.38]])
        faces=np.array([[0,2,1],[0,1,3],[0,3,2],[1,2,3]])
        scale=np.array([.4,.7,1.1])
        angle=.25
        quat=[np.cos(angle/2),0,0,np.sin(angle/2)]
        vertex_text=' '.join(map(str,vertices.ravel()))
        face_text=' '.join(map(str,faces.ravel()))
        xml=f'''<mujoco><asset><mesh name="test" vertex="{vertex_text}"
            face="{face_text}" scale=".4 .7 1.1"/></asset><worldbody>
            <body name="owner" pos=".3 -.1 .2"><freejoint/><geom name="mesh" type="mesh"
            mesh="test" pos=".05 .02 -.01" quat="{' '.join(map(str,quat))}"/>
            </body></worldbody></mujoco>'''
        model=mujoco.MjModel.from_xml_string(xml)
        data=mujoco.MjData(model)
        mujoco.mj_forward(model,data)
        gid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,"mesh")
        surface=surface_from_geom(model,gid)
        rotation=np.array([[np.cos(angle),-np.sin(angle),0],
                           [np.sin(angle),np.cos(angle),0],[0,0,1]])
        source_body_vertices=(vertices*scale)@rotation.T+np.array([.05,.02,-.01])
        expected=source_body_vertices[faces].mean(axis=1)
        order=lambda a:a[np.lexsort((a[:,2],a[:,1],a[:,0]))]
        np.testing.assert_allclose(order(surface.centers),order(expected),atol=1e-8,rtol=1e-7)

    def test_proxy_units_and_transform_are_explicit(self):
        import open3d as o3d

        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            mesh = o3d.geometry.TriangleMesh.create_box(20, 30, 40)
            o3d.io.write_triangle_mesh(str(folder/"box.ply"),mesh)
            config = folder/"robot.yaml"
            config.write_text('surfaces:\n  - id: test\n    body: probe\n    mesh: box.ply\n    units: mm\n'
                              '    transform_to_body: {pos_m: [0.1, 0.2, 0.3]}\n',encoding="utf-8")
            surfaces, _ = load_surfaces(self.model, config)
            np.testing.assert_allclose(surfaces[0].centers.mean(axis=0), [0.11,0.215,0.32],atol=1e-12)
            self.assertAlmostEqual(surfaces[0].areas.sum(), 2*(.02*.03+.03*.04+.04*.02))

    def test_imported_urdf_can_receive_rft_without_actuators_or_sites(self):
        xml='''<robot name="passive_arm"><link name="base"/>
        <link name="tool"><inertial><mass value="0.1"/>
        <inertia ixx="0.001" iyy="0.001" izz="0.001" ixy="0" ixz="0" iyz="0"/></inertial>
        <collision><origin xyz="0.05 0 0"/><geometry><box size="0.02 0.03 0.01"/></geometry></collision></link>
        <joint name="arm" type="revolute"><parent link="base"/><child link="tool"/>
        <origin xyz="0 0 0.1"/><axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="1" velocity="2"/>
        </joint></robot>'''
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/"arm.urdf"
            path.write_text(xml,encoding="utf-8")
            model=load_model(path)
            data=mujoco.MjData(model)
            data.qvel[0]=0.1
            mujoco.mj_forward(model,data)
            surface=surface_from_geom(model,0)
            result=RFTForceProvider(model,[surface],sand_height=0.12,alpha=1).compute(data)
            self.assertEqual(model.nu,0)
            self.assertEqual(model.nsite,0)
            self.assertGreater(np.linalg.norm(result.qfrc),0)
            self.assertLess(float(np.dot(result.qfrc,data.qvel)),0)


if __name__ == "__main__":
    unittest.main()
