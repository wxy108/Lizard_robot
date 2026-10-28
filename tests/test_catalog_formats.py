"""No-download known-answer tests for format/package/deployment contracts."""
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
import zipfile

import mujoco
import numpy as np

from controllers.catalog_controllers import DifferentialDriveController
from rft.binding import load_model,primitive_mesh
from rft.formats import check_archive,urdf_spec
from rft.geometry import make_mesh,read_mesh,qualify
from rft.pipeline import import_robot,fingerprint
from rft.records import write_json,artifact_manifest
from rft.robot_package import import_package

URDF = '''<robot name="probe"><link name="base"><inertial><mass value="1"/>
<inertia ixx=".01" iyy=".01" izz=".01" ixy="0" ixz="0" iyz="0"/></inertial>
<collision><geometry><box size=".1 .1 .1"/></geometry></collision></link>
<link name="fixed"><collision><geometry><sphere radius=".02"/></geometry></collision></link>
<joint name="weld" type="fixed"><parent link="base"/><child link="fixed"/><origin xyz=".1 0 0"/></joint></robot>'''
MODEL = '''<mujoco><worldbody><geom name="floor" type="plane" size="2 2 .01"/>
<body name="base" pos="0 0 .1"><freejoint/><geom name="shell" size=".1" mass="1"/>
<body name="left"><joint name="l" axis="0 1 0"/><geom size=".02"/></body>
<body name="right"><joint name="r" axis="0 1 0"/><geom size=".02"/></body>
</body></worldbody><actuator><velocity name="l_motor" joint="l" kv=".2"/>
<velocity name="r_motor" joint="r" kv=".2"/></actuator><keyframe><key name="home"/></keyframe></mujoco>'''


class CatalogFormatTests(unittest.TestCase):
    def test_urdf_detected_by_root_even_when_named_xml(self):
        with tempfile.TemporaryDirectory() as tmp:
            file = Path(tmp)/"robot.xml"; file.write_text(URDF)
            model = load_model(file)
            self.assertGreaterEqual(model.nbody,3)
            self.assertGreater(model.body("fixed").id,0)

    @unittest.skipUnless(importlib.util.find_spec("xacro"),"Optional xacro not installed")
    def test_xacro_expansion_and_native_fixed_link_ownership(self):
        with tempfile.TemporaryDirectory() as tmp:
            file = Path(tmp)/"robot.xacro"
            file.write_text(URDF.replace('<robot name="probe">','<robot name="probe" xmlns:xacro="http://ros.org/wiki/xacro"><xacro:property name="mass" value="2"/>').replace('mass value="1"','mass value="${mass}"'))
            spec,report,_ = urdf_spec(file,{}, {})
            model = spec.compile()
            self.assertAlmostEqual(model.body_mass[model.body("base").id],2)
            self.assertEqual(report["source_format"],"xacro")
            self.assertGreater(model.body("fixed").id,0)

    def test_explicit_urdf_adapter_is_opt_in_and_source_bytes_unchanged(self):
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp); file = base/"robot.urdf"; file.write_text(URDF)
            source = file.read_bytes()
            self.assertEqual(load_model(file).nv,0)
            write_json(base/"config.json",{"model":"robot.urdf","model_adapter":{"floating_body":"base",
                       "add_floor":True,"add_keyframe":"home"}})
            model,config,files = import_robot(base/"config.json")
            self.assertEqual(model.nv,6); self.assertEqual(model.nkey,1)
            self.assertEqual(source,file.read_bytes()); self.assertIn("effective_model_adapter",config)

    def test_binary_roundtrip_preserves_compiled_physical_identity(self):
        model = mujoco.MjModel.from_xml_string(MODEL)
        with tempfile.TemporaryDirectory() as tmp:
            file = Path(tmp)/"model.mjb"; mujoco.mj_saveModel(model,str(file))
            self.assertEqual(fingerprint(model),fingerprint(load_model(file)))
            write_json(Path(tmp)/"config.json",{"model":"model.mjb","compiled_mujoco_version":"wrong"})
            with self.assertRaisesRegex(ValueError,"matching compiled"):
                import_robot(Path(tmp)/"config.json")

    def test_native_mjz_roundtrip_includes_own_assets(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/"model.mjz"
            spec = mujoco.MjSpec.from_string(MODEL); expected = spec.compile()
            with path.open("wb") as stream: spec.to_zip(stream)
            self.assertEqual(fingerprint(expected),fingerprint(load_model(path)))

    def test_archive_traversal_drive_alias_and_symlink_are_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            for i,name in enumerate(("../escape","C:/escape","a/./b","file. ","/absolute")):
                path = Path(tmp)/f"bad{i}.zip"
                with zipfile.ZipFile(path,"w") as z: z.writestr(name,b"x")
                with self.assertRaises(ValueError): check_archive(path)
            path = Path(tmp)/"link.zip"
            info = zipfile.ZipInfo("link"); info.external_attr = (0o120777 << 16)
            with zipfile.ZipFile(path,"w") as z: z.writestr(info,b"target")
            with self.assertRaisesRegex(ValueError,"symlink"): check_archive(path)

    def test_archive_budgets_and_case_aliases(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/"large.zip"
            with zipfile.ZipFile(path,"w") as z: z.writestr("blob",b"x"*100)
            with self.assertRaisesRegex(ValueError,"budget"): check_archive(path,max_bytes=10)
            path = Path(tmp)/"duplicate.zip"
            with zipfile.ZipFile(path,"w") as z:
                z.writestr("Model.xml",b"x"); z.writestr("model.xml",b"y")
            with self.assertRaisesRegex(ValueError,"ambiguous"): check_archive(path)

    def package(self,directory,mutate=None):
        model = mujoco.MjModel.from_xml_string(MODEL)
        mujoco.mj_saveModel(model,str(directory/"model.mjb"))
        write_json(directory/"recipe.json",{"model":"model.mjb","compiled_mujoco_version":mujoco.__version__})
        metadata = {"schema_version":1,"mujoco_version":mujoco.__version__,"recipe":"recipe.json",
                    "physical_fingerprint":fingerprint(model),"artifacts":artifact_manifest(directory)}
        if mutate: mutate(metadata)
        write_json(directory/"robot_package.json",metadata)
        file = directory/"robot.zip"
        with zipfile.ZipFile(file,"w") as z:
            for name in ("model.mjb","recipe.json","robot_package.json"): z.write(directory/name,name)
        return file

    def test_self_contained_package_and_cache_reverification(self):
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp); file = self.package(base)
            recipe = import_package(file,base/"extract")
            self.assertTrue(recipe.exists())
            self.assertEqual(recipe,import_package(file,base/"extract"))
            (base/"extract/model.mjb").write_bytes(b"tampered")
            with self.assertRaisesRegex(ValueError,"stale/tampered"):
                import_package(file,base/"extract")

    def test_package_wrong_version_and_bad_hash_rejected(self):
        for mutation,expected in ((lambda m:m.update(mujoco_version="wrong"),"version"),
                                 (lambda m:m["artifacts"]["model.mjb"].update(sha256="0"*64),"artifact mismatch")):
            with tempfile.TemporaryDirectory() as tmp:
                file = self.package(Path(tmp),mutation)
                with self.assertRaisesRegex(ValueError,expected): import_package(file,Path(tmp)/"extract")

    def test_stl_obj_ply_are_equivalent_closed_geometry(self):
        import open3d as o3d
        mesh = make_mesh(*primitive_mesh(mujoco.mjtGeom.mjGEOM_BOX,[.02,.03,.01]))
        with tempfile.TemporaryDirectory() as tmp:
            for ext in ("stl","obj","ply"):
                path = Path(tmp)/("mesh."+ext)
                mesh.compute_triangle_normals()
                self.assertTrue(o3d.io.write_triangle_mesh(str(path),mesh))
                report = qualify(read_mesh(path,"m"),{})
                self.assertAlmostEqual(report["area_m2"],.0088,places=8)

    def test_model_clone_does_not_alias_source(self):
        model = mujoco.MjModel.from_xml_string(MODEL); other = copy.copy(model)
        other.opt.timestep = .0005; other.geom_contype[:] = 0
        self.assertNotEqual(model.opt.timestep,other.opt.timestep)
        self.assertTrue(np.any(model.geom_contype != other.geom_contype))

    def test_differential_drive_forward_and_yaw_equations(self):
        model = mujoco.MjModel.from_xml_string(MODEL)
        spec = {"wheel_radius_m":.1,"wheel_separation_m":.4,"command":[.2,0,1],"joints":["l","r"]}
        controller = DifferentialDriveController(model,spec)
        np.testing.assert_allclose(controller.update(model,mujoco.MjData(model)),[0,4])
        spec["command"] = [.2,.1,0]
        with self.assertRaisesRegex(ValueError,"lateral"): DifferentialDriveController(model,spec)

    def test_json_bytes_are_lf_without_windows_filter_dependency(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/"metadata.json"; write_json(path,{"value":1})
            self.assertNotIn(b"\r",path.read_bytes())

    def profile_probe(self,kind,command):
        from controllers.catalog_controllers import ProfilePolicyController,PROFILES
        joints = json.loads(PROFILES.read_text())[kind]["joints"]
        bodies = ''.join(f'<body name="b{i}"><joint name="{j}"/><geom size=".01"/></body>' for i,j in enumerate(joints))
        actuators = ''.join(f'<position joint="{j}" kp="35"/>' for j in joints)
        xml = '<mujoco><worldbody><body name="root"><freejoint/><geom size=".1"/><site name="imu"/>'+bodies+\
              '</body></worldbody><actuator>'+actuators+'</actuator><sensor><gyro name="gyro" site="imu"/>'+\
              '<framelinvel name="local_linvel" objtype="site" objname="imu"/></sensor><keyframe><key name="home"/></keyframe></mujoco>'
        model = mujoco.MjModel.from_xml_string(xml); data = mujoco.MjData(model)
        count = len(joints)
        class Session:
            def get_inputs(self):
                return [__import__('types').SimpleNamespace(name='obs',shape=[1,16+3*count])]
            def get_outputs(self): return [__import__('types').SimpleNamespace(name='continuous_actions')]
            def run(self,*a): return [np.zeros((1,count),dtype=np.float32)]
        ctrl = ProfilePolicyController(model,data,{"profile":kind,"command":command,"control_dt":.02,"default_keyframe":"home"},"unused",Session())
        return model,data,ctrl

    def test_t1_head_mask_does_not_zero_physical_velocity(self):
        model,data,controller = self.profile_probe("t1",[.2,0,0])
        data.qvel[6:8] = [.2,.3]; mujoco.mj_forward(model,data)
        before = data.qvel.copy(); obs = controller.observation(model,data)
        np.testing.assert_array_equal(data.qvel,before)
        np.testing.assert_array_equal(obs[12:14],[0,0])

    def test_apollo_near_zero_command_phase_matches_original_branch(self):
        model,data,controller = self.profile_probe("apollo",[.005,0,0])
        mujoco.mj_forward(model,data); controller.update(model,data)
        expected = np.fmod(np.array([0.,np.pi])+2*np.pi*.02*1.25+np.pi,2*np.pi)-np.pi
        np.testing.assert_array_equal(controller.phase,expected)

    def test_undeclared_dynamic_actor_rejected_before_control(self):
        from rft.ground import run
        model = mujoco.MjModel.from_xml_string(MODEL.replace('</worldbody>',
            '<body name="extra" pos="0 0 2"><freejoint/><geom size=".1"/></body></worldbody>'))
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaisesRegex(ValueError,"dynamic actor"):
                run(model,{"controller":{"control_dt":.02},"root_body":"base"},Path(tmp)/"recipe",[],Path(tmp)/"out",mode="sand",duration=.1,dt=.001)


if __name__ == "__main__":
    unittest.main()
