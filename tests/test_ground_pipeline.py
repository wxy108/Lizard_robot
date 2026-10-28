"""Known-answer geometry/cache/collision/control contracts; no external downloads."""
import json
from pathlib import Path
import tempfile
import types
import unittest

import mujoco
import numpy as np

from rft.binding import primitive_mesh
from rft.geometry import GeometryRejected, audit, make_mesh, prepare, qualify, read_mesh
from rft.ground import contact_forces, support_ids
from rft.pipeline import build_bundle, convert_support_spec, fingerprint, import_robot, load_bundle, select_meshes
from controllers.playground_onnx import PlaygroundController, joint_order

PROBE = """<mujoco><worldbody><geom name="floor" type="plane" size="1 1 .1"/>
<body name="robot" pos="0 0 .1"><freejoint/><geom name="foot" type="box" size=".02 .03 .01" mass="1"/></body>
</worldbody></mujoco>"""


class PipelineTests(unittest.TestCase):
    def setUp(self):
        self.model = mujoco.MjModel.from_xml_string(PROBE)
        self.mesh = make_mesh(*primitive_mesh(mujoco.mjtGeom.mjGEOM_BOX,[.02,.03,.01]))
        self.config = {"surfaces":[{"id":"foot","body":"robot","geom":"foot"}],
                       "remesh":{"method":"preserve"}}

    def test_closed_outward_box_known_area_volume(self):
        result = qualify(self.mesh,{})
        self.assertAlmostEqual(result["area_m2"],8*(.02*.03+.02*.01+.03*.01))
        self.assertAlmostEqual(result["component_signed_volumes_m3"][0],8*.02*.03*.01)

    def test_reject_open_and_inward_before_remeshing(self):
        vertices,faces = np.asarray(self.mesh.vertices),np.asarray(self.mesh.triangles)
        for bad in (make_mesh(vertices,faces[:-1]),make_mesh(vertices,faces[:,::-1])):
            with self.assertRaises(GeometryRejected):
                qualify(bad,{})

    def test_reject_multicomponent_assembly_no_largest_component_shortcut(self):
        vertices,faces = np.asarray(self.mesh.vertices),np.asarray(self.mesh.triangles)
        assembly = make_mesh(np.vstack([vertices,vertices+[.1,0,0]]),np.vstack([faces,faces+8]))
        with self.assertRaises(GeometryRejected) as caught:
            qualify(assembly,{})
        self.assertEqual(caught.exception.metrics["components"],2)

    def test_reject_crossing_and_duplicate_faces(self):
        vertices,faces = np.asarray(self.mesh.vertices),np.asarray(self.mesh.triangles)
        crossing = make_mesh(np.vstack([vertices,vertices+[.01,.01,0]]),np.vstack([faces,faces+8]))
        self.assertGreater(audit(crossing)["self_intersection_pairs"],0)
        with self.assertRaises(GeometryRejected):
            qualify(crossing,{})
        with self.assertRaises(GeometryRejected):
            qualify(make_mesh(vertices,np.vstack([faces,faces[:1]])),{})

    def test_array_input_checks_before_native_geometry(self):
        with self.assertRaises(ValueError):
            make_mesh([[np.nan,0,0]],[[0,0,0]])
        with self.assertRaises(ValueError):
            make_mesh([[0,0,0]],[[0,1,2]])

    def test_mesh_units_are_explicit_and_scaled(self):
        import open3d as o3d
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/"box.ply"
            o3d.io.write_triangle_mesh(str(path),self.mesh)
            with self.assertRaises(ValueError):
                read_mesh(path,None)
            result = read_mesh(path,"mm")
            np.testing.assert_allclose(np.ptp(np.asarray(result.vertices),axis=0),[4e-5,6e-5,2e-5])

    def test_verified_bundle_roundtrip_and_tamper_rejection(self):
        with tempfile.TemporaryDirectory() as tmp:
            config = Path(tmp)/"config.yaml"; config.write_text("probe",encoding="utf-8")
            directory = Path(tmp)/"bundle"
            before = fingerprint(self.model)
            build_bundle(self.model,self.config,config,directory)
            surfaces,metadata = load_bundle(self.model,directory)
            self.assertEqual(surfaces[0].body_name,"robot")
            self.assertEqual(before,fingerprint(self.model))
            with (directory/"foot/surface.npz").open("ab") as stream:
                stream.write(b"tampered")
            with self.assertRaisesRegex(ValueError,"Artifact mismatch"):
                load_bundle(self.model,directory)

    def test_wrong_model_rejects_prepared_bundle(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/"config.yaml"; path.write_text("probe")
            build_bundle(self.model,self.config,path,Path(tmp)/"bundle")
            self.model.body_mass[1] *= 2
            with self.assertRaisesRegex(ValueError,"different compiled model"):
                load_bundle(self.model,Path(tmp)/"bundle")

    def test_wrong_owner_and_duplicate_physical_geom_rejected(self):
        config = {"surfaces":[{"id":"first","body":"robot","geom":"foot"},
                              {"id":"second","body":"robot","geom":"foot"}]}
        with self.assertRaisesRegex(ValueError,"Duplicate physical geom"):
            select_meshes(self.model,config,Path("probe.yaml"))
        config["surfaces"][0]["body"] = "missing"
        with self.assertRaisesRegex(ValueError,"Invalid body owner"):
            select_meshes(self.model,config,Path("probe.yaml"))

    def test_world_floor_policy_rejects_unlisted_support(self):
        self.assertEqual(support_ids(self.model,["floor"]),{0})
        model = mujoco.MjModel.from_xml_string(PROBE.replace('<body name="robot"',
            '<geom name="hidden_floor" type="plane" pos="0 0 -.1" size="1 1 .1"/><body name="robot"'))
        with self.assertRaisesRegex(ValueError,"Undeclared rigid"):
            support_ids(model,["floor"])
        with self.assertRaisesRegex(ValueError,"named world geom"):
            support_ids(self.model,["foot"])

    def test_explicit_contact_pairs_cannot_bypass_ground_masks(self):
        model = mujoco.MjModel.from_xml_string(PROBE.replace('</mujoco>',
            '<contact><pair geom1="floor" geom2="foot"/></contact></mujoco>'))
        with self.assertRaisesRegex(ValueError,"Explicit contact pairs"):
            support_ids(model,["floor"])

    def test_native_rigid_contact_force_has_upward_sign(self):
        data = mujoco.MjData(self.model)
        data.qpos[2] = .009
        mujoco.mj_forward(self.model,data)
        for _ in range(100):
            mujoco.mj_step(self.model,data)
        active,force,pairs = contact_forces(self.model,data,{0},[{1}])
        self.assertTrue(pairs)
        self.assertEqual(active[0],1)
        self.assertGreater(force[2],0)

    def test_reviewed_pair_conversion_keeps_unrelated_pairs_and_model_identity(self):
        xml = PROBE.replace('</body>','<geom name="other" size=".01" pos=".1 0 0"/></body>').replace('</mujoco>',
            '<contact><pair name="floor_pair" geom1="floor" geom2="foot"/>'
            '<pair name="self_pair" geom1="foot" geom2="other"/></contact></mujoco>')
        spec = mujoco.MjSpec.from_string(xml)
        reference = fingerprint(spec.compile())
        with self.assertRaisesRegex(ValueError,"exact reviewed names"):
            convert_support_spec(spec,{"support_geoms":["floor"]})
        removed = convert_support_spec(spec,{"support_geoms":["floor"],"disable_support_pairs":["floor_pair"]})
        self.assertEqual(removed,["floor_pair"])
        compiled = spec.compile()
        self.assertEqual(compiled.npair,1)
        self.assertEqual(compiled.pair(0).name,"self_pair")
        self.assertEqual(reference,fingerprint(compiled))

    def test_nested_fixed_body_and_mocap_ground_are_not_hidden(self):
        for extra in ('<body name="terrain"><geom name="extra" type="plane" size="1 1 .1"/></body>',
                      '<body name="platform" mocap="true"><geom name="extra" type="box" size="1 1 .1"/></body>'):
            model = mujoco.MjModel.from_xml_string(PROBE.replace('<body name="robot"',extra+'<body name="robot"'))
            with self.assertRaisesRegex(ValueError,"Undeclared rigid"):
                support_ids(model,["floor"])

    def test_unnamed_support_pair_cannot_be_silently_removed(self):
        spec = mujoco.MjSpec.from_string(PROBE.replace('</mujoco>',
            '<contact><pair geom1="floor" geom2="foot"/></contact></mujoco>'))
        with self.assertRaisesRegex(ValueError,"unique named pairs"):
            convert_support_spec(spec,{"support_geoms":["floor"],"disable_support_pairs":[""]})

    def test_virtual_asset_aliases_are_not_duplicated(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)
            (path/"assets/nested").mkdir(parents=True)
            (path/"assets/nested/include.xml").write_text('<mujoco><worldbody><body name="b"><geom size=".1"/></body></worldbody></mujoco>')
            (path/"model.xml").write_text('<mujoco><include file="nested/include.xml"/></mujoco>')
            (path/"config.yaml").write_text('model: model.xml\nasset_dirs: [assets]\n')
            model,_,_ = import_robot(path/"config.yaml")
            self.assertEqual(model.nbody,2)

    def test_isotropic_remesh_checked_against_original_bound(self):
        with tempfile.TemporaryDirectory() as tmp:
            prepared,report = prepare(self.mesh,{"method":"isotropic","target_edge_m":.01,
                "max_deviation_m":.0005,"iterations":3},Path(tmp)/"candidate")
            self.assertGreater(len(prepared.triangles),len(self.mesh.triangles))
            self.assertLessEqual(max(v["max_sampled_m"] for v in report["sampled_deviation"].values()),.0005001)

    def test_unbounded_remesh_request_is_rejected_before_generation(self):
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaisesRegex(ValueError,"Estimated remesh size"):
                prepare(self.mesh,{"method":"isotropic","target_edge_m":1e-5,
                        "max_deviation_m":1e-6},Path(tmp)/"candidate")


class FakeSession:
    def get_inputs(self): return [types.SimpleNamespace(name="obs",shape=[1,48])]
    def get_outputs(self): return [types.SimpleNamespace(name="continuous_actions")]
    def run(self,names,values): return [np.ones((1,12),dtype=np.float32)*.2]


class ControllerContractTests(unittest.TestCase):
    def make_probe(self):
        # Independent synthetic model: named sensor order and servo mappings.
        bodies = ''.join(f'<body name="b{i}"><joint name="{name}"/><geom size=".02"/></body>'
                         for i,name in enumerate(joint_order("go1")))
        motors = ''.join(f'<position name="a{i}" joint="{name}" kp="35"/>' for i,name in enumerate(joint_order("go1")))
        return mujoco.MjModel.from_xml_string('<mujoco><worldbody><body name="root"><freejoint/>'
            '<geom size=".1"/><site name="imu"/>'+bodies+'</body></worldbody><actuator>'+motors+'</actuator>'
            '<sensor><framelinvel name="local_linvel" objtype="site" objname="imu" reftype="site" refname="imu"/>'
            '<gyro name="gyro" site="imu"/></sensor></mujoco>')

    def test_controller_obs_order_and_action_scale(self):
        model = self.make_probe(); data = mujoco.MjData(model); mujoco.mj_forward(model,data)
        controller = PlaygroundController(model,data,"go1","unused",session=FakeSession(),command=[.4,.1,.2])
        obs = controller.observation(model,data)
        np.testing.assert_allclose(obs[6:9],[0,0,-1])
        np.testing.assert_allclose(obs[-3:],[.4,.1,.2])
        np.testing.assert_allclose(controller.update(model,data),np.ones(12)*.1)
        self.assertEqual(controller.updates,1)

    def test_controller_rejects_wrong_actuator_mapping_or_frequency(self):
        model = self.make_probe(); data = mujoco.MjData(model)
        with self.assertRaisesRegex(ValueError,"50 Hz"):
            PlaygroundController(model,data,"go1","unused",control_dt=.01,session=FakeSession())
        model.actuator_trnid[0,0] = model.actuator_trnid[1,0]
        with self.assertRaisesRegex(ValueError,"order mismatch"):
            PlaygroundController(model,data,"go1","unused",session=FakeSession())


if __name__ == "__main__":
    unittest.main()
