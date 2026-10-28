"""Known-answer tests for D-022 geometry and motion presentation, no downloads."""
import ast
from pathlib import Path
import unittest

import numpy as np

from scripts.generate_robot_comparisons import (camera_basis,compose_motion_frame,
    contact_bins,frame_sample_indices,geometry_frame,geometry_sheet,project,
    proxy_metrics,quadrature,row_source_index,spacing_stats)

VERTICES = np.array([[0.,0,0],[1,0,0],[1,1,0],[0,1,0]])
FACES = np.array([[0,1,2],[0,2,3]],dtype=np.int32)


def surface():
    centers,normals,areas = quadrature(VERTICES,FACES)
    colors,_ = spacing_stats(centers)
    mesh = {"vertices":VERTICES,"faces":FACES,"centers":centers,"normals":normals,
            "areas":areas,"colors":colors}
    item = {"id":"left_sole","body":"foot","before_mesh":mesh,"after_mesh":mesh,
            "family":"same","frame":geometry_frame(VERTICES,VERTICES),
            "before":{"triangles":2},"after":{"triangles":2}}
    item["metrics"] = proxy_metrics(item)
    return item


def motion(mode):
    trace = np.zeros((10,16)); trace[:,0] = np.arange(10)*.1; trace[:,1] = trace[:,0]*.2
    trace[:,4] = .5; trace[:,5] = 1
    summary = {"mode":mode,"status":"passed_screen","physics_timestep_s":.1,
        "recorded_duration_s":1.,"requested_duration_s":1.,"effective_controller":{"command":[.2,0,0]}}
    return {"summary":summary,"trace":trace,"indices":frame_sample_indices(10,3),
            "contact":np.ones((10,1),dtype=bool),"groups":np.array(["foot"])}


class RobotComparisonTests(unittest.TestCase):
    def test_quadrature_known_square(self):
        centers,normals,areas = quadrature(VERTICES,FACES)
        np.testing.assert_allclose(centers,[[2/3,1/3,0],[1/3,2/3,0]])
        np.testing.assert_allclose(normals,[[0,0,1],[0,0,1]])
        np.testing.assert_allclose(areas,[.5,.5])

    def test_invalid_triangles_stop_instead_of_color_sanitization(self):
        with self.assertRaises(ValueError): quadrature(VERTICES,np.array([[0,0,1],[0,2,3]]))
        with self.assertRaises(ValueError): spacing_stats(np.zeros((2,3)))

    def test_spacing_metrics_are_3d_and_in_mm(self):
        _,stats = spacing_stats([[0,0,0],[0,0,1],[0,0,3]])
        self.assertEqual(stats["median_spacing_mm"],1000)
        self.assertAlmostEqual(stats["spacing_p95_p05"],1.9)

    def test_relative_color_rule_preserves_absolute_scale_as_separate_metric(self):
        a = np.array([[0,0,0],[1,0,0],[3,0,0]],dtype=float)
        c1,m1 = spacing_stats(a); c2,m2 = spacing_stats(a*2)
        np.testing.assert_array_equal(c1,c2)
        self.assertEqual(m2["median_spacing_mm"],2*m1["median_spacing_mm"])

    def test_camera_basis_is_orthonormal(self):
        matrix = camera_basis(37,-75)
        np.testing.assert_allclose(matrix@matrix.T,np.eye(3),atol=1e-15)

    def test_shared_frame_and_projection_do_not_independently_autoscale(self):
        a,b = VERTICES,VERTICES+.05
        center,radius = geometry_frame(a,b)
        self.assertTrue(np.all(np.linalg.norm(np.vstack([a,b])-center,axis=1)<=radius+1e-15))
        p1,_ = project(a,center,radius,(0,0,300,240),camera_basis(35,25))
        p2,_ = project(a,center,radius,(0,0,300,240),camera_basis(35,25))
        np.testing.assert_array_equal(p1,p2)
        self.assertTrue(np.all((p1>=0)&(p1<300)))

    def test_saved_frame_mapping_includes_both_endpoints(self):
        np.testing.assert_array_equal(frame_sample_indices(10,3),[0,4,9])

    def test_stopped_row_never_reuses_last_frame(self):
        self.assertEqual(row_source_index(70,71),70)
        self.assertIsNone(row_source_index(71,71))
        self.assertIsNone(row_source_index(120,71))

    def test_contact_display_preserves_short_event_and_no_data_region(self):
        bits,valid = contact_bins([0,.001,.002],[0,1,0],10,1)
        self.assertEqual(bits[0],1)
        self.assertFalse(bits[1:].any())
        self.assertTrue(valid[0]); self.assertFalse(valid[1:].any())

    def test_motion_layout_rigid_above_sand_and_missing_scene_is_neutral(self):
        red = np.full((540,960,3),(0,0,255),dtype=np.uint8)
        blue = np.full((540,960,3),(255,0,0),dtype=np.uint8)
        items = [motion("rigid"),motion("sand")]
        image = compose_motion_frame("probe",items,[red,blue],[1,1],.5,1,(-.02,.22))
        self.assertEqual(image.shape,(944,1280,3))
        np.testing.assert_array_equal(image[300,100],(0,0,255))
        np.testing.assert_array_equal(image[720,100],(255,0,0))
        stopped = compose_motion_frame("probe",items,[red,None],[1,None],.5,1,(-.02,.22))
        np.testing.assert_array_equal(stopped[850,100],(234,238,241))

    def test_geometry_sheet_has_context_mesh_clouds_and_per_surface_metrics(self):
        item = surface()
        image = geometry_sheet("probe",[item],np.full((400,360,3),180,dtype=np.uint8))
        self.assertEqual(image.shape,(960,1600,3))
        self.assertGreater(float(image[100:840,400:1580].std()),10)
        self.assertEqual(item["metrics"]["before"]["triangles_and_centroids"],2)
        self.assertEqual(item["metrics"]["after"]["area_m2"],1)

    def test_comparison_generator_has_no_step_or_remesh_calls(self):
        source = Path(__file__).resolve().parents[1]/"scripts/generate_robot_comparisons.py"
        tree = ast.parse(source.read_text())
        forbidden = {"mj_step","mj_step1","mj_step2","meshing_isotropic_explicit_remeshing","build_bundle"}
        calls = {n.func.attr if isinstance(n.func,ast.Attribute) else n.func.id
                 for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,(ast.Attribute,ast.Name))}
        self.assertFalse(calls&forbidden)


if __name__ == "__main__":
    unittest.main()
