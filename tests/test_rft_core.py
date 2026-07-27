import unittest

import numpy as np

from sim_fxn_lib import rft_3D_body_full_mat


class RftCoreTests(unittest.TestCase):
    def setUp(self):
        # A 1 cm^2 horizontal plate whose outward normal points downward.
        self.body = {
            "r": np.array([[0.0, 0.0, 0.0]]),
            "n": np.array([[0.0, 0.0, -1.0]]),
            "A": np.array([1e-4]),
        }

    def test_sand_reaction_opposes_downward_intrusion(self):
        velocity = np.array([0.0, 0.0, -0.1])
        raw_force, _, _, _, include, _, _ = rft_3D_body_full_mat(
            self.body,
            np.array([0.0, 0.0, -0.01]),
            None,
            velocity,
            np.zeros(3),
            sand_height_m=0.0,
            rotation_matrix=np.eye(3),
        )

        reaction_force = -raw_force
        self.assertTrue(include[0])
        self.assertGreater(reaction_force[2], 0.0)
        self.assertLess(np.dot(reaction_force, velocity), 0.0)

    def test_plate_above_sand_has_no_force(self):
        raw_force, _, _, _, include, full_force, _ = rft_3D_body_full_mat(
            self.body,
            np.array([0.0, 0.0, 0.01]),
            None,
            np.array([0.0, 0.0, -0.1]),
            np.zeros(3),
            sand_height_m=0.0,
            rotation_matrix=np.eye(3),
        )

        self.assertFalse(include[0])
        np.testing.assert_array_equal(raw_force, np.zeros(3))
        np.testing.assert_array_equal(full_force, np.zeros((1, 3)))

    def test_rotation_matrix_matches_legacy_yaw_pitch_roll(self):
        yaw, pitch, roll = 0.31, -0.22, 0.17
        cz, sz = np.cos(yaw), np.sin(yaw)
        cy, sy = np.cos(pitch), np.sin(pitch)
        cx, sx = np.cos(roll), np.sin(roll)
        rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
        ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
        rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
        world_from_body = rz @ ry @ rx

        kwargs = dict(
            body_m=self.body,
            origin_m=np.array([0.0, 0.0, -0.02]),
            vel_mps=np.array([0.0, 0.0, -0.1]),
            ang_vel_radps=np.zeros(3),
            sand_height_m=0.0,
        )
        legacy = rft_3D_body_full_mat(
            orientation_rad=[yaw, pitch, roll],
            **kwargs,
        )
        direct = rft_3D_body_full_mat(
            orientation_rad=None,
            rotation_matrix=world_from_body,
            **kwargs,
        )

        np.testing.assert_allclose(legacy[0], direct[0], rtol=1e-12, atol=1e-12)
        np.testing.assert_array_equal(legacy[4], direct[4])


if __name__ == "__main__":
    unittest.main()
