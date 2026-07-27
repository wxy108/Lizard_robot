import unittest

import numpy as np

from scripts.analyze_video_matrix import (
    component_metrics,
    trajectory_metrics,
)
from scripts.generate_video_matrix import (
    binary_intervals,
    compose_dashboard,
    named_component_for_body,
    simulation_frame_steps,
)


class VideoMatrixTests(unittest.TestCase):
    def test_binary_intervals_records_start_and_end(self):
        time = np.arange(6, dtype=float) * 0.1
        contact = np.array([0, 1, 1, 0, 1, 0], dtype=np.uint8)

        intervals = binary_intervals(time, contact, dt=0.1)

        np.testing.assert_allclose(intervals, [(0.1, 0.3), (0.4, 0.5)])

    def test_binary_intervals_closes_contact_at_end_of_run(self):
        time = np.arange(4, dtype=float) * 0.1
        contact = np.array([0, 0, 1, 1], dtype=np.uint8)

        intervals = binary_intervals(time, contact, dt=0.1)

        np.testing.assert_allclose(intervals, [(0.2, 0.4)])

    def test_nearest_named_component_wins(self):
        parents = np.array([0, 0, 1, 2, 2])
        named = {"Mid": 1, "FR": 2}

        self.assertEqual(
            named_component_for_body(3, parents, named),
            "FR",
        )
        self.assertEqual(
            named_component_for_body(1, parents, named),
            "Mid",
        )
        self.assertIsNone(named_component_for_body(0, parents, named))

    def test_frame_steps_include_final_state(self):
        steps = simulation_frame_steps(n_steps=101, dt=0.001, fps=20)

        self.assertEqual(steps[0], 0)
        self.assertEqual(steps[-1], 100)
        self.assertTrue(np.all(np.diff(steps) > 0))

    def test_dashboard_has_expected_shape_and_content(self):
        frame = np.zeros((240, 320, 3), dtype=np.uint8)
        time = np.linspace(0.01, 0.2, 20)
        com = np.column_stack(
            (
                np.linspace(0.0, 0.1, 20),
                np.linspace(0.0, -0.02, 20),
                np.linspace(0.03, 0.04, 20),
            )
        )
        contact = np.zeros((20, 8), dtype=np.uint8)
        contact[4:9, 2] = 1

        result = compose_dashboard(
            frame,
            scenario_label="test scenario",
            view="side",
            current_step=10,
            time_s=time,
            com=com,
            contact=contact,
            panel_width=320,
        )

        self.assertEqual(result.shape, (240, 640, 3))
        self.assertGreater(int(np.count_nonzero(result[:, 320:])), 0)

    def test_analysis_metrics_include_contact_and_penetration(self):
        time = np.arange(5, dtype=float) * 0.1
        contact = np.array([[0], [1], [1], [0], [0]], dtype=np.uint8)
        penetration = np.array([[0], [0.001], [0.003], [0], [0]])
        active = np.array([[0], [2], [4], [0], [0]])

        rows = component_metrics(
            time,
            contact,
            ["FR"],
            max_penetration_m=penetration,
            active_triangles=active,
        )

        self.assertAlmostEqual(rows[0]["contact_duty"], 0.4)
        self.assertEqual(rows[0]["contact_events"], 1)
        self.assertAlmostEqual(rows[0]["maximum_penetration_mm"], 3.0)
        self.assertAlmostEqual(
            rows[0]["mean_active_triangles_when_contacting"],
            3.0,
        )

    def test_trajectory_metrics_use_center_of_mass(self):
        time = np.array([0.1, 0.2, 0.3])
        com = np.array(
            [[0.0, 0.0, 0.0], [0.03, 0.04, 0.0], [0.03, 0.04, 0.12]]
        )

        result = trajectory_metrics(time, com)

        np.testing.assert_allclose(
            result["com_displacement_m"],
            [0.03, 0.04, 0.12],
        )
        self.assertAlmostEqual(result["com_path_length_m"], 0.17)


if __name__ == "__main__":
    unittest.main()
