import unittest

import numpy as np

from scripts.analyze_video_matrix import (
    component_metrics,
    trajectory_metrics,
)
from scripts.compose_video_matrix import compose_master_frame
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

    def test_master_frame_is_three_views_plus_one_panel_per_row(self):
        render_width = 160
        render_height = 100
        panel_width = 80
        frames = {}
        scenarios = (
            "rigid_original",
            "sand_simplified",
            "sand_simplified_sites",
        )
        views = ("top", "side", "diag45")
        for row, scenario in enumerate(scenarios):
            for column, view in enumerate(views):
                frame = np.full(
                    (render_height, render_width + panel_width, 3),
                    20 + row * 30 + column * 5,
                    dtype=np.uint8,
                )
                frame[:, render_width:] = 180 + row * 20
                frames[(scenario, view)] = frame

        result = compose_master_frame(
            frames,
            render_width=render_width,
            render_height=render_height,
            panel_width=panel_width,
            row_height=50,
            header_height=24,
        )

        self.assertEqual(result.shape, (174, 280, 3))
        # The final 40 px are one scenario-level analysis panel per row.
        self.assertTrue(np.all(result[49, 260] == 180))
        self.assertTrue(np.all(result[99, 260] == 200))
        self.assertTrue(np.all(result[149, 260] == 220))

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
