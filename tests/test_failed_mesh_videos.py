import sys
import unittest
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from generate_failed_mesh_videos import (  # noqa: E402
    case_definitions,
    project_centroids,
    projection_center_radius,
)


class FailedMeshVideoTests(unittest.TestCase):
    def test_case_inputs_are_portable_repository_paths(self):
        cases = case_definitions()
        self.assertEqual(len(cases), 3)
        for case in cases:
            self.assertTrue(case.rejected_path.is_file())
            self.assertTrue(case.accepted_path.is_file())
            case.rejected_path.resolve().relative_to(ROOT)
            case.accepted_path.resolve().relative_to(ROOT)

    def test_projection_returns_finite_pixels_and_depth(self):
        first = np.array(
            [[-1.0, -0.5, 0.0], [1.0, 0.5, 0.25]],
            dtype=float,
        )
        second = np.array(
            [[-0.8, -0.4, -0.1], [0.8, 0.4, 0.2]],
            dtype=float,
        )
        center, radius = projection_center_radius(first, second)
        x_pixels, y_pixels, depth = project_centroids(
            first,
            center=center,
            radius=radius,
            azimuth_rad=0.4,
            elevation_rad=0.3,
            rectangle=(10, 20, 400, 300),
        )
        self.assertEqual(x_pixels.shape, (2,))
        self.assertEqual(y_pixels.shape, (2,))
        self.assertEqual(depth.shape, (2,))
        self.assertTrue(np.isfinite(depth).all())
        self.assertTrue(((x_pixels >= 10) & (x_pixels <= 410)).all())
        self.assertTrue(((y_pixels >= 20) & (y_pixels <= 320)).all())


if __name__ == "__main__":
    unittest.main()
