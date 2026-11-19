from __future__ import annotations

import unittest

from pathlib import Path

from vision_function.ball_detection.benchmark_rpi import (
    load_fisheye_balance,
    load_roi,
    scale_roi,
)


class BallBenchmarkTest(unittest.TestCase):
    def test_loads_actual_reference_roi_and_balance(self) -> None:
        config = (
            Path(__file__).parents[1]
            / "ball_detection"
            / "config"
            / "actual_roi.json"
        )
        self.assertEqual(load_roi(config), (120, 280, 1260, 440))
        self.assertEqual(load_fisheye_balance(config), 0.2)

    def test_scales_reference_roi_for_reduced_jpeg_decode(self) -> None:
        self.assertEqual(
            scale_roi((300, 270, 1150, 420), 2),
            (150, 135, 575, 210),
        )

    def test_loads_region_extending_beyond_nominal_canvas(self) -> None:
        config = (
            Path(__file__).parents[1]
            / "ball_detection"
            / "config"
            / "tube_local_rectification.json"
        )
        self.assertEqual(load_roi(config), (80, 252, 1360, 452))
        self.assertEqual(scale_roi(load_roi(config), 2), (40, 126, 680, 226))
        self.assertEqual(load_fisheye_balance(config), 0.2)

    def test_rejects_roi_that_cannot_be_scaled_exactly(self) -> None:
        with self.assertRaisesRegex(ValueError, "not exactly divisible"):
            scale_roi((301, 270, 1150, 420), 2)

    def test_rejects_unsupported_decode_scale(self) -> None:
        with self.assertRaisesRegex(ValueError, "Unsupported decode scale"):
            scale_roi((300, 270, 1150, 420), 4)


if __name__ == "__main__":
    unittest.main()
