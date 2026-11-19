from __future__ import annotations

from pathlib import Path
import unittest

import cv2
import numpy as np

from vision_function.ball_detection.preprocess import (
    TubePreprocessor,
    load_tube_rectification,
)


class BallPreprocessTest(unittest.TestCase):
    def setUp(self) -> None:
        self.config_path = (
            Path(__file__).parents[1]
            / "ball_detection"
            / "config"
            / "tube_local_rectification.json"
        )

    def test_loads_tube_local_rectification(self) -> None:
        config = load_tube_rectification(self.config_path)
        self.assertEqual(config.reference_size, (1280, 720))
        self.assertEqual(config.input_size, (640, 360))
        self.assertEqual(config.reference_origin_xy, (80, 252))
        self.assertEqual(config.origin_xy, (40, 126))
        self.assertEqual(config.output_size, (640, 100))
        self.assertEqual(config.balance, 0.2)

    def test_preprocess_accepts_reference_and_reduced_frames(self) -> None:
        config = load_tube_rectification(self.config_path)
        processor = TubePreprocessor(config)
        rng = np.random.default_rng(2028)
        reference = rng.integers(0, 256, size=(720, 1280, 3), dtype=np.uint8)
        reduced = cv2.resize(reference, (640, 360), interpolation=cv2.INTER_AREA)
        expected = processor(reduced)
        actual = processor(reference)
        self.assertEqual(actual.shape, (100, 640, 3))
        np.testing.assert_array_equal(actual, expected)

    def test_legacy_roi_config_remains_supported(self) -> None:
        legacy = self.config_path.with_name("actual_roi.json")
        config = load_tube_rectification(legacy)
        self.assertEqual(config.origin_xy, (60, 140))
        self.assertEqual(config.output_size, (570, 80))


if __name__ == "__main__":
    unittest.main()
