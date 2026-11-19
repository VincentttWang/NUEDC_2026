from __future__ import annotations

import argparse
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from vision_function.ball_detection.preprocess import TubeRectificationConfig
from vision_function.scale_location.calibrate_scale import (
    CalibrationPoint,
    build_payload,
    expand_image_paths,
    fit_axis,
    load_points,
    parse_marks,
    render_preview,
)
from vision_function.scale_location.calibration import load_scale_calibration


class ScaleCalibrationToolTests(unittest.TestCase):
    def setUp(self) -> None:
        self.points = (
            CalibrationPoint(-12.0, 60.0, 44.0),
            CalibrationPoint(-6.0, 170.0, 47.0),
            CalibrationPoint(0.0, 286.0, 50.0),
            CalibrationPoint(6.0, 430.0, 54.0),
            CalibrationPoint(12.0, 580.0, 58.0),
        )
        self.config = TubeRectificationConfig(
            reference_size=(1280, 720),
            decode_scale=2,
            balance=0.2,
            reference_origin_xy=(80, 252),
            reference_output_size=(1280, 200),
            coordinate_space="tube_local_rectified_plane",
        )

    def test_parse_marks_requires_strict_increase(self) -> None:
        self.assertEqual(parse_marks("-12, 0, 12"), (-12.0, 0.0, 12.0))
        with self.assertRaisesRegex(argparse.ArgumentTypeError, "strictly increasing"):
            parse_marks("-12,0,0,12")

    def test_expand_image_directory_ignores_non_images(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "b.jpg").write_bytes(b"jpg")
            (root / "a.png").write_bytes(b"png")
            (root / "notes.txt").write_text("ignored", encoding="utf-8")
            self.assertEqual(
                [path.name for path in expand_image_paths([root])],
                ["a.png", "b.jpg"],
            )

    def test_fit_axis_uses_zero_mark_and_points_image_right(self) -> None:
        origin, direction, axis_values = fit_axis(self.points)
        self.assertEqual(origin, (286.0, 50.0))
        self.assertGreater(direction[0], 0.99)
        self.assertAlmostEqual(np.hypot(*direction), 1.0)
        self.assertAlmostEqual(axis_values[2], 0.0)
        self.assertTrue(
            all(left < right for left, right in zip(axis_values, axis_values[1:]))
        )

    def test_fit_axis_reports_the_first_reversed_pair(self) -> None:
        points = (
            CalibrationPoint(-3.0, 200.0, 50.0),
            CalibrationPoint(0.0, 300.0, 50.0),
            CalibrationPoint(3.0, 280.0, 50.0),
            CalibrationPoint(6.0, 400.0, 50.0),
        )
        with self.assertRaisesRegex(ValueError, r"0 cm .* but 3 cm .* left to right"):
            fit_axis(points)

    def test_load_points_and_build_runtime_compatible_payload(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            points_path = root / "points.json"
            source_path = root / "source.png"
            rectification_path = root / "rectification.json"
            output_path = root / "calibration.json"
            points_path.write_text(
                json.dumps(
                    [
                        {"scale_cm": point.scale_cm, "x": point.x, "y": point.y}
                        for point in self.points
                    ]
                ),
                encoding="utf-8",
            )
            source_path.write_bytes(b"source")
            rectification_path.write_text("{}", encoding="utf-8")

            points = load_points(points_path)
            payload = build_payload(
                points,
                self.config,
                [source_path],
                rectification_path,
                (-12.0, 12.0),
                0.75,
            )
            output_path.write_text(json.dumps(payload), encoding="utf-8")
            calibration = load_scale_calibration(output_path)

            self.assertEqual(calibration.status, "final")
            self.assertEqual(calibration.image_size, (640, 100))
            self.assertAlmostEqual(calibration.measure((286.0, 50.0)).position_cm, 0.0)
            self.assertEqual(len(payload["source"]["images"][0]["sha256"]), 64)

    def test_preview_has_readable_scaled_shape(self) -> None:
        image = np.zeros((100, 640, 3), dtype=np.uint8)
        origin, direction, _axis_values = fit_axis(self.points)
        preview = render_preview(image, self.points, origin, direction)
        self.assertEqual(preview.shape, (300, 1920, 3))
        self.assertGreater(int(preview.max()), 0)


if __name__ == "__main__":
    unittest.main()
