from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest

from vision_function.scale_location.calibration import load_scale_calibration


ROOT = Path(__file__).resolve().parents[1]
TEMPORARY_CALIBRATION = (
    ROOT / "scale_location/config/scale_calibration_temporary_25cm.json"
)
FINAL_CALIBRATION = ROOT / "scale_location/config/scale_calibration.json"


class ScaleCalibrationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.calibration = load_scale_calibration(TEMPORARY_CALIBRATION)

    def test_temporary_anchor_measurements(self) -> None:
        cases = (
            ((59.0, 50.0), -12.5),
            ((286.0, 50.0), 0.0),
            ((582.0, 50.0), 12.5),
            ((172.5, 50.0), -6.25),
            ((434.0, 50.0), 6.25),
        )
        for point, expected in cases:
            with self.subTest(point=point):
                measurement = self.calibration.measure(point)
                self.assertTrue(measurement.valid)
                self.assertAlmostEqual(measurement.position_cm, expected)

    def test_final_calibration_is_accepted_and_records_validation(self) -> None:
        calibration = load_scale_calibration(FINAL_CALIBRATION)
        payload = json.loads(FINAL_CALIBRATION.read_text(encoding="utf-8"))
        self.assertEqual(calibration.status, "final")
        self.assertEqual(calibration.image_size, (640, 100))
        self.assertEqual(calibration.valid_range_cm, (-12.0, 12.0))
        self.assertEqual(len(calibration.anchors), 9)
        self.assertEqual(
            payload["validation"]["status"], "accepted_for_current_iteration"
        )
        self.assertLess(payload["validation"]["maximum_absolute_bias_cm"], 1.0)

    def test_projection_ignores_y_for_horizontal_temporary_axis(self) -> None:
        self.assertAlmostEqual(self.calibration.measure((286.0, 10.0)).position_cm, 0.0)
        self.assertAlmostEqual(self.calibration.measure((286.0, 90.0)).position_cm, 0.0)

    def test_outside_anchors_is_clamped_and_marks_edge(self) -> None:
        for point, expected_cm in (((58.9, 50.0), -12.5), ((582.1, 50.0), 12.5)):
            measurement = self.calibration.measure(point)
            self.assertTrue(measurement.valid)
            self.assertAlmostEqual(measurement.position_cm, expected_cm)
            self.assertTrue(measurement.edge)

    def test_edge_margin(self) -> None:
        self.assertTrue(self.calibration.measure((59.0, 50.0)).edge)
        self.assertFalse(self.calibration.measure((286.0, 50.0)).edge)

    def test_rejects_non_monotonic_anchors(self) -> None:
        payload = json.loads(TEMPORARY_CALIBRATION.read_text(encoding="utf-8"))
        payload["anchors"][1]["axis_px"] = payload["anchors"][0]["axis_px"]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "invalid.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "strictly increasing"):
                load_scale_calibration(path)


if __name__ == "__main__":
    unittest.main()
