from __future__ import annotations

from pathlib import Path
import unittest

import cv2
import numpy as np

from vision_function.scale_location.dynamic_calibration import ColorScaleTracker
from vision_function.scale_location.marker_detection import (
    ColorMarkerDetector,
    MarkerDetectionConfig,
    MarkerGeometry,
    MarkerRun,
)


PACKAGE_ROOT = Path(__file__).parents[1]
CONFIG_PATH = PACKAGE_ROOT / "config" / "color_markers.json"
REAL_SAMPLE_DIR = PACKAGE_ROOT / "calibration_assets" / "color_ruler"
REGRESSION_SAMPLE_DIR = PACKAGE_ROOT / "data" / "距离误识别"


def read_image(path: Path) -> np.ndarray | None:
    """Read paths containing non-ASCII characters on Windows OpenCV builds."""
    encoded = np.fromfile(path, dtype=np.uint8)
    return cv2.imdecode(encoded, cv2.IMREAD_COLOR) if encoded.size else None


def synthetic_scale(
    config: MarkerDetectionConfig,
    *,
    replace_colors: dict[int, str] | None = None,
) -> np.ndarray:
    image = np.full((100, 640, 3), 245, dtype=np.uint8)
    color_by_name = {color.name: color.design_rgb for color in config.colors}
    x = config.roi_xywh[0] + 10
    y = config.roi_xywh[1]
    band_width = 20
    gap = 2
    replace_colors = replace_colors or {}
    for scale_cm in range(config.minimum_cm, config.maximum_cm + 1):
        name = replace_colors.get(scale_cm, config.expected_color(scale_cm))
        red, green, blue = color_by_name[name]
        image[y : y + config.roi_xywh[3], x : x + band_width] = (blue, green, red)
        x += band_width + gap
    return image


def synthetic_runs(
    config: MarkerDetectionConfig,
    *,
    width_by_scale: dict[int, int] | None = None,
) -> tuple[MarkerRun, ...]:
    width_by_scale = width_by_scale or {}
    runs = []
    for index, scale_cm in enumerate(
        range(config.minimum_cm, config.maximum_cm + 1)
    ):
        center_x = 60.0 + 22.0 * index
        width = width_by_scale.get(scale_cm, 20)
        runs.append(
            MarkerRun(
                color=config.expected_color(scale_cm),
                start_px=round(center_x - width / 2.0),
                end_px=round(center_x + width / 2.0),
                center_xy=(center_x, 75.0),
            )
        )
    return tuple(runs)


class ColorMarkerDetectorTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = MarkerDetectionConfig.load(CONFIG_PATH)
        self.detector = ColorMarkerDetector(self.config)

    def detect(self, image: np.ndarray, sequence: int = 0) -> MarkerGeometry:
        return self.detector.detect(
            image,
            frame_sequence=sequence,
            capture_timestamp_s=sequence / 30.0,
        )

    def test_detects_ordered_anchors_and_zero(self) -> None:
        geometry = self.detect(synthetic_scale(self.config))
        self.assertTrue(geometry.valid, geometry.reason)
        self.assertEqual(len(geometry.runs), 23)
        self.assertEqual(len(geometry.anchors), 23)
        self.assertEqual(
            [anchor.scale_cm for anchor in geometry.anchors], list(range(-11, 12))
        )
        self.assertAlmostEqual(geometry.anchors[11].axis_px, 0.0)
        self.assertEqual(geometry.runs[11].color, "magenta")

    def test_missing_magenta_is_invalid(self) -> None:
        geometry = self.detect(
            synthetic_scale(self.config, replace_colors={0: "green"})
        )
        self.assertFalse(geometry.valid)
        self.assertIn("found 0", geometry.reason)

    def test_duplicate_magenta_is_invalid(self) -> None:
        geometry = self.detect(
            synthetic_scale(self.config, replace_colors={2: "magenta"})
        )
        self.assertFalse(geometry.valid)
        self.assertIn("found 2", geometry.reason)

    def test_unique_magenta_without_neighbouring_marks_is_invalid(self) -> None:
        image = np.full((100, 640, 3), 245, dtype=np.uint8)
        color_by_name = {color.name: color.design_rgb for color in self.config.colors}
        red, green, blue = color_by_name["magenta"]
        x = 300
        y = self.config.roi_xywh[1]
        image[y : y + self.config.roi_xywh[3], x : x + 20] = (blue, green, red)

        geometry = self.detect(image)

        self.assertFalse(geometry.valid)
        self.assertIn("one consecutive marker", geometry.reason)

    def test_truncated_scale_uses_visible_marker_centers(self) -> None:
        image = synthetic_scale(self.config)
        image[:, 450:] = 245

        geometry = self.detect(image)

        self.assertTrue(geometry.valid, geometry.reason)
        self.assertIn("partial visible scale", geometry.reason)
        self.assertEqual(geometry.anchors[0].scale_cm, -11.0)
        self.assertLess(geometry.anchors[-1].scale_cm, 11.0)
        calibration = geometry.to_calibration(self.config)
        self.assertEqual(calibration.valid_range_cm[0], -11.0)
        self.assertEqual(
            calibration.valid_range_cm[1], geometry.anchors[-1].scale_cm
        )
        for anchor, run in zip(geometry.anchors, geometry.runs):
            measurement = calibration.measure(run.center_xy)
            self.assertTrue(measurement.valid)
            self.assertAlmostEqual(measurement.position_cm, anchor.scale_cm)

    def test_left_truncated_scale_keeps_zero_anchor(self) -> None:
        image = synthetic_scale(self.config)
        image[:, :160] = 245

        geometry = self.detect(image)

        self.assertTrue(geometry.valid, geometry.reason)
        self.assertGreater(geometry.anchors[0].scale_cm, -11.0)
        zero_anchor = next(
            anchor for anchor in geometry.anchors if anchor.scale_cm == 0.0
        )
        self.assertAlmostEqual(zero_anchor.axis_px, 0.0)
        calibration = geometry.to_calibration(self.config)
        measurement = calibration.measure(geometry.axis_origin_xy)
        self.assertTrue(measurement.valid)
        self.assertAlmostEqual(measurement.position_cm, 0.0)

    def test_stops_before_a_missing_marker_instead_of_shifting_values(self) -> None:
        image = synthetic_scale(self.config)
        x = self.config.roi_xywh[0] + 10 + (20 + 2) * 18
        image[self.config.roi_xywh[1] :, x : x + 20] = 245

        geometry = self.detect(image)

        self.assertTrue(geometry.valid, geometry.reason)
        self.assertEqual(geometry.anchors[-1].scale_cm, 6.0)

    def test_recovers_one_narrow_internal_marker_from_neighbours(self) -> None:
        image = synthetic_scale(self.config)
        plus_one_x = self.config.roi_xywh[0] + 10 + (20 + 2) * 12
        image[
            self.config.roi_xywh[1] :
            self.config.roi_xywh[1] + self.config.roi_xywh[3],
            plus_one_x : plus_one_x + 13,
        ] = 245

        geometry = self.detect(image)

        self.assertTrue(geometry.valid, geometry.reason)
        self.assertIn("recovered 1 narrow marker", geometry.reason)
        plus_one = geometry.anchors[12]
        self.assertEqual(plus_one.scale_cm, 1.0)
        self.assertAlmostEqual(plus_one.axis_px, 22.0)

    def test_rejects_multiple_narrow_internal_markers(self) -> None:
        image = synthetic_scale(self.config)
        for scale_cm in (1, 3):
            index = scale_cm - self.config.minimum_cm
            marker_x = self.config.roi_xywh[0] + 10 + (20 + 2) * index
            image[
                self.config.roi_xywh[1] :
                self.config.roi_xywh[1] + self.config.roi_xywh[3],
                marker_x : marker_x + 13,
            ] = 245

        geometry = self.detect(image)

        self.assertFalse(geometry.valid)
        self.assertIn("width is outside", geometry.reason)

    def test_accepts_one_wide_run_when_centres_remain_regular(self) -> None:
        geometry = self.detector._geometry(
            synthetic_runs(self.config, width_by_scale={10: 32}),
            frame_sequence=0,
            capture_timestamp_s=0.0,
        )

        self.assertTrue(geometry.valid, geometry.reason)
        self.assertEqual(geometry.anchors[-1].scale_cm, 11.0)
        self.assertIn("accepted 1 wide marker", geometry.reason)

    def test_trims_one_narrow_edge_run(self) -> None:
        geometry = self.detector._geometry(
            synthetic_runs(self.config, width_by_scale={11: 8}),
            frame_sequence=0,
            capture_timestamp_s=0.0,
        )

        self.assertTrue(geometry.valid, geometry.reason)
        self.assertEqual(geometry.anchors[-1].scale_cm, 10.0)
        self.assertIn("trimmed 1 anomalous edge marker", geometry.reason)

    def test_rejects_multiple_wide_internal_runs(self) -> None:
        geometry = self.detector._geometry(
            synthetic_runs(self.config, width_by_scale={8: 32, 10: 32}),
            frame_sequence=0,
            capture_timestamp_s=0.0,
        )

        self.assertFalse(geometry.valid)
        self.assertIn("width is outside", geometry.reason)

    def test_nominal_fallback_rejects_magenta_outside_center_window(self) -> None:
        image = np.full((100, 640, 3), 245, dtype=np.uint8)
        color_by_name = {color.name: color.design_rgb for color in self.config.colors}
        red, green, blue = color_by_name["magenta"]
        x = 100
        y = self.config.roi_xywh[1]
        image[y : y + self.config.roi_xywh[3], x : x + 20] = (blue, green, red)

        geometry = self.detect(image)

        self.assertFalse(geometry.valid)
        self.assertIn("outside the configured window", geometry.reason)

    def test_generated_calibration_maps_integer_centers(self) -> None:
        geometry = self.detect(synthetic_scale(self.config))
        calibration = geometry.to_calibration(self.config)
        for anchor, run in zip(geometry.anchors, geometry.runs):
            measurement = calibration.measure(run.center_xy)
            self.assertTrue(measurement.valid)
            self.assertAlmostEqual(measurement.position_cm, anchor.scale_cm, places=6)

    def test_all_rectified_real_samples(self) -> None:
        paths = sorted(REAL_SAMPLE_DIR.glob("tube-roi-*.png"))
        self.assertEqual(len(paths), 5)
        zero_x = []
        for sequence, path in enumerate(paths):
            reference = read_image(path)
            self.assertIsNotNone(reference, path.name)
            production = cv2.resize(
                reference, self.config.image_size, interpolation=cv2.INTER_AREA
            )
            geometry = self.detect(production, sequence)
            self.assertTrue(geometry.valid, f"{path.name}: {geometry.reason}")
            self.assertEqual(len(geometry.anchors), 23)
            self.assertLess(geometry.spacing_cv, 0.08)
            zero_x.append(geometry.axis_origin_xy[0])
        self.assertLess(max(zero_x) - min(zero_x), 2.0)

    def test_sloped_distance_regression_samples_recover_full_scale(self) -> None:
        paths = sorted(REGRESSION_SAMPLE_DIR.glob("tube-roi-*.png"))
        if not paths:
            self.skipTest("distance regression samples are not present")
        self.assertEqual(len(paths), 4)

        geometries = []
        for sequence, path in enumerate(paths):
            reference = read_image(path)
            self.assertIsNotNone(reference, path.name)
            production = cv2.resize(
                reference, self.config.image_size, interpolation=cv2.INTER_AREA
            )
            geometry = self.detect(production, sequence)
            self.assertTrue(geometry.valid, f"{path.name}: {geometry.reason}")
            geometries.append(geometry)

        self.assertEqual(
            [geometry.anchors[-1].scale_cm for geometry in geometries],
            [11.0, 11.0, 11.0, 11.0],
        )
        for geometry in geometries[2:]:
            self.assertEqual(geometry.reason, "ok")
            self.assertLess(geometry.spacing_mean_px, 21.0)
            self.assertLess(geometry.axis_direction_xy[1], -0.005)
            calibration = geometry.to_calibration(self.config)
            for anchor, run in zip(geometry.anchors, geometry.runs):
                measurement = calibration.measure(run.center_xy)
                self.assertTrue(measurement.valid)
                self.assertAlmostEqual(measurement.position_cm, anchor.scale_cm)


class ScriptedDetector:
    def __init__(self, valid_by_sequence: dict[int, bool]) -> None:
        self.valid_by_sequence = valid_by_sequence
        self.calls: list[int] = []

    def detect(
        self,
        image: np.ndarray,
        *,
        frame_sequence: int,
        capture_timestamp_s: float,
    ) -> MarkerGeometry:
        self.calls.append(frame_sequence)
        valid = self.valid_by_sequence.get(frame_sequence, True)
        return MarkerGeometry(
            valid=valid,
            reason="ok" if valid else "synthetic failure",
            frame_sequence=frame_sequence,
            capture_timestamp_s=capture_timestamp_s,
            axis_origin_xy=(0.0, 0.0) if valid else None,
            axis_direction_xy=(1.0, 0.0) if valid else None,
            anchors=(),
            runs=(),
            spacing_mean_px=20.0 if valid else None,
            spacing_cv=0.0 if valid else None,
        )


class ColorScaleTrackerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = MarkerDetectionConfig.load(CONFIG_PATH)
        self.image = np.zeros((100, 640, 3), dtype=np.uint8)

    def test_regular_detection_cadence(self) -> None:
        detector = ScriptedDetector({})
        tracker = ColorScaleTracker(detector, self.config)
        attempted = []
        for sequence in range(7):
            status = tracker.update(
                self.image,
                frame_sequence=sequence,
                capture_timestamp_s=sequence / 100.0,
            )
            if status.attempted:
                attempted.append(sequence)
        self.assertEqual(attempted, [0, 2, 4, 6])

    def test_failure_retries_on_next_frame(self) -> None:
        detector = ScriptedDetector({2: False})
        tracker = ColorScaleTracker(detector, self.config)
        attempted = []
        for sequence in range(5):
            status = tracker.update(
                self.image,
                frame_sequence=sequence,
                capture_timestamp_s=sequence / 100.0,
            )
            if status.attempted:
                attempted.append(sequence)
        self.assertEqual(attempted, [0, 2, 3, 4])
        self.assertTrue(status.geometry_valid)

    def test_large_marker_angle_uses_oriented_strip_and_maps_back(self) -> None:
        image = synthetic_scale(self.config)
        zero = (304.0, 76.0)
        original = ColorMarkerDetector(self.config).detect(
            image, frame_sequence=0, capture_timestamp_s=0.0
        )
        self.assertTrue(original.valid)
        matrix = cv2.getRotationMatrix2D(zero, 3.0, 1.0)
        sloped = cv2.warpAffine(
            image,
            matrix,
            self.config.image_size,
            flags=cv2.INTER_NEAREST,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(245, 245, 245),
        )
        tracker = ColorScaleTracker(ColorMarkerDetector(self.config), self.config)

        status = tracker.update(
            sloped,
            frame_sequence=0,
            capture_timestamp_s=0.0,
        )

        self.assertTrue(status.geometry_valid, status.reason)
        self.assertTrue(status.reason.startswith("oriented strip:"), status.reason)
        self.assertEqual(tracker.oriented_attempts, 1)
        self.assertEqual(tracker.oriented_accepted, 1)
        self.assertEqual(tracker.oriented_fallbacks, 0)
        assert status.geometry is not None
        self.assertEqual(status.geometry.anchors[0].scale_cm, -11.0)
        self.assertEqual(status.geometry.anchors[-1].scale_cm, 11.0)
        # OpenCV's positive image rotation produces a negative image-space slope.
        angle = np.degrees(np.arctan2(*status.geometry.axis_direction_xy[::-1]))
        self.assertAlmostEqual(angle, -3.0, delta=1.0)
        expected_origin = cv2.transform(
            np.asarray([[original.axis_origin_xy]], dtype=np.float64), matrix
        )[0, 0]
        np.testing.assert_allclose(
            status.geometry.axis_origin_xy, expected_origin, atol=1.5
        )
        calibration = status.geometry.to_calibration(self.config)
        for run, anchor in zip(status.geometry.runs, status.geometry.anchors):
            self.assertAlmostEqual(
                calibration.project(run.center_xy), anchor.axis_px, places=5
            )

    def test_oriented_failure_falls_back_to_fixed_detection(self) -> None:
        image = synthetic_scale(self.config)
        matrix = cv2.getRotationMatrix2D((304.0, 76.0), 3.0, 1.0)
        sloped = cv2.warpAffine(
            image,
            matrix,
            self.config.image_size,
            flags=cv2.INTER_NEAREST,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(245, 245, 245),
        )
        tracker = ColorScaleTracker(ColorMarkerDetector(self.config), self.config)

        class FailingRefiner:
            def refine(self, *args, **kwargs):
                raise ValueError("synthetic oriented failure")

        tracker.oriented_refiner = FailingRefiner()
        status = tracker.update(
            sloped,
            frame_sequence=0,
            capture_timestamp_s=0.0,
        )

        self.assertTrue(status.geometry_valid, status.reason)
        self.assertFalse(status.reason.startswith("oriented strip:"))
        self.assertEqual(tracker.oriented_attempts, 1)
        self.assertEqual(tracker.oriented_accepted, 0)
        self.assertEqual(tracker.oriented_fallbacks, 1)


if __name__ == "__main__":
    unittest.main()
