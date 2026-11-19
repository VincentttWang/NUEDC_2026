from __future__ import annotations

import json
from pathlib import Path
import unittest

import cv2
import numpy as np

from vision_function.fisheye import FisheyeUndistorter, load_fisheye_calibration


class FisheyeUndistorterTest(unittest.TestCase):
    def test_loads_reused_camera_calibration(self) -> None:
        calibration = load_fisheye_calibration()
        self.assertEqual(calibration.image_size, (1280, 720))
        self.assertAlmostEqual(calibration.rms_px, 0.34363786897152304)
        self.assertEqual(calibration.distortion.shape, (4, 1))

    def test_maps_are_cached_for_realtime_processing(self) -> None:
        undistorter = FisheyeUndistorter()
        first = undistorter.maps((1280, 720))
        second = undistorter.maps((1280, 720))
        self.assertIs(first, second)
        self.assertEqual(first[0].shape[:2], (720, 1280))

    def test_default_map_matches_opencv_fisheye_model(self) -> None:
        calibration = load_fisheye_calibration()
        undistorter = FisheyeUndistorter(calibration)
        map1, map2, output = undistorter.maps((1280, 720))
        float_x, float_y = cv2.convertMaps(map1, map2, cv2.CV_32FC1)
        expected_x, expected_y = cv2.fisheye.initUndistortRectifyMap(
            calibration.camera_matrix,
            calibration.distortion,
            np.eye(3),
            output,
            (1280, 720),
            cv2.CV_32FC1,
        )
        np.testing.assert_allclose(float_x, expected_x, atol=1 / 32)
        np.testing.assert_allclose(float_y, expected_y, atol=1 / 32)

    def test_undistort_preserves_frame_shape(self) -> None:
        frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        corrected = FisheyeUndistorter().undistort(frame)
        self.assertEqual(corrected.shape, frame.shape)
        self.assertEqual(corrected.dtype, frame.dtype)

    def test_roi_maps_are_cached_and_have_roi_shape(self) -> None:
        undistorter = FisheyeUndistorter()
        roi = (300, 270, 1150, 420)
        first = undistorter.roi_maps((1280, 720), roi)
        second = undistorter.roi_maps((1280, 720), roi)
        self.assertIs(first, second)
        self.assertEqual(first[0].shape[:2], (150, 850))

    def test_roi_remap_matches_full_remap_then_crop(self) -> None:
        rng = np.random.default_rng(2026)
        frame = rng.integers(0, 256, size=(720, 1280, 3), dtype=np.uint8)
        roi = (300, 270, 1150, 420)
        undistorter = FisheyeUndistorter()
        expected = undistorter.undistort(frame)[270:420, 300:1150]
        actual = undistorter.undistort_roi(frame, roi)
        np.testing.assert_array_equal(actual, expected)

    def test_roi_remap_rejects_out_of_bounds_roi(self) -> None:
        frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        with self.assertRaisesRegex(ValueError, "超出图像范围"):
            FisheyeUndistorter().undistort_roi(frame, (300, 270, 1300, 420))

    def test_rectified_region_can_extend_beyond_nominal_canvas(self) -> None:
        frame = np.zeros((360, 640, 3), dtype=np.uint8)
        undistorter = FisheyeUndistorter(balance=0.2)
        first = undistorter.rectified_region_maps(
            (640, 360), (40.0, 126.0), (640, 100)
        )
        second = undistorter.rectified_region_maps(
            (640, 360), (40.0, 126.0), (640, 100)
        )
        corrected = undistorter.undistort_rectified_region(
            frame, (40.0, 126.0), (640, 100)
        )
        self.assertIs(first, second)
        self.assertEqual(first[0].shape[:2], (100, 640))
        self.assertEqual(corrected.shape, (100, 640, 3))

    def test_rectified_region_matches_in_bounds_full_frame_crop(self) -> None:
        rng = np.random.default_rng(2027)
        frame = rng.integers(0, 256, size=(360, 640, 3), dtype=np.uint8)
        undistorter = FisheyeUndistorter(balance=0.2)
        expected = undistorter.undistort(frame)[126:226, 40:640]
        actual = undistorter.undistort_rectified_region(
            frame, (40.0, 126.0), (600, 100)
        )
        np.testing.assert_array_equal(actual, expected)

    def test_rectified_region_rejects_invalid_output_size(self) -> None:
        with self.assertRaisesRegex(ValueError, "输出尺寸"):
            FisheyeUndistorter().rectified_region_maps(
                (640, 360), (0.0, 0.0), (640, 0)
            )

    def test_supports_same_aspect_ratio_scaling(self) -> None:
        map1, _, _ = FisheyeUndistorter().maps((1920, 1080))
        self.assertEqual(map1.shape[:2], (1080, 1920))

    def test_rejects_unverified_aspect_ratio(self) -> None:
        with self.assertRaisesRegex(ValueError, "宽高比"):
            FisheyeUndistorter().maps((640, 480))

    def test_undistort_points_preserves_input_shape(self) -> None:
        points = np.asarray([[100.0, 200.0], [640.0, 360.0]])
        corrected = FisheyeUndistorter().undistort_points(points, (1280, 720))
        self.assertEqual(corrected.shape, points.shape)
        self.assertTrue(np.all(np.isfinite(corrected)))

    def test_actual_vehicle_roi_matches_half_resolution_pipeline(self) -> None:
        path = (
            Path(__file__).resolve().parents[1]
            / "ball_detection"
            / "config"
            / "actual_roi.json"
        )
        payload = json.loads(path.read_text(encoding="utf-8"))
        reference = payload["reference_roi"]
        production = payload["production_roi"]
        scale = payload["production_decode_scale"]
        for key in ("left", "top", "right", "bottom", "width", "height"):
            self.assertEqual(reference[key] // scale, production[key])
        map1, _, _ = FisheyeUndistorter(
            balance=payload["fisheye_balance"]
        ).roi_maps(
            (640, 360),
            (
                production["left"],
                production["top"],
                production["right"],
                production["bottom"],
            ),
        )
        self.assertEqual(map1.shape[:2], (80, 570))


if __name__ == "__main__":
    unittest.main()
