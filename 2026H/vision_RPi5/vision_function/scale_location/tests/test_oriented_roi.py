from __future__ import annotations

import unittest

import cv2
import numpy as np

from vision_function.scale_location.marker_detection import MarkerGeometry, MarkerRun
from vision_function.scale_location.oriented_roi import (
    extract_oriented_strip,
    fit_marker_axis,
)


def sloped_geometry() -> MarkerGeometry:
    direction = np.asarray((0.98, 0.20), dtype=np.float64)
    direction /= np.linalg.norm(direction)
    zero = np.asarray((320.0, 72.0), dtype=np.float64)
    runs = []
    for value in range(-3, 4):
        center = zero + direction * (value * 22.0)
        runs.append(
            MarkerRun(
                color=(
                    "magenta"
                    if value == 0
                    else ("yellow" if abs(value) % 2 else "green")
                ),
                start_px=round(center[0] - 10),
                end_px=round(center[0] + 10),
                center_xy=(float(center[0]), float(center[1])),
            )
        )
    return MarkerGeometry(
        valid=True,
        reason="synthetic",
        frame_sequence=0,
        capture_timestamp_s=0.0,
        axis_origin_xy=(320.0, 72.0),
        axis_direction_xy=(float(direction[0]), float(direction[1])),
        anchors=(),
        runs=tuple(runs),
        spacing_mean_px=22.0,
        spacing_cv=0.0,
    )


class OrientedRoiTests(unittest.TestCase):
    def test_axis_fit_uses_all_marker_centres(self) -> None:
        origin, direction = fit_marker_axis(sloped_geometry())

        np.testing.assert_allclose(origin, (320.0, 72.0), atol=1e-6)
        np.testing.assert_allclose(direction, (0.979804, 0.19996), atol=1e-5)

    def test_extracts_sloped_line_as_horizontal_strip(self) -> None:
        geometry = sloped_geometry()
        image = np.zeros((100, 640, 3), dtype=np.uint8)
        points = np.rint([run.center_xy for run in geometry.runs]).astype(np.int32)
        cv2.polylines(image, [points], False, (0, 255, 0), thickness=3)

        strip = extract_oriented_strip(image, geometry, output_height=30)
        green_y, _ = np.nonzero(strip.image[:, :, 1] > 100)

        self.assertTrue(len(green_y))
        self.assertLessEqual(int(green_y.max() - green_y.min()), 5)
        mapped = strip.points_to_source(np.asarray([[320.0, 14.5]]))
        np.testing.assert_allclose(mapped[0], (320.0, 72.0), atol=1e-6)


if __name__ == "__main__":
    unittest.main()
