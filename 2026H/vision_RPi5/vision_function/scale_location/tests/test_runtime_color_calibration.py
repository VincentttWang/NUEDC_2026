from __future__ import annotations

from pathlib import Path
import sys
import types
import unittest

import cv2
import numpy as np

try:
    import termios  # noqa: F401
except ImportError:
    sys.modules["termios"] = types.ModuleType("termios")

from vision_function.scale_location.calibration import load_scale_calibration
from vision_function.scale_location.dynamic_calibration import ColorScaleTracker
from vision_function.scale_location.estimator import MotionEstimator, MotionFilterConfig
from vision_function.scale_location.marker_detection import (
    ColorMarkerDetector,
    MarkerDetectionConfig,
)
from vision_function.scale_location.pipeline import ScalePipeline
from vision_function.scale_location.protocol import FrameFlags
from vision_function.scale_location.runtime import (
    HexPacketWriter,
    VisionUartRuntime,
)
from vision_function.scale_location.types import BallObservation
from vision_function.scale_location.uart_transport import FrameLockedSender


PACKAGE_ROOT = Path(__file__).parents[1]
SAMPLE_DIR = PACKAGE_ROOT / "calibration_assets" / "color_ruler"


class FakeSource:
    def __init__(self) -> None:
        self.sequence = 0
        self.current_timestamp_s = 1.0

    def fetch(self) -> tuple[bytes, float]:
        self.current_timestamp_s = 1.0 + self.sequence * 0.02
        self.sequence += 1
        return b"unused", self.current_timestamp_s

    def clock(self) -> float:
        return self.current_timestamp_s

    def close(self) -> None:
        pass


class FakeBallDetector:
    def __init__(self, crop: np.ndarray) -> None:
        self.crop = crop

    def detect(
        self,
        frame: np.ndarray,
        *,
        capture_timestamp_s: float,
        frame_sequence: int,
    ) -> tuple[BallObservation, np.ndarray]:
        return (
            BallObservation(
                frame_sequence=frame_sequence,
                capture_timestamp_s=capture_timestamp_s,
                found=True,
                center_local_xy=(310.0, 80.0),
                confidence=0.9,
                candidate_count=1,
            ),
            self.crop.copy(),
        )


class RuntimeColorCalibrationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.marker_config = MarkerDetectionConfig.load(
            PACKAGE_ROOT / "config" / "color_markers.json"
        )
        reference_path = sorted(SAMPLE_DIR.glob("tube-roi-*.png"))[0]
        reference = cv2.imread(str(reference_path), cv2.IMREAD_COLOR)
        self.assertIsNotNone(reference)
        self.valid_crop = cv2.resize(
            reference,
            self.marker_config.image_size,
            interpolation=cv2.INTER_AREA,
        )

    def build_runtime(
        self, crop: np.ndarray
    ) -> tuple[VisionUartRuntime, ScalePipeline, HexPacketWriter]:
        source = FakeSource()
        fixed = load_scale_calibration(
            PACKAGE_ROOT / "config" / "scale_calibration.json"
        )
        motion = MotionFilterConfig.load(
            PACKAGE_ROOT / "config" / "motion_filter.json"
        )
        pipeline = ScalePipeline(
            fixed,
            MotionEstimator(motion),
            position_sign=-1,
        )
        tracker = ColorScaleTracker(
            ColorMarkerDetector(self.marker_config),
            self.marker_config,
        )
        writer = HexPacketWriter()
        sender = FrameLockedSender(writer, clock=source.clock)
        runtime = VisionUartRuntime(
            source,
            FakeBallDetector(crop),
            pipeline,
            sender,
            decoder=lambda encoded: np.zeros((360, 640, 3), dtype=np.uint8),
            clock=source.clock,
            marker_tracker=tracker,
            marker_calibration_required=True,
        )
        return runtime, pipeline, writer

    def test_real_marker_calibration_replaces_fixed_mapping_before_uart(self) -> None:
        runtime, pipeline, writer = self.build_runtime(self.valid_crop)
        results = [runtime.process_next() for _ in range(5)]
        self.assertTrue(all(result is not None for result in results))
        final = results[-1]
        self.assertEqual(pipeline.calibration.calibration_type, "runtime_color_markers")
        self.assertEqual(pipeline.calibration.status, self.marker_config.status)
        self.assertTrue(final.marker.geometry_valid)
        self.assertTrue(final.send.frame.flags & FrameFlags.VALID)
        self.assertEqual(writer.last_packet, final.send.packet)

    def test_invalid_marker_geometry_never_falls_back_to_fixed_measurement(self) -> None:
        runtime, pipeline, _ = self.build_runtime(
            np.zeros((100, 640, 3), dtype=np.uint8)
        )
        results = [runtime.process_next() for _ in range(5)]
        self.assertEqual(pipeline.calibration.calibration_type, "multi_mark_piecewise_linear")
        self.assertTrue(all(not result.state.valid for result in results))
        self.assertTrue(
            all(not (result.send.frame.flags & FrameFlags.VALID) for result in results)
        )
        self.assertEqual(runtime.stats.marker_blocked_frames, 5)


if __name__ == "__main__":
    unittest.main()
