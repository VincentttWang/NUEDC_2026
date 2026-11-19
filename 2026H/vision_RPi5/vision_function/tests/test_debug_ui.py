from pathlib import Path
from tempfile import TemporaryDirectory
import threading
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from vision_function.ball_detection.preprocess import TubePreprocessor, load_tube_rectification
from vision_function.debug_ui.core import (
    DEFAULT_ROI,
    DebugProcessor,
    draw_horizontal_reference_lines,
    draw_debug_overlay,
    normalize_tube_frame,
)
from vision_function.debug_bridge import DebugFramePublisher
from vision_function.debug_ui.app import FrameReader, ServiceDebugReader, SourceSpec
from vision_function.scale_location.marker_detection import (
    ColorMarkerDetector,
    MarkerDetectionConfig,
)
from vision_function.scale_location.types import BallObservation


ROOT = Path(__file__).resolve().parents[2]
SAMPLE = ROOT / "vision_function" / "scale_location" / "calibration_assets" / "color_ruler" / "tube-roi-2026-07-31T10-43-52-300Z.png"


class DebugUiTests(unittest.TestCase):
    def test_horizontal_lines_match_html_full_and_tube_coordinates(self) -> None:
        full = draw_horizontal_reference_lines(
            np.zeros((720, 1280, 3), dtype=np.uint8), tube_local=False
        )
        tube = draw_horizontal_reference_lines(
            np.zeros((100, 640, 3), dtype=np.uint8), tube_local=True
        )
        self.assertTupleEqual(tuple(full[307, 10]), (0, 0, 255))
        self.assertTupleEqual(tuple(full[422, 10]), (0, 0, 255))
        self.assertTupleEqual(tuple(tube[28, 10]), (0, 0, 255))
        self.assertTupleEqual(tuple(tube[85, 10]), (0, 0, 255))

    def test_service_bridge_reuses_uart_runtime_result(self) -> None:
        source = np.zeros((720, 1280, 3), dtype=np.uint8)
        roi = np.zeros((100, 640, 3), dtype=np.uint8)
        ok, source_jpeg = cv2.imencode(".jpg", source)
        self.assertTrue(ok)
        payload = {
            "observation": {
                "frame_sequence": 42,
                "capture_timestamp_s": 10.0,
                "found": False,
                "center_local_xy": None,
                "confidence": None,
                "box_local_xyxy": None,
                "candidate_count": 0,
            },
            "state": {
                "valid": False,
                "position_at_capture_cm": 0.0,
                "velocity_at_capture_cm_s": 0.0,
            },
            "uart": {"age_ms": 25, "flags": 0},
            "processing_ms": 26.0,
            "color_markers": {
                "geometry_valid": True,
                "reason": "ok",
                "processing_ms": 4.0,
                "geometry": {"runs": [], "spacing_cv": 0.02},
            },
        }
        with TemporaryDirectory() as directory:
            DebugFramePublisher(directory).publish(
                source_jpeg.tobytes(), roi, payload
            )
            displayed_source, result, _timestamp = ServiceDebugReader(
                directory
            ).read(threading.Event())

        self.assertEqual(displayed_source.shape, (360, 640, 3))
        self.assertEqual(result.annotated_frame.shape, (100, 640, 3))
        self.assertTrue(result.markers.valid)
        self.assertEqual(result.total_ms, 26.0)
        self.assertEqual(result.uart["age_ms"], 25)

    def test_service_reader_stops_without_waiting_for_metadata(self) -> None:
        stopped = threading.Event()
        stopped.set()
        with TemporaryDirectory() as directory:
            with self.assertRaises(InterruptedError):
                ServiceDebugReader(directory).read(stopped)

    def test_snapshot_reader_reuses_connection_and_half_decodes(self) -> None:
        source = np.zeros((720, 1280, 3), dtype=np.uint8)
        ok, jpeg = cv2.imencode(".jpg", source)
        self.assertTrue(ok)

        class Response:
            status = 200

            def __init__(self, timestamp: int) -> None:
                self.timestamp = timestamp

            def read(self) -> bytes:
                return jpeg.tobytes()

            def getheader(self, name: str):
                return str(self.timestamp) if name == "X-Timestamp" else None

        class Connection:
            def __init__(self, *_args, **_kwargs) -> None:
                self.requests = 0

            def request(self, _method: str, _path: str) -> None:
                self.requests += 1

            def getresponse(self) -> Response:
                return Response(self.requests)

            def close(self) -> None:
                pass

        connection = Connection()
        with patch(
            "vision_function.debug_ui.app.http.client.HTTPConnection",
            return_value=connection,
        ) as constructor:
            reader = FrameReader(SourceSpec("snapshot", "http://127.0.0.1:8080/snapshot"))
            first, first_timestamp = reader.read()
            second, second_timestamp = reader.read()
            reader.close()

        constructor.assert_called_once()
        self.assertEqual(connection.requests, 2)
        self.assertEqual(first.shape, (360, 640, 3))
        self.assertEqual(second.shape, (360, 640, 3))
        self.assertEqual((first_timestamp, second_timestamp), (1.0, 2.0))

    def test_normalize_accepts_saved_reference_roi(self) -> None:
        frame = np.zeros((200, 1280, 3), dtype=np.uint8)
        preprocessor = TubePreprocessor(load_tube_rectification(DEFAULT_ROI))
        output = normalize_tube_frame(frame, preprocessor)
        self.assertEqual(output.shape, (100, 640, 3))

    def test_headless_processor_draws_marker_overlay_without_ball_model(self) -> None:
        frame = cv2.imread(str(SAMPLE), cv2.IMREAD_COLOR)
        self.assertIsNotNone(frame)
        result = DebugProcessor(enable_ball=False).process(
            frame, sequence=7, capture_timestamp_s=10.0
        )
        self.assertEqual(result.annotated_frame.shape, (100, 640, 3))
        self.assertEqual(result.sequence, 7)
        self.assertIsNone(result.ball)
        self.assertGreater(len(result.markers.runs), 0)
        self.assertGreaterEqual(result.marker_ms, 0.0)

    def test_ball_overlay_marks_detected_center(self) -> None:
        frame = cv2.resize(cv2.imread(str(SAMPLE)), (640, 100))
        config_path = ROOT / "vision_function" / "scale_location" / "config" / "color_markers.json"
        config = MarkerDetectionConfig.load(config_path)
        markers = ColorMarkerDetector(config).detect(
            frame, frame_sequence=1, capture_timestamp_s=1.0
        )
        ball = BallObservation(
            frame_sequence=1,
            capture_timestamp_s=1.0,
            found=True,
            center_local_xy=(320.0, 50.0),
            confidence=0.9,
            box_local_xyxy=(305.0, 35.0, 335.0, 65.0),
            candidate_count=1,
        )
        overlay = draw_debug_overlay(frame, ball, markers, config)
        self.assertFalse(np.array_equal(overlay[50, 320], frame[50, 320]))


if __name__ == "__main__":
    unittest.main()
