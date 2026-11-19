from __future__ import annotations

import unittest

import numpy as np

from vision_function.scale_location.calibration import load_scale_calibration
from vision_function.scale_location.estimator import MotionEstimator, MotionFilterConfig
from vision_function.scale_location.pipeline import ScalePipeline
from vision_function.scale_location.protocol import decode_frame
from vision_function.scale_location.runtime import (
    DEFAULT_CONFIDENCE,
    VisionUartRuntime,
    build_parser,
)
from vision_function.scale_location.types import BallObservation
from vision_function.scale_location.uart_transport import FrameLockedSender


CALIBRATION = (
    "vision_function/scale_location/config/scale_calibration_temporary_25cm.json"
)


class FakeSource:
    def __init__(self, rows: list[tuple[bytes, float | None]]) -> None:
        self.rows = iter(rows)

    def fetch(self) -> tuple[bytes, float | None]:
        return next(self.rows)

    def close(self) -> None:
        pass


class FakeDetector:
    def detect(self, frame, *, capture_timestamp_s, frame_sequence):
        x = 286.0 + frame_sequence * 5.0
        return (
            BallObservation(
                frame_sequence,
                capture_timestamp_s,
                True,
                (x, 50.0),
                0.8,
                (x - 8.0, 42.0, x + 8.0, 58.0),
                1,
            ),
            frame,
        )


class FakeWriter:
    def __init__(self) -> None:
        self.packets: list[bytes] = []

    def write(self, packet: bytes) -> int:
        self.packets.append(packet)
        return len(packet)


class ScaleRuntimeTests(unittest.TestCase):
    def test_default_confidence_matches_dynamic_validation(self) -> None:
        self.assertEqual(DEFAULT_CONFIDENCE, 0.10)
        self.assertEqual(build_parser().parse_args([]).conf, 0.10)

    def test_duplicate_frame_is_not_inferred_or_transmitted(self) -> None:
        source = FakeSource(
            [(b"a", 100.0), (b"duplicate", 100.0), (b"b", 100.03)]
        )
        writer = FakeWriter()
        calibration = load_scale_calibration(CALIBRATION)
        pipeline = ScalePipeline(
            calibration, MotionEstimator(MotionFilterConfig()), target_cm=0.0
        )
        sender = FrameLockedSender(writer, clock=lambda: 100.03)
        runtime = VisionUartRuntime(
            source,
            FakeDetector(),
            pipeline,
            sender,
            decoder=lambda _: np.zeros((360, 640, 3), dtype=np.uint8),
            clock=lambda: 100.03,
        )

        first = runtime.process_next()
        duplicate = runtime.process_next()
        second = runtime.process_next()

        self.assertIsNotNone(first)
        self.assertIsNone(duplicate)
        self.assertIsNotNone(second)
        self.assertEqual(runtime.stats.duplicate_frames, 1)
        self.assertEqual(runtime.stats.inferred_frames, 2)
        self.assertEqual(runtime.stats.tx_frames, 2)
        self.assertEqual(len(writer.packets), 2)
        self.assertEqual(decode_frame(writer.packets[0]).sequence, 0)
        self.assertEqual(decode_frame(writer.packets[1]).sequence, 1)
        self.assertEqual(decode_frame(writer.packets[0]).age_ms, 30)

    def test_missing_timestamp_is_neither_inferred_nor_sent(self) -> None:
        source = FakeSource([(b"a", None)])
        writer = FakeWriter()
        runtime = VisionUartRuntime(
            source,
            FakeDetector(),
            ScalePipeline(
                load_scale_calibration(CALIBRATION),
                MotionEstimator(MotionFilterConfig()),
            ),
            FrameLockedSender(writer, clock=lambda: 1.0),
            decoder=lambda _: np.zeros((360, 640, 3), dtype=np.uint8),
            clock=lambda: 1.0,
        )
        self.assertIsNone(runtime.process_next())
        self.assertEqual(runtime.stats.missing_timestamps, 1)
        self.assertEqual(writer.packets, [])


if __name__ == "__main__":
    unittest.main()
