from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from vision_function.scale_location.diagnostics import DiagnosticCapture
from vision_function.scale_location.types import BallObservation


def observation(sequence: int, found: bool, confidence: float | None) -> BallObservation:
    center = (320.0, 50.0) if found else None
    box = (312.0, 42.0, 328.0, 58.0) if found else None
    return BallObservation(
        sequence,
        float(sequence),
        found,
        center,
        confidence,
        box,
        1 if found else 0,
    )


class DiagnosticCaptureTests(unittest.TestCase):
    def test_saves_only_misses_and_low_confidence_up_to_limit(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            capture = DiagnosticCapture(output, confidence_below=0.5, max_images=2)
            crop = np.zeros((100, 640, 3), dtype=np.uint8)

            capture.capture(observation(0, True, 0.8), crop)
            capture.capture(observation(1, True, 0.4), crop)
            capture.capture(observation(2, False, None), crop)
            capture.capture(observation(3, False, None), crop)
            capture.close()

            payload = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
            self.assertEqual(payload["eligible_frames"], 3)
            self.assertEqual(payload["saved_images"], 2)
            self.assertEqual(len(payload["frames"]), 2)
            self.assertEqual(len(list(output.glob("*.jpg"))), 2)
            self.assertIn("conf_0.400", payload["frames"][0]["image"])
            self.assertIn("miss", payload["frames"][1]["image"])

    def test_rejects_invalid_limits(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            with self.assertRaisesRegex(ValueError, "confidence"):
                DiagnosticCapture(output, confidence_below=1.1, max_images=1)
            with self.assertRaisesRegex(ValueError, "max images"):
                DiagnosticCapture(output, confidence_below=0.5, max_images=0)


if __name__ == "__main__":
    unittest.main()
