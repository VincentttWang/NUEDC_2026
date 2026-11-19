"""Optional runtime image capture for missed and low-confidence detections."""

from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np

from .types import BallObservation


class DiagnosticCapture:
    """Save selected tube crops and a machine-readable manifest."""

    def __init__(self, output: Path, confidence_below: float, max_images: int) -> None:
        if not 0.0 <= confidence_below <= 1.0:
            raise ValueError("diagnostic confidence must be in [0, 1]")
        if max_images < 1:
            raise ValueError("diagnostic max images must be positive")
        self.output = output
        self.confidence_below = confidence_below
        self.max_images = max_images
        self.eligible_frames = 0
        self.rows: list[dict[str, object]] = []
        output.mkdir(parents=True, exist_ok=True)

    def capture(self, observation: BallObservation, crop: np.ndarray) -> None:
        confidence = observation.confidence
        eligible = not observation.found or (
            confidence is not None and confidence < self.confidence_below
        )
        if not eligible:
            return
        self.eligible_frames += 1
        if len(self.rows) >= self.max_images:
            return
        label = "miss" if not observation.found else f"conf_{confidence:.3f}"
        filename = f"frame_{observation.frame_sequence:06d}_{label}.jpg"
        path = self.output / filename
        if not cv2.imwrite(str(path), crop, [cv2.IMWRITE_JPEG_QUALITY, 92]):
            raise RuntimeError(f"failed to save diagnostic crop: {path}")
        self.rows.append(
            {
                "frame_sequence": observation.frame_sequence,
                "capture_timestamp_s": observation.capture_timestamp_s,
                "found": observation.found,
                "confidence": confidence,
                "center_local_xy": (
                    list(observation.center_local_xy)
                    if observation.center_local_xy is not None
                    else None
                ),
                "image": filename,
            }
        )

    def as_dict(self) -> dict[str, object]:
        return {
            "directory": str(self.output),
            "confidence_below": self.confidence_below,
            "max_images": self.max_images,
            "eligible_frames": self.eligible_frames,
            "saved_images": len(self.rows),
        }

    def close(self) -> None:
        payload = {**self.as_dict(), "frames": self.rows}
        (self.output / "manifest.json").write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
