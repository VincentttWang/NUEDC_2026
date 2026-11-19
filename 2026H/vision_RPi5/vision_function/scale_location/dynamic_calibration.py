"""Schedule colour-marker detection and retain the latest valid geometry."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from time import perf_counter
from typing import Any

import cv2
import numpy as np

from .calibration import ScaleCalibration
from .marker_detection import (
    ColorMarkerDetector,
    MarkerDetectionConfig,
    MarkerGeometry,
)
from .oriented_roi import OrientedMarkerRefiner, marker_axis_angle_degrees


@dataclass(frozen=True)
class MarkerTrackStatus:
    attempted: bool
    detection_valid: bool | None
    geometry_valid: bool
    geometry_age_ms: float | None
    reason: str
    processing_ms: float
    geometry: MarkerGeometry | None

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


class ColorScaleTracker:
    """Detect at a fixed cadence, retry failures and expire stale geometry."""

    def __init__(
        self,
        detector: ColorMarkerDetector,
        config: MarkerDetectionConfig,
    ) -> None:
        self.detector = detector
        self.config = config
        self.oriented_refiner = (
            OrientedMarkerRefiner(config) if config.oriented_roi_enabled else None
        )
        self.reset()

    def reset(self) -> None:
        self._geometry: MarkerGeometry | None = None
        self._retry = True
        self.attempts = 0
        self.accepted = 0
        self.rejected = 0
        self.total_processing_ms = 0.0
        self.oriented_attempts = 0
        self.oriented_accepted = 0
        self.oriented_fallbacks = 0

    def _orientation_seed(
        self, capture_timestamp_s: float
    ) -> MarkerGeometry | None:
        if self.oriented_refiner is None or self._geometry is None:
            return None
        age_ms = max(
            0.0,
            (float(capture_timestamp_s) - self._geometry.capture_timestamp_s) * 1000.0,
        )
        if age_ms > self.config.max_geometry_age_ms:
            return None
        angle = marker_axis_angle_degrees(self._geometry)
        return (
            self._geometry
            if abs(angle) >= self.config.oriented_angle_threshold_degrees
            else None
        )

    def _try_oriented(
        self,
        image: np.ndarray,
        seed: MarkerGeometry,
        *,
        frame_sequence: int,
        capture_timestamp_s: float,
    ) -> MarkerGeometry | None:
        assert self.oriented_refiner is not None
        self.oriented_attempts += 1
        try:
            result = self.oriented_refiner.refine(
                image,
                seed,
                frame_sequence=frame_sequence,
                capture_timestamp_s=capture_timestamp_s,
            )
        except (ValueError, cv2.error, np.linalg.LinAlgError):
            return None
        if result.valid:
            self.oriented_accepted += 1
            return result
        return None

    def _detect(
        self,
        image: np.ndarray,
        *,
        frame_sequence: int,
        capture_timestamp_s: float,
    ) -> MarkerGeometry:
        seed = self._orientation_seed(capture_timestamp_s)
        if seed is not None:
            refined = self._try_oriented(
                image,
                seed,
                frame_sequence=frame_sequence,
                capture_timestamp_s=capture_timestamp_s,
            )
            if refined is not None:
                return refined
            self.oriented_fallbacks += 1

        fixed = self.detector.detect(
            image,
            frame_sequence=frame_sequence,
            capture_timestamp_s=capture_timestamp_s,
        )
        if (
            seed is None
            and self.oriented_refiner is not None
            and fixed.valid
            and abs(marker_axis_angle_degrees(fixed))
            >= self.config.oriented_angle_threshold_degrees
        ):
            refined = self._try_oriented(
                image,
                fixed,
                frame_sequence=frame_sequence,
                capture_timestamp_s=capture_timestamp_s,
            )
            if refined is not None:
                return refined
            self.oriented_fallbacks += 1
        return fixed

    @property
    def geometry(self) -> MarkerGeometry | None:
        return self._geometry

    @property
    def calibration(self) -> ScaleCalibration | None:
        if self._geometry is None:
            return None
        return self._geometry.to_calibration(self.config)

    def update(
        self,
        image: np.ndarray,
        *,
        frame_sequence: int,
        capture_timestamp_s: float,
    ) -> MarkerTrackStatus:
        due = self._retry or frame_sequence % self.config.detect_every_n_frames == 0
        detection: MarkerGeometry | None = None
        processing_ms = 0.0
        if due:
            started = perf_counter()
            detection = self._detect(
                image,
                frame_sequence=frame_sequence,
                capture_timestamp_s=capture_timestamp_s,
            )
            processing_ms = (perf_counter() - started) * 1000.0
            self.attempts += 1
            self.total_processing_ms += processing_ms
            if detection.valid:
                self._geometry = detection
                self._retry = False
                self.accepted += 1
            else:
                self._retry = True
                self.rejected += 1

        age_ms = (
            None
            if self._geometry is None
            else max(
                0.0,
                (float(capture_timestamp_s) - self._geometry.capture_timestamp_s)
                * 1000.0,
            )
        )
        geometry_valid = bool(
            self._geometry is not None
            and age_ms is not None
            and age_ms <= self.config.max_geometry_age_ms
        )
        if detection is not None:
            reason = detection.reason
        elif self._geometry is None:
            reason = "waiting for first marker detection"
        elif not geometry_valid:
            reason = "marker geometry is stale"
        else:
            reason = "using retained marker geometry"
        return MarkerTrackStatus(
            attempted=due,
            detection_valid=None if detection is None else detection.valid,
            geometry_valid=geometry_valid,
            geometry_age_ms=age_ms,
            reason=reason,
            processing_ms=processing_ms,
            geometry=self._geometry if geometry_valid else None,
        )

    def stats(self) -> dict[str, int | float]:
        return {
            "attempts": self.attempts,
            "accepted": self.accepted,
            "rejected": self.rejected,
            "total_processing_ms": self.total_processing_ms,
            "mean_processing_ms": (
                self.total_processing_ms / self.attempts if self.attempts else 0.0
            ),
            "oriented_attempts": self.oriented_attempts,
            "oriented_accepted": self.oriented_accepted,
            "oriented_fallbacks": self.oriented_fallbacks,
        }
