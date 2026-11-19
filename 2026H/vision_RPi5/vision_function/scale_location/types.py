"""Typed data contracts between detection, scale estimation and UART output."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Any


@dataclass(frozen=True)
class BallObservation:
    """One detector result expressed in the 640x100 tube-local plane."""

    frame_sequence: int
    capture_timestamp_s: float
    found: bool
    center_local_xy: tuple[float, float] | None
    confidence: float | None
    box_local_xyxy: tuple[float, float, float, float] | None = None
    candidate_count: int = 0

    def __post_init__(self) -> None:
        if self.frame_sequence < 0:
            raise ValueError("frame_sequence must be non-negative")
        if not math.isfinite(self.capture_timestamp_s):
            raise ValueError("capture_timestamp_s must be finite")
        if self.candidate_count < 0:
            raise ValueError("candidate_count must be non-negative")
        if self.found:
            if self.center_local_xy is None or self.confidence is None:
                raise ValueError("A found observation requires center and confidence")
            if self.candidate_count < 1:
                raise ValueError("A found observation requires at least one candidate")
        elif any(
            value is not None
            for value in (self.center_local_xy, self.confidence, self.box_local_xyxy)
        ):
            raise ValueError("A missing observation cannot contain detection geometry")

    def as_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        for key in ("center_local_xy", "box_local_xyxy"):
            if payload[key] is not None:
                payload[key] = list(payload[key])
        return payload


@dataclass(frozen=True)
class ScaleState:
    """Physical state produced for one distinct camera frame."""

    frame_sequence: int
    valid: bool
    velocity_valid: bool
    moving: bool
    edge: bool
    quality: int
    target_cm: float
    position_at_capture_cm: float
    velocity_at_capture_cm_s: float
    acceleration_cm_s2: float
    position_now_cm: float
    velocity_now_cm_s: float
    displacement_cm: float
    control_error_cm: float
    capture_timestamp_s: float
    output_timestamp_s: float
    measurement_age_ms: float
    confidence: float | None
    innovation_cm: float | None

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)
