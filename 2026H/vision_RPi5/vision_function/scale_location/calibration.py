"""Load a scale calibration and map a tube-local ball center to centimetres."""

from __future__ import annotations

from bisect import bisect_right
from dataclasses import dataclass
import json
import math
from pathlib import Path


@dataclass(frozen=True)
class ScaleAnchor:
    axis_px: float
    scale_cm: float


@dataclass(frozen=True)
class ScaleMeasurement:
    """One point mapped onto the calibrated one-dimensional tube axis."""

    valid: bool
    position_cm: float | None
    axis_px: float
    edge: bool


@dataclass(frozen=True)
class ScaleCalibration:
    schema_version: int
    calibration_type: str
    status: str
    coordinate_space: str
    image_size: tuple[int, int]
    axis_origin_xy: tuple[float, float]
    axis_direction_xy: tuple[float, float]
    anchors: tuple[ScaleAnchor, ...]
    valid_range_cm: tuple[float, float]
    edge_margin_cm: float

    def project(self, center_xy: tuple[float, float]) -> float:
        """Project an image point onto the calibrated tube axis."""
        x, y = _finite_pair(center_xy, "center_xy")
        origin_x, origin_y = self.axis_origin_xy
        direction_x, direction_y = self.axis_direction_xy
        return (x - origin_x) * direction_x + (y - origin_y) * direction_y

    def measure(self, center_xy: tuple[float, float]) -> ScaleMeasurement:
        """Map a point, clamping to anchor range and valid_range_cm instead of rejecting."""
        axis_px = self.project(center_xy)
        axis_values = tuple(anchor.axis_px for anchor in self.anchors)

        # Clamp axis_px to the anchor range instead of discarding the measurement.
        clamped_axis_px = max(axis_values[0], min(axis_values[-1], axis_px))

        if clamped_axis_px == axis_values[-1]:
            position_cm = self.anchors[-1].scale_cm
        else:
            right = bisect_right(axis_values, clamped_axis_px)
            left_anchor = self.anchors[right - 1]
            right_anchor = self.anchors[right]
            ratio = (clamped_axis_px - left_anchor.axis_px) / (
                right_anchor.axis_px - left_anchor.axis_px
            )
            position_cm = left_anchor.scale_cm + ratio * (
                right_anchor.scale_cm - left_anchor.scale_cm
            )

        valid_min, valid_max = self.valid_range_cm
        clamped_cm = max(valid_min, min(valid_max, position_cm))
        edge = (
            axis_px != clamped_axis_px
            or clamped_cm <= valid_min + self.edge_margin_cm
            or clamped_cm >= valid_max - self.edge_margin_cm
        )
        return ScaleMeasurement(True, clamped_cm, axis_px, edge)


def _finite_pair(payload: object, name: str) -> tuple[float, float]:
    if not isinstance(payload, (list, tuple)) or len(payload) != 2:
        raise ValueError(f"{name} must contain exactly two numbers")
    pair = float(payload[0]), float(payload[1])
    if not all(math.isfinite(value) for value in pair):
        raise ValueError(f"{name} must contain finite numbers")
    return pair


def load_scale_calibration(path: Path | str) -> ScaleCalibration:
    """Load and validate a temporary or final piecewise-linear calibration."""
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    schema_version = int(payload["schema_version"])
    if schema_version != 1:
        raise ValueError(f"Unsupported scale calibration schema {schema_version}")

    image_size = tuple(int(value) for value in payload["image_size"])
    if len(image_size) != 2 or any(value <= 0 for value in image_size):
        raise ValueError("image_size must contain two positive integers")

    origin = _finite_pair(payload["axis_origin_xy"], "axis_origin_xy")
    direction = _finite_pair(payload["axis_direction_xy"], "axis_direction_xy")
    norm = math.hypot(*direction)
    if norm <= 1e-12:
        raise ValueError("axis_direction_xy must not be zero")
    direction = direction[0] / norm, direction[1] / norm

    anchors = tuple(
        ScaleAnchor(float(item["axis_px"]), float(item["scale_cm"]))
        for item in payload["anchors"]
    )
    if len(anchors) < 2:
        raise ValueError("At least two scale anchors are required")
    if not all(
        math.isfinite(anchor.axis_px) and math.isfinite(anchor.scale_cm)
        for anchor in anchors
    ):
        raise ValueError("Scale anchors must be finite")
    if any(
        current.axis_px >= following.axis_px
        or current.scale_cm >= following.scale_cm
        for current, following in zip(anchors, anchors[1:])
    ):
        raise ValueError("Scale anchors must be strictly increasing")

    valid_range = _finite_pair(payload["valid_range_cm"], "valid_range_cm")
    if valid_range[0] >= valid_range[1]:
        raise ValueError("valid_range_cm must be increasing")
    if (
        valid_range[0] < anchors[0].scale_cm
        or valid_range[1] > anchors[-1].scale_cm
    ):
        raise ValueError("valid_range_cm must be covered by the anchors")

    edge_margin = float(payload.get("edge_margin_cm", 0.0))
    if not math.isfinite(edge_margin) or edge_margin < 0:
        raise ValueError("edge_margin_cm must be finite and non-negative")

    return ScaleCalibration(
        schema_version=schema_version,
        calibration_type=str(payload["calibration_type"]),
        status=str(payload["status"]),
        coordinate_space=str(payload["coordinate_space"]),
        image_size=(image_size[0], image_size[1]),
        axis_origin_xy=origin,
        axis_direction_xy=direction,
        anchors=anchors,
        valid_range_cm=valid_range,
        edge_margin_cm=edge_margin,
    )
