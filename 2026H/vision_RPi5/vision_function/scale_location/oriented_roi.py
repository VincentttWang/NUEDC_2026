"""Extraction of a ruler-aligned colour-marker strip.

The production detector first obtains coarse marker centres from its fixed ROI.
This module robustly refits their common axis and remaps a thin, rotated strip so
the existing one-dimensional marker detector can run on a horizontal ruler.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import math

import cv2
import numpy as np

from .marker_detection import (
    ColorMarkerDetector,
    MarkerDetectionConfig,
    MarkerGeometry,
    MarkerRun,
)


@dataclass(frozen=True)
class OrientedStrip:
    """A horizontal strip and its affine transforms to/from the source image."""

    image: np.ndarray
    source_to_strip: np.ndarray
    strip_to_source: np.ndarray
    source_axis_origin_xy: tuple[float, float]
    source_axis_direction_xy: tuple[float, float]
    angle_degrees: float

    def points_to_source(self, points_xy: np.ndarray) -> np.ndarray:
        """Map one or more strip-space points back to source-image coordinates."""
        points = np.asarray(points_xy, dtype=np.float64).reshape(-1, 1, 2)
        return cv2.transform(points, self.strip_to_source).reshape(-1, 2)


class OrientedMarkerRefiner:
    """Detect markers in a strip guided by a previously validated geometry."""

    def __init__(self, config: MarkerDetectionConfig) -> None:
        height = config.oriented_strip_height_px
        x, _, width, _ = config.roi_xywh
        strip_config = replace(
            config,
            image_size=(config.image_size[0], height),
            roi_xywh=(x, 0, width, height),
            oriented_roi_enabled=False,
        )
        strip_config.validate()
        self.config = strip_config
        self.detector = ColorMarkerDetector(strip_config)

    def refine(
        self,
        image: np.ndarray,
        seed: MarkerGeometry,
        *,
        frame_sequence: int,
        capture_timestamp_s: float,
    ) -> MarkerGeometry:
        strip = extract_oriented_strip(
            image,
            seed,
            output_height=self.config.image_size[1],
            interpolation=cv2.INTER_NEAREST,
        )
        geometry = self.detector.detect(
            strip.image,
            frame_sequence=frame_sequence,
            capture_timestamp_s=capture_timestamp_s,
        )
        return geometry_to_source(geometry, strip)


def fit_marker_axis(geometry: MarkerGeometry) -> tuple[np.ndarray, np.ndarray]:
    """Fit the marker-centre axis with total least squares (PCA)."""
    if not geometry.valid or len(geometry.runs) < 3:
        raise ValueError("a valid geometry with at least three marker runs is required")
    centers = np.asarray([run.center_xy for run in geometry.runs], dtype=np.float64)
    center = np.mean(centers, axis=0)
    _, _, axes = np.linalg.svd(centers - center, full_matrices=False)
    direction = axes[0]
    if direction[0] < 0:
        direction = -direction
    norm = float(np.linalg.norm(direction))
    if norm <= 1e-9:
        raise ValueError("marker axis is degenerate")
    direction /= norm

    # Keep zero centimetres as the longitudinal origin.  Projecting it onto the
    # robust line prevents one damaged zero component from shifting the strip.
    zero = np.asarray(geometry.axis_origin_xy, dtype=np.float64)
    origin = center + direction * float((zero - center) @ direction)
    return origin, direction


def marker_axis_angle_degrees(geometry: MarkerGeometry) -> float:
    """Return the robust signed marker-axis angle in source-image coordinates."""
    if len(geometry.runs) >= 3:
        _, direction = fit_marker_axis(geometry)
    elif geometry.valid and geometry.axis_direction_xy is not None:
        direction = np.asarray(geometry.axis_direction_xy, dtype=np.float64)
        norm = float(np.linalg.norm(direction))
        if norm <= 1e-9:
            raise ValueError("marker axis is degenerate")
        direction /= norm
        if direction[0] < 0:
            direction = -direction
    else:
        raise ValueError("a valid marker axis is required")
    return math.degrees(math.atan2(float(direction[1]), float(direction[0])))


def geometry_to_source(
    geometry: MarkerGeometry,
    strip: OrientedStrip,
) -> MarkerGeometry:
    """Map refined strip geometry back into the source 640x100 coordinates."""
    if not geometry.valid:
        return replace(geometry, reason=f"oriented strip rejected: {geometry.reason}")
    assert geometry.axis_origin_xy is not None
    assert geometry.axis_direction_xy is not None
    source_origin = strip.points_to_source(np.asarray([geometry.axis_origin_xy]))[0]
    source_direction = (
        strip.strip_to_source[:, :2]
        @ np.asarray(geometry.axis_direction_xy, dtype=np.float64)
    )
    source_direction /= np.linalg.norm(source_direction)
    if source_direction[0] < 0:
        source_direction = -source_direction

    mapped_runs: list[MarkerRun] = []
    for run in geometry.runs:
        source_center = strip.points_to_source(np.asarray([run.center_xy]))[0]
        endpoints = strip.points_to_source(
            np.asarray(
                [
                    [float(run.start_px), run.center_xy[1]],
                    [float(run.end_px), run.center_xy[1]],
                ]
            )
        )
        mapped_runs.append(
            replace(
                run,
                start_px=int(round(float(np.min(endpoints[:, 0])))),
                end_px=int(round(float(np.max(endpoints[:, 0])))),
                center_xy=(float(source_center[0]), float(source_center[1])),
            )
        )
    return replace(
        geometry,
        reason=f"oriented strip: {geometry.reason}",
        axis_origin_xy=(float(source_origin[0]), float(source_origin[1])),
        axis_direction_xy=(float(source_direction[0]), float(source_direction[1])),
        runs=tuple(mapped_runs),
    )


def extract_oriented_strip(
    image: np.ndarray,
    geometry: MarkerGeometry,
    *,
    output_width: int | None = None,
    output_height: int = 30,
    interpolation: int = cv2.INTER_NEAREST,
) -> OrientedStrip:
    """Remap a rotated ruler strip into a horizontal image.

    Longitudinal strip coordinates retain the source x coordinate at the fitted
    zero marker.  This keeps the configured zero-centre window and marker spacing
    directly usable by a detector configured for the thinner output image.
    """
    if image is None or image.size == 0:
        raise ValueError("source image is empty")
    if output_height <= 0:
        raise ValueError("output_height must be positive")
    width = image.shape[1] if output_width is None else int(output_width)
    if width <= 0:
        raise ValueError("output_width must be positive")

    origin, direction = fit_marker_axis(geometry)
    dx, dy = float(direction[0]), float(direction[1])
    ox, oy = float(origin[0]), float(origin[1])
    target_y = (output_height - 1) / 2.0
    source_to_strip = np.asarray(
        [
            [dx, dy, ox - dx * ox - dy * oy],
            [-dy, dx, target_y + dy * ox - dx * oy],
        ],
        dtype=np.float64,
    )
    strip_to_source = cv2.invertAffineTransform(source_to_strip)
    strip = cv2.warpAffine(
        image,
        source_to_strip,
        (width, output_height),
        flags=interpolation,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=(0, 0, 0),
    )
    return OrientedStrip(
        image=strip,
        source_to_strip=source_to_strip,
        strip_to_source=strip_to_source,
        source_axis_origin_xy=(ox, oy),
        source_axis_direction_xy=(dx, dy),
        angle_degrees=math.degrees(math.atan2(dy, dx)),
    )
