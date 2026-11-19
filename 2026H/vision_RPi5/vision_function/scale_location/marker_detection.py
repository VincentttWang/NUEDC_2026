"""Detect a coloured scale pattern in the rectified tube-local image."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from .calibration import ScaleAnchor, ScaleCalibration


@dataclass(frozen=True)
class MarkerColorConfig:
    name: str
    design_rgb: tuple[int, int, int]
    max_distance: float


@dataclass(frozen=True)
class MarkerDetectionConfig:
    schema_version: int
    status: str
    coordinate_space: str
    image_size: tuple[int, int]
    roi_xywh: tuple[int, int, int, int]
    detect_every_n_frames: int
    max_geometry_age_ms: float
    lightness_weight: float
    minimum_column_fraction: float
    maximum_unknown_gap_px: int
    maximum_recoverable_narrow_runs: int
    minimum_run_width_px: int
    maximum_spacing_cv: float
    minimum_width_ratio: float
    maximum_width_ratio: float
    nominal_spacing_px: float
    maximum_zero_run_width_px: int
    zero_center_x_range_px: tuple[int, int]
    minimum_cm: int
    maximum_cm: int
    zero_color: str
    odd_color: str
    even_color: str
    edge_margin_cm: float
    oriented_roi_enabled: bool
    oriented_angle_threshold_degrees: float
    oriented_strip_height_px: int
    colors: tuple[MarkerColorConfig, ...]

    @classmethod
    def load(cls, path: Path | str) -> MarkerDetectionConfig:
        payload = json.loads(Path(path).read_text(encoding="utf-8"))
        colors = tuple(
            MarkerColorConfig(
                name=str(name),
                design_rgb=_rgb_triplet(item["design_rgb"], f"colors.{name}"),
                max_distance=float(item["max_distance"]),
            )
            for name, item in payload["colors"].items()
        )
        config = cls(
            schema_version=int(payload["schema_version"]),
            status=str(payload["status"]),
            coordinate_space=str(payload["coordinate_space"]),
            image_size=_positive_pair(payload["image_size"], "image_size"),
            roi_xywh=_roi(payload["roi_xywh"]),
            detect_every_n_frames=int(payload["detect_every_n_frames"]),
            max_geometry_age_ms=float(payload["max_geometry_age_ms"]),
            lightness_weight=float(payload["lightness_weight"]),
            minimum_column_fraction=float(payload["minimum_column_fraction"]),
            maximum_unknown_gap_px=int(payload["maximum_unknown_gap_px"]),
            maximum_recoverable_narrow_runs=int(
                payload.get("maximum_recoverable_narrow_runs", 0)
            ),
            minimum_run_width_px=int(payload["minimum_run_width_px"]),
            maximum_spacing_cv=float(payload["maximum_spacing_cv"]),
            minimum_width_ratio=float(payload["minimum_width_ratio"]),
            maximum_width_ratio=float(payload["maximum_width_ratio"]),
            nominal_spacing_px=float(payload["nominal_spacing_px"]),
            maximum_zero_run_width_px=int(payload["maximum_zero_run_width_px"]),
            zero_center_x_range_px=_integer_pair(
                payload["zero_center_x_range_px"], "zero_center_x_range_px"
            ),
            minimum_cm=int(payload["minimum_cm"]),
            maximum_cm=int(payload["maximum_cm"]),
            zero_color=str(payload["zero_color"]),
            odd_color=str(payload["odd_color"]),
            even_color=str(payload["even_color"]),
            edge_margin_cm=float(payload.get("edge_margin_cm", 0.0)),
            oriented_roi_enabled=bool(payload.get("oriented_roi_enabled", False)),
            oriented_angle_threshold_degrees=float(
                payload.get("oriented_angle_threshold_degrees", 1.0)
            ),
            oriented_strip_height_px=int(payload.get("oriented_strip_height_px", 30)),
            colors=colors,
        )
        config.validate()
        return config

    def validate(self) -> None:
        if self.schema_version != 1:
            raise ValueError(f"Unsupported marker schema {self.schema_version}")
        width, height = self.image_size
        x, y, roi_width, roi_height = self.roi_xywh
        if x + roi_width > width or y + roi_height > height:
            raise ValueError("roi_xywh must fit inside image_size")
        if self.detect_every_n_frames < 1:
            raise ValueError("detect_every_n_frames must be positive")
        if not math.isfinite(self.max_geometry_age_ms) or self.max_geometry_age_ms <= 0:
            raise ValueError("max_geometry_age_ms must be finite and positive")
        if not 0.0 < self.lightness_weight <= 1.0:
            raise ValueError("lightness_weight must be in (0, 1]")
        if not 0.0 < self.minimum_column_fraction <= 1.0:
            raise ValueError("minimum_column_fraction must be in (0, 1]")
        if self.maximum_unknown_gap_px < 0 or self.minimum_run_width_px < 1:
            raise ValueError("run filtering settings are invalid")
        if self.maximum_recoverable_narrow_runs < 0:
            raise ValueError("maximum_recoverable_narrow_runs cannot be negative")
        if not 0.0 < self.maximum_spacing_cv < 1.0:
            raise ValueError("maximum_spacing_cv must be in (0, 1)")
        if not 0.0 < self.minimum_width_ratio <= 1.0:
            raise ValueError("minimum_width_ratio must be in (0, 1]")
        if self.maximum_width_ratio < 1.0:
            raise ValueError("maximum_width_ratio must be at least 1")
        if not math.isfinite(self.nominal_spacing_px) or self.nominal_spacing_px <= 0:
            raise ValueError("nominal_spacing_px must be finite and positive")
        if self.maximum_zero_run_width_px < self.minimum_run_width_px:
            raise ValueError(
                "maximum_zero_run_width_px must be at least minimum_run_width_px"
            )
        zero_x_low, zero_x_high = self.zero_center_x_range_px
        if not 0 <= zero_x_low < zero_x_high <= width:
            raise ValueError("zero_center_x_range_px must be increasing within image_size")
        if self.minimum_cm >= 0 or self.maximum_cm <= 0:
            raise ValueError("marker range must straddle zero")
        if abs(self.minimum_cm) != self.maximum_cm:
            raise ValueError("the current marker pattern must be symmetric around zero")
        names = [color.name for color in self.colors]
        if len(names) != len(set(names)):
            raise ValueError("marker color names must be unique")
        required = {self.zero_color, self.odd_color, self.even_color}
        if not required.issubset(names):
            raise ValueError(f"Missing marker colors: {sorted(required - set(names))}")
        if len(required) != 3:
            raise ValueError("zero, odd and even markers must use distinct colors")
        if any(
            not math.isfinite(color.max_distance) or color.max_distance <= 0
            for color in self.colors
        ):
            raise ValueError("color max_distance values must be finite and positive")
        if not math.isfinite(self.edge_margin_cm) or self.edge_margin_cm < 0:
            raise ValueError("edge_margin_cm must be finite and non-negative")
        if (
            not math.isfinite(self.oriented_angle_threshold_degrees)
            or not 0.0 < self.oriented_angle_threshold_degrees < 45.0
        ):
            raise ValueError("oriented_angle_threshold_degrees must be in (0, 45)")
        if not 3 <= self.oriented_strip_height_px <= height:
            raise ValueError("oriented_strip_height_px must fit inside image height")

    def expected_color(self, scale_cm: int) -> str:
        if scale_cm == 0:
            return self.zero_color
        return self.odd_color if abs(scale_cm) % 2 else self.even_color


@dataclass(frozen=True)
class MarkerRun:
    color: str
    start_px: int
    end_px: int
    center_xy: tuple[float, float]

    @property
    def width_px(self) -> int:
        return self.end_px - self.start_px


@dataclass(frozen=True)
class MarkerGeometry:
    valid: bool
    reason: str
    frame_sequence: int
    capture_timestamp_s: float
    axis_origin_xy: tuple[float, float] | None
    axis_direction_xy: tuple[float, float] | None
    anchors: tuple[ScaleAnchor, ...]
    runs: tuple[MarkerRun, ...]
    spacing_mean_px: float | None
    spacing_cv: float | None

    def as_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        if self.axis_origin_xy is not None:
            payload["axis_origin_xy"] = list(self.axis_origin_xy)
        if self.axis_direction_xy is not None:
            payload["axis_direction_xy"] = list(self.axis_direction_xy)
        return payload

    def to_calibration(self, config: MarkerDetectionConfig) -> ScaleCalibration:
        if not self.valid or self.axis_origin_xy is None or self.axis_direction_xy is None:
            raise ValueError("Cannot build a calibration from invalid marker geometry")
        anchor_low = self.anchors[0].scale_cm
        anchor_high = self.anchors[-1].scale_cm
        return ScaleCalibration(
            schema_version=1,
            calibration_type="runtime_color_markers",
            status=config.status,
            coordinate_space=config.coordinate_space,
            image_size=config.image_size,
            axis_origin_xy=self.axis_origin_xy,
            axis_direction_xy=self.axis_direction_xy,
            anchors=self.anchors,
            valid_range_cm=(anchor_low, anchor_high),
            edge_margin_cm=config.edge_margin_cm,
        )


class ColorMarkerDetector:
    """Classify a thin Lab ROI and reconstruct the ordered centimetre bands."""

    UNKNOWN = -1

    def __init__(self, config: MarkerDetectionConfig) -> None:
        self.config = config
        self._names = tuple(color.name for color in config.colors)
        self._centers = np.asarray(
            [_rgb_to_lab(color.design_rgb) for color in config.colors],
            dtype=np.float32,
        )
        self._limits = np.asarray(
            [color.max_distance for color in config.colors], dtype=np.float32
        )

    def detect(
        self,
        image: np.ndarray,
        *,
        frame_sequence: int,
        capture_timestamp_s: float,
    ) -> MarkerGeometry:
        if image is None or image.size == 0:
            raise ValueError("marker input image is empty")
        if (image.shape[1], image.shape[0]) != self.config.image_size:
            raise ValueError(
                f"marker input must be {self.config.image_size}, got "
                f"{(image.shape[1], image.shape[0])}"
            )
        x, y, width, height = self.config.roi_xywh
        roi = image[y : y + height, x : x + width]
        labels = self._classify(roi)
        columns = self._column_labels(labels)
        columns = _fill_short_unknown_gaps(
            columns, self.config.maximum_unknown_gap_px, self.UNKNOWN
        )
        runs = self._runs(columns, labels, (x, y))
        return self._geometry(runs, frame_sequence, capture_timestamp_s)

    def _classify(self, roi: np.ndarray) -> np.ndarray:
        lab = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB).astype(np.float32)
        delta = lab[:, :, None, :] - self._centers[None, None, :, :]
        delta[:, :, :, 0] *= math.sqrt(self.config.lightness_weight)
        distances = np.sqrt(np.sum(delta * delta, axis=3))
        nearest = np.argmin(distances, axis=2)
        best = np.take_along_axis(distances, nearest[:, :, None], axis=2)[:, :, 0]
        accepted = best <= self._limits[nearest]
        return np.where(accepted, nearest, self.UNKNOWN).astype(np.int16)

    def _column_labels(self, labels: np.ndarray) -> np.ndarray:
        output = np.full(labels.shape[1], self.UNKNOWN, dtype=np.int16)
        best_votes = np.zeros(labels.shape[1], dtype=np.int32)
        minimum_votes = max(
            1, math.ceil(labels.shape[0] * self.config.minimum_column_fraction)
        )
        for index in range(len(self._names)):
            votes = np.count_nonzero(labels == index, axis=0)
            replace = (votes >= minimum_votes) & (votes > best_votes)
            output[replace] = index
            best_votes[replace] = votes[replace]
        return output

    def _runs(
        self,
        columns: np.ndarray,
        labels: np.ndarray,
        offset_xy: tuple[int, int],
    ) -> tuple[MarkerRun, ...]:
        runs: list[MarkerRun] = []
        start = 0
        for end in range(1, len(columns) + 1):
            if end < len(columns) and columns[end] == columns[start]:
                continue
            label = int(columns[start])
            if label != self.UNKNOWN and end - start >= self.config.minimum_run_width_px:
                ys, xs = np.nonzero(labels[:, start:end] == label)
                if len(xs):
                    center = (
                        offset_xy[0] + start + float(np.mean(xs)),
                        offset_xy[1] + float(np.mean(ys)),
                    )
                    runs.append(
                        MarkerRun(
                            color=self._names[label],
                            start_px=offset_xy[0] + start,
                            end_px=offset_xy[0] + end,
                            center_xy=center,
                        )
                    )
            start = end
        return tuple(runs)

    def _geometry(
        self,
        runs: tuple[MarkerRun, ...],
        frame_sequence: int,
        capture_timestamp_s: float,
    ) -> MarkerGeometry:
        zeros = [index for index, run in enumerate(runs) if run.color == self.config.zero_color]
        if len(zeros) != 1:
            return _invalid_geometry(
                f"expected one {self.config.zero_color} run, found {len(zeros)}",
                frame_sequence,
                capture_timestamp_s,
                runs,
            )
        radius = self.config.maximum_cm
        zero_index = zeros[0]
        zero_run = runs[zero_index]
        zero_x = zero_run.center_xy[0]
        zero_x_low, zero_x_high = self.config.zero_center_x_range_px
        if zero_run.width_px > self.config.maximum_zero_run_width_px:
            return _invalid_geometry(
                f"{self.config.zero_color} run is too wide",
                frame_sequence,
                capture_timestamp_s,
                runs,
            )
        if not zero_x_low <= zero_x <= zero_x_high:
            return _invalid_geometry(
                f"{self.config.zero_color} center is outside the configured window",
                frame_sequence,
                capture_timestamp_s,
                runs,
            )

        left_count = 0
        for offset in range(1, min(radius, zero_index) + 1):
            if runs[zero_index - offset].color != self.config.expected_color(-offset):
                break
            left_count = offset
        right_count = 0
        for offset in range(1, min(radius, len(runs) - zero_index - 1) + 1):
            if runs[zero_index + offset].color != self.config.expected_color(offset):
                break
            right_count = offset
        if left_count < 1 or right_count < 1:
            return _invalid_geometry(
                "need at least one consecutive marker on both sides of zero",
                frame_sequence,
                capture_timestamp_s,
                runs,
            )

        selected = runs[
            zero_index - left_count : zero_index + right_count + 1
        ]
        values = tuple(range(-left_count, right_count + 1))

        trimmed_edges = 0
        while len(selected) >= 3:
            edge_widths = np.asarray(
                [run.width_px for run in selected], dtype=np.float64
            )
            edge_ratios = edge_widths / float(np.median(edge_widths))
            left_bad = not (
                self.config.minimum_width_ratio
                <= edge_ratios[0]
                <= self.config.maximum_width_ratio
            )
            right_bad = not (
                self.config.minimum_width_ratio
                <= edge_ratios[-1]
                <= self.config.maximum_width_ratio
            )
            if left_bad and left_count > 1:
                selected = selected[1:]
                values = values[1:]
                left_count -= 1
                trimmed_edges += 1
                continue
            if right_bad and right_count > 1:
                selected = selected[:-1]
                values = values[:-1]
                right_count -= 1
                trimmed_edges += 1
                continue
            break

        centers = np.asarray([run.center_xy for run in selected], dtype=np.float64)
        widths = np.asarray([run.width_px for run in selected], dtype=np.float64)
        median_width = float(np.median(widths))
        ratios = widths / median_width
        narrow = np.flatnonzero(ratios < self.config.minimum_width_ratio)
        wide = np.flatnonzero(ratios > self.config.maximum_width_ratio)
        zero_selected_index = left_count
        can_recover_narrow = bool(
            len(narrow) <= self.config.maximum_recoverable_narrow_runs
            and len(narrow) + len(wide) == 1
            and all(
                0 < index < len(selected) - 1
                and index != zero_selected_index
                and self.config.minimum_width_ratio
                <= ratios[index - 1]
                <= self.config.maximum_width_ratio
                and self.config.minimum_width_ratio
                <= ratios[index + 1]
                <= self.config.maximum_width_ratio
                for index in narrow
            )
        )
        # Width is a secondary quality signal.  A single wide internal run can
        # result from column quantisation or colour bleed while its centre is
        # still geometrically correct; the spacing gate below decides whether
        # it is safe.  Multiple width outliers remain ambiguous and are rejected.
        if len(narrow) + len(wide) > 1 or (len(narrow) and not can_recover_narrow):
            return _invalid_geometry(
                "marker run width is outside the configured ratio",
                frame_sequence,
                capture_timestamp_s,
                selected,
            )
        if len(narrow):
            selected_mutable = list(selected)
            for index in narrow:
                recovered_center = (centers[index - 1] + centers[index + 1]) / 2.0
                centers[index] = recovered_center
                run = selected[index]
                selected_mutable[index] = MarkerRun(
                    color=run.color,
                    start_px=run.start_px,
                    end_px=run.end_px,
                    center_xy=(float(recovered_center[0]), float(recovered_center[1])),
                )
            selected = tuple(selected_mutable)

        direction = centers[-1] - centers[0]
        norm = float(np.linalg.norm(direction))
        if norm <= 1e-9:
            return _invalid_geometry(
                "marker axis is degenerate",
                frame_sequence,
                capture_timestamp_s,
                selected,
            )
        direction /= norm
        if direction[0] < 0:
            direction = -direction
        zero_center = np.asarray(selected[left_count].center_xy, dtype=np.float64)
        axis_values = (centers - zero_center) @ direction
        spacing = np.diff(axis_values)
        if np.any(spacing <= 0):
            return _invalid_geometry(
                "marker centers are not strictly increasing",
                frame_sequence,
                capture_timestamp_s,
                selected,
            )
        spacing_mean = float(np.mean(spacing))
        spacing_cv = float(np.std(spacing) / spacing_mean)
        if spacing_cv > self.config.maximum_spacing_cv:
            return _invalid_geometry(
                f"marker spacing CV {spacing_cv:.3f} exceeds limit",
                frame_sequence,
                capture_timestamp_s,
                selected,
                spacing_mean,
                spacing_cv,
            )
        anchors = tuple(
            ScaleAnchor(float(axis_px), float(scale_cm))
            for axis_px, scale_cm in zip(axis_values, values)
        )
        return MarkerGeometry(
            valid=True,
            reason="; ".join(
                part
                for part in (
                    (
                        "ok"
                        if left_count == radius and right_count == radius
                        else (
                            f"partial visible scale "
                            f"{-left_count:+d}..{right_count:+d} cm"
                        )
                    ),
                    (
                        f"trimmed {trimmed_edges} anomalous edge marker run(s)"
                        if trimmed_edges
                        else ""
                    ),
                    (
                        f"recovered {len(narrow)} narrow marker run(s)"
                        if len(narrow)
                        else ""
                    ),
                    (
                        f"accepted {len(wide)} wide marker run(s) after spacing check"
                        if len(wide)
                        else ""
                    ),
                )
                if part
            ),
            frame_sequence=frame_sequence,
            capture_timestamp_s=float(capture_timestamp_s),
            axis_origin_xy=(float(zero_center[0]), float(zero_center[1])),
            axis_direction_xy=(float(direction[0]), float(direction[1])),
            anchors=anchors,
            runs=selected,
            spacing_mean_px=spacing_mean,
            spacing_cv=spacing_cv,
        )

def _invalid_geometry(
    reason: str,
    frame_sequence: int,
    capture_timestamp_s: float,
    runs: tuple[MarkerRun, ...],
    spacing_mean_px: float | None = None,
    spacing_cv: float | None = None,
) -> MarkerGeometry:
    return MarkerGeometry(
        valid=False,
        reason=reason,
        frame_sequence=frame_sequence,
        capture_timestamp_s=float(capture_timestamp_s),
        axis_origin_xy=None,
        axis_direction_xy=None,
        anchors=(),
        runs=runs,
        spacing_mean_px=spacing_mean_px,
        spacing_cv=spacing_cv,
    )


def _fill_short_unknown_gaps(
    values: np.ndarray, maximum_gap: int, unknown: int
) -> np.ndarray:
    output = values.copy()
    if maximum_gap == 0:
        return output
    index = 0
    while index < len(output):
        if output[index] != unknown:
            index += 1
            continue
        end = index + 1
        while end < len(output) and output[end] == unknown:
            end += 1
        if (
            end - index <= maximum_gap
            and index > 0
            and end < len(output)
            and output[index - 1] == output[end]
        ):
            output[index:end] = output[index - 1]
        index = end
    return output


def _rgb_to_lab(rgb: tuple[int, int, int]) -> tuple[float, float, float]:
    red, green, blue = rgb
    pixel = np.asarray([[[blue, green, red]]], dtype=np.uint8)
    lab = cv2.cvtColor(pixel, cv2.COLOR_BGR2LAB)[0, 0]
    return float(lab[0]), float(lab[1]), float(lab[2])


def _rgb_triplet(payload: object, name: str) -> tuple[int, int, int]:
    if not isinstance(payload, (list, tuple)) or len(payload) != 3:
        raise ValueError(f"{name}.design_rgb must contain three integers")
    values = tuple(int(value) for value in payload)
    if any(value < 0 or value > 255 for value in values):
        raise ValueError(f"{name}.design_rgb values must be in [0, 255]")
    return values  # type: ignore[return-value]


def _positive_pair(payload: object, name: str) -> tuple[int, int]:
    if not isinstance(payload, (list, tuple)) or len(payload) != 2:
        raise ValueError(f"{name} must contain two integers")
    values = int(payload[0]), int(payload[1])
    if any(value <= 0 for value in values):
        raise ValueError(f"{name} values must be positive")
    return values


def _integer_pair(payload: object, name: str) -> tuple[int, int]:
    if not isinstance(payload, (list, tuple)) or len(payload) != 2:
        raise ValueError(f"{name} must contain two integers")
    return int(payload[0]), int(payload[1])


def _roi(payload: object) -> tuple[int, int, int, int]:
    if not isinstance(payload, (list, tuple)) or len(payload) != 4:
        raise ValueError("roi_xywh must contain four integers")
    x, y, width, height = (int(value) for value in payload)
    if x < 0 or y < 0 or width <= 0 or height <= 0:
        raise ValueError("roi_xywh must contain a non-negative origin and positive size")
    return x, y, width, height
