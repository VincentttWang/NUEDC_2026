"""Headless processing and drawing helpers shared by the Tk desktop UI."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from time import perf_counter

import cv2
import numpy as np

from vision_function.ball_detection.preprocess import (
    TubePreprocessor,
    load_tube_rectification,
)
from vision_function.scale_location.marker_detection import (
    ColorMarkerDetector,
    MarkerDetectionConfig,
    MarkerGeometry,
)
from vision_function.scale_location.types import BallObservation


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ROI = ROOT / "ball_detection" / "config" / "tube_local_rectification.json"
DEFAULT_MARKERS = ROOT / "scale_location" / "config" / "color_markers.json"
DEFAULT_MODEL = (
    ROOT
    / "ball_detection"
    / "model"
    / "actual_yolo11n_local_v3_ncnn_model"
)

# Keep these coordinates identical to deploy/www/index.html.  They are defined
# in the 1280x720 rectified output plane, while the production tube ROI begins
# at y=252 and is 200 pixels high in that same reference plane.
REFERENCE_HEIGHT_PX = 720
HORIZONTAL_REFERENCE_Y_PX = (307, 422)
TUBE_REFERENCE_TOP_PX = 252
TUBE_REFERENCE_HEIGHT_PX = 200
REFERENCE_LINE_BGR = (0, 0, 255)


@dataclass(frozen=True)
class DebugFrameResult:
    sequence: int
    tube_frame: np.ndarray
    annotated_frame: np.ndarray
    ball: BallObservation | None
    markers: MarkerGeometry
    ball_ms: float
    marker_ms: float
    total_ms: float
    ball_error: str | None = None


def normalize_tube_frame(frame: np.ndarray, preprocessor: TubePreprocessor) -> np.ndarray:
    """Accept raw camera frames as well as saved 640x100/1280x200 ROI images."""
    if frame is None or frame.size == 0:
        raise ValueError("输入画面为空")
    height, width = frame.shape[:2]
    if (width, height) == (640, 100):
        return frame.copy()
    if (width, height) == (1280, 200):
        return cv2.resize(frame, (640, 100), interpolation=cv2.INTER_AREA)
    return preprocessor(frame)


def draw_horizontal_reference_lines(
    frame: np.ndarray, *, tube_local: bool
) -> np.ndarray:
    """Draw the two fixed HTML reference lines in full-frame or tube coordinates."""
    if frame is None or frame.size == 0:
        raise ValueError("参考线输入画面为空")
    output = frame.copy()
    height, width = output.shape[:2]
    for reference_y in HORIZONTAL_REFERENCE_Y_PX:
        if tube_local:
            scaled_y = (
                (reference_y - TUBE_REFERENCE_TOP_PX)
                * height
                / TUBE_REFERENCE_HEIGHT_PX
            )
        else:
            scaled_y = reference_y * height / REFERENCE_HEIGHT_PX
        y = int(round(scaled_y))
        if 0 <= y < height:
            cv2.line(
                output,
                (0, y),
                (width - 1, y),
                REFERENCE_LINE_BGR,
                3,
                cv2.LINE_8,
            )
    return output


class DebugProcessor:
    """Run production preprocessing, ball detection and colour-marker detection."""

    def __init__(
        self,
        *,
        roi_path: Path = DEFAULT_ROI,
        marker_path: Path = DEFAULT_MARKERS,
        model_path: Path = DEFAULT_MODEL,
        enable_ball: bool = True,
        opencv_threads: int = 1,
        ncnn_threads: int = 2,
    ) -> None:
        if opencv_threads < 1 or ncnn_threads < 1:
            raise ValueError("线程数必须为正数")
        cv2.setNumThreads(opencv_threads)
        self.preprocessor = TubePreprocessor(load_tube_rectification(roi_path))
        self.marker_config = MarkerDetectionConfig.load(marker_path)
        self.marker_detector = ColorMarkerDetector(self.marker_config)
        self.model_path = Path(model_path)
        self.enable_ball = enable_ball
        self.ncnn_threads = ncnn_threads
        self._ball_detector = None
        self._ball_error: str | None = None
        self._ncnn_configured = False

    @property
    def ball_error(self) -> str | None:
        return self._ball_error

    def _load_ball_detector(self):
        if not self.enable_ball or self._ball_error is not None:
            return None
        if self._ball_detector is None:
            try:
                from vision_function.ball_detection.detect_ball import BallDetector

                self._ball_detector = BallDetector(self.model_path)
            except Exception as exc:  # optional backend must not take down the UI
                self._ball_error = f"{type(exc).__name__}: {exc}"
        return self._ball_detector

    def process(
        self,
        frame: np.ndarray,
        *,
        sequence: int,
        capture_timestamp_s: float,
    ) -> DebugFrameResult:
        started = perf_counter()
        tube = normalize_tube_frame(frame, self.preprocessor)

        ball_started = perf_counter()
        ball: BallObservation | None = None
        detector = self._load_ball_detector()
        if detector is not None:
            try:
                ball = detector.detect_crop(
                    tube,
                    capture_timestamp_s=capture_timestamp_s,
                    frame_sequence=sequence,
                )
                # NCNN exposes its network options only after the first
                # prediction.  Match the production runtime's validated
                # two-thread configuration from then on.
                if not self._ncnn_configured:
                    detector.configure_ncnn_threads(self.ncnn_threads)
                    self._ncnn_configured = True
            except Exception as exc:
                self._ball_error = f"{type(exc).__name__}: {exc}"
        ball_ms = (perf_counter() - ball_started) * 1000.0

        marker_started = perf_counter()
        markers = self.marker_detector.detect(
            tube,
            frame_sequence=sequence,
            capture_timestamp_s=capture_timestamp_s,
        )
        marker_ms = (perf_counter() - marker_started) * 1000.0
        annotated = draw_debug_overlay(tube, ball, markers, self.marker_config)
        return DebugFrameResult(
            sequence=sequence,
            tube_frame=tube,
            annotated_frame=annotated,
            ball=ball,
            markers=markers,
            ball_ms=ball_ms,
            marker_ms=marker_ms,
            total_ms=(perf_counter() - started) * 1000.0,
            ball_error=self._ball_error,
        )


def draw_debug_overlay(
    tube: np.ndarray,
    ball: BallObservation | None,
    markers: MarkerGeometry,
    config: MarkerDetectionConfig,
) -> np.ndarray:
    """Draw stable, high-contrast overlays in tube-local coordinates."""
    output = draw_horizontal_reference_lines(tube, tube_local=True)
    palette = {
        "yellow": (30, 225, 240),
        "green": (80, 220, 110),
        "magenta": (220, 80, 220),
    }
    x, y, width, height = config.roi_xywh
    cv2.rectangle(output, (x, y), (x + width, y + height), (170, 170, 170), 1)

    for run in markers.runs:
        color = palette.get(run.color, (255, 255, 255))
        cx, cy = (int(round(value)) for value in run.center_xy)
        cv2.line(output, (cx, y), (cx, y + height), color, 2)
        cv2.circle(output, (cx, cy), 3, color, -1, cv2.LINE_AA)

    if markers.valid and markers.axis_origin_xy is not None:
        origin = tuple(int(round(value)) for value in markers.axis_origin_xy)
        cv2.drawMarker(output, origin, (255, 255, 255), cv2.MARKER_CROSS, 14, 2)

    if ball is not None and ball.found:
        assert ball.center_local_xy is not None
        center = tuple(int(round(value)) for value in ball.center_local_xy)
        if ball.box_local_xyxy is not None:
            left, top, right, bottom = (
                int(round(value)) for value in ball.box_local_xyxy
            )
            cv2.rectangle(output, (left, top), (right, bottom), (60, 210, 255), 2)
        cv2.drawMarker(output, center, (60, 210, 255), cv2.MARKER_CROSS, 18, 2)
    return output
