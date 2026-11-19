"""PyTorch detector adapter compatible with the current BallObservation API."""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
import torch

from vision_function.ball_detection.preprocess import (
    TubePreprocessor,
    load_tube_rectification,
)
from vision_function.scale_location.types import BallObservation

from .decode import decode_predictions
from .evaluate import load_checkpoint


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_WEIGHTS = ROOT / "model" / "shallow_cnn_v1.pt"
DEFAULT_ROI = ROOT / "config" / "tube_local_rectification.json"


class ShallowCNNBallDetector:
    def __init__(
        self,
        weights_path: Path = DEFAULT_WEIGHTS,
        roi_path: Path = DEFAULT_ROI,
        *,
        device: str = "cpu",
        presence_threshold: float = 0.5,
        heatmap_threshold: float = 0.25,
        calibrated_axis_y: float = 75.0,
        nominal_ball_diameter_px: float = 32.0,
    ) -> None:
        self.device = torch.device(device)
        self.model, _checkpoint = load_checkpoint(Path(weights_path), self.device)
        self.model.eval()
        self.preprocessor = TubePreprocessor(load_tube_rectification(roi_path))
        self.presence_threshold = presence_threshold
        self.heatmap_threshold = heatmap_threshold
        self.calibrated_axis_y = calibrated_axis_y
        self.nominal_ball_diameter_px = nominal_ball_diameter_px

    @staticmethod
    def roi_to_tensor(crop: np.ndarray) -> torch.Tensor:
        if crop.shape != (100, 640, 3):
            raise ValueError(f"Expected a 640x100 BGR ROI, got {crop.shape}")
        rgb = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)
        tensor = torch.from_numpy(np.ascontiguousarray(rgb.transpose(2, 0, 1)))
        return tensor.to(torch.float32).div_(255.0).unsqueeze(0)

    @torch.inference_mode()
    def detect(
        self,
        frame: np.ndarray,
        *,
        capture_timestamp_s: float,
        frame_sequence: int,
    ) -> tuple[BallObservation, np.ndarray]:
        crop = self.preprocessor(frame)
        outputs = self.model(self.roi_to_tensor(crop).to(self.device))
        prediction = decode_predictions(
            *outputs,
            presence_threshold=self.presence_threshold,
            heatmap_threshold=self.heatmap_threshold,
        )[0]
        if not prediction.found or prediction.x_local is None:
            return BallObservation(
                frame_sequence=frame_sequence,
                capture_timestamp_s=capture_timestamp_s,
                found=False,
                center_local_xy=None,
                confidence=None,
                candidate_count=0,
            ), crop

        x = prediction.x_local
        y = self.calibrated_axis_y
        radius = self.nominal_ball_diameter_px / 2.0
        return BallObservation(
            frame_sequence=frame_sequence,
            capture_timestamp_s=capture_timestamp_s,
            found=True,
            center_local_xy=(x, y),
            confidence=prediction.confidence,
            box_local_xyxy=(x - radius, y - radius, x + radius, y + radius),
            candidate_count=1,
        ), crop
