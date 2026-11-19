"""Torch-free NCNN runtime for the shallow steel-ball localizer."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import sys

import cv2
import ncnn
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = ROOT.parents[1]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

DEFAULT_MODEL = ROOT / "model" / "shallow_cnn_v1_ncnn_model"
DEFAULT_ROI = ROOT / "config" / "tube_local_rectification.json"


@dataclass(frozen=True)
class NCNNPositionPrediction:
    found: bool
    x_local: float | None
    presence_probability: float
    heatmap_peak: float
    bin_index: int

    @property
    def confidence(self) -> float:
        return float((self.presence_probability * self.heatmap_peak) ** 0.5)


def sigmoid(value: np.ndarray | np.float32 | float) -> np.ndarray:
    clipped = np.clip(value, -80.0, 80.0)
    return 1.0 / (1.0 + np.exp(-clipped))


class ShallowCNNNCNN:
    """Run the exported 640x100 model without PyTorch or Ultralytics."""

    input_width = 640
    input_height = 100

    def __init__(
        self,
        model_dir: Path = DEFAULT_MODEL,
        *,
        threads: int = 2,
        use_fp16: bool = True,
    ) -> None:
        if threads < 1:
            raise ValueError("threads must be positive")
        model_dir = Path(model_dir)
        param_path = model_dir / "model.ncnn.param"
        bin_path = model_dir / "model.ncnn.bin"
        if not param_path.is_file() or not bin_path.is_file():
            raise FileNotFoundError(f"NCNN model files are missing below {model_dir}")

        self.model_dir = model_dir
        self.threads = threads
        self.use_fp16 = use_fp16
        self.net = ncnn.Net()
        self.net.opt.num_threads = threads
        self.net.opt.use_vulkan_compute = False
        self.net.opt.use_fp16_packed = use_fp16
        self.net.opt.use_fp16_storage = use_fp16
        self.net.opt.use_fp16_arithmetic = use_fp16
        if self.net.load_param(str(param_path)) != 0:
            raise RuntimeError(f"Cannot load NCNN parameters: {param_path}")
        if self.net.load_model(str(bin_path)) != 0:
            raise RuntimeError(f"Cannot load NCNN weights: {bin_path}")

    @staticmethod
    def prepare_roi(crop: np.ndarray) -> np.ndarray:
        """Convert a native-resolution BGR ROI to contiguous RGB CHW float32."""
        if crop.shape != (100, 640, 3):
            raise ValueError(f"Expected a 640x100 BGR ROI, got {crop.shape}")
        rgb = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)
        chw = np.ascontiguousarray(rgb.transpose(2, 0, 1), dtype=np.float32)
        chw *= 1.0 / 255.0
        return chw

    def infer_prepared(
        self, chw_rgb: np.ndarray
    ) -> tuple[np.float32, np.ndarray, np.ndarray]:
        if chw_rgb.shape != (3, self.input_height, self.input_width):
            raise ValueError(
                "Expected prepared input shape "
                f"(3, {self.input_height}, {self.input_width}), got {chw_rgb.shape}"
            )
        if chw_rgb.dtype != np.float32 or not chw_rgb.flags.c_contiguous:
            chw_rgb = np.ascontiguousarray(chw_rgb, dtype=np.float32)

        with self.net.create_extractor() as extractor:
            extractor.set_light_mode(True)
            if extractor.input("in0", ncnn.Mat(chw_rgb).clone()) != 0:
                raise RuntimeError("NCNN rejected input blob in0")
            status, presence = extractor.extract("out0")
            if status != 0:
                raise RuntimeError("NCNN failed to extract out0")
            status, heatmap = extractor.extract("out1")
            if status != 0:
                raise RuntimeError("NCNN failed to extract out1")
            status, offsets = extractor.extract("out2")
            if status != 0:
                raise RuntimeError("NCNN failed to extract out2")

        return (
            np.asarray(presence, dtype=np.float32).reshape(-1)[0],
            np.asarray(heatmap, dtype=np.float32).reshape(-1),
            np.asarray(offsets, dtype=np.float32).reshape(-1),
        )

    @staticmethod
    def decode(
        presence_logit: np.float32,
        heatmap_logits: np.ndarray,
        offset_logits: np.ndarray,
        *,
        presence_threshold: float = 0.5,
        heatmap_threshold: float = 0.25,
        input_width: int = 640,
    ) -> NCNNPositionPrediction:
        if heatmap_logits.ndim != 1 or offset_logits.shape != heatmap_logits.shape:
            raise ValueError("heatmap and offset outputs must be equally sized vectors")
        bins = heatmap_logits.size
        if bins < 2 or input_width % bins:
            raise ValueError("input_width must be exactly divisible by output bins")

        presence = float(sigmoid(presence_logit))
        heatmap = sigmoid(heatmap_logits)
        index = int(np.argmax(heatmap))
        peak = float(heatmap[index])
        offset = float(sigmoid(offset_logits[index]) - 0.5)
        bin_size = float(input_width) / bins
        x_local = (index + 0.5 + offset) * bin_size
        found = presence >= presence_threshold and peak >= heatmap_threshold
        return NCNNPositionPrediction(
            found=found,
            x_local=float(x_local) if found else None,
            presence_probability=presence,
            heatmap_peak=peak,
            bin_index=index,
        )

    def predict(
        self,
        crop: np.ndarray,
        *,
        presence_threshold: float = 0.5,
        heatmap_threshold: float = 0.25,
    ) -> NCNNPositionPrediction:
        outputs = self.infer_prepared(self.prepare_roi(crop))
        return self.decode(
            *outputs,
            presence_threshold=presence_threshold,
            heatmap_threshold=heatmap_threshold,
        )


class ShallowCNNBallDetectorNCNN:
    """NCNN-based shallow CNN detector implementing the Detector Protocol.

    This adapter wraps ``ShallowCNNNCNN`` so it can be used as a drop-in
    replacement for ``BallDetector`` (YOLO) in the live ``VisionUartRuntime``
    pipeline.  The shallow CNN only localises the ball along the horizontal
    tube axis; the vertical coordinate is held at a calibrated constant.
    """

    def __init__(
        self,
        model_dir: Path = DEFAULT_MODEL,
        roi_path: Path = DEFAULT_ROI,
        *,
        threads: int = 2,
        use_fp16: bool = True,
        presence_threshold: float = 0.5,
        heatmap_threshold: float = 0.25,
        calibrated_axis_y: float = 75.0,
        nominal_ball_diameter_px: float = 32.0,
    ) -> None:
        from vision_function.ball_detection.preprocess import (
            TubePreprocessor,
            load_tube_rectification,
        )

        self.model_dir = Path(model_dir)
        self.roi_path = Path(roi_path)
        self.presence_threshold = float(presence_threshold)
        self.heatmap_threshold = float(heatmap_threshold)
        self.calibrated_axis_y = float(calibrated_axis_y)
        self.radius = float(nominal_ball_diameter_px) / 2.0

        self.preprocessor = TubePreprocessor(load_tube_rectification(self.roi_path))
        self.ncnn = ShallowCNNNCNN(
            self.model_dir,
            threads=threads,
            use_fp16=use_fp16,
        )

    @property
    def production_size(self) -> tuple[int, int]:
        return (640, 100)

    def configure_ncnn_threads(self, threads: int) -> int | None:
        """Reconfigure NCNN thread count after the first warm-up inference."""
        if threads < 1:
            raise ValueError("threads must be positive")
        net = self.ncnn.net
        if int(net.opt.num_threads) != threads:
            net.clear()
            net.opt.num_threads = threads
            net.load_param(str(self.model_dir / "model.ncnn.param"))
            net.load_model(str(self.model_dir / "model.ncnn.bin"))
        return int(net.opt.num_threads)

    def detect(
        self,
        frame: np.ndarray,
        *,
        capture_timestamp_s: float,
        frame_sequence: int,
    ) -> tuple:
        """Return a typed ``BallObservation`` compatible with the scale pipeline."""
        from vision_function.scale_location.types import BallObservation

        crop = self.preprocessor(frame)
        prediction = self.ncnn.predict(
            crop,
            presence_threshold=self.presence_threshold,
            heatmap_threshold=self.heatmap_threshold,
        )
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
        return BallObservation(
            frame_sequence=frame_sequence,
            capture_timestamp_s=capture_timestamp_s,
            found=True,
            center_local_xy=(x, y),
            confidence=prediction.confidence,
            box_local_xyxy=(x - self.radius, y - self.radius, x + self.radius, y + self.radius),
            candidate_count=1,
        ), crop
