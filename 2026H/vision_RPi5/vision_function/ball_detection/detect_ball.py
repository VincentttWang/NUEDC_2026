"""Detect the single steel ball and return its center in corrected image coordinates."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO


ROOT = Path(__file__).resolve().parent
REPOSITORY_ROOT = ROOT.parents[1]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from vision_function.ball_detection.preprocess import (  # noqa: E402
    TubePreprocessor,
    load_tube_rectification,
)
from vision_function.scale_location.types import BallObservation  # noqa: E402


DEFAULT_MODEL = ROOT / "model" / "actual_yolo11n_local_v3_ncnn_model"
DEFAULT_ROI = ROOT / "config" / "tube_local_rectification.json"
# Validated on Raspberry Pi dynamic replay and labelled no-ball samples.
DEFAULT_CONFIDENCE = 0.10


def restore_xyxy(
    box_xyxy: tuple[float, float, float, float],
    roi_offset: tuple[int, int],
    scale: int,
) -> tuple[float, float, float, float]:
    left, top, right, bottom = box_xyxy
    offset_x, offset_y = roi_offset
    return (
        (left + offset_x) * scale,
        (top + offset_y) * scale,
        (right + offset_x) * scale,
        (bottom + offset_y) * scale,
    )


class BallDetector:
    """Strictly serial single-ball detector matching the Raspberry Pi pipeline."""

    def __init__(
        self,
        model_path: Path = DEFAULT_MODEL,
        roi_path: Path = DEFAULT_ROI,
        *,
        imgsz: int = 320,
        confidence: float = DEFAULT_CONFIDENCE,
    ) -> None:
        self.model_path = Path(model_path)
        self.rectification = load_tube_rectification(roi_path)
        self.reference_size = self.rectification.reference_size
        self.decode_scale = self.rectification.decode_scale
        origin_x, origin_y = self.rectification.origin_xy
        output_width, output_height = self.rectification.output_size
        self.roi_xyxy = (
            origin_x,
            origin_y,
            origin_x + output_width,
            origin_y + output_height,
        )
        self.balance = self.rectification.balance
        self.imgsz = int(imgsz)
        self.confidence = float(confidence)
        self.preprocessor = TubePreprocessor(self.rectification)
        self.model = YOLO(str(model_path), task="detect")

    def configure_ncnn_threads(self, threads: int) -> int | None:
        """Fix NCNN worker count after the predictor has completed its first load."""
        if threads < 1:
            raise ValueError("threads must be positive")
        predictor = getattr(self.model, "predictor", None)
        backend = getattr(predictor, "model", None)
        net = getattr(backend, "net", None)
        options = getattr(net, "opt", None)
        if options is None or not hasattr(options, "num_threads"):
            return None
        if int(options.num_threads) != threads:
            param_path = self.model_path
            if not param_path.is_file():
                param_path = next(param_path.glob("*.param"))
            net.clear()
            options.num_threads = threads
            net.load_param(str(param_path))
            net.load_model(str(param_path.with_suffix(".bin")))
        return int(options.num_threads)

    @property
    def production_size(self) -> tuple[int, int]:
        return (
            self.reference_size[0] // self.decode_scale,
            self.reference_size[1] // self.decode_scale,
        )

    def preprocess(self, frame: np.ndarray) -> np.ndarray:
        return self.preprocessor(frame)

    def detect(
        self,
        frame: np.ndarray,
        *,
        capture_timestamp_s: float,
        frame_sequence: int,
    ) -> tuple[BallObservation, np.ndarray]:
        """Return the typed detector-to-scale API described by scale_location."""
        crop = self.preprocess(frame)
        observation = self.detect_crop(
            crop,
            capture_timestamp_s=capture_timestamp_s,
            frame_sequence=frame_sequence,
        )
        return observation, crop

    def detect_crop(
        self,
        crop: np.ndarray,
        *,
        capture_timestamp_s: float,
        frame_sequence: int,
    ) -> BallObservation:
        """Detect in an already rectified 640x100 tube-local image.

        The desktop debugger and offline calibration tools sometimes already own
        the rectified ROI.  Keeping that path here avoids remapping the image a
        second time while preserving the production detector's thresholds and
        coordinate contract.
        """
        if crop is None or crop.shape != (100, 640, 3):
            shape = None if crop is None else crop.shape
            raise ValueError(f"Expected a 640x100 BGR ROI, got {shape}")
        result = self.model.predict(
            crop,
            imgsz=self.imgsz,
            conf=self.confidence,
            device="cpu",
            max_det=10,
            verbose=False,
        )[0]
        if result.boxes is None or len(result.boxes) == 0:
            return BallObservation(
                frame_sequence=frame_sequence,
                capture_timestamp_s=capture_timestamp_s,
                found=False,
                center_local_xy=None,
                confidence=None,
                box_local_xyxy=None,
                candidate_count=0,
            )

        confidences = result.boxes.conf.cpu().numpy()
        selected_index = int(np.argmax(confidences))
        box = tuple(
            float(value)
            for value in result.boxes.xyxy[selected_index].cpu().tolist()
        )
        left, top, right, bottom = box
        center_roi = (left + right) / 2.0, (top + bottom) / 2.0
        return BallObservation(
            frame_sequence=frame_sequence,
            capture_timestamp_s=capture_timestamp_s,
            found=True,
            center_local_xy=center_roi,
            confidence=float(confidences[selected_index]),
            box_local_xyxy=box,
            candidate_count=len(result.boxes),
        )

    def to_legacy_dict(self, observation: BallObservation) -> dict[str, object]:
        """Convert the typed API for older offline reports during migration."""
        if not observation.found:
            return {
                "found": False,
                "candidate_count": 0,
                "confidence": None,
                "center_reference_xy": None,
                "center_production_xy": None,
                "center_local_xy": None,
                "center_roi_xy": None,
                "box_reference_xyxy": None,
            }
        assert observation.center_local_xy is not None
        assert observation.box_local_xyxy is not None
        center_production = (
            observation.center_local_xy[0] + self.roi_xyxy[0],
            observation.center_local_xy[1] + self.roi_xyxy[1],
        )
        center_reference = tuple(value * self.decode_scale for value in center_production)
        reference_box = restore_xyxy(
            observation.box_local_xyxy,
            (self.roi_xyxy[0], self.roi_xyxy[1]),
            self.decode_scale,
        )
        center_local = [round(value, 3) for value in observation.center_local_xy]
        return {
            "found": True,
            "candidate_count": observation.candidate_count,
            "confidence": round(float(observation.confidence), 6),
            "center_reference_xy": [round(value, 3) for value in center_reference],
            "center_production_xy": [round(value, 3) for value in center_production],
            "center_local_xy": center_local,
            "center_roi_xy": center_local,
            "box_reference_xyxy": [round(value, 3) for value in reference_box],
        }

    def detect_legacy(self, frame: np.ndarray) -> tuple[dict[str, object], np.ndarray]:
        observation, crop = self.detect(
            frame, capture_timestamp_s=0.0, frame_sequence=0
        )
        return self.to_legacy_dict(observation), crop


def input_paths(path: Path) -> list[Path]:
    if path.is_file():
        return [path]
    paths = sorted(
        candidate
        for candidate in path.iterdir()
        if candidate.suffix.lower() in {".png", ".jpg", ".jpeg"}
    )
    if not paths:
        raise RuntimeError(f"No images found in {path}")
    return paths


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--roi", type=Path, default=DEFAULT_ROI)
    parser.add_argument("--imgsz", type=int, default=320)
    parser.add_argument("--conf", type=float, default=DEFAULT_CONFIDENCE)
    parser.add_argument("--report", type=Path)
    parser.add_argument("--preview-dir", type=Path)
    args = parser.parse_args()

    detector = BallDetector(args.model, args.roi, imgsz=args.imgsz, confidence=args.conf)
    rows = []
    if args.preview_dir is not None:
        args.preview_dir.mkdir(parents=True, exist_ok=True)
    for sequence, path in enumerate(input_paths(args.input)):
        frame = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if frame is None:
            raise RuntimeError(f"Cannot read {path}")
        observation, crop = detector.detect(
            frame, capture_timestamp_s=0.0, frame_sequence=sequence
        )
        detection = observation.as_dict()
        detection["image"] = path.name
        rows.append(detection)
        if args.preview_dir is not None and observation.found:
            assert observation.box_local_xyxy is not None
            left, top, right, bottom = observation.box_local_xyxy
            x1, y1, x2, y2 = map(round, (left, top, right, bottom))
            cv2.rectangle(crop, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.imwrite(str(args.preview_dir / path.name), crop)

    report = {
        "model": str(args.model),
        "roi_config": str(args.roi),
        "imgsz": args.imgsz,
        "confidence_threshold": args.conf,
        "images": rows,
    }
    text = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.report is not None:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(text, encoding="utf-8")
    print(text, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
