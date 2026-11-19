"""Evaluate a ball detector at the production confidence on a YOLO ROI dataset."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import yaml
from ultralytics import YOLO


def percentile(values: list[float], q: float) -> float | None:
    return None if not values else round(float(np.percentile(values, q)), 4)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--split", default="val")
    parser.add_argument("--imgsz", type=int, default=320)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()

    payload = yaml.safe_load(args.data.read_text(encoding="utf-8"))
    dataset_root = Path(payload["path"])
    if not dataset_root.is_absolute():
        dataset_root = (args.data.parent / dataset_root).resolve()
    image_dir = dataset_root / str(payload[args.split])
    label_dir = dataset_root / "labels" / args.split
    image_paths = sorted(image_dir.glob("*.png"))
    if not image_paths:
        raise SystemExit(f"No PNG images found in {image_dir}")

    model = YOLO(str(args.model), task="detect")
    results = model.predict(
        [str(path) for path in image_paths],
        imgsz=args.imgsz,
        conf=args.conf,
        device="cpu",
        max_det=10,
        stream=True,
        verbose=False,
    )

    rows: list[dict[str, object]] = []
    positive_count = 0
    detected_positive = 0
    negative_count = 0
    false_positive = 0
    center_errors: list[float] = []
    selected_confidences: list[float] = []
    multiple_candidates = 0
    for path, result in zip(image_paths, results, strict=True):
        label_path = label_dir / f"{path.stem}.txt"
        label = label_path.read_text(encoding="utf-8").strip()
        ground_truth_center: tuple[float, float] | None = None
        if label:
            _, center_x, center_y, _, _ = map(float, label.split())
            ground_truth_center = center_x * result.orig_shape[1], center_y * result.orig_shape[0]
            positive_count += 1
        else:
            negative_count += 1

        candidate_count = 0 if result.boxes is None else len(result.boxes)
        if candidate_count == 0:
            rows.append(
                {
                    "image": path.name,
                    "has_ground_truth": ground_truth_center is not None,
                    "found": False,
                    "candidate_count": 0,
                }
            )
            continue

        if candidate_count > 1:
            multiple_candidates += 1
        confidences = result.boxes.conf.cpu().numpy()
        index = int(np.argmax(confidences))
        confidence = float(confidences[index])
        left, top, right, bottom = (
            float(value) for value in result.boxes.xyxy[index].cpu().tolist()
        )
        center = (left + right) / 2.0, (top + bottom) / 2.0
        selected_confidences.append(confidence)
        error = None
        if ground_truth_center is None:
            false_positive += 1
        else:
            detected_positive += 1
            error = float(np.hypot(
                center[0] - ground_truth_center[0],
                center[1] - ground_truth_center[1],
            ))
            center_errors.append(error)
        rows.append(
            {
                "image": path.name,
                "has_ground_truth": ground_truth_center is not None,
                "found": True,
                "candidate_count": candidate_count,
                "confidence": round(confidence, 6),
                "center_roi_xy": [round(value, 3) for value in center],
                "center_error_px": None if error is None else round(error, 4),
            }
        )

    summary = {
        "positive_detected": detected_positive,
        "positive_total": positive_count,
        "recall_at_confidence": round(detected_positive / positive_count, 6),
        "negative_false_positive": false_positive,
        "negative_total": negative_count,
        "multiple_candidate_images": multiple_candidates,
        "selected_confidence_mean": (
            None if not selected_confidences else round(float(np.mean(selected_confidences)), 6)
        ),
        "selected_confidence_min": (
            None if not selected_confidences else round(min(selected_confidences), 6)
        ),
        "center_error_roi_px_mean": (
            None if not center_errors else round(float(np.mean(center_errors)), 4)
        ),
        "center_error_roi_px_p95": percentile(center_errors, 95),
        "center_error_roi_px_max": None if not center_errors else round(max(center_errors), 4),
    }
    report = {
        "model": str(args.model),
        "data": str(args.data),
        "split": args.split,
        "imgsz": args.imgsz,
        "confidence_threshold": args.conf,
        "summary": summary,
        "images": rows,
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
