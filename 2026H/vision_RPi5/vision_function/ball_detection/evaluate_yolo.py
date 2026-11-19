"""Compare YOLO steel-ball detection on full images and a calibrated ROI."""

from __future__ import annotations

import argparse
import json
import statistics
from pathlib import Path
from time import perf_counter

import cv2
from ultralytics import YOLO


ROOT = Path(__file__).resolve().parent
DEFAULT_DATA = ROOT / "data" / "raw_similar"
DEFAULT_MODEL = ROOT / "model" / "best.pt"
DEFAULT_ROI = ROOT / "config" / "raw_similar_roi.json"
DEFAULT_OUTPUT = ROOT / "output" / "raw_similar_yolo"


def load_roi(path: Path) -> tuple[int, int, int, int]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    roi = payload["roi"]
    return int(roi["left"]), int(roi["top"]), int(roi["right"]), int(roi["bottom"])


def detections(
    result, offset: tuple[int, int] = (0, 0), coordinate_scale: int = 1
) -> list[dict[str, object]]:
    output: list[dict[str, object]] = []
    if result.boxes is None:
        return output
    offset_x, offset_y = offset
    for xyxy, confidence, class_id in zip(
        result.boxes.xyxy.cpu().tolist(),
        result.boxes.conf.cpu().tolist(),
        result.boxes.cls.cpu().tolist(),
        strict=True,
    ):
        left, top, right, bottom = xyxy
        left += offset_x
        right += offset_x
        top += offset_y
        bottom += offset_y
        left *= coordinate_scale
        right *= coordinate_scale
        top *= coordinate_scale
        bottom *= coordinate_scale
        output.append(
            {
                "class_id": int(class_id),
                "confidence": round(float(confidence), 6),
                "box_xyxy": [round(left, 2), round(top, 2), round(right, 2), round(bottom, 2)],
                "center_xy": [round((left + right) / 2, 2), round((top + bottom) / 2, 2)],
                "size_wh": [round(right - left, 2), round(bottom - top, 2)],
            }
        )
    return output


def draw_box(image, detection: dict[str, object], color: tuple[int, int, int], label: str) -> None:
    left, top, right, bottom = (round(value) for value in detection["box_xyxy"])
    confidence = float(detection["confidence"])
    cv2.rectangle(image, (left, top), (right, bottom), color, 2, cv2.LINE_AA)
    cv2.putText(
        image,
        f"{label} {confidence:.3f}",
        (left, max(20, top - 7)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        color,
        2,
        cv2.LINE_AA,
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", type=Path, default=DEFAULT_DATA)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--roi", type=Path, default=DEFAULT_ROI)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--source-scale", type=int, choices=(1, 2), default=1)
    args = parser.parse_args()

    paths = sorted(args.data.glob("*.png"))
    if not paths:
        raise SystemExit(f"No PNG images found in {args.data}")
    reference_left, reference_top, reference_right, reference_bottom = load_roi(args.roi)
    reference_roi = (reference_left, reference_top, reference_right, reference_bottom)
    if any(value % args.source_scale for value in reference_roi):
        raise RuntimeError(f"ROI {reference_roi} is not divisible by {args.source_scale}")
    left, top, right, bottom = (
        value // args.source_scale for value in reference_roi
    )

    images = []
    crops = []
    image_size: tuple[int, int] | None = None
    for path in paths:
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f"Cannot read {path}")
        reference_height, reference_width = image.shape[:2]
        if image_size is None:
            image_size = (reference_width, reference_height)
        elif image_size != (reference_width, reference_height):
            raise RuntimeError(
                f"Image size mismatch: {path} is {reference_width}x{reference_height}, "
                f"expected {image_size}"
            )
        if args.source_scale != 1:
            image = cv2.resize(
                image,
                (reference_width // args.source_scale, reference_height // args.source_scale),
                interpolation=cv2.INTER_AREA,
            )
        height, width = image.shape[:2]
        if not (0 <= left < right <= width and 0 <= top < bottom <= height):
            raise RuntimeError(f"ROI {(left, top, right, bottom)} is outside {path} ({width}x{height})")
        images.append(image)
        crops.append(image[top:bottom, left:right])

    model = YOLO(str(args.model))
    # Warm up both input shapes so wall-clock timings do not include model setup.
    model.predict(images[:1], imgsz=args.imgsz, conf=args.conf, device="cpu", verbose=False)
    model.predict(crops[:1], imgsz=args.imgsz, conf=args.conf, device="cpu", verbose=False)
    started = perf_counter()
    full_results = model.predict(images, imgsz=args.imgsz, conf=args.conf, device="cpu", verbose=False)
    full_seconds = perf_counter() - started
    started = perf_counter()
    roi_results = model.predict(crops, imgsz=args.imgsz, conf=args.conf, device="cpu", verbose=False)
    roi_seconds = perf_counter() - started

    args.output.mkdir(parents=True, exist_ok=True)
    rows = []
    for path, image, full_result, roi_result in zip(
        paths, images, full_results, roi_results, strict=True
    ):
        full = detections(full_result, coordinate_scale=args.source_scale)
        roi = detections(
            roi_result, (left, top), coordinate_scale=args.source_scale
        )
        annotated = cv2.resize(
            image,
            image_size,
            interpolation=cv2.INTER_LINEAR,
        ) if args.source_scale != 1 else image.copy()
        cv2.rectangle(
            annotated,
            (reference_left, reference_top),
            (reference_right, reference_bottom),
            (255, 128, 0),
            2,
            cv2.LINE_AA,
        )
        for item in full:
            draw_box(annotated, item, (0, 165, 255), "full")
        for item in roi:
            draw_box(annotated, item, (0, 220, 0), "roi")
        cv2.imwrite(str(args.output / path.name), annotated)
        rows.append({"image": path.name, "full": full, "roi": roi})

    full_confidences = [item["confidence"] for row in rows for item in row["full"]]
    roi_confidences = [item["confidence"] for row in rows for item in row["roi"]]
    report = {
        "model": str(args.model),
        "model_names": model.names,
        "image_count": len(paths),
        "image_size": list(image_size) if image_size else None,
        "source_scale": args.source_scale,
        "inference_image_size": list(reversed(images[0].shape[:2])),
        "roi_xyxy": list(reference_roi),
        "inference_roi_xyxy": [left, top, right, bottom],
        "imgsz": args.imgsz,
        "confidence_threshold": args.conf,
        "full_total_seconds": round(full_seconds, 6),
        "roi_total_seconds": round(roi_seconds, 6),
        "full_ms_per_image": round(full_seconds / len(paths) * 1000, 3),
        "roi_ms_per_image": round(roi_seconds / len(paths) * 1000, 3),
        "full_images_with_detection": sum(bool(row["full"]) for row in rows),
        "roi_images_with_detection": sum(bool(row["roi"]) for row in rows),
        "full_detection_count": len(full_confidences),
        "roi_detection_count": len(roi_confidences),
        "full_mean_confidence": (
            round(statistics.mean(full_confidences), 6) if full_confidences else None
        ),
        "roi_mean_confidence": (
            round(statistics.mean(roi_confidences), 6) if roi_confidences else None
        ),
        "images": rows,
    }
    (args.output / "report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
