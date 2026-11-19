"""Extract labelled positive and negative ROI frames from the raw_new videos.

The output is an unsplit sample pool.  Keeping it unsplit avoids pretending that
adjacent frames from the only positive/negative recordings form an independent
validation set.  A later dataset-building step can add these samples to the
training split while retaining the repository's existing validation set.
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
import sys
from pathlib import Path
from time import perf_counter
from typing import Any

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parent
REPOSITORY_ROOT = ROOT.parents[1]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from vision_function.ball_detection.extract_actual_video_frames import (  # noqa: E402
    choose_annotation,
    predict_yolo,
    sampling_interval,
)
from vision_function.ball_detection.prepare_actual_dataset import (  # noqa: E402
    detect_circle,
    preprocess,
    yolo_label,
)
from vision_function.ball_detection.preprocess import (  # noqa: E402
    TubePreprocessor,
    load_tube_rectification,
)


DEFAULT_RAW_NEW = ROOT / "data" / "raw_new"
DEFAULT_OUTPUT = ROOT / "data" / "actual_video_frames_raw_new"
DEFAULT_MODEL = ROOT / "model" / "actual_yolo11n_local_v3.pt"
DEFAULT_ROI = ROOT / "config" / "tube_local_rectification.json"


def _safe_fps(capture: cv2.VideoCapture) -> float:
    fps = float(capture.get(cv2.CAP_PROP_FPS))
    return fps if math.isfinite(fps) and fps > 0.0 else 30.0


def _write_sample(
    output: Path,
    stem: str,
    crop: np.ndarray,
    label: str,
) -> tuple[Path, Path]:
    image_relative = Path("images") / "all" / f"{stem}.png"
    label_relative = Path("labels") / "all" / f"{stem}.txt"
    if not cv2.imwrite(str(output / image_relative), crop):
        raise RuntimeError(f"Cannot write {output / image_relative}")
    (output / label_relative).write_text(label, encoding="utf-8")
    return image_relative, label_relative


def _sharpness(image: np.ndarray) -> float:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def _preview(
    output: Path,
    rows: list[dict[str, object]],
    *,
    name: str,
    page_size: int = 120,
) -> None:
    preview_dir = output / name
    preview_dir.mkdir(parents=True, exist_ok=True)
    thumb_width, thumb_height, columns = 640, 125, 3
    for page_index, start in enumerate(range(0, len(rows), page_size)):
        page = rows[start : start + page_size]
        canvas_rows = (len(page) + columns - 1) // columns
        canvas = np.full(
            (canvas_rows * thumb_height, columns * thumb_width, 3), 28, np.uint8
        )
        for index, row in enumerate(page):
            image = cv2.imread(str(output / str(row["image"])))
            annotation = row.get("annotation_roi_xyr")
            if annotation is not None:
                x, y, radius = (float(value) for value in annotation)
                cv2.circle(image, (round(x), round(y)), round(radius), (0, 255, 0), 2)
                cv2.circle(image, (round(x), round(y)), 2, (0, 255, 0), -1)
            false_box = row.get("negative_yolo_box_xyxy")
            if false_box is not None:
                left, top, right, bottom = (round(float(value)) for value in false_box)
                cv2.rectangle(image, (left, top), (right, bottom), (0, 0, 255), 2)
            label = (
                f'{row["kind"]} f={row["frame_index"]} '
                f't={row["timestamp_s"]:.2f}s {row["annotation_source"]}'
            )
            cv2.putText(
                image,
                label,
                (5, 17),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 255, 255),
                1,
                cv2.LINE_AA,
            )
            row_index, column_index = divmod(index, columns)
            canvas[
                row_index * thumb_height : (row_index + 1) * thumb_height,
                column_index * thumb_width : (column_index + 1) * thumb_width,
            ] = cv2.resize(
                image, (thumb_width, thumb_height), interpolation=cv2.INTER_NEAREST
            )
        if not cv2.imwrite(str(preview_dir / f"page_{page_index:03d}.jpg"), canvas):
            raise RuntimeError(f"Cannot write preview page {page_index}")


def _extract_positive(
    video_path: Path,
    output: Path,
    processor: TubePreprocessor,
    roi_size: tuple[int, int],
    model: Any,
    *,
    sample_prefix: str,
    scan_rate_hz: float,
    position_step_px: float,
    imgsz: int,
) -> tuple[list[dict[str, object]], dict[str, object]]:
    capture = cv2.VideoCapture(str(video_path))
    if not capture.isOpened():
        raise RuntimeError(f"Cannot open {video_path}")
    fps = _safe_fps(capture)
    scan_stride = max(1, round(fps / scan_rate_hz))
    frame_index = examined = retained = missing = 0
    last_observation: tuple[float, float] | None = None
    last_kept: tuple[float, float] | None = None
    source_counts: dict[str, int] = {}
    rows: list[dict[str, object]] = []
    started = perf_counter()
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        timestamp_s = frame_index / fps
        if frame_index % scan_stride:
            frame_index += 1
            continue
        examined += 1
        crop = preprocess(frame, processor)
        hough = detect_circle(crop)
        yolo_box, yolo_confidence, candidate_count = predict_yolo(model, crop, imgsz)
        annotation, source, agreement_distance = choose_annotation(
            hough, yolo_box, yolo_confidence
        )
        if (
            annotation is not None
            and source == "hough_only"
            and last_observation is not None
            and abs(annotation[0] - last_observation[1]) > 60.0
        ):
            annotation = None
            source = "temporal_reject"
        source_counts[source] = source_counts.get(source, 0) + 1
        if annotation is None:
            missing += 1
            frame_index += 1
            continue

        center_x, center_y, radius = annotation
        if last_observation is None:
            speed = 0.0
        else:
            dt = timestamp_s - last_observation[0]
            speed = abs(center_x - last_observation[1]) / dt if dt > 0.0 else 0.0
        last_observation = timestamp_s, center_x
        elapsed = math.inf if last_kept is None else timestamp_s - last_kept[0]
        displacement = (
            math.inf if last_kept is None else abs(center_x - last_kept[1])
        )
        difficult = source in {"yolo_only", "disagreement_yolo"}
        keep = (
            last_kept is None
            or displacement >= position_step_px
            or elapsed >= sampling_interval(speed)
            or (difficult and elapsed >= 1.0 / scan_rate_hz)
        )
        if not keep:
            frame_index += 1
            continue

        # Keep generated paths ASCII-only.  Some Windows OpenCV builds cannot
        # read relative paths containing non-ASCII characters even though the
        # same files can be opened through an absolute path.
        stem = f"{sample_prefix}_f{frame_index:06d}"
        image_relative, label_relative = _write_sample(
            output,
            stem,
            crop,
            yolo_label(center_x, center_y, radius, roi_size),
        )
        rows.append(
            {
                "image": str(image_relative),
                "label": str(label_relative),
                "kind": "positive",
                "source_video": video_path.name,
                "frame_index": frame_index,
                "timestamp_s": round(timestamp_s, 6),
                "speed_px_s": round(speed, 3),
                "sharpness": round(_sharpness(crop), 3),
                "annotation_roi_xyr": [round(float(value), 3) for value in annotation],
                "annotation_source": source,
                "hough_roi_xyr": (
                    None if hough is None else [round(float(value), 3) for value in hough]
                ),
                "yolo_roi_xyxy": (
                    None
                    if yolo_box is None
                    else [round(float(value), 3) for value in yolo_box]
                ),
                "detector_agreement_px": (
                    None if agreement_distance is None else round(agreement_distance, 3)
                ),
                "yolo_confidence": (
                    None if yolo_confidence is None else round(yolo_confidence, 6)
                ),
                "yolo_candidate_count": candidate_count,
            }
        )
        retained += 1
        last_kept = timestamp_s, center_x
        frame_index += 1
    capture.release()
    return rows, {
        "video": video_path.name,
        "kind": "positive",
        "fps": fps,
        "decoded_frames": frame_index,
        "duration_s": round(frame_index / fps, 6),
        "scan_stride_frames": scan_stride,
        "examined_frames": examined,
        "retained_frames": retained,
        "missing_annotations": missing,
        "annotation_sources_examined": source_counts,
        "processing_s": round(perf_counter() - started, 3),
    }


def _extract_negative(
    video_path: Path,
    output: Path,
    processor: TubePreprocessor,
    model: Any,
    *,
    sample_prefix: str,
    sample_rate_hz: float,
    imgsz: int,
) -> tuple[list[dict[str, object]], dict[str, object]]:
    capture = cv2.VideoCapture(str(video_path))
    if not capture.isOpened():
        raise RuntimeError(f"Cannot open {video_path}")
    fps = _safe_fps(capture)
    stride = max(1, round(fps / sample_rate_hz))
    frame_index = retained = flagged = 0
    rows: list[dict[str, object]] = []
    started = perf_counter()
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        if frame_index % stride:
            frame_index += 1
            continue
        timestamp_s = frame_index / fps
        crop = preprocess(frame, processor)
        yolo_box, yolo_confidence, candidate_count = predict_yolo(model, crop, imgsz)
        if yolo_box is not None:
            flagged += 1
        stem = f"{sample_prefix}_f{frame_index:06d}"
        image_relative, label_relative = _write_sample(output, stem, crop, "")
        rows.append(
            {
                "image": str(image_relative),
                "label": str(label_relative),
                "kind": "negative",
                "source_video": video_path.name,
                "frame_index": frame_index,
                "timestamp_s": round(timestamp_s, 6),
                "speed_px_s": 0.0,
                "sharpness": round(_sharpness(crop), 3),
                "annotation_roi_xyr": None,
                "annotation_source": "negative_empty_label",
                "negative_yolo_box_xyxy": (
                    None
                    if yolo_box is None
                    else [round(float(value), 3) for value in yolo_box]
                ),
                "yolo_confidence": (
                    None if yolo_confidence is None else round(yolo_confidence, 6)
                ),
                "yolo_candidate_count": candidate_count,
            }
        )
        retained += 1
        frame_index += 1
    capture.release()
    return rows, {
        "video": video_path.name,
        "kind": "negative",
        "fps": fps,
        "decoded_frames": frame_index,
        "duration_s": round(frame_index / fps, 6),
        "sample_stride_frames": stride,
        "retained_frames": retained,
        "production_yolo_flagged_frames": flagged,
        "processing_s": round(perf_counter() - started, 3),
    }


def main() -> int:
    from ultralytics import YOLO

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--positive-video",
        dest="positive_videos",
        type=Path,
        action="append",
        help="positive source video; repeat for multiple videos",
    )
    parser.add_argument(
        "--negative-video",
        dest="negative_videos",
        type=Path,
        action="append",
        help="known-empty source video; repeat for multiple videos",
    )
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--roi", type=Path, default=DEFAULT_ROI)
    parser.add_argument("--positive-scan-rate", type=float, default=10.0)
    parser.add_argument("--negative-sample-rate", type=float, default=5.0)
    parser.add_argument("--position-step", type=float, default=10.0)
    parser.add_argument("--imgsz", type=int, default=320)
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    if (
        args.positive_scan_rate <= 0.0
        or args.negative_sample_rate <= 0.0
        or args.position_step <= 0.0
    ):
        raise SystemExit("Sampling rates and position step must be positive")
    positive_videos = args.positive_videos or [DEFAULT_RAW_NEW / "带球的视频.webm"]
    negative_videos = args.negative_videos or [DEFAULT_RAW_NEW / "无球的视频.webm"]
    for path in (*positive_videos, *negative_videos, args.model, args.roi):
        if not path.exists():
            raise SystemExit(f"Required input does not exist: {path}")
    if args.output.exists():
        if not args.overwrite:
            raise SystemExit(f"Output already exists: {args.output}")
        shutil.rmtree(args.output)
    for kind in ("images", "labels"):
        (args.output / kind / "all").mkdir(parents=True, exist_ok=True)

    rectification = load_tube_rectification(args.roi)
    processor = TubePreprocessor(rectification)
    model = YOLO(str(args.model), task="detect")
    positive_rows: list[dict[str, object]] = []
    positive_reports: list[dict[str, object]] = []
    for video_index, video_path in enumerate(positive_videos):
        video_rows, video_report = _extract_positive(
            video_path,
            args.output,
            processor,
            rectification.output_size,
            model,
            sample_prefix=f"positive_v{video_index}",
            scan_rate_hz=args.positive_scan_rate,
            position_step_px=args.position_step,
            imgsz=args.imgsz,
        )
        positive_rows.extend(video_rows)
        positive_reports.append(video_report)
    negative_rows: list[dict[str, object]] = []
    negative_reports: list[dict[str, object]] = []
    for video_index, video_path in enumerate(negative_videos):
        video_rows, video_report = _extract_negative(
            video_path,
            args.output,
            processor,
            model,
            sample_prefix=f"negative_v{video_index}",
            sample_rate_hz=args.negative_sample_rate,
            imgsz=args.imgsz,
        )
        negative_rows.extend(video_rows)
        negative_reports.append(video_report)
    rows = positive_rows + negative_rows
    manifest = {
        "format_version": 1,
        "purpose": "Unsplit 640x100 ROI sample pool from raw_new tilt recordings",
        "split_policy": (
            "unsplit: both classes have only one source video; add to training and "
            "retain an existing independent validation set"
        ),
        "positive_videos": [str(path) for path in positive_videos],
        "negative_videos": [str(path) for path in negative_videos],
        "model_for_positive_pseudo_labels": str(args.model),
        "negative_labels": "empty because the source recording is known to contain no ball",
        "roi_config": str(args.roi),
        "coordinate_space": rectification.coordinate_space,
        "output_size": list(rectification.output_size),
        "positive_scan_rate_hz": args.positive_scan_rate,
        "negative_sample_rate_hz": args.negative_sample_rate,
        "position_step_px": args.position_step,
        "videos": positive_reports + negative_reports,
        "positive_samples": len(positive_rows),
        "negative_samples": len(negative_rows),
        "retained_total": len(rows),
        "frames": rows,
    }
    (args.output / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    _preview(args.output, positive_rows, name="previews_positive")
    _preview(args.output, negative_rows, name="previews_negative")
    flagged_rows = [
        row for row in negative_rows if row["negative_yolo_box_xyxy"] is not None
    ]
    if flagged_rows:
        _preview(args.output, flagged_rows, name="previews_negative_flagged")
    summary = {
        "output": str(args.output),
        "positive_samples": len(positive_rows),
        "negative_samples": len(negative_rows),
        "negative_yolo_flagged": len(flagged_rows),
        "retained_total": len(rows),
        "videos": positive_reports + negative_reports,
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
