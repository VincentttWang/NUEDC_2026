"""Persist adaptively sampled, corrected ROI frames from the vehicle videos."""

from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path
from typing import Any

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parent
REPOSITORY_ROOT = ROOT.parents[1]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from vision_function.ball_detection.prepare_actual_dataset import (  # noqa: E402
    detect_circle,
    preprocess,
    yolo_label,
)
from vision_function.ball_detection.preprocess import (  # noqa: E402
    TubePreprocessor,
    load_tube_rectification,
)


DEFAULT_RAW_VIDEOS = ROOT / "data" / "raw" / "videos"
DEFAULT_OUTPUT = ROOT / "data" / "actual_video_frames_local_v3"
DEFAULT_MODEL = ROOT / "model" / "actual_yolo11n_v1.pt"
DEFAULT_ROI = ROOT / "config" / "tube_local_rectification.json"


def sampling_interval(speed_px_s: float) -> float:
    """Return the maximum keep interval for motion in the 640x100 local image."""
    if speed_px_s < 20.0:
        return 1.0
    if speed_px_s < 80.0:
        return 0.5
    if speed_px_s < 160.0:
        return 0.25
    return 0.1


def choose_annotation(
    hough: tuple[float, float, float] | None,
    yolo_box: tuple[float, float, float, float] | None,
    yolo_confidence: float | None,
) -> tuple[tuple[float, float, float] | None, str, float | None]:
    """Choose a pseudo-label and report detector agreement in ROI pixels."""
    if yolo_box is None:
        return hough, "hough_only" if hough is not None else "missing", None

    left, top, right, bottom = yolo_box
    yolo_circle = (
        (left + right) / 2.0,
        (top + bottom) / 2.0,
        max(right - left, bottom - top) / 2.0,
    )
    if hough is None:
        if yolo_confidence is not None and yolo_confidence >= 0.25:
            return yolo_circle, "yolo_only", None
        return None, "missing", None

    distance = float(np.hypot(hough[0] - yolo_circle[0], hough[1] - yolo_circle[1]))
    if distance <= 12.0:
        # Hough gives a more direct circle center and radius when both methods
        # identify the same object.
        return hough, "agreement", distance
    # In the actual videos, large disagreements were caused by Hough locking on
    # a tube fixture or reflection. The fine-tuned detector remained on the ball.
    return yolo_circle, "disagreement_yolo", distance


def predict_yolo(
    model: Any,
    crop: np.ndarray,
    imgsz: int,
) -> tuple[tuple[float, float, float, float] | None, float | None, int]:
    result = model.predict(
        crop,
        imgsz=imgsz,
        conf=0.10,
        device="cpu",
        max_det=10,
        verbose=False,
    )[0]
    if result.boxes is None or len(result.boxes) == 0:
        return None, None, 0
    confidences = result.boxes.conf.cpu().numpy()
    index = int(np.argmax(confidences))
    box = tuple(float(value) for value in result.boxes.xyxy[index].cpu().tolist())
    return box, float(confidences[index]), len(result.boxes)


def draw_preview(
    output: Path,
    rows: list[dict[str, object]],
    page_size: int = 120,
    *,
    preview_name: str = "previews",
) -> None:
    preview_dir = output / preview_name
    preview_dir.mkdir(parents=True, exist_ok=True)
    thumb_width, thumb_height, columns = 570, 115, 3
    for page_index, start in enumerate(range(0, len(rows), page_size)):
        page_rows = rows[start:start + page_size]
        canvas_rows = (len(page_rows) + columns - 1) // columns
        canvas = np.full(
            (canvas_rows * thumb_height, columns * thumb_width, 3), 32, np.uint8
        )
        for index, row in enumerate(page_rows):
            image = cv2.imread(str(output / str(row["image"])))
            annotation = row.get("annotation_roi_xyr")
            color = (0, 255, 0)
            if row["annotation_source"] in {"disagreement_yolo", "yolo_only"}:
                color = (0, 165, 255)
            if annotation is not None:
                x, y, radius = (float(value) for value in annotation)
                cv2.circle(image, (round(x), round(y)), round(radius), color, 2)
                cv2.circle(image, (round(x), round(y)), 2, color, -1)
            label = (
                f'{row["split"]} f={row["frame_index"]} '
                f'v={row["speed_px_s"]:.0f} {row["annotation_source"]}'
            )
            cv2.putText(
                image,
                label,
                (5, 16),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.43,
                (0, 255, 255),
                1,
                cv2.LINE_AA,
            )
            row_index, column_index = divmod(index, columns)
            canvas[
                row_index * thumb_height:(row_index + 1) * thumb_height,
                column_index * thumb_width:(column_index + 1) * thumb_width,
            ] = cv2.resize(
                image, (thumb_width, thumb_height), interpolation=cv2.INTER_NEAREST
            )
        cv2.imwrite(str(preview_dir / f"page_{page_index:03d}.jpg"), canvas)


def main() -> int:
    from ultralytics import YOLO

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--videos", type=Path, default=DEFAULT_RAW_VIDEOS)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--roi", type=Path, default=DEFAULT_ROI)
    parser.add_argument("--scan-stride", type=int, default=3)
    parser.add_argument("--position-step", type=float, default=10.0)
    parser.add_argument("--imgsz", type=int, default=320)
    parser.add_argument(
        "--validation-video-index",
        type=int,
        default=1,
        help="Zero-based sorted video index kept entirely out of training.",
    )
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()

    if args.scan_stride < 1 or args.position_step <= 0:
        raise SystemExit("Sampling arguments must be positive")
    video_paths = sorted(path for path in args.videos.iterdir() if path.is_file())
    if not video_paths:
        raise SystemExit(f"No videos found in {args.videos}")
    if not 0 <= args.validation_video_index < len(video_paths):
        raise SystemExit("validation-video-index is outside the video list")
    if args.output.exists():
        if not args.overwrite:
            raise SystemExit(f"Output already exists: {args.output}")
        shutil.rmtree(args.output)

    rectification = load_tube_rectification(args.roi)
    roi_size = rectification.output_size
    processor = TubePreprocessor(rectification)
    model = YOLO(str(args.model), task="detect")

    for split in ("train", "val"):
        (args.output / "images" / split).mkdir(parents=True, exist_ok=True)
        (args.output / "labels" / split).mkdir(parents=True, exist_ok=True)

    rows: list[dict[str, object]] = []
    video_reports: list[dict[str, object]] = []
    for video_index, video_path in enumerate(video_paths):
        split = "val" if video_index == args.validation_video_index else "train"
        capture = cv2.VideoCapture(str(video_path))
        if not capture.isOpened():
            raise RuntimeError(f"Cannot open {video_path}")

        frame_index = 0
        examined = 0
        retained = 0
        missing = 0
        last_observation: tuple[int, float] | None = None
        last_kept: tuple[int, float] | None = None
        source_counts: dict[str, int] = {}
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            if frame_index % args.scan_stride != 0:
                frame_index += 1
                continue

            examined += 1
            crop = preprocess(frame, processor)
            hough = detect_circle(crop)
            yolo_box, yolo_confidence, candidate_count = predict_yolo(
                model, crop, args.imgsz
            )
            annotation, source, agreement_distance = choose_annotation(
                hough, yolo_box, yolo_confidence
            )
            if (
                annotation is not None
                and source == "hough_only"
                and last_observation is not None
                and abs(annotation[0] - last_observation[1]) > 60.0
            ):
                # Reject an isolated Hough lock on a fixture. At a 0.1 s scan
                # interval this jump is outside the observed ball dynamics.
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
                delta_frames = frame_index - last_observation[0]
                speed = abs(center_x - last_observation[1]) * 30.0 / delta_frames
            last_observation = frame_index, center_x

            elapsed = (
                float("inf")
                if last_kept is None
                else (frame_index - last_kept[0]) / 30.0
            )
            displacement = (
                float("inf")
                if last_kept is None
                else abs(center_x - last_kept[1])
            )
            # The wider local canvas contains stable end-cap/fixture circles, so
            # Hough disagreement is common and no longer makes every 0.1 s frame
            # a difficult sample. A YOLO-only observation is still kept densely.
            difficult = source == "yolo_only"
            keep = (
                last_kept is None
                or displacement >= args.position_step
                or elapsed >= sampling_interval(speed)
                or (difficult and elapsed >= args.scan_stride / 30.0)
            )
            if not keep:
                frame_index += 1
                continue

            stem = f"v{video_index}_{video_path.stem}_f{frame_index:06d}"
            image_relative = Path("images") / split / f"{stem}.png"
            label_relative = Path("labels") / split / f"{stem}.txt"
            if not cv2.imwrite(str(args.output / image_relative), crop):
                raise RuntimeError(f"Cannot write {args.output / image_relative}")
            (args.output / label_relative).write_text(
                yolo_label(center_x, center_y, radius, roi_size),
                encoding="utf-8",
            )
            gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
            row: dict[str, object] = {
                "image": str(image_relative),
                "label": str(label_relative),
                "split": split,
                "source_video": video_path.name,
                "video_index": video_index,
                "frame_index": frame_index,
                "timestamp_s": round(frame_index / 30.0, 3),
                "speed_px_s": round(speed, 3),
                "sharpness": round(float(cv2.Laplacian(gray, cv2.CV_64F).var()), 3),
                "annotation_roi_xyr": [round(value, 3) for value in annotation],
                "annotation_source": source,
                "hough_roi_xyr": (
                    None
                    if hough is None
                    else [round(float(value), 3) for value in hough]
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
            rows.append(row)
            retained += 1
            last_kept = frame_index, center_x
            frame_index += 1

        capture.release()
        video_reports.append(
            {
                "video": video_path.name,
                "split": split,
                "decoded_frames": frame_index,
                "duration_s_at_30fps": round(frame_index / 30.0, 3),
                "examined_frames": examined,
                "retained_frames": retained,
                "missing_annotations": missing,
                "annotation_sources_examined": source_counts,
            }
        )

    manifest = {
        "format_version": 1,
        "purpose": "Reusable tube-local rectified frames selected from vehicle videos",
        "source_videos": str(args.videos),
        "model_for_pseudo_labels": str(args.model),
        "roi_config": str(args.roi),
        "scan_stride_frames": args.scan_stride,
        "assumed_fps": 30.0,
        "position_step_px": args.position_step,
        "sampling_intervals": {
            "speed_lt_20_px_s": 1.0,
            "speed_20_to_80_px_s": 0.5,
            "speed_80_to_160_px_s": 0.25,
            "speed_ge_160_px_s": 0.1,
        },
        "validation_video_index": args.validation_video_index,
        "coordinate_space": rectification.coordinate_space,
        "rectified_origin_xy": list(rectification.origin_xy),
        "output_size": list(roi_size),
        "videos": video_reports,
        "retained_total": len(rows),
        "frames": rows,
    }
    (args.output / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    draw_preview(args.output, rows)
    draw_preview(
        args.output,
        [row for row in rows if row["annotation_source"] != "agreement"],
        preview_name="previews_review",
    )
    print(json.dumps({"videos": video_reports, "retained_total": len(rows)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
