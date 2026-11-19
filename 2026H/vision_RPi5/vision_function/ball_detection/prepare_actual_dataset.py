"""Build a YOLO dataset from the final vehicle photos and a raw motion video."""

from __future__ import annotations

import argparse
import json
import shutil
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np
import yaml


ROOT = Path(__file__).resolve().parent
REPOSITORY_ROOT = ROOT.parents[1]
DEFAULT_RAW = ROOT / "data" / "raw"
DEFAULT_OUTPUT = ROOT / "data" / "actual_yolo_v1"
DEFAULT_ROI = ROOT / "config" / "tube_local_rectification.json"
DEFAULT_ANNOTATIONS = ROOT / "config" / "actual_photo_annotations.json"
DEFAULT_BOUNDARY_ANNOTATIONS = ROOT / "config" / "boundary_annotations.json"
DEFAULT_LIVE_ANNOTATIONS = ROOT / "config" / "live_stationary_annotations.json"

import sys

if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from vision_function.ball_detection.preprocess import (  # noqa: E402
    TubePreprocessor,
    load_tube_rectification,
)


VAL_POSITIVE_INDICES = {1, 5, 9, 13, 17, 20}
VAL_NEGATIVE_INDICES = {2, 6, 9}


def load_config(path: Path) -> dict[str, object]:
    return json.loads(path.read_text(encoding="utf-8"))


def yolo_label(
    center_x: float,
    center_y: float,
    radius: float,
    image_size: tuple[int, int],
) -> str:
    width, height = image_size
    half_side = max(radius + 2.0, 10.0)
    left = max(0.0, center_x - half_side)
    top = max(0.0, center_y - half_side)
    right = min(float(width), center_x + half_side)
    bottom = min(float(height), center_y + half_side)
    box_center_x = (left + right) / 2.0 / width
    box_center_y = (top + bottom) / 2.0 / height
    box_width = (right - left) / width
    box_height = (bottom - top) / height
    return (
        f"0 {box_center_x:.8f} {box_center_y:.8f} "
        f"{box_width:.8f} {box_height:.8f}\n"
    )


def preprocess(
    frame: np.ndarray,
    processor: TubePreprocessor,
) -> np.ndarray:
    return processor(frame)


def detect_circle(crop: np.ndarray) -> tuple[float, float, float] | None:
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 1.2)
    circles = cv2.HoughCircles(
        gray,
        cv2.HOUGH_GRADIENT,
        dp=1,
        minDist=25,
        param1=80,
        param2=18,
        minRadius=8,
        maxRadius=18,
    )
    if circles is None:
        return None
    width, height = crop.shape[1], crop.shape[0]
    candidates = [
        (float(x), float(y), float(radius))
        for x, y, radius in circles[0]
        if 55 <= x <= width - 20 and 15 <= y <= height - 15
    ]
    if not candidates:
        return None
    # The left fixture creates a stable circle near x=30 and is rejected above.
    # Exactly one steel ball exists, so the strongest remaining Hough result is used.
    return candidates[0]


def write_sample(
    output: Path,
    split: str,
    stem: str,
    crop: np.ndarray,
    label: str,
) -> None:
    image_path = output / "images" / split / f"{stem}.png"
    label_path = output / "labels" / split / f"{stem}.txt"
    if not cv2.imwrite(str(image_path), crop):
        raise RuntimeError(f"Cannot write {image_path}")
    label_path.write_text(label, encoding="utf-8")


def draw_preview(output: Path, split: str, limit: int = 180) -> None:
    image_paths = sorted((output / "images" / split).glob("*.png"))[:limit]
    if not image_paths:
        return
    thumb_width, thumb_height, columns = 640, 120, 3
    rows = (len(image_paths) + columns - 1) // columns
    canvas = np.full((rows * thumb_height, columns * thumb_width, 3), 32, np.uint8)
    for index, image_path in enumerate(image_paths):
        image = cv2.imread(str(image_path))
        label_path = output / "labels" / split / f"{image_path.stem}.txt"
        label = label_path.read_text(encoding="utf-8").strip()
        if label:
            _, cx, cy, width, height = map(float, label.split())
            x1 = round((cx - width / 2) * image.shape[1])
            y1 = round((cy - height / 2) * image.shape[0])
            x2 = round((cx + width / 2) * image.shape[1])
            y2 = round((cy + height / 2) * image.shape[0])
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        thumb = cv2.resize(image, (thumb_width, thumb_height), interpolation=cv2.INTER_NEAREST)
        cv2.putText(
            thumb,
            image_path.stem[-24:],
            (5, 17),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )
        row, column = divmod(index, columns)
        canvas[
            row * thumb_height:(row + 1) * thumb_height,
            column * thumb_width:(column + 1) * thumb_width,
        ] = thumb
    cv2.imwrite(str(output / f"preview_{split}.jpg"), canvas)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw", type=Path, default=DEFAULT_RAW)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--roi", type=Path, default=DEFAULT_ROI)
    parser.add_argument("--annotations", type=Path, default=DEFAULT_ANNOTATIONS)
    parser.add_argument(
        "--boundary-annotations", type=Path, default=DEFAULT_BOUNDARY_ANNOTATIONS
    )
    parser.add_argument(
        "--live-annotations", type=Path, default=DEFAULT_LIVE_ANNOTATIONS
    )
    parser.add_argument(
        "--extracted-video",
        type=Path,
        help="Reuse persisted video frames instead of sampling raw videos.",
    )
    parser.add_argument("--video-stride", type=int, default=10)
    parser.add_argument("--video-max-per-position-bin", type=int, default=6)
    parser.add_argument(
        "--boundary-train-repeat",
        type=int,
        default=10,
        help="Repeat scarce physical-limit samples in train; validation is never repeated.",
    )
    parser.add_argument(
        "--live-train-repeat",
        type=int,
        default=3,
        help="Repeat the verified stationary live capture in train.",
    )
    args = parser.parse_args()

    if args.output.exists() and any(args.output.iterdir()):
        raise SystemExit(f"Output is not empty: {args.output}")
    if (
        args.video_stride < 1
        or args.video_max_per_position_bin < 1
        or args.boundary_train_repeat < 1
        or args.live_train_repeat < 1
    ):
        raise SystemExit("Video sampling arguments must be positive")

    annotation_payload = load_config(args.annotations)
    boundary_payload = load_config(args.boundary_annotations)
    live_payload = load_config(args.live_annotations)
    rectification = load_tube_rectification(args.roi)
    reference_size = rectification.reference_size
    decode_scale = rectification.decode_scale
    origin = rectification.origin_xy
    roi_size = rectification.output_size
    balance = rectification.balance
    annotation_origin = tuple(
        int(value) for value in annotation_payload.get("rectified_origin_xy", origin)
    )
    if float(annotation_payload.get("fisheye_balance", balance)) != balance:
        raise RuntimeError("Annotation and output fisheye balances do not match")
    annotations = {
        name: [
            float(values[0]) + annotation_origin[0] - origin[0],
            float(values[1]) + annotation_origin[1] - origin[1],
            float(values[2]),
        ]
        for name, values in annotation_payload["annotations"].items()
    }
    processor = TubePreprocessor(rectification)

    for split in ("train", "val"):
        (args.output / "images" / split).mkdir(parents=True, exist_ok=True)
        (args.output / "labels" / split).mkdir(parents=True, exist_ok=True)

    report: dict[str, object] = {
        "reference_image_size": list(reference_size),
        "decode_scale": decode_scale,
        "fisheye_balance": balance,
        "coordinate_space": rectification.coordinate_space,
        "rectified_origin_xy": list(origin),
        "production_output_size": list(roi_size),
        "static_positive": {"train": 0, "val": 0},
        "static_negative": {"train": 0, "val": 0},
    }

    positive_paths = sorted((args.raw / "photos").glob("*.png"))
    if set(path.name for path in positive_paths) != set(annotations):
        raise RuntimeError("Positive photos and curated annotations do not match")
    for index, path in enumerate(positive_paths):
        frame = cv2.imread(str(path))
        if frame is None:
            raise RuntimeError(f"Cannot read {path}")
        crop = preprocess(frame, processor)
        center_x, center_y, radius = (float(value) for value in annotations[path.name])
        split = "val" if index in VAL_POSITIVE_INDICES else "train"
        write_sample(
            args.output,
            split,
            f"photo_{path.stem}",
            crop,
            yolo_label(center_x, center_y, radius, roi_size),
        )
        report["static_positive"][split] += 1

    negative_paths = sorted((args.raw / "photos_without_ball").glob("*.png"))
    for index, path in enumerate(negative_paths):
        frame = cv2.imread(str(path))
        if frame is None:
            raise RuntimeError(f"Cannot read {path}")
        crop = preprocess(frame, processor)
        split = "val" if index in VAL_NEGATIVE_INDICES else "train"
        write_sample(args.output, split, f"negative_{path.stem}", crop, "")
        report["static_negative"][split] += 1

    boundary_annotations = boundary_payload["annotations"]
    if tuple(boundary_payload["rectified_origin_xy"]) != origin:
        raise RuntimeError("Boundary annotations use a different rectified origin")
    if tuple(boundary_payload["image_size"]) != roi_size:
        raise RuntimeError("Boundary annotations use a different output size")
    report["boundary_positive"] = {"train": 0, "val": 0}
    report["boundary_negative"] = {"train": 0, "val": 0}
    report["boundary_train_repeat"] = args.boundary_train_repeat
    validation_indices = {
        "left": {2, 4},
        "center": {2},
        "right": {2, 4},
        "empty": {2},
    }
    boundary_root = args.raw / "boundary"
    for category in ("left", "center", "right"):
        paths = sorted((boundary_root / category).glob("*.png"))
        expected = {
            f"{category}/{path.name}" for path in paths
        }
        configured = {
            name for name in boundary_annotations if name.startswith(f"{category}/")
        }
        if expected != configured:
            raise RuntimeError(f"Boundary annotations do not match {category} images")
        for index, path in enumerate(paths):
            frame = cv2.imread(str(path))
            if frame is None:
                raise RuntimeError(f"Cannot read {path}")
            crop = preprocess(frame, processor)
            center_x, center_y, radius = (
                float(value)
                for value in boundary_annotations[f"{category}/{path.name}"]
            )
            split = "val" if index in validation_indices[category] else "train"
            write_sample(
                args.output,
                split,
                f"boundary_{category}_{path.stem}",
                crop,
                yolo_label(center_x, center_y, radius, roi_size),
            )
            if split == "train":
                for repeat in range(1, args.boundary_train_repeat):
                    write_sample(
                        args.output,
                        split,
                        f"boundary_{category}_{path.stem}_repeat{repeat:02d}",
                        crop,
                        yolo_label(center_x, center_y, radius, roi_size),
                    )
            report["boundary_positive"][split] += 1

    for index, path in enumerate(sorted((boundary_root / "empty").glob("*.png"))):
        frame = cv2.imread(str(path))
        if frame is None:
            raise RuntimeError(f"Cannot read {path}")
        crop = preprocess(frame, processor)
        split = "val" if index in validation_indices["empty"] else "train"
        write_sample(
            args.output, split, f"boundary_empty_{path.stem}", crop, ""
        )
        if split == "train":
            for repeat in range(1, args.boundary_train_repeat):
                write_sample(
                    args.output,
                    split,
                    f"boundary_empty_{path.stem}_repeat{repeat:02d}",
                    crop,
                    "",
                )
        report["boundary_negative"][split] += 1

    live_folder = args.raw / str(live_payload["source_folder"])
    live_paths = sorted(live_folder.glob("*.jpg"))
    if len(live_paths) != int(live_payload["expected_image_count"]):
        raise RuntimeError("Stationary live capture count does not match its annotation")
    if tuple(live_payload["rectified_origin_xy"]) != origin:
        raise RuntimeError("Stationary live annotations use a different rectified origin")
    if tuple(live_payload["image_size"]) != roi_size:
        raise RuntimeError("Stationary live annotations use a different output size")
    if float(live_payload["fisheye_balance"]) != balance:
        raise RuntimeError("Stationary live annotations use a different fisheye balance")
    live_center_x, live_center_y, live_radius = (
        float(value) for value in live_payload["stationary_ball_xyr"]
    )
    for path in live_paths:
        frame = cv2.imread(str(path))
        if frame is None:
            raise RuntimeError(f"Cannot read {path}")
        crop = preprocess(frame, processor)
        label = yolo_label(live_center_x, live_center_y, live_radius, roi_size)
        stem = f"live_{path.stem}"
        write_sample(args.output, "train", stem, crop, label)
        for repeat in range(1, args.live_train_repeat):
            write_sample(
                args.output,
                "train",
                f"{stem}_repeat{repeat:02d}",
                crop,
                label,
            )
    report["stationary_live_capture"] = {
        "source_folder": str(live_folder),
        "unique_train_images": len(live_paths),
        "train_repeat": args.live_train_repeat,
        "annotation_xyr": [live_center_x, live_center_y, live_radius],
        "annotation_note": live_payload["annotation_note"],
    }

    if args.extracted_video is not None:
        manifest = load_config(args.extracted_video / "manifest.json")
        manifest_size = manifest.get("output_size", manifest.get("roi_size"))
        if tuple(manifest_size or ()) != roi_size:
            raise RuntimeError(
                "Persisted video frames use a different rectified output; "
                "rerun extract_actual_video_frames.py with the current config"
            )
        manifest_origin = manifest.get("rectified_origin_xy")
        if manifest_origin is not None and tuple(manifest_origin) != origin:
            raise RuntimeError("Persisted video frames use a different rectified origin")
        split_counts = {"train": 0, "val": 0}
        for row in manifest["frames"]:
            split = str(row["split"])
            source_image = args.extracted_video / str(row["image"])
            source_label = args.extracted_video / str(row["label"])
            stem = f"video_{source_image.stem}"
            shutil.copyfile(
                source_image, args.output / "images" / split / f"{stem}.png"
            )
            shutil.copyfile(
                source_label, args.output / "labels" / split / f"{stem}.txt"
            )
            split_counts[split] += 1
        report["video"] = {
            "mode": "persisted_adaptive_frames",
            "source": str(args.extracted_video),
            "selected_train_frames": split_counts["train"],
            "selected_val_frames": split_counts["val"],
            "source_reports": manifest["videos"],
            "sampling_note": (
                "One complete source video is held out for dynamic validation."
            ),
        }
    else:
        video_candidates: dict[
            int, list[tuple[int, np.ndarray, tuple[float, float, float]]]
        ] = defaultdict(list)
        video_total_frames = 0
        video_examined_frames = 0
        video_hough_misses = 0
        video_paths = sorted((args.raw / "videos").glob("*"))
        for video_index, path in enumerate(video_paths):
            capture = cv2.VideoCapture(str(path))
            if not capture.isOpened():
                raise RuntimeError(f"Cannot open {path}")
            frame_index = 0
            while True:
                ok, frame = capture.read()
                if not ok:
                    break
                if frame_index % args.video_stride == 0:
                    video_examined_frames += 1
                    crop = preprocess(frame, processor)
                    circle = detect_circle(crop)
                    if circle is None:
                        video_hough_misses += 1
                    else:
                        position_bin = int(circle[0] // 20)
                        video_candidates[position_bin].append(
                            (video_index * 1_000_000 + frame_index, crop, circle)
                        )
                frame_index += 1
            capture.release()
            video_total_frames += frame_index

        selected_video: list[
            tuple[int, np.ndarray, tuple[float, float, float]]
        ] = []
        for candidates in video_candidates.values():
            count = min(len(candidates), args.video_max_per_position_bin)
            indices = np.linspace(0, len(candidates) - 1, count, dtype=int)
            selected_video.extend(candidates[int(index)] for index in indices)
        selected_video.sort(key=lambda item: item[0])
        for combined_index, crop, circle in selected_video:
            center_x, center_y, radius = circle
            write_sample(
                args.output,
                "train",
                f"video_{combined_index:09d}",
                crop,
                yolo_label(center_x, center_y, radius, roi_size),
            )

        report["video"] = {
            "mode": "legacy_raw_sampling",
            "source_count": len(video_paths),
            "decoded_frames": video_total_frames,
            "examined_frames": video_examined_frames,
            "hough_misses": video_hough_misses,
            "selected_train_frames": len(selected_video),
            "position_bin_count": len(video_candidates),
            "sampling_note": (
                "All video frames stay in train; validation uses held-out photos."
            ),
        }
    report["dataset"] = {
        "train_images": len(list((args.output / "images" / "train").glob("*.png"))),
        "val_images": len(list((args.output / "images" / "val").glob("*.png"))),
    }

    yaml_payload = {
        "path": str(args.output.resolve()),
        "train": "images/train",
        "val": "images/val",
        "names": {0: "steel_ball"},
    }
    (args.output / "data.yaml").write_text(
        yaml.safe_dump(yaml_payload, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )
    (args.output / "report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    draw_preview(args.output, "train")
    draw_preview(args.output, "val")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
