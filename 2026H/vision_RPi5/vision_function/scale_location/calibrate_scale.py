"""Interactively build a piecewise-linear tube scale calibration."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime
import hashlib
import json
from pathlib import Path
import sys

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parent
REPOSITORY_ROOT = ROOT.parents[1]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from vision_function.ball_detection.preprocess import (  # noqa: E402
    TubePreprocessor,
    TubeRectificationConfig,
    load_tube_rectification,
)
from vision_function.scale_location.calibration import (  # noqa: E402
    load_scale_calibration,
)


DEFAULT_RECTIFICATION = (
    ROOT.parent / "ball_detection" / "config" / "tube_local_rectification.json"
)
DEFAULT_OUTPUT = ROOT / "config" / "scale_calibration.json"
DEFAULT_PREVIEW = ROOT / "output" / "scale_calibration_preview.png"
DEFAULT_MARKS = "-12,-9,-6,-3,0,3,6,9,12"
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


@dataclass(frozen=True)
class CalibrationPoint:
    scale_cm: float
    x: float
    y: float


def parse_marks(value: str) -> tuple[float, ...]:
    """Parse a strictly increasing comma-separated scale list."""
    try:
        marks = tuple(float(item.strip()) for item in value.split(",") if item.strip())
    except ValueError as exc:
        raise argparse.ArgumentTypeError("marks must be comma-separated numbers") from exc
    if len(marks) < 2:
        raise argparse.ArgumentTypeError("at least two marks are required")
    if not all(np.isfinite(mark) for mark in marks):
        raise argparse.ArgumentTypeError("marks must be finite")
    if any(left >= right for left, right in zip(marks, marks[1:])):
        raise argparse.ArgumentTypeError("marks must be strictly increasing")
    return marks


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _portable_path(path: Path) -> str:
    """Prefer a repository-relative POSIX path for reproducible metadata."""
    resolved = path.resolve()
    try:
        return resolved.relative_to(REPOSITORY_ROOT).as_posix()
    except ValueError:
        return str(resolved)


def _read_image(path: Path) -> np.ndarray:
    encoded = np.fromfile(path, dtype=np.uint8)
    image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"cannot decode image: {path}")
    return image


def expand_image_paths(inputs: list[Path]) -> list[Path]:
    """Expand image files and non-recursive image directories deterministically."""
    paths: list[Path] = []
    for value in inputs:
        path = value.resolve()
        if path.is_dir():
            paths.extend(
                child
                for child in sorted(path.iterdir())
                if child.is_file() and child.suffix.lower() in IMAGE_SUFFIXES
            )
        elif path.is_file():
            paths.append(path)
        else:
            raise ValueError(f"image path not found: {path}")
    if not paths:
        raise ValueError("no supported images were found")
    return paths


def load_calibration_image(
    paths: list[Path], rectification_path: Path
) -> tuple[np.ndarray, TubeRectificationConfig]:
    """Load images, map raw frames to the production ROI, and take their median."""
    if not paths:
        raise ValueError("at least one source image is required")
    config = load_tube_rectification(rectification_path)
    preprocessor = TubePreprocessor(config)
    local_images: list[np.ndarray] = []
    for path in paths:
        image = _read_image(path)
        size = image.shape[1], image.shape[0]
        if size == config.output_size:
            local = image
        else:
            local = preprocessor(image)
        if (local.shape[1], local.shape[0]) != config.output_size:
            raise ValueError(
                f"unexpected local image size for {path}: "
                f"{local.shape[1]}x{local.shape[0]}"
            )
        local_images.append(local)
    median = np.median(np.stack(local_images), axis=0).astype(np.uint8)
    return median, config


def load_points(path: Path) -> tuple[CalibrationPoint, ...]:
    """Load reproducible non-interactive clicks from a JSON list."""
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, list):
        raise ValueError("points JSON must be a list")
    points = tuple(
        CalibrationPoint(
            scale_cm=float(item["scale_cm"]),
            x=float(item["x"]),
            y=float(item["y"]),
        )
        for item in payload
    )
    _validate_points(points)
    return points


def _validate_points(points: tuple[CalibrationPoint, ...]) -> None:
    if len(points) < 2:
        raise ValueError("at least two calibration points are required")
    values = np.asarray(
        [(point.scale_cm, point.x, point.y) for point in points], dtype=np.float64
    )
    if not np.all(np.isfinite(values)):
        raise ValueError("calibration points must be finite")
    if any(
        left.scale_cm >= right.scale_cm for left, right in zip(points, points[1:])
    ):
        raise ValueError("point scale_cm values must be strictly increasing")


def collect_points(
    image: np.ndarray, marks: tuple[float, ...], display_scale: int
) -> tuple[CalibrationPoint, ...]:
    """Collect one image click for each physical mark using an OpenCV window."""
    if display_scale < 1:
        raise ValueError("display_scale must be positive")
    clicks: list[tuple[float, float]] = []
    zoom = [display_scale]
    notice = [""]
    window = "Scale calibration"

    def current_points() -> tuple[CalibrationPoint, ...]:
        return tuple(
            CalibrationPoint(scale_cm=mark, x=click[0], y=click[1])
            for mark, click in zip(marks, clicks)
        )

    def redraw() -> None:
        scale = zoom[0]
        scaled_image = cv2.resize(
            image,
            None,
            fx=scale,
            fy=scale,
            interpolation=cv2.INTER_NEAREST,
        )
        for index, (x, y) in enumerate(clicks):
            center = (round(x * scale), round(y * scale))
            cv2.drawMarker(
                scaled_image,
                center,
                (0, 255, 255),
                cv2.MARKER_CROSS,
                14,
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                scaled_image,
                f"{marks[index]:g}",
                (center[0] + 6, max(16, center[1] - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 255, 255),
                1,
                cv2.LINE_AA,
            )
        if notice[0]:
            pending = notice[0]
        elif len(clicks) == len(marks):
            pending = "done - Enter to validate"
        else:
            pending = (
                f"click {marks[len(clicks)]:g} cm  "
                f"({len(clicks) + 1}/{len(marks)})"
            )
        footer = np.zeros((30, scaled_image.shape[1], 3), dtype=np.uint8)
        canvas = np.vstack((scaled_image, footer))
        cv2.putText(
            canvas,
            f"{pending}   zoom={scale}x [1/2/3]",
            (7, scaled_image.shape[0] + 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        cv2.imshow(window, canvas)

    def on_mouse(event: int, x: int, y: int, _flags: int, _data: object) -> None:
        scale = zoom[0]
        inside_image = x < image.shape[1] * scale and y < image.shape[0] * scale
        if (
            event == cv2.EVENT_LBUTTONDOWN
            and inside_image
            and len(clicks) < len(marks)
        ):
            clicks.append((x / scale, y / scale))
            notice[0] = ""
            redraw()

    print("Click marks from left to right in the displayed order.")
    print(
        "Keys: Enter=accept, U/Backspace=undo, R=reset, "
        "1/2/3=zoom, Esc=cancel"
    )
    cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(window, on_mouse)
    redraw()
    try:
        while True:
            if cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                raise RuntimeError("calibration window was closed")
            key = cv2.waitKey(20) & 0xFF
            if key in (8, ord("u"), ord("U")):
                if clicks:
                    clicks.pop()
                    notice[0] = ""
                    redraw()
            elif key in (ord("r"), ord("R")):
                clicks.clear()
                notice[0] = ""
                redraw()
            elif key in (ord("1"), ord("2"), ord("3")):
                zoom[0] = int(chr(key))
                redraw()
            elif key in (10, 13):
                if len(clicks) == len(marks):
                    try:
                        fit_axis(current_points())
                    except ValueError as exc:
                        notice[0] = "invalid order - press U to undo incorrect marks"
                        print(f"Cannot accept clicks: {exc}")
                        redraw()
                        continue
                    else:
                        break
                print(f"Need {len(marks) - len(clicks)} more click(s).")
            elif key == 27:
                raise KeyboardInterrupt("calibration cancelled")
    finally:
        cv2.destroyWindow(window)
    points = current_points()
    _validate_points(points)
    return points


def fit_axis(
    points: tuple[CalibrationPoint, ...]
) -> tuple[tuple[float, float], tuple[float, float], tuple[float, ...]]:
    """Fit the tube axis and project clicks into strictly increasing coordinates."""
    _validate_points(points)
    xy = np.asarray([(point.x, point.y) for point in points], dtype=np.float64)
    centered = xy - np.mean(xy, axis=0)
    covariance = centered.T @ centered
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    if float(eigenvalues[-1]) <= 1e-9:
        raise ValueError("clicked points do not define a usable tube axis")
    direction = eigenvectors[:, -1]
    if direction[0] < 0:
        direction = -direction

    zero_points = [point for point in points if abs(point.scale_cm) <= 1e-12]
    if zero_points:
        origin = np.asarray((zero_points[0].x, zero_points[0].y), dtype=np.float64)
    else:
        origin = np.mean(xy, axis=0)
    axis_values = tuple(float((position - origin) @ direction) for position in xy)
    violation = next(
        (
            index
            for index, (left, right) in enumerate(
                zip(axis_values, axis_values[1:])
            )
            if left >= right
        ),
        None,
    )
    if violation is not None:
        left_point = points[violation]
        right_point = points[violation + 1]
        raise ValueError(
            f"{left_point.scale_cm:g} cm at ({left_point.x:.1f}, "
            f"{left_point.y:.1f}) projects to {axis_values[violation]:.1f} px, "
            f"but {right_point.scale_cm:g} cm at ({right_point.x:.1f}, "
            f"{right_point.y:.1f}) projects to "
            f"{axis_values[violation + 1]:.1f} px; clicks must move left to right"
        )
    return (
        (float(origin[0]), float(origin[1])),
        (float(direction[0]), float(direction[1])),
        axis_values,
    )


def build_payload(
    points: tuple[CalibrationPoint, ...],
    config: TubeRectificationConfig,
    source_paths: list[Path],
    rectification_path: Path,
    valid_range_cm: tuple[float, float] | None,
    edge_margin_cm: float,
) -> dict[str, object]:
    """Build a schema-v1 final calibration payload accepted by the runtime."""
    if not np.isfinite(edge_margin_cm) or edge_margin_cm < 0:
        raise ValueError("edge_margin_cm must be finite and non-negative")
    width, height = config.output_size
    outside = [
        point
        for point in points
        if not (0 <= point.x < width and 0 <= point.y < height)
    ]
    if outside:
        point = outside[0]
        raise ValueError(
            f"point {point.scale_cm:g} cm is outside the {width}x{height} local image"
        )
    origin, direction, axis_values = fit_axis(points)
    mark_range = points[0].scale_cm, points[-1].scale_cm
    valid_range = valid_range_cm or mark_range
    if (
        not all(np.isfinite(value) for value in valid_range)
        or valid_range[0] >= valid_range[1]
        or valid_range[0] < mark_range[0]
        or valid_range[1] > mark_range[1]
    ):
        raise ValueError("valid range must be increasing and covered by clicked marks")

    return {
        "schema_version": 1,
        "calibration_type": "multi_mark_piecewise_linear",
        "status": "final",
        "coordinate_space": config.coordinate_space,
        "image_size": list(config.output_size),
        "positive_direction": "image_right",
        "axis_origin_xy": [round(value, 6) for value in origin],
        "axis_direction_xy": [round(value, 9) for value in direction],
        "anchors": [
            {
                "axis_px": round(axis_px, 6),
                "scale_cm": point.scale_cm,
            }
            for point, axis_px in zip(points, axis_values)
        ],
        "valid_range_cm": list(valid_range),
        "edge_margin_cm": edge_margin_cm,
        "measured_points_local_xy": [
            {"scale_cm": point.scale_cm, "xy": [point.x, point.y]}
            for point in points
        ],
        "source": {
            "method": (
                "single_frame_manual_marks"
                if len(source_paths) == 1
                else f"median_of_{len(source_paths)}_frames_then_manual_marks"
            ),
            "generated_at": datetime.now().astimezone().isoformat(timespec="seconds"),
            "rectification_config": _portable_path(rectification_path),
            "rectification_sha256": _sha256(rectification_path),
            "images": [
                {"path": _portable_path(path), "sha256": _sha256(path)}
                for path in source_paths
            ],
        },
    }


def render_preview(
    image: np.ndarray,
    points: tuple[CalibrationPoint, ...],
    origin: tuple[float, float],
    direction: tuple[float, float],
    scale: int = 3,
) -> np.ndarray:
    """Render clicked marks and the fitted axis at a readable scale."""
    canvas = cv2.resize(
        image, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST
    )
    origin_array = np.asarray(origin)
    direction_array = np.asarray(direction)
    start = origin_array - direction_array * 2000
    end = origin_array + direction_array * 2000
    cv2.line(
        canvas,
        tuple(np.rint(start * scale).astype(int)),
        tuple(np.rint(end * scale).astype(int)),
        (255, 255, 0),
        1,
        cv2.LINE_AA,
    )
    for point in points:
        center = (round(point.x * scale), round(point.y * scale))
        cv2.circle(canvas, center, 5, (0, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(
            canvas,
            f"{point.scale_cm:+g} cm",
            (center[0] + 7, max(16, center[1] - 7)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )
    return canvas


def _write_image(path: Path, image: np.ndarray) -> None:
    suffix = path.suffix or ".png"
    ok, encoded = cv2.imencode(suffix, image)
    if not ok:
        raise ValueError(f"unsupported preview image extension: {suffix}")
    encoded.tofile(path)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "images", nargs="+", type=Path, help="raw/640x100 images or image directories"
    )
    parser.add_argument("--rectification", type=Path, default=DEFAULT_RECTIFICATION)
    parser.add_argument("--marks", type=parse_marks, default=parse_marks(DEFAULT_MARKS))
    parser.add_argument(
        "--points",
        type=Path,
        help="JSON click list for reproducible non-interactive generation",
    )
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--preview", type=Path, default=DEFAULT_PREVIEW)
    parser.add_argument(
        "--display-scale",
        type=int,
        choices=(1, 2, 3),
        default=2,
        help="initial GUI zoom; press 1/2/3 to change it while clicking",
    )
    parser.add_argument("--valid-range", nargs=2, type=float, metavar=("MIN", "MAX"))
    parser.add_argument("--edge-margin-cm", type=float, default=0.75)
    parser.add_argument("--force", action="store_true", help="overwrite output files")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        paths = expand_image_paths(args.images)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc
    rectification_path = args.rectification.resolve()
    output_path = args.output.resolve()
    preview_path = args.preview.resolve()
    for path in [*paths, rectification_path]:
        if not path.is_file():
            raise SystemExit(f"file not found: {path}")
    if args.points and not args.points.is_file():
        raise SystemExit(f"file not found: {args.points}")
    existing = [path for path in (output_path, preview_path) if path.exists()]
    if existing and not args.force:
        names = ", ".join(str(path) for path in existing)
        raise SystemExit(f"refusing to overwrite {names}; pass --force to replace")

    image, config = load_calibration_image(paths, rectification_path)
    if args.points:
        points = load_points(args.points)
    else:
        points = collect_points(image, args.marks, args.display_scale)
    valid_range = tuple(args.valid_range) if args.valid_range else None
    payload = build_payload(
        points,
        config,
        paths,
        rectification_path,
        valid_range,
        args.edge_margin_cm,
    )
    origin = tuple(payload["axis_origin_xy"])
    direction = tuple(payload["axis_direction_xy"])
    preview = render_preview(image, points, origin, direction)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    preview_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    _write_image(preview_path, preview)
    load_scale_calibration(output_path)
    print(f"calibration={output_path}")
    print(f"preview={preview_path}")
    print(f"anchors={len(points)} valid_range_cm={payload['valid_range_cm']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
