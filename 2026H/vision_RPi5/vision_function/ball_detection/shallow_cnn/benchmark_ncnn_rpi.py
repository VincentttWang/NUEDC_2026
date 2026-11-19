"""Benchmark the shallow NCNN localizer on Raspberry Pi static and live inputs."""

from __future__ import annotations

import argparse
import http.client
import json
import os
import platform
import subprocess
import sys
from datetime import datetime
from pathlib import Path
from time import perf_counter, time
from urllib.parse import urlsplit

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parent
REPOSITORY_ROOT = Path(
    os.environ.get(
        "NUEDC_REPOSITORY_ROOT",
        ROOT.parents[2] if len(ROOT.parents) > 2 else ROOT,
    )
)
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from vision_function.ball_detection.preprocess import (  # noqa: E402
    TubePreprocessor,
    load_tube_rectification,
)
try:
    from vision_function.ball_detection.shallow_cnn.ncnn_runtime import (  # noqa: E402
        DEFAULT_MODEL,
        ShallowCNNNCNN,
    )
except ModuleNotFoundError:
    # Allow the benchmark and runtime to be copied into an isolated remote folder.
    from ncnn_runtime import DEFAULT_MODEL, ShallowCNNNCNN  # type: ignore[no-redef]


DEFAULT_DATA = ROOT.parent / "data" / "actual_yolo_local_v3" / "images" / "val"
DEFAULT_ROI = ROOT.parent / "config" / "tube_local_rectification.json"
DEFAULT_OUTPUT = ROOT.parent / "output" / "shallow_cnn_v1" / "rpi_benchmark"


def summarize(values: list[float]) -> dict[str, float | int | None]:
    if not values:
        return {
            "count": 0,
            "mean": None,
            "p50": None,
            "p90": None,
            "p95": None,
            "p99": None,
            "minimum": None,
            "maximum": None,
        }
    samples = np.asarray(values, dtype=np.float64)
    return {
        "count": len(values),
        "mean": round(float(np.mean(samples)), 4),
        "p50": round(float(np.percentile(samples, 50)), 4),
        "p90": round(float(np.percentile(samples, 90)), 4),
        "p95": round(float(np.percentile(samples, 95)), 4),
        "p99": round(float(np.percentile(samples, 99)), 4),
        "minimum": round(float(np.min(samples)), 4),
        "maximum": round(float(np.max(samples)), 4),
    }


def command_output(command: list[str]) -> str | None:
    try:
        result = subprocess.run(
            command, check=False, capture_output=True, text=True, timeout=3
        )
    except (FileNotFoundError, subprocess.SubprocessError):
        return None
    output = result.stdout.strip()
    return output or None


def thermal_state() -> dict[str, str | None]:
    return {
        "temperature": command_output(["vcgencmd", "measure_temp"]),
        "throttled": command_output(["vcgencmd", "get_throttled"]),
    }


def service_state() -> dict[str, str | None]:
    return {
        name: command_output(["systemctl", "is-active", name])
        for name in ("ustreamer.service", "vision-scale-uart.service")
    }


def platform_info() -> dict[str, object]:
    try:
        import ncnn

        ncnn_version = getattr(ncnn, "__version__", None)
    except ImportError:
        ncnn_version = None
    return {
        "hostname": platform.node(),
        "platform": platform.platform(),
        "machine": platform.machine(),
        "python": platform.python_version(),
        "opencv": cv2.__version__,
        "ncnn": ncnn_version,
        "logical_cpu_count": os.cpu_count(),
    }


def max_rss_kib() -> int | None:
    try:
        import resource

        return int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)
    except ImportError:
        return None


def save_report(payload: dict[str, object], output: Path, label: str) -> Path:
    output.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().astimezone().strftime("%Y%m%dT%H%M%S%z")
    path = output / f"{label}_{timestamp}.json"
    text = json.dumps(payload, ensure_ascii=False, indent=2) + "\n"
    path.write_text(text, encoding="utf-8")
    print(text, end="")
    print(f"report={path}")
    return path


def load_static_roi(path: Path) -> tuple[np.ndarray, Path]:
    image_path = path
    if path.is_dir():
        candidates = sorted(path.glob("*.png"))
        if not candidates:
            raise RuntimeError(f"No PNG images found in {path}")
        image_path = candidates[0]
    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f"Cannot decode {image_path}")
    if image.shape != (100, 640, 3):
        raise ValueError(f"Static input must be a 640x100 ROI, got {image.shape}")
    return image, image_path


def run_static(args: argparse.Namespace) -> int:
    cv2.setNumThreads(args.opencv_threads)
    roi, image_path = load_static_roi(args.input)
    started = perf_counter()
    model = ShallowCNNNCNN(
        args.model,
        threads=args.ncnn_threads,
        use_fp16=args.precision == "fp16",
    )
    model_load_ms = (perf_counter() - started) * 1000.0
    prepared = model.prepare_roi(roi)

    for _ in range(args.warmup):
        model.infer_prepared(prepared)
        model.predict(roi)

    inference_ms: list[float] = []
    complete_ms: list[float] = []
    decode_ms: list[float] = []
    prediction = None
    thermal_start = thermal_state()
    for _ in range(args.iterations):
        started = perf_counter()
        raw = model.infer_prepared(prepared)
        inference_ms.append((perf_counter() - started) * 1000.0)

        started = perf_counter()
        prediction = model.decode(*raw)
        decode_ms.append((perf_counter() - started) * 1000.0)

        started = perf_counter()
        prediction = model.predict(roi)
        complete_ms.append((perf_counter() - started) * 1000.0)

    assert prediction is not None
    report = {
        "kind": "shallow_cnn_static",
        "platform": platform_info(),
        "model": str(args.model),
        "model_bytes": {
            "param": (args.model / "model.ncnn.param").stat().st_size,
            "bin": (args.model / "model.ncnn.bin").stat().st_size,
        },
        "model_load_ms": round(model_load_ms, 4),
        "input": str(image_path),
        "input_shape_hwc": list(roi.shape),
        "input_resized": False,
        "ncnn_threads": args.ncnn_threads,
        "ncnn_precision": args.precision,
        "opencv_threads": args.opencv_threads,
        "warmup_iterations": args.warmup,
        "measured_iterations": args.iterations,
        "timing_ms": {
            "inference_only_prepared_chw": summarize(inference_ms),
            "coordinate_decode_only": summarize(decode_ms),
            "bgr_to_rgb_chw_inference_decode": summarize(complete_ms),
        },
        "equivalent_fps_from_complete_p95": round(
            1000.0 / float(np.percentile(complete_ms, 95)), 3
        ),
        "last_prediction": {
            "found": prediction.found,
            "x_local": prediction.x_local,
            "presence_probability": prediction.presence_probability,
            "heatmap_peak": prediction.heatmap_peak,
        },
        "services": service_state(),
        "thermal_start": thermal_start,
        "thermal_end": thermal_state(),
        "max_rss_kib": max_rss_kib(),
    }
    save_report(report, args.output, f"static_t{args.ncnn_threads}")
    return 0


class SnapshotClient:
    def __init__(self, url: str, timeout: float) -> None:
        parsed = urlsplit(url)
        if parsed.scheme != "http" or not parsed.hostname:
            raise ValueError("Only an http:// snapshot URL is supported")
        self.host = parsed.hostname
        self.port = parsed.port or 80
        self.path = parsed.path or "/"
        if parsed.query:
            self.path += f"?{parsed.query}"
        self.timeout = timeout
        self.connection: http.client.HTTPConnection | None = None

    def fetch(self) -> tuple[bytes, float | None]:
        if self.connection is None:
            self.connection = http.client.HTTPConnection(
                self.host, self.port, timeout=self.timeout
            )
        try:
            self.connection.request("GET", self.path)
            response = self.connection.getresponse()
            if response.status != 200:
                raise RuntimeError(f"Snapshot HTTP status is {response.status}")
            timestamp = response.getheader("X-Timestamp")
            return response.read(), float(timestamp) if timestamp is not None else None
        except (OSError, http.client.HTTPException):
            self.close()
            raise

    def close(self) -> None:
        if self.connection is not None:
            self.connection.close()
            self.connection = None


def fetch_distinct(
    client: SnapshotClient, previous_timestamp: float | None
) -> tuple[bytes, float | None, int]:
    requests = 0
    while True:
        encoded, timestamp = client.fetch()
        requests += 1
        if (
            previous_timestamp is None
            or timestamp is None
            or timestamp != previous_timestamp
        ):
            return encoded, timestamp, requests


def run_camera(args: argparse.Namespace) -> int:
    cv2.setNumThreads(args.opencv_threads)
    config = load_tube_rectification(args.roi)
    preprocessor = TubePreprocessor(config)
    model = ShallowCNNNCNN(
        args.model,
        threads=args.ncnn_threads,
        use_fp16=args.precision == "fp16",
    )
    client = SnapshotClient(args.url, args.timeout)
    previous_timestamp: float | None = None

    try:
        for _ in range(args.warmup):
            encoded, previous_timestamp, _requests = fetch_distinct(
                client, previous_timestamp
            )
            frame = cv2.imdecode(
                np.frombuffer(encoded, dtype=np.uint8), cv2.IMREAD_REDUCED_COLOR_2
            )
            if frame is None:
                raise RuntimeError("Cannot decode camera snapshot")
            model.predict(preprocessor(frame))

        acquire_ms: list[float] = []
        decode_ms: list[float] = []
        rectify_roi_ms: list[float] = []
        model_prepare_ms: list[float] = []
        inference_ms: list[float] = []
        coordinate_decode_ms: list[float] = []
        control_total_ms: list[float] = []
        frame_to_result_ms: list[float] = []
        frame_age_at_acquire_ms: list[float] = []
        request_counts: list[int] = []
        input_sizes: list[float] = []
        positions: list[float] = []
        found_count = 0
        frame_shape = None
        roi_shape = None
        thermal_start = thermal_state()

        for _ in range(args.iterations):
            total_started = perf_counter()

            started = perf_counter()
            encoded, frame_timestamp, requests = fetch_distinct(
                client, previous_timestamp
            )
            previous_timestamp = frame_timestamp
            acquire_ms.append((perf_counter() - started) * 1000.0)
            request_counts.append(requests)
            input_sizes.append(float(len(encoded)))
            if frame_timestamp is not None:
                frame_age_at_acquire_ms.append((time() - frame_timestamp) * 1000.0)

            started = perf_counter()
            frame = cv2.imdecode(
                np.frombuffer(encoded, dtype=np.uint8), cv2.IMREAD_REDUCED_COLOR_2
            )
            decode_ms.append((perf_counter() - started) * 1000.0)
            if frame is None:
                raise RuntimeError("Cannot decode camera snapshot")
            frame_shape = list(frame.shape)

            started = perf_counter()
            roi = preprocessor(frame)
            rectify_roi_ms.append((perf_counter() - started) * 1000.0)
            roi_shape = list(roi.shape)

            started = perf_counter()
            prepared = model.prepare_roi(roi)
            model_prepare_ms.append((perf_counter() - started) * 1000.0)

            started = perf_counter()
            raw = model.infer_prepared(prepared)
            inference_ms.append((perf_counter() - started) * 1000.0)

            started = perf_counter()
            prediction = model.decode(*raw)
            coordinate_decode_ms.append((perf_counter() - started) * 1000.0)
            found_count += int(prediction.found)
            if prediction.x_local is not None:
                positions.append(prediction.x_local)

            control_total_ms.append((perf_counter() - total_started) * 1000.0)
            if frame_timestamp is not None:
                frame_to_result_ms.append((time() - frame_timestamp) * 1000.0)
    finally:
        client.close()

    report = {
        "kind": "shallow_cnn_live_camera",
        "platform": platform_info(),
        "model": str(args.model),
        "source_url": args.url,
        "source": "snapshot_distinct_frames",
        "decode_mode": "IMREAD_REDUCED_COLOR_2",
        "input_frame_shape_hwc": frame_shape,
        "rectified_roi_shape_hwc": roi_shape,
        "model_input_resized": False,
        "ncnn_threads": args.ncnn_threads,
        "ncnn_precision": args.precision,
        "opencv_threads": args.opencv_threads,
        "warmup_iterations": args.warmup,
        "measured_distinct_frames": args.iterations,
        "timing_ms": {
            "acquire_distinct_snapshot": summarize(acquire_ms),
            "frame_age_at_acquire_end": summarize(frame_age_at_acquire_ms),
            "jpeg_half_decode": summarize(decode_ms),
            "fisheye_rectify_roi": summarize(rectify_roi_ms),
            "bgr_to_rgb_chw": summarize(model_prepare_ms),
            "ncnn_inference": summarize(inference_ms),
            "coordinate_decode": summarize(coordinate_decode_ms),
            "camera_to_coordinate_control_total": summarize(control_total_ms),
            "frame_timestamp_to_coordinate": summarize(frame_to_result_ms),
        },
        "equivalent_fps_from_control_p95": round(
            1000.0 / float(np.percentile(control_total_ms, 95)), 3
        ),
        "snapshot_requests_per_distinct_frame": summarize(
            [float(value) for value in request_counts]
        ),
        "input_jpeg_bytes": summarize(input_sizes),
        "frames_found": found_count,
        "x_local": summarize(positions),
        "services": service_state(),
        "thermal_start": thermal_start,
        "thermal_end": thermal_state(),
        "max_rss_kib": max_rss_kib(),
        "note": (
            "The existing production service was not stopped or modified. Its CPU load is "
            "therefore included when it is active."
        ),
    }
    save_report(report, args.output, f"camera_t{args.ncnn_threads}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    static = subparsers.add_parser("static", help="Benchmark a prepared ROI")
    static.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    static.add_argument("--input", type=Path, default=DEFAULT_DATA)
    static.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    static.add_argument("--ncnn-threads", type=int, default=2)
    static.add_argument("--precision", choices=("fp16", "fp32"), default="fp16")
    static.add_argument("--opencv-threads", type=int, default=1)
    static.add_argument("--warmup", type=int, default=30)
    static.add_argument("--iterations", type=int, default=500)
    static.set_defaults(function=run_static)

    camera = subparsers.add_parser("camera", help="Benchmark distinct live snapshots")
    camera.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    camera.add_argument("--roi", type=Path, default=DEFAULT_ROI)
    camera.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    camera.add_argument("--url", default="http://127.0.0.1:8080/snapshot")
    camera.add_argument("--timeout", type=float, default=2.0)
    camera.add_argument("--ncnn-threads", type=int, default=2)
    camera.add_argument("--precision", choices=("fp16", "fp32"), default="fp16")
    camera.add_argument("--opencv-threads", type=int, default=1)
    camera.add_argument("--warmup", type=int, default=10)
    camera.add_argument("--iterations", type=int, default=130)
    camera.set_defaults(function=run_camera)
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    if args.ncnn_threads < 1 or args.opencv_threads < 1:
        parser.error("thread counts must be positive")
    if args.warmup < 0 or args.iterations < 1:
        parser.error("warmup must be non-negative and iterations positive")
    return int(args.function(args))


if __name__ == "__main__":
    raise SystemExit(main())
