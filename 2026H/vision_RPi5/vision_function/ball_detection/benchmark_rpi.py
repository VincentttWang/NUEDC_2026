"""Raspberry Pi benchmarks for steel-ball inference and the live camera pipeline."""

from __future__ import annotations

import argparse
import http.client
import json
import os
import platform
import resource
import statistics
import subprocess
import sys
from datetime import datetime
from pathlib import Path
from time import perf_counter, time
from urllib.parse import urlsplit

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parent
REPOSITORY_ROOT = ROOT.parents[1]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from vision_function.fisheye import FisheyeUndistorter  # noqa: E402


DEFAULT_DATA = ROOT / "data" / "raw" / "photos"
DEFAULT_MODEL = ROOT / "model" / "actual_yolo11n_local_v3_ncnn_model"
DEFAULT_ROI = ROOT / "config" / "tube_local_rectification.json"
DEFAULT_OUTPUT = ROOT / "output" / "rpi_benchmark"
DEFAULT_CONFIDENCE = 0.25

JPEG_DECODE_FLAGS = {
    1: cv2.IMREAD_COLOR,
    2: cv2.IMREAD_REDUCED_COLOR_2,
}


def load_roi(path: Path) -> tuple[int, int, int, int]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    roi = payload.get(
        "reference_rectified_region",
        payload.get("reference_roi", payload.get("roi")),
    )
    if roi is None:
        raise ValueError(
            f"ROI config has no reference_rectified_region/reference_roi: {path}"
        )
    left, top = int(roi["left"]), int(roi["top"])
    right = int(roi["right"]) if "right" in roi else left + int(roi["width"])
    bottom = int(roi["bottom"]) if "bottom" in roi else top + int(roi["height"])
    return left, top, right, bottom


def load_fisheye_balance(path: Path) -> float:
    payload = json.loads(path.read_text(encoding="utf-8"))
    balance = float(payload.get("fisheye_balance", 0.0))
    if not 0.0 <= balance <= 1.0:
        raise ValueError(f"fisheye_balance must be in [0, 1], got {balance}")
    return balance


def scale_roi(
    roi_xyxy: tuple[int, int, int, int], divisor: int
) -> tuple[int, int, int, int]:
    """Scale calibrated coordinates for an integer JPEG decode reduction."""
    if divisor not in JPEG_DECODE_FLAGS:
        raise ValueError(f"Unsupported decode scale: {divisor}")
    if any(value % divisor for value in roi_xyxy):
        raise ValueError(f"ROI {roi_xyxy} is not exactly divisible by {divisor}")
    return tuple(value // divisor for value in roi_xyxy)


def summarize(values: list[float]) -> dict[str, float | int | None]:
    if not values:
        return {"count": 0, "mean": None, "p50": None, "p90": None, "p95": None,
                "p99": None, "minimum": None, "maximum": None}
    samples = np.asarray(values, dtype=np.float64)
    return {
        "count": len(values),
        "mean": round(float(np.mean(samples)), 3),
        "p50": round(float(np.percentile(samples, 50)), 3),
        "p90": round(float(np.percentile(samples, 90)), 3),
        "p95": round(float(np.percentile(samples, 95)), 3),
        "p99": round(float(np.percentile(samples, 99)), 3),
        "minimum": round(float(np.min(samples)), 3),
        "maximum": round(float(np.max(samples)), 3),
    }


def command_output(command: list[str]) -> str | None:
    try:
        result = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=3,
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


def platform_info() -> dict[str, object]:
    return {
        "hostname": platform.node(),
        "platform": platform.platform(),
        "machine": platform.machine(),
        "python": platform.python_version(),
        "opencv": cv2.__version__,
        "logical_cpu_count": os.cpu_count(),
    }


def configure_ncnn_threads(
    model: object, model_path: Path, threads: int | None
) -> int | None:
    """Set and report NCNN threads, reloading weights when an override is requested."""
    predictor = getattr(model, "predictor", None)
    backend = getattr(predictor, "model", None)
    net = getattr(backend, "net", None)
    options = getattr(net, "opt", None)
    if options is None or not hasattr(options, "num_threads"):
        return None
    if threads is not None and int(options.num_threads) != threads:
        param_path = model_path
        if not param_path.is_file():
            param_path = next(param_path.glob("*.param"))
        net.clear()
        options.num_threads = threads
        net.load_param(str(param_path))
        net.load_model(str(param_path.with_suffix(".bin")))
    return int(options.num_threads)


def save_report(payload: dict[str, object], output: Path, label: str) -> Path:
    output.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().astimezone().strftime("%Y%m%dT%H%M%S%z")
    path = output / f"{label}_{timestamp}.json"
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    print(f"report={path}")
    return path


def load_images(path: Path) -> list[np.ndarray]:
    paths = sorted(path.glob("*.png"))
    if not paths:
        raise RuntimeError(f"No PNG files found in {path}")
    images = []
    for image_path in paths:
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f"Cannot decode {image_path}")
        images.append(image)
    return images


def run_static(args: argparse.Namespace) -> int:
    import torch
    from ultralytics import YOLO

    torch.set_num_threads(args.threads)
    try:
        torch.set_num_interop_threads(1)
    except RuntimeError:
        pass
    cv2.setNumThreads(args.opencv_threads)

    images = load_images(args.data)
    reference_size = list(reversed(images[0].shape[:2]))
    reference_roi = load_roi(args.roi)
    left, top, right, bottom = scale_roi(reference_roi, args.source_scale)
    if args.source_scale != 1:
        images = [
            cv2.resize(
                image,
                (image.shape[1] // args.source_scale, image.shape[0] // args.source_scale),
                interpolation=cv2.INTER_AREA,
            )
            for image in images
        ]
    if args.input == "roi":
        undistorter = FisheyeUndistorter(
            balance=load_fisheye_balance(args.roi)
        )
        inputs = [
            undistorter.undistort_rectified_region(
                image,
                (left, top),
                (right - left, bottom - top),
            )
            for image in images
        ]
    else:
        inputs = images

    started = perf_counter()
    model = YOLO(str(args.model), task="detect")
    model_load_ms = (perf_counter() - started) * 1000

    ncnn_threads = None
    for index in range(args.warmup):
        model.predict(
            inputs[index % len(inputs)],
            imgsz=args.imgsz,
            conf=args.conf,
            device="cpu",
            verbose=False,
        )
        if index == 0:
            ncnn_threads = configure_ncnn_threads(model, args.model, args.ncnn_threads)

    wall_ms: list[float] = []
    preprocess_ms: list[float] = []
    inference_ms: list[float] = []
    postprocess_ms: list[float] = []
    confidences: list[float] = []
    detection_counts: list[int] = []
    thermal_start = thermal_state()
    for index in range(args.iterations):
        frame = inputs[index % len(inputs)]
        started = perf_counter()
        result = model.predict(
            frame,
            imgsz=args.imgsz,
            conf=args.conf,
            device="cpu",
            verbose=False,
        )[0]
        wall_ms.append((perf_counter() - started) * 1000)
        preprocess_ms.append(float(result.speed.get("preprocess", 0.0)))
        inference_ms.append(float(result.speed.get("inference", 0.0)))
        postprocess_ms.append(float(result.speed.get("postprocess", 0.0)))
        count = 0 if result.boxes is None else len(result.boxes)
        detection_counts.append(count)
        if count:
            confidences.extend(float(value) for value in result.boxes.conf.cpu().tolist())

    report = {
        "kind": "static_yolo_batch1",
        "platform": platform_info(),
        "torch": torch.__version__,
        "ultralytics_model": str(args.model),
        "inference_backend": "ncnn" if ncnn_threads is not None else "pytorch",
        "model_load_ms": round(model_load_ms, 3),
        "input": args.input,
        "source_image_count": len(images),
        "reference_image_size": reference_size,
        "source_scale": args.source_scale,
        "source_image_size": list(reversed(images[0].shape[:2])),
        "reference_roi_xyxy": list(reference_roi),
        "roi_xyxy": [left, top, right, bottom],
        "imgsz": args.imgsz,
        "confidence_threshold": args.conf,
        "warmup_iterations": args.warmup,
        "measured_iterations": args.iterations,
        "torch_threads": args.threads if ncnn_threads is None else None,
        "ncnn_threads": ncnn_threads,
        "opencv_threads": args.opencv_threads,
        "timing_ms": {
            "predict_wall": summarize(wall_ms),
            "preprocess": summarize(preprocess_ms),
            "inference": summarize(inference_ms),
            "postprocess": summarize(postprocess_ms),
        },
        "equivalent_fps_from_wall_p95": round(1000 / float(np.percentile(wall_ms, 95)), 3),
        "frames_with_detection": sum(count > 0 for count in detection_counts),
        "total_detections": sum(detection_counts),
        "mean_detection_confidence": (
            round(statistics.mean(confidences), 6) if confidences else None
        ),
        "thermal_start": thermal_start,
        "thermal_end": thermal_state(),
        "max_rss_kib": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
    }
    save_report(report, args.output, f"static_{args.input}_{args.imgsz}")
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


class MjpegStreamClient:
    """Read uStreamer multipart frames serially without a background queue."""

    def __init__(self, url: str, timeout: float) -> None:
        parsed = urlsplit(url)
        if parsed.scheme != "http" or not parsed.hostname:
            raise ValueError("Only an http:// stream URL is supported")
        self.host = parsed.hostname
        self.port = parsed.port or 80
        self.path = parsed.path or "/"
        if parsed.query:
            self.path += f"?{parsed.query}"
        self.timeout = timeout
        self.connection: http.client.HTTPConnection | None = None
        self.response: http.client.HTTPResponse | None = None

    def _connect(self) -> None:
        self.connection = http.client.HTTPConnection(
            self.host, self.port, timeout=self.timeout
        )
        self.connection.request("GET", self.path)
        self.response = self.connection.getresponse()
        if self.response.status != 200:
            raise RuntimeError(f"MJPEG stream HTTP status is {self.response.status}")

    def fetch(self) -> tuple[bytes, float | None]:
        if self.response is None:
            self._connect()
        assert self.response is not None
        try:
            boundary = self.response.fp.readline()
            while boundary in (b"\r\n", b"\n"):
                boundary = self.response.fp.readline()
            if not boundary.startswith(b"--"):
                raise RuntimeError(f"Invalid MJPEG boundary: {boundary[:80]!r}")

            headers: dict[str, str] = {}
            while True:
                line = self.response.fp.readline()
                if line in (b"\r\n", b"\n"):
                    break
                if not line:
                    raise RuntimeError("MJPEG stream ended while reading headers")
                name, separator, value = line.decode("latin-1").partition(":")
                if not separator:
                    raise RuntimeError(f"Invalid MJPEG part header: {line[:80]!r}")
                headers[name.strip().lower()] = value.strip()

            length = int(headers["content-length"])
            encoded = self.response.read(length)
            if len(encoded) != length:
                raise RuntimeError("MJPEG stream ended while reading JPEG payload")
            timestamp = headers.get("x-timestamp")
            return encoded, float(timestamp) if timestamp is not None else None
        except (OSError, http.client.HTTPException):
            self.close()
            raise

    def close(self) -> None:
        if self.connection is not None:
            self.connection.close()
        self.connection = None
        self.response = None


def run_camera(args: argparse.Namespace) -> int:
    cv2.setNumThreads(args.opencv_threads)
    reference_roi = load_roi(args.roi)
    left, top, right, bottom = scale_roi(reference_roi, args.decode_scale)
    if args.remap_mode == "full" and (
        right > 1280 // args.decode_scale
        or bottom > 720 // args.decode_scale
    ):
        raise ValueError(
            "The tube-local region extends beyond the nominal full-frame canvas; "
            "use --remap-mode roi"
        )
    decode_flag = JPEG_DECODE_FLAGS[args.decode_scale]
    source_url = args.url
    if args.source == "stream" and source_url.endswith("/snapshot"):
        source_url = f"{source_url[:-len('/snapshot')]}/stream"
    client = (
        SnapshotClient(source_url, args.timeout)
        if args.source == "snapshot"
        else MjpegStreamClient(source_url, args.timeout)
    )
    fisheye_balance = load_fisheye_balance(args.roi)
    undistorter = FisheyeUndistorter(balance=fisheye_balance)
    model = None
    model_load_ms = None
    if args.with_yolo:
        import torch
        from ultralytics import YOLO

        torch.set_num_threads(args.threads)
        try:
            torch.set_num_interop_threads(1)
        except RuntimeError:
            pass
        started = perf_counter()
        model = YOLO(str(args.model), task="detect")
        model_load_ms = (perf_counter() - started) * 1000

    try:
        # Warm up HTTP, JPEG decoding, map construction, remap and JPEG encoding.
        map_build_ms = None
        ncnn_threads = None
        for index in range(args.warmup):
            encoded, _frame_timestamp = client.fetch()
            frame = cv2.imdecode(np.frombuffer(encoded, dtype=np.uint8), decode_flag)
            if frame is None:
                raise RuntimeError("Cannot decode camera snapshot")
            started = perf_counter()
            if args.remap_mode == "roi":
                roi = undistorter.undistort_rectified_region(
                    frame, (left, top), (right - left, bottom - top)
                )
                diagnostic_frame = roi
            else:
                corrected = undistorter.undistort(frame)
                roi = corrected[top:bottom, left:right].copy()
                diagnostic_frame = corrected
            elapsed_ms = (perf_counter() - started) * 1000
            if index == 0:
                map_build_ms = elapsed_ms
            if model is not None:
                model.predict(
                    roi,
                    imgsz=args.imgsz,
                    conf=args.conf,
                    device="cpu",
                    verbose=False,
                )
                if index == 0:
                    ncnn_threads = configure_ncnn_threads(model, args.model, args.ncnn_threads)
            cv2.imencode(
                ".jpg", diagnostic_frame, [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality]
            )

        acquire_ms: list[float] = []
        frame_age_at_acquire_ms: list[float] = []
        frame_to_control_result_ms: list[float] = []
        frame_to_total_result_ms: list[float] = []
        decode_ms: list[float] = []
        undistort_ms: list[float] = []
        crop_ms: list[float] = []
        encode_ms: list[float] = []
        control_total_ms: list[float] = []
        total_ms: list[float] = []
        yolo_wall_ms: list[float] = []
        yolo_inference_ms: list[float] = []
        detection_counts: list[int] = []
        input_sizes: list[int] = []
        output_sizes: list[int] = []
        frame_size = None
        thermal_start = thermal_state()

        for _ in range(args.iterations):
            total_started = perf_counter()
            started = perf_counter()
            encoded, frame_timestamp = client.fetch()
            acquire_ms.append((perf_counter() - started) * 1000)
            if frame_timestamp is not None:
                frame_age_at_acquire_ms.append((time() - frame_timestamp) * 1000)
            input_sizes.append(len(encoded))

            started = perf_counter()
            frame = cv2.imdecode(np.frombuffer(encoded, dtype=np.uint8), decode_flag)
            decode_ms.append((perf_counter() - started) * 1000)
            if frame is None:
                raise RuntimeError("Cannot decode camera snapshot")
            frame_size = [frame.shape[1], frame.shape[0]]

            started = perf_counter()
            if args.remap_mode == "roi":
                _roi = undistorter.undistort_rectified_region(
                    frame, (left, top), (right - left, bottom - top)
                )
                diagnostic_frame = _roi
            else:
                corrected = undistorter.undistort(frame)
            undistort_ms.append((perf_counter() - started) * 1000)

            if args.remap_mode == "roi":
                crop_ms.append(0.0)
            else:
                started = perf_counter()
                _roi = corrected[top:bottom, left:right].copy()
                diagnostic_frame = corrected
                crop_ms.append((perf_counter() - started) * 1000)

            if model is not None:
                started = perf_counter()
                result = model.predict(
                    _roi,
                    imgsz=args.imgsz,
                    conf=args.conf,
                    device="cpu",
                    verbose=False,
                )[0]
                yolo_wall_ms.append((perf_counter() - started) * 1000)
                yolo_inference_ms.append(float(result.speed.get("inference", 0.0)))
                detection_counts.append(0 if result.boxes is None else len(result.boxes))
            control_total_ms.append((perf_counter() - total_started) * 1000)
            if frame_timestamp is not None:
                frame_to_control_result_ms.append((time() - frame_timestamp) * 1000)

            started = perf_counter()
            ok, output = cv2.imencode(
                ".jpg", diagnostic_frame, [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality]
            )
            encode_ms.append((perf_counter() - started) * 1000)
            if not ok:
                raise RuntimeError("Cannot encode corrected camera frame")
            output_sizes.append(int(output.size))
            total_ms.append((perf_counter() - total_started) * 1000)
            if frame_timestamp is not None:
                frame_to_total_result_ms.append((time() - frame_timestamp) * 1000)
    finally:
        client.close()

    report = {
        "kind": "live_camera_preprocess",
        "platform": platform_info(),
        "acquisition_source": args.source,
        "source_url": source_url,
        "decode_scale": args.decode_scale,
        "coordinate_restore_scale": args.decode_scale,
        "frame_size": frame_size,
        "reference_roi_xyxy": list(reference_roi),
        "roi_xyxy": [left, top, right, bottom],
        "fisheye_balance": fisheye_balance,
        "remap_mode": args.remap_mode,
        "diagnostic_encode_scope": "roi" if args.remap_mode == "roi" else "full_frame",
        "warmup_iterations": args.warmup,
        "measured_iterations": args.iterations,
        "opencv_threads": args.opencv_threads,
        "with_yolo": args.with_yolo,
        "model": str(args.model) if args.with_yolo else None,
        "inference_backend": (
            "ncnn" if ncnn_threads is not None else "pytorch" if args.with_yolo else None
        ),
        "model_load_ms": round(float(model_load_ms), 3) if model_load_ms is not None else None,
        "imgsz": args.imgsz if args.with_yolo else None,
        "confidence_threshold": args.conf if args.with_yolo else None,
        "torch_threads": args.threads if args.with_yolo and ncnn_threads is None else None,
        "ncnn_threads": ncnn_threads,
        "jpeg_quality": args.jpeg_quality,
        "first_undistort_with_map_build_ms": round(float(map_build_ms), 3),
        "timing_ms": {
            "acquire_encoded_jpeg": summarize(acquire_ms),
            "frame_age_at_acquire_end": summarize(frame_age_at_acquire_ms),
            "jpeg_decode": summarize(decode_ms),
            "fisheye_remap": summarize(undistort_ms),
            "roi_copy": summarize(crop_ms),
            "yolo_wall": summarize(yolo_wall_ms),
            "yolo_inference": summarize(yolo_inference_ms),
            "control_total_without_diagnostic_encode": summarize(control_total_ms),
            "frame_timestamp_to_control_result": summarize(frame_to_control_result_ms),
            "diagnostic_jpeg_encode": summarize(encode_ms),
            "total_including_diagnostic_encode": summarize(total_ms),
            "frame_timestamp_to_total_result": summarize(frame_to_total_result_ms),
        },
        "input_jpeg_bytes": summarize([float(value) for value in input_sizes]),
        "diagnostic_jpeg_bytes": summarize([float(value) for value in output_sizes]),
        "equivalent_fps_from_control_p95": round(
            1000 / float(np.percentile(control_total_ms, 95)), 3
        ),
        "frames_with_detection": sum(count > 0 for count in detection_counts),
        "total_detections": sum(detection_counts),
        "thermal_start": thermal_start,
        "thermal_end": thermal_state(),
        "max_rss_kib": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
        "note": (
            "uStreamer receives hardware-encoded MJPEG; corrected JPEG encoding is measured "
            "separately for diagnostics and is not required by the control loop."
        ),
    }
    save_report(report, args.output, "camera_preprocess")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    static = subparsers.add_parser("static", help="Benchmark batch=1 YOLO inference")
    static.add_argument("--data", type=Path, default=DEFAULT_DATA)
    static.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    static.add_argument("--roi", type=Path, default=DEFAULT_ROI)
    static.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    static.add_argument("--input", choices=("roi", "full"), default="roi")
    static.add_argument(
        "--source-scale",
        type=int,
        choices=tuple(JPEG_DECODE_FLAGS),
        default=1,
        help="Downscale fixed source images before ROI inference",
    )
    static.add_argument("--imgsz", type=int, default=320)
    static.add_argument("--conf", type=float, default=DEFAULT_CONFIDENCE)
    static.add_argument("--warmup", type=int, default=10)
    static.add_argument("--iterations", type=int, default=130)
    static.add_argument("--threads", type=int, default=4)
    static.add_argument(
        "--ncnn-threads",
        type=int,
        help="Override NCNN worker threads; by default the NCNN runtime chooses",
    )
    static.add_argument("--opencv-threads", type=int, default=1)
    static.set_defaults(func=run_static)

    camera = subparsers.add_parser("camera", help="Benchmark live camera preprocessing")
    camera.add_argument("--url", default="http://127.0.0.1:8080/snapshot")
    camera.add_argument(
        "--source",
        choices=("snapshot", "stream"),
        default="snapshot",
        help="Read latest snapshots or serial frames from a continuous MJPEG stream",
    )
    camera.add_argument("--roi", type=Path, default=DEFAULT_ROI)
    camera.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    camera.add_argument("--warmup", type=int, default=5)
    camera.add_argument("--iterations", type=int, default=100)
    camera.add_argument("--timeout", type=float, default=3.0)
    camera.add_argument("--jpeg-quality", type=int, default=85)
    camera.add_argument(
        "--decode-scale",
        type=int,
        choices=tuple(JPEG_DECODE_FLAGS),
        default=1,
        help="Use JPEG IDCT scaling while decoding (2 decodes 1280x720 as 640x360)",
    )
    camera.add_argument("--opencv-threads", type=int, default=1)
    camera.add_argument("--remap-mode", choices=("full", "roi"), default="full")
    camera.add_argument("--with-yolo", action="store_true")
    camera.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    camera.add_argument("--imgsz", type=int, default=320)
    camera.add_argument("--conf", type=float, default=DEFAULT_CONFIDENCE)
    camera.add_argument("--threads", type=int, default=4)
    camera.add_argument(
        "--ncnn-threads",
        type=int,
        help="Override NCNN worker threads; by default the NCNN runtime chooses",
    )
    camera.set_defaults(func=run_camera)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.warmup < 1 or args.iterations < 1:
        raise SystemExit("--warmup and --iterations must be positive")
    if args.ncnn_threads is not None and args.ncnn_threads < 1:
        raise SystemExit("--ncnn-threads must be positive")
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
