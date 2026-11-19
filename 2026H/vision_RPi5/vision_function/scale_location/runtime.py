"""Live camera -> ball detector -> scale state -> 11-byte UART stream."""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import http.client
import json
import math
from pathlib import Path
import sys
from time import perf_counter, time
from typing import Callable, Protocol
from urllib.parse import urlsplit

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parent
REPOSITORY_ROOT = ROOT.parents[1]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from vision_function.scale_location.calibration import (  # noqa: E402
    load_scale_calibration,
)
from vision_function.scale_location.diagnostics import DiagnosticCapture  # noqa: E402
from vision_function.scale_location.dynamic_calibration import (  # noqa: E402
    ColorScaleTracker,
    MarkerTrackStatus,
)
from vision_function.scale_location.estimator import (  # noqa: E402
    MotionEstimator,
    MotionFilterConfig,
)
from vision_function.scale_location.marker_detection import (  # noqa: E402
    ColorMarkerDetector,
    MarkerDetectionConfig,
)
from vision_function.scale_location.pipeline import ScalePipeline  # noqa: E402
from vision_function.scale_location.protocol import FrameFlags  # noqa: E402
from vision_function.scale_location.types import BallObservation, ScaleState  # noqa: E402
from vision_function.scale_location.uart_transport import (  # noqa: E402
    FrameLockedSender,
    PosixUart,
    SendResult,
    UartSettings,
)
from vision_function.debug_bridge import DebugFramePublisher  # noqa: E402


DEFAULT_CALIBRATION = ROOT / "config" / "scale_calibration.json"
DEFAULT_MOTION_FILTER = ROOT / "config" / "motion_filter.json"
DEFAULT_UART = ROOT / "config" / "uart.json"
DEFAULT_YOLO_MODEL = ROOT.parent / "ball_detection" / "model" / "actual_yolo11n_local_v3_ncnn_model"
DEFAULT_SHALLOW_CNN_MODEL = ROOT.parent / "ball_detection" / "model" / "shallow_cnn_v3_ncnn_model"
DEFAULT_MARKERS = ROOT / "config" / "color_markers.json"
DEFAULT_MODEL = ROOT.parent / "ball_detection" / "model" / "actual_yolo11n_local_v3_ncnn_model"
DEFAULT_ROI = ROOT.parent / "ball_detection" / "config" / "tube_local_rectification.json"
# Dynamic replay recovered 84.5% of misses at 0.10, while all 31 labelled
# no-ball samples stayed below 0.065. Keep the threshold explicit and tested.
DEFAULT_CONFIDENCE = 0.10
# Shallow CNN defaults match the v3 training config thresholds.
DEFAULT_SHALLOW_CNN_PRESENCE = 0.5
DEFAULT_SHALLOW_CNN_HEATMAP = 0.25


class FrameSource(Protocol):
    def fetch(self) -> tuple[bytes, float | None]: ...
    def close(self) -> None: ...


class Detector(Protocol):
    def detect(
        self,
        frame: np.ndarray,
        *,
        capture_timestamp_s: float,
        frame_sequence: int,
    ) -> tuple[BallObservation, np.ndarray]: ...


class SnapshotClient:
    """Fetch the newest completed uStreamer JPEG and its capture timestamp."""

    def __init__(self, url: str, timeout_s: float = 3.0) -> None:
        parsed = urlsplit(url)
        if parsed.scheme != "http" or not parsed.hostname:
            raise ValueError("Snapshot URL must be an http:// URL")
        self.host = parsed.hostname
        self.port = parsed.port or 80
        self.path = parsed.path or "/"
        if parsed.query:
            self.path += f"?{parsed.query}"
        self.timeout_s = timeout_s
        self._connection: http.client.HTTPConnection | None = None

    def fetch(self) -> tuple[bytes, float | None]:
        if self._connection is None:
            self._connection = http.client.HTTPConnection(
                self.host, self.port, timeout=self.timeout_s
            )
        try:
            self._connection.request("GET", self.path)
            response = self._connection.getresponse()
            if response.status != 200:
                raise RuntimeError(f"Snapshot HTTP status is {response.status}")
            timestamp = response.getheader("X-Timestamp")
            return response.read(), float(timestamp) if timestamp is not None else None
        except (OSError, http.client.HTTPException):
            self.close()
            raise

    def close(self) -> None:
        if self._connection is not None:
            self._connection.close()
            self._connection = None


class HexPacketWriter:
    """A non-hardware writer used to verify the complete stream safely."""

    def __init__(self) -> None:
        self.last_packet: bytes | None = None

    def write(self, packet: bytes) -> int:
        self.last_packet = bytes(packet)
        return len(packet)


@dataclass
class RuntimeStats:
    fetched: int = 0
    duplicate_frames: int = 0
    missing_timestamps: int = 0
    nonmonotonic_timestamps: int = 0
    decode_failures: int = 0
    inferred_frames: int = 0
    detected_frames: int = 0
    tx_frames: int = 0
    invalid: int = 0
    write_fail: int = 0
    marker_blocked_frames: int = 0
    max_age_ms: int = 0
    total_processing_ms: float = 0.0

    def as_dict(self) -> dict[str, int | float]:
        payload = asdict(self)
        payload["mean_processing_ms"] = (
            self.total_processing_ms / self.tx_frames if self.tx_frames else 0.0
        )
        return payload


@dataclass(frozen=True)
class RuntimeResult:
    observation: BallObservation
    state: ScaleState
    send: SendResult
    processing_ms: float
    marker: MarkerTrackStatus | None = None

    def as_dict(self) -> dict[str, object]:
        payload: dict[str, object] = {
            "observation": self.observation.as_dict(),
            "state": self.state.as_dict(),
            "uart": {
                "packet_hex": self.send.packet.hex(" ").upper(),
                "sequence": self.send.frame.sequence,
                "age_ms": self.send.frame.age_ms,
                "flags": self.send.frame.flags,
            },
            "processing_ms": self.processing_ms,
        }
        if self.marker is not None:
            payload["color_markers"] = self.marker.as_dict()
        return payload


class VisionUartRuntime:
    """Strictly serial, frame-locked live processing path."""

    def __init__(
        self,
        source: FrameSource,
        detector: Detector,
        pipeline: ScalePipeline,
        sender: FrameLockedSender,
        *,
        decoder: Callable[[bytes], np.ndarray | None] | None = None,
        clock: Callable[[], float] = time,
        diagnostic_capture: DiagnosticCapture | None = None,
        marker_tracker: ColorScaleTracker | None = None,
        marker_calibration_required: bool = False,
        debug_publisher: DebugFramePublisher | None = None,
    ) -> None:
        self.source = source
        self.detector = detector
        self.pipeline = pipeline
        self.sender = sender
        self.decoder = decoder or self._decode_half
        self.clock = clock
        self.diagnostic_capture = diagnostic_capture
        self.marker_tracker = marker_tracker
        self.marker_calibration_required = marker_calibration_required
        self.debug_publisher = debug_publisher
        self._applied_marker_sequence: int | None = None
        self.stats = RuntimeStats()
        self.last_capture_timestamp_s: float | None = None
        self.frame_sequence = 0

    @staticmethod
    def _decode_half(encoded: bytes) -> np.ndarray | None:
        return cv2.imdecode(
            np.frombuffer(encoded, dtype=np.uint8), cv2.IMREAD_REDUCED_COLOR_2
        )

    def process_next(self) -> RuntimeResult | None:
        started = perf_counter()
        encoded, timestamp = self.source.fetch()
        self.stats.fetched += 1
        if timestamp is None or not math.isfinite(timestamp):
            self.stats.missing_timestamps += 1
            return None
        timestamp = float(timestamp)
        if self.last_capture_timestamp_s is not None:
            if timestamp == self.last_capture_timestamp_s:
                self.stats.duplicate_frames += 1
                return None
            if timestamp < self.last_capture_timestamp_s:
                self.stats.nonmonotonic_timestamps += 1
                self.last_capture_timestamp_s = None
                self.pipeline.reset()
                if self.marker_tracker is not None:
                    self.marker_tracker.reset()
                self._applied_marker_sequence = None
                return None
        self.last_capture_timestamp_s = timestamp

        frame = self.decoder(encoded)
        marker_status: MarkerTrackStatus | None = None
        crop: np.ndarray | None = None
        if frame is None:
            self.stats.decode_failures += 1
            observation = BallObservation(
                frame_sequence=self.frame_sequence,
                capture_timestamp_s=timestamp,
                found=False,
                center_local_xy=None,
                confidence=None,
                candidate_count=0,
            )
        else:
            observation, crop = self.detector.detect(
                frame,
                capture_timestamp_s=timestamp,
                frame_sequence=self.frame_sequence,
            )
            self.stats.inferred_frames += 1
            self.stats.detected_frames += int(observation.found)
            if self.marker_tracker is not None:
                marker_status = self.marker_tracker.update(
                    crop,
                    frame_sequence=self.frame_sequence,
                    capture_timestamp_s=timestamp,
                )
            if self.diagnostic_capture is not None:
                self.diagnostic_capture.capture(observation, crop)

        pipeline_observation = observation
        if self.marker_calibration_required:
            geometry = marker_status.geometry if marker_status is not None else None
            if geometry is None:
                self.stats.marker_blocked_frames += 1
                if observation.found:
                    pipeline_observation = BallObservation(
                        frame_sequence=observation.frame_sequence,
                        capture_timestamp_s=observation.capture_timestamp_s,
                        found=False,
                        center_local_xy=None,
                        confidence=None,
                        candidate_count=0,
                    )
            elif geometry.frame_sequence != self._applied_marker_sequence:
                assert self.marker_tracker is not None
                self.pipeline.set_calibration(
                    geometry.to_calibration(self.marker_tracker.config)
                )
                self._applied_marker_sequence = geometry.frame_sequence

        state = self.pipeline.update(
            pipeline_observation, output_timestamp_s=self.clock()
        )
        try:
            sent = self.sender.send(
                capture_timestamp_s=timestamp,
                position_cm=state.position_at_capture_cm,
                velocity_cm_s=state.velocity_at_capture_cm_s,
                valid=state.valid,
                moving=state.moving,
                velocity_valid=state.velocity_valid,
                edge=state.edge,
                quality=state.quality,
            )
        except Exception:
            self.stats.write_fail += 1
            raise

        self.frame_sequence += 1
        self.stats.tx_frames += 1
        self.stats.invalid += int(not (sent.frame.flags & FrameFlags.VALID))
        self.stats.max_age_ms = max(self.stats.max_age_ms, sent.frame.age_ms)
        elapsed_ms = (perf_counter() - started) * 1000.0
        self.stats.total_processing_ms += elapsed_ms
        result = RuntimeResult(observation, state, sent, elapsed_ms, marker_status)
        if self.debug_publisher is not None:
            try:
                self.debug_publisher.publish(encoded, crop, result.as_dict())
            except (OSError, ValueError):
                # Preview is diagnostic-only and must never interrupt UART.
                pass
        return result


def _settings_with_override(config_path: Path, device: str | None) -> UartSettings:
    settings = UartSettings.load(config_path)
    return UartSettings(
        device=device or settings.device,
        baudrate=settings.baudrate,
        position_sign=settings.position_sign,
    )


def _warm_up(
    source: SnapshotClient,
    detector: Detector,
    count: int,
    ncnn_threads: int,
) -> float | None:
    last_timestamp = None
    completed = 0
    while completed < count:
        encoded, timestamp = source.fetch()
        if timestamp is None or timestamp == last_timestamp:
            continue
        frame = VisionUartRuntime._decode_half(encoded)
        if frame is None:
            raise RuntimeError("Cannot decode a camera warm-up frame")
        detector.detect(
            frame,
            capture_timestamp_s=float(timestamp),
            frame_sequence=completed,
        )
        if completed == 0:
            configure = getattr(detector, "configure_ncnn_threads", None)
            if configure is not None:
                actual_threads = configure(ncnn_threads)
                if actual_threads is not None and actual_threads != ncnn_threads:
                    raise RuntimeError(
                        f"NCNN thread override failed: {actual_threads} != {ncnn_threads}"
                    )
        last_timestamp = float(timestamp)
        completed += 1
    return last_timestamp


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="http://127.0.0.1:8080/snapshot")
    parser.add_argument(
        "--detector",
        choices=("yolo", "shallow_cnn"),
        default="shallow_cnn",
        help="Ball detector backend (default: shallow_cnn)",
    )
    parser.add_argument("--model", type=Path, default=DEFAULT_YOLO_MODEL)
    parser.add_argument("--roi", type=Path, default=DEFAULT_ROI)
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument(
        "--calibration-source",
        choices=("color-markers", "fixed"),
        default="color-markers",
        help="pixel-to-centimetre calibration source (default: color-markers)",
    )
    parser.add_argument("--motion-filter", type=Path, default=DEFAULT_MOTION_FILTER)
    parser.add_argument("--uart-config", type=Path, default=DEFAULT_UART)
    parser.add_argument(
        "--marker-shadow",
        action="store_true",
        help="detect color scale geometry while continuing to use fixed calibration",
    )
    parser.add_argument("--marker-config", type=Path, default=DEFAULT_MARKERS)
    parser.add_argument("--device", help="Override the UART /dev node from uart.json")
    parser.add_argument("--target-cm", type=float, default=0.0)
    parser.add_argument("--imgsz", type=int, default=320)
    parser.add_argument("--conf", type=float, default=DEFAULT_CONFIDENCE)
    parser.add_argument(
        "--shallow-cnn-model",
        type=Path,
        default=DEFAULT_SHALLOW_CNN_MODEL,
        help="NCNN model directory for shallow CNN detector",
    )
    parser.add_argument(
        "--shallow-cnn-presence",
        type=float,
        default=DEFAULT_SHALLOW_CNN_PRESENCE,
        help="Presence threshold for shallow CNN",
    )
    parser.add_argument(
        "--shallow-cnn-heatmap",
        type=float,
        default=DEFAULT_SHALLOW_CNN_HEATMAP,
        help="Heatmap threshold for shallow CNN",
    )
    parser.add_argument("--opencv-threads", type=int, default=1)
    parser.add_argument("--torch-threads", type=int, default=2)
    parser.add_argument("--ncnn-threads", type=int, default=2)
    parser.add_argument("--warmup", type=int, default=5)
    parser.add_argument(
        "--iterations", type=int, default=0, help="0 runs until interrupted"
    )
    parser.add_argument("--timeout", type=float, default=3.0)
    parser.add_argument("--status-every", type=int, default=30)
    parser.add_argument("--json-lines", action="store_true")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Run the complete protocol path without opening GPIO UART",
    )
    parser.add_argument("--report", type=Path)
    parser.add_argument(
        "--diagnostic-dir",
        type=Path,
        help="save missed and low-confidence 640x100 crops for diagnosis",
    )
    parser.add_argument("--diagnostic-confidence-below", type=float, default=0.5)
    parser.add_argument("--diagnostic-max-images", type=int, default=500)
    parser.add_argument(
        "--debug-preview-dir",
        type=Path,
        help="publish source/ROI/result snapshots for the desktop monitor",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.warmup < 0 or args.iterations < 0:
        raise SystemExit("--warmup and --iterations cannot be negative")
    if args.ncnn_threads < 1:
        raise SystemExit("--ncnn-threads must be positive")
    if args.status_every < 0:
        raise SystemExit("--status-every cannot be negative")
    if not 0.0 <= args.diagnostic_confidence_below <= 1.0:
        raise SystemExit("--diagnostic-confidence-below must be in [0, 1]")
    if args.diagnostic_max_images < 1:
        raise SystemExit("--diagnostic-max-images must be positive")

    cv2.setNumThreads(args.opencv_threads)
    try:
        import torch

        torch.set_num_threads(args.torch_threads)
        try:
            torch.set_num_interop_threads(1)
        except RuntimeError:
            pass
    except ImportError:
        pass

    calibration = load_scale_calibration(args.calibration)
    if calibration.status == "temporary_test_only":
        print(
            "warning: using temporary assumed-25cm calibration; UART X is not a final "
            "metrology result",
            file=sys.stderr,
        )
    interface_settings = _settings_with_override(args.uart_config, args.device)
    motion_config = MotionFilterConfig.load(args.motion_filter)
    pipeline = ScalePipeline(
        calibration,
        MotionEstimator(motion_config),
        target_cm=args.target_cm,
        position_sign=interface_settings.position_sign,
    )

    if args.detector == "shallow_cnn":
        from vision_function.ball_detection.shallow_cnn.ncnn_runtime import (
            ShallowCNNBallDetectorNCNN,
        )

        detector: Detector = ShallowCNNBallDetectorNCNN(
            model_dir=args.shallow_cnn_model,
            roi_path=args.roi,
            threads=args.ncnn_threads,
            use_fp16=True,
            presence_threshold=args.shallow_cnn_presence,
            heatmap_threshold=args.shallow_cnn_heatmap,
        )
    else:
        from vision_function.ball_detection.detect_ball import BallDetector

        detector = BallDetector(
            args.model,
            args.roi,
            imgsz=args.imgsz,
            confidence=args.conf,
        )
    marker_apply = (
        args.calibration_source == "color-markers" and not args.marker_shadow
    )
    marker_enabled = marker_apply or args.marker_shadow
    marker_tracker = None
    if marker_enabled:
        marker_config = MarkerDetectionConfig.load(args.marker_config)
        marker_low = min(
            interface_settings.position_sign * marker_config.minimum_cm,
            interface_settings.position_sign * marker_config.maximum_cm,
        )
        marker_high = max(
            interface_settings.position_sign * marker_config.minimum_cm,
            interface_settings.position_sign * marker_config.maximum_cm,
        )
        if marker_apply and not marker_low <= args.target_cm <= marker_high:
            raise SystemExit(
                f"--target-cm must be in [{marker_low}, {marker_high}] for "
                "color-marker calibration"
            )
        marker_tracker = ColorScaleTracker(
            ColorMarkerDetector(marker_config), marker_config
        )
    source = SnapshotClient(args.url, args.timeout)

    writer: HexPacketWriter | PosixUart
    uart: PosixUart | None = None
    if args.dry_run:
        writer = HexPacketWriter()
    else:
        uart = PosixUart(interface_settings)
        uart.open()
        writer = uart
    sender = FrameLockedSender(writer)
    diagnostic = (
        DiagnosticCapture(
            args.diagnostic_dir,
            args.diagnostic_confidence_below,
            args.diagnostic_max_images,
        )
        if args.diagnostic_dir is not None
        else None
    )
    runtime = VisionUartRuntime(
        source,
        detector,
        pipeline,
        sender,
        diagnostic_capture=diagnostic,
        marker_tracker=marker_tracker,
        marker_calibration_required=marker_apply,
        debug_publisher=(
            DebugFramePublisher(args.debug_preview_dir)
            if args.debug_preview_dir is not None
            else None
        ),
    )

    try:
        if args.warmup:
            runtime.last_capture_timestamp_s = _warm_up(
                source, detector, args.warmup, args.ncnn_threads
            )
        while args.iterations == 0 or runtime.stats.tx_frames < args.iterations:
            result = runtime.process_next()
            if result is None:
                continue
            if args.json_lines:
                print(json.dumps(result.as_dict(), ensure_ascii=False, separators=(",", ":")))
            elif args.status_every and runtime.stats.tx_frames % args.status_every == 0:
                state = result.state
                print(
                    f"tx={runtime.stats.tx_frames} seq={result.send.frame.sequence} "
                    f"age={result.send.frame.age_ms}ms valid={int(state.valid)} "
                    f"x={state.position_at_capture_cm:+.3f}cm "
                    f"v={state.velocity_at_capture_cm_s:+.2f}cm/s "
                    f"flags=0x{result.send.frame.flags:02X} "
                    f"packet={result.send.packet.hex(' ').upper()}"
                    + (
                        f" marker={'valid' if result.marker.geometry_valid else 'invalid'}"
                        if result.marker is not None
                        else ""
                    )
                )
    except KeyboardInterrupt:
        pass
    finally:
        source.close()
        if uart is not None:
            uart.close()
        if diagnostic is not None:
            diagnostic.close()

    report = {
        "source_url": args.url,
        "detector": args.detector,
        "model": (
            str(args.shallow_cnn_model)
            if args.detector == "shallow_cnn"
            else str(args.model)
        ),
        "calibration": str(args.calibration),
        "active_calibration_source": (
            "color-markers" if marker_apply else "fixed"
        ),
        "motion_filter": str(args.motion_filter),
        "uart": "dry-run" if args.dry_run else interface_settings.device,
        "position_sign": interface_settings.position_sign,
        "target_cm": args.target_cm,
        "stats": runtime.stats.as_dict(),
    }
    if marker_tracker is not None:
        report["color_markers"] = {
            "enabled": True,
            "config": str(args.marker_config),
            "affects_uart": marker_apply,
            "stats": marker_tracker.stats(),
        }
    if diagnostic is not None:
        report["diagnostic"] = diagnostic.as_dict()
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if args.report is not None:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
