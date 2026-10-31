#!/usr/bin/env python3
"""View, capture, or measure a K230 RTSP/MJPEG stream with OpenCV."""

from __future__ import annotations

import argparse
import os
import statistics
import time
from pathlib import Path


def rotate_if_needed(frame, disabled: bool):
    if not disabled and frame.shape[0] > frame.shape[1]:
        return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    return frame


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    index = min(len(ordered) - 1, int(round((len(ordered) - 1) * fraction)))
    return ordered[index]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("url", help="An rtsp:// or http://.../stream.mjpg URL")
    parser.add_argument(
        "--transport",
        choices=("tcp", "udp"),
        default="tcp",
        help="RTSP/RTP transport; try udp for lower latency on a reliable LAN",
    )
    parser.add_argument(
        "--buffer-size",
        type=int,
        default=1,
        help="Requested OpenCV capture queue size (the FFmpeg backend may ignore it)",
    )
    parser.add_argument("--snapshot", type=Path, help="Save one decoded frame and exit")
    parser.add_argument("--stats-frames", type=int, help="Measure N decoded frames and exit")
    parser.add_argument(
        "--warmup-frames",
        type=int,
        default=30,
        help="Discard this many decoded frames before measuring stats",
    )
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument(
        "--no-auto-rotate",
        action="store_true",
        help="Keep native orientation instead of rotating portrait WBC frames",
    )
    args = parser.parse_args()

    # OpenCV reads these FFmpeg options only when cv2 is imported. UDP avoids
    # TCP head-of-line blocking, while TCP remains the safer default on a noisy
    # hotspot. Both modes ask FFmpeg not to build a playback buffer.
    os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
        "rtsp_transport;%s|fflags;nobuffer|flags;low_delay" % args.transport
    )
    global cv2
    import cv2

    timeout_ms = max(1, int(args.timeout * 1000))
    capture = cv2.VideoCapture(
        args.url,
        cv2.CAP_FFMPEG,
        [
            cv2.CAP_PROP_OPEN_TIMEOUT_MSEC,
            timeout_ms,
            cv2.CAP_PROP_READ_TIMEOUT_MSEC,
            timeout_ms,
        ],
    )
    capture.set(cv2.CAP_PROP_BUFFERSIZE, max(1, args.buffer_size))
    deadline = time.monotonic() + args.timeout
    frame_times: list[float] = []
    try:
        while time.monotonic() < deadline:
            ok, frame = capture.read()
            if not ok or frame is None:
                time.sleep(0.01)
                continue

            frame = rotate_if_needed(frame, args.no_auto_rotate)
            now = time.monotonic()
            if args.snapshot is not None:
                args.snapshot.parent.mkdir(parents=True, exist_ok=True)
                if not cv2.imwrite(str(args.snapshot), frame):
                    raise RuntimeError("failed to write snapshot")
                print(
                    "VIDEO_FRAME_OK width=%d height=%d snapshot=%s"
                    % (frame.shape[1], frame.shape[0], args.snapshot)
                )
                return 0

            if args.stats_frames is not None:
                if args.warmup_frames > 0:
                    args.warmup_frames -= 1
                    continue
                frame_times.append(now)
                if len(frame_times) >= max(2, args.stats_frames):
                    intervals = [
                        b - a for a, b in zip(frame_times, frame_times[1:])
                    ]
                    elapsed = frame_times[-1] - frame_times[0]
                    print(
                        "VIDEO_STATS_OK frames=%d fps=%.2f mean_ms=%.1f "
                        "p95_ms=%.1f p99_ms=%.1f max_ms=%.1f"
                        % (
                            len(frame_times),
                            (len(frame_times) - 1) / elapsed,
                            statistics.mean(intervals) * 1000,
                            percentile(intervals, 0.95) * 1000,
                            percentile(intervals, 0.99) * 1000,
                            max(intervals) * 1000,
                        )
                    )
                    return 0
                continue

            cv2.imshow("K230 video", frame)
            if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                return 0

        print("VIDEO_FRAME_TIMEOUT url=%s frames=%d" % (args.url, len(frame_times)))
        return 1
    finally:
        capture.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    raise SystemExit(main())
