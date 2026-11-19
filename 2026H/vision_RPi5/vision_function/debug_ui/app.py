"""Tk desktop application for Raspberry Pi vision debugging.

Run from the repository root with::

    python -m vision_function.debug_ui.app
"""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
from datetime import datetime
import http.client
import json
import os
from pathlib import Path
import queue
import threading
import time
from urllib.parse import urlsplit

import cv2
import numpy as np

from vision_function.debug_bridge import decode_published_image
from vision_function.scale_location.types import BallObservation

from .core import (
    DebugFrameResult,
    DebugProcessor,
    draw_horizontal_reference_lines,
)


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_DEMO = (
    ROOT
    / "vision_function"
    / "scale_location"
    / "calibration_assets"
    / "color_ruler"
    / "tube-roi-2026-07-31T10-43-52-300Z.png"
)


@dataclass(frozen=True)
class SourceSpec:
    kind: str
    value: str


@dataclass(frozen=True)
class PublishedMarkers:
    valid: bool
    runs: tuple[dict[str, object], ...]
    reason: str
    spacing_cv: float | None


@dataclass(frozen=True)
class PublishedResult:
    annotated_frame: np.ndarray
    ball: BallObservation
    markers: PublishedMarkers
    total_ms: float
    ball_ms: float | None
    marker_ms: float | None
    ball_error: str | None
    state: dict[str, object]
    uart: dict[str, object]


class ServiceDebugReader:
    """Read frames already processed by the UART service from shared memory."""

    def __init__(self, directory: Path | str) -> None:
        self.directory = Path(directory)
        self.last_generation = -1

    def read(
        self, stop_event: threading.Event
    ) -> tuple[np.ndarray, PublishedResult, float]:
        metadata_path = self.directory / "latest.json"
        while not stop_event.is_set():
            try:
                payload = json.loads(metadata_path.read_text(encoding="utf-8"))
                generation = int(payload["generation"])
                if generation == self.last_generation:
                    stop_event.wait(0.01)
                    continue
                source = decode_published_image(
                    self.directory / str(payload["source_image"]),
                    cv2.IMREAD_REDUCED_COLOR_2,
                )
                roi = decode_published_image(
                    self.directory / str(payload["roi_image"])
                )
                result = self._result(roi, payload["result"])
                self.last_generation = generation
                return source, result, float(payload["published_ns"]) / 1e9
            except (FileNotFoundError, KeyError, ValueError, OSError):
                stop_event.wait(0.05)
        raise InterruptedError("调试连接已停止")

    @staticmethod
    def _result(roi: np.ndarray, payload: dict[str, object]) -> PublishedResult:
        observation = dict(payload["observation"])
        center = observation.get("center_local_xy")
        box = observation.get("box_local_xyxy")
        ball = BallObservation(
            frame_sequence=0,
            capture_timestamp_s=float(observation["capture_timestamp_s"]),
            found=bool(observation["found"]),
            center_local_xy=None if center is None else tuple(float(v) for v in center),
            confidence=(
                None
                if observation.get("confidence") is None
                else float(observation["confidence"])
            ),
            box_local_xyxy=None if box is None else tuple(float(v) for v in box),
            candidate_count=int(observation.get("candidate_count", 0)),
        )
        marker_status = dict(payload.get("color_markers") or {})
        geometry = dict(marker_status.get("geometry") or {})
        runs = tuple(geometry.get("runs") or ())
        markers = PublishedMarkers(
            valid=bool(marker_status.get("geometry_valid", False)),
            runs=runs,
            reason=str(marker_status.get("reason", "等待色块检测")),
            spacing_cv=(
                None
                if geometry.get("spacing_cv") is None
                else float(geometry["spacing_cv"])
            ),
        )
        annotated = draw_published_overlay(roi, ball, geometry)
        marker_ms = marker_status.get("processing_ms")
        return PublishedResult(
            annotated_frame=annotated,
            ball=ball,
            markers=markers,
            total_ms=float(payload.get("processing_ms", 0.0)),
            ball_ms=None,
            marker_ms=None if marker_ms is None else float(marker_ms),
            ball_error=None,
            state=dict(payload.get("state") or {}),
            uart=dict(payload.get("uart") or {}),
        )


def draw_published_overlay(
    roi: np.ndarray,
    ball: BallObservation,
    geometry: dict[str, object],
) -> np.ndarray:
    output = draw_horizontal_reference_lines(roi, tube_local=True)
    palette = {
        "yellow": (30, 225, 240),
        "green": (80, 220, 110),
        "magenta": (220, 80, 220),
    }
    for run in geometry.get("runs") or ():
        center = run.get("center_xy")
        if not center:
            continue
        cx, cy = (int(round(float(value))) for value in center)
        color = palette.get(str(run.get("color")), (255, 255, 255))
        cv2.line(output, (cx, max(0, cy - 18)), (cx, min(99, cy + 18)), color, 2)
        cv2.circle(output, (cx, cy), 3, color, -1, cv2.LINE_AA)
    origin = geometry.get("axis_origin_xy")
    if origin:
        point = tuple(int(round(float(value))) for value in origin)
        cv2.drawMarker(output, point, (255, 255, 255), cv2.MARKER_CROSS, 14, 2)
    if ball.found and ball.center_local_xy is not None:
        center = tuple(int(round(value)) for value in ball.center_local_xy)
        if ball.box_local_xyxy is not None:
            left, top, right, bottom = (
                int(round(value)) for value in ball.box_local_xyxy
            )
            cv2.rectangle(output, (left, top), (right, bottom), (60, 210, 255), 2)
        cv2.drawMarker(output, center, (60, 210, 255), cv2.MARKER_CROSS, 18, 2)
    return output


class FrameReader:
    """Uniform reader for local media, USB cameras and uStreamer snapshots."""

    IMAGE_SUFFIXES = {".png", ".jpg", ".jpeg", ".bmp", ".webp"}

    def __init__(self, source: SourceSpec) -> None:
        self.source = source
        self._capture: cv2.VideoCapture | None = None
        self._image: np.ndarray | None = None
        self._http: http.client.HTTPConnection | None = None
        self._snapshot_host = ""
        self._snapshot_port = 80
        self._snapshot_path = "/"
        if source.kind == "camera":
            self._capture = cv2.VideoCapture(int(source.value))
        elif source.kind == "snapshot":
            parsed = urlsplit(source.value)
            if parsed.scheme != "http" or not parsed.hostname:
                raise ValueError("图传快照地址必须是 http:// URL")
            self._snapshot_host = parsed.hostname
            self._snapshot_port = parsed.port or 80
            self._snapshot_path = parsed.path or "/"
            if parsed.query:
                self._snapshot_path += "?" + parsed.query
        elif source.kind == "file":
            path = Path(source.value)
            if path.suffix.lower() in self.IMAGE_SUFFIXES:
                self._image = cv2.imread(str(path), cv2.IMREAD_COLOR)
                if self._image is None:
                    raise RuntimeError(f"无法读取图片：{path}")
            else:
                self._capture = cv2.VideoCapture(str(path))
        if self._capture is not None and not self._capture.isOpened():
            raise RuntimeError(f"无法打开视频源：{source.value}")

    def read(self) -> tuple[np.ndarray, float]:
        timestamp = time.time()
        if self.source.kind == "snapshot":
            if self._http is None:
                self._http = http.client.HTTPConnection(
                    self._snapshot_host, self._snapshot_port, timeout=2.0
                )
            try:
                self._http.request("GET", self._snapshot_path)
                response = self._http.getresponse()
                if response.status != 200:
                    raise RuntimeError(f"uStreamer HTTP 状态码 {response.status}")
                payload = response.read()
                header = response.getheader("X-Timestamp")
            except (OSError, http.client.HTTPException):
                self._close_http()
                raise
            # Match the production path: JPEG 1/2 IDCT decode directly to
            # 640x360 instead of decoding 1280x720 and resizing afterwards.
            encoded = np.frombuffer(payload, dtype=np.uint8)
            frame = cv2.imdecode(encoded, cv2.IMREAD_REDUCED_COLOR_2)
            if frame is None:
                raise RuntimeError("uStreamer 返回的 JPEG 无法解码")
            if header:
                try:
                    timestamp = float(header)
                except ValueError:
                    pass
            return frame, timestamp
        if self._image is not None:
            return self._image.copy(), timestamp
        assert self._capture is not None
        ok, frame = self._capture.read()
        if not ok and self.source.kind == "file":
            self._capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = self._capture.read()
        if not ok or frame is None:
            raise RuntimeError("视频源取帧失败")
        return frame, timestamp

    def close(self) -> None:
        if self._capture is not None:
            self._capture.release()
        self._close_http()

    def _close_http(self) -> None:
        if self._http is not None:
            self._http.close()
            self._http = None


class ProcessingWorker(threading.Thread):
    def __init__(self, source: SourceSpec, events: queue.Queue, stop: threading.Event):
        super().__init__(daemon=True, name="vision-debug-worker")
        self.source = source
        self.events = events
        self.stop_event = stop

    def emit(self, kind: str, payload: object) -> None:
        try:
            self.events.put_nowait((self, kind, payload))
        except queue.Full:
            try:
                self.events.get_nowait()
            except queue.Empty:
                pass
            self.events.put_nowait((self, kind, payload))

    def run(self) -> None:
        reader: FrameReader | ServiceDebugReader | None = None
        try:
            if self.source.kind == "service":
                self.emit("status", "正在连接正式识别服务…")
                reader = ServiceDebugReader(self.source.value)
                while not self.stop_event.is_set():
                    try:
                        frame, result, _timestamp = reader.read(self.stop_event)
                    except InterruptedError:
                        break
                    if self.stop_event.is_set():
                        break
                    self.emit(
                        "frame",
                        (frame, result, 0.0, max(result.total_ms, 0.001)),
                    )
                return

            self.emit("status", "正在加载独立识别器…")
            processor = DebugProcessor()
            reader = FrameReader(self.source)
            last_timestamp: float | None = None
            while not self.stop_event.is_set():
                loop_started = time.perf_counter()
                fetch_started = time.perf_counter()
                try:
                    frame, timestamp = reader.read()
                except Exception as exc:
                    if self.source.kind != "snapshot":
                        raise
                    self.emit("status", f"图传暂时断开，正在重连：{exc}")
                    self.stop_event.wait(0.5)
                    continue
                fetch_ms = (time.perf_counter() - fetch_started) * 1000.0
                if (
                    self.source.kind == "snapshot"
                    and last_timestamp is not None
                    and timestamp <= last_timestamp
                ):
                    continue
                last_timestamp = timestamp
                result = processor.process(
                    frame, sequence=0, capture_timestamp_s=timestamp
                )
                loop_ms = (time.perf_counter() - loop_started) * 1000.0
                if self.stop_event.is_set():
                    break
                self.emit("frame", (frame, result, fetch_ms, loop_ms))
                elapsed = time.perf_counter() - loop_started
                if self.source.kind == "file" and Path(self.source.value).suffix.lower() in FrameReader.IMAGE_SUFFIXES:
                    self.stop_event.wait(max(0.0, 0.10 - elapsed))
        except Exception as exc:
            self.emit("error", f"{type(exc).__name__}: {exc}")
        finally:
            if isinstance(reader, FrameReader):
                reader.close()
            self.emit("stopped", None)


class VisionDebugApp:
    BG = "#0d0f12"
    PANEL = "#16191f"
    LINE = "#2b313b"
    FG = "#e6e9ef"
    DIM = "#929aa8"
    ACCENT = "#60a5fa"
    OK = "#4ade80"
    BAD = "#f87171"

    def __init__(self, root) -> None:
        import tkinter as tk
        from tkinter import ttk

        self.tk = tk
        self.root = root
        root.title("H题视觉调试台")
        root.configure(bg=self.BG)
        root.geometry("1024x600")
        root.minsize(860, 500)
        root.protocol("WM_DELETE_WINDOW", self.close)

        self.events: queue.Queue = queue.Queue(maxsize=2)
        self.stop_event = threading.Event()
        self.worker: ProcessingWorker | None = None
        self.last_source: np.ndarray | None = None
        self.last_result: DebugFrameResult | PublishedResult | None = None
        self._source_photo = None
        self._roi_photo = None
        self.started_at = time.perf_counter()
        self.received_frames = 0
        self.frame_times: deque[float] = deque()

        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TButton", padding=(10, 6), background="#222833", foreground=self.FG)
        style.configure("TCombobox", fieldbackground="#222833", foreground=self.FG)

        monitor_dir = os.environ.get("NUEDC_DEBUG_MONITOR_DIR")
        self.source_kind = tk.StringVar(
            value="正式服务（UART保持）" if monitor_dir else "演示图片"
        )
        self.source_value = tk.StringVar(value=monitor_dir or str(DEFAULT_DEMO))
        self.status_text = tk.StringVar(
            value=(
                "监看模式 · 正式识别与 UART 保持运行"
                if monitor_dir
                else "就绪（本机演示模式）"
            )
        )
        self.ball_text = tk.StringVar(value="尚未运行")
        self.marker_text = tk.StringVar(value="尚未运行")
        self.performance_text = tk.StringVar(value="FPS --  |  总耗时 --")
        self.detail_text = tk.StringVar(value="等待画面")
        self._build_layout(ttk)
        self.root.after(40, self._poll_events)

    def _build_layout(self, ttk) -> None:
        tk = self.tk
        header = tk.Frame(self.root, bg=self.PANEL, highlightbackground=self.LINE, highlightthickness=1)
        header.pack(fill="x")
        self.status_dot = tk.Label(header, text="●", fg=self.BAD, bg=self.PANEL, font=("Segoe UI", 12))
        self.status_dot.pack(side="left", padx=(14, 5), pady=10)
        tk.Label(header, text="H题视觉调试台", fg=self.FG, bg=self.PANEL, font=("Microsoft YaHei UI", 13, "bold")).pack(side="left")
        tk.Label(header, text="树莓派桌面版", fg=self.DIM, bg=self.PANEL).pack(side="left", padx=10)

        self.kind_box = ttk.Combobox(
            header,
            textvariable=self.source_kind,
            state="readonly",
            width=17,
            values=(
                "正式服务（UART保持）",
                "演示图片",
                "本地文件",
                "USB摄像头",
                "图传快照",
            ),
        )
        self.kind_box.pack(side="left", padx=(10, 4))
        self.kind_box.bind("<<ComboboxSelected>>", self._source_kind_changed)
        self.value_entry = tk.Entry(header, textvariable=self.source_value, bg="#222833", fg=self.FG, insertbackground=self.FG, relief="flat", width=24)
        self.value_entry.pack(side="left", padx=4, ipady=5)
        ttk.Button(header, text="浏览", command=self.browse).pack(side="left", padx=4)
        self.start_button = ttk.Button(header, text="开始", command=self.toggle)
        self.start_button.pack(side="left", padx=4)
        ttk.Button(header, text="保存截图", command=self.snapshot).pack(side="left", padx=4)

        body = tk.Frame(self.root, bg=self.BG)
        body.pack(fill="both", expand=True, padx=6, pady=6)
        views = tk.Frame(body, bg=self.BG)
        views.pack(side="left", fill="both", expand=True)
        views.grid_columnconfigure(0, weight=1)
        views.grid_rowconfigure(0, weight=3)
        views.grid_rowconfigure(1, weight=1)
        self.source_canvas = self._view(views, "原始画面", 700, 320, row=0)
        self.roi_canvas = self._view(
            views, "识别 ROI · 小球 / 色块叠加", 700, 120, row=1
        )

        side = tk.Frame(body, bg=self.PANEL, width=260, highlightbackground=self.LINE, highlightthickness=1)
        side.pack(side="right", fill="y", padx=(6, 0))
        side.pack_propagate(False)
        self._section(side, "小球识别", self.ball_text, self.ACCENT)
        self._section(side, "色块识别", self.marker_text, self.OK)
        self._section(side, "实时性能", self.performance_text, self.FG)
        self._section(side, "详细信息", self.detail_text, self.DIM)
        legend = tk.Frame(side, bg=self.PANEL)
        legend.pack(fill="x", padx=14, pady=8)
        for label, color in (("奇数刻度", "#e1dc4c"), ("偶数刻度", "#6edc50"), ("零点", "#dc50dc"), ("小球", "#ffd23c")):
            row = tk.Frame(legend, bg=self.PANEL)
            row.pack(fill="x", pady=2)
            tk.Label(row, text="■", fg=color, bg=self.PANEL).pack(side="left")
            tk.Label(row, text=label, fg=self.DIM, bg=self.PANEL).pack(side="left", padx=6)

        footer = tk.Frame(self.root, bg=self.PANEL, highlightbackground=self.LINE, highlightthickness=1)
        footer.pack(fill="x")
        tk.Label(footer, textvariable=self.status_text, fg=self.DIM, bg=self.PANEL, anchor="w").pack(fill="x", padx=14, pady=7)

    def _view(self, parent, title: str, width: int, height: int, *, row: int):
        tk = self.tk
        frame = tk.Frame(parent, bg=self.PANEL, highlightbackground=self.LINE, highlightthickness=1)
        frame.grid(row=row, column=0, sticky="nsew", pady=(0, 5))
        tk.Label(frame, text=title, fg=self.DIM, bg=self.PANEL, anchor="w").pack(fill="x", padx=10, pady=5)
        canvas = tk.Canvas(frame, width=width, height=height, bg="#000000", highlightthickness=0)
        canvas.pack(fill="both", expand=True)
        canvas.create_text(width // 2, height // 2, text="等待画面", fill=self.DIM, tags="placeholder")
        return canvas

    def _section(self, parent, title: str, variable, color: str) -> None:
        tk = self.tk
        tk.Label(parent, text=title, fg=self.DIM, bg=self.PANEL, anchor="w", font=("Microsoft YaHei UI", 10, "bold")).pack(fill="x", padx=14, pady=(15, 4))
        tk.Label(parent, textvariable=variable, fg=color, bg=self.PANEL, anchor="nw", justify="left", wraplength=255).pack(fill="x", padx=14)

    def _source_kind_changed(self, _event=None) -> None:
        defaults = {
            "正式服务（UART保持）": os.environ.get(
                "NUEDC_DEBUG_MONITOR_DIR", "/dev/shm/nuedc-vision-debug"
            ),
            "演示图片": str(DEFAULT_DEMO),
            "本地文件": str(DEFAULT_DEMO),
            "USB摄像头": "0",
            "图传快照": "http://127.0.0.1:8080/snapshot",
        }
        self.source_value.set(defaults[self.source_kind.get()])

    def _source_spec(self) -> SourceSpec:
        kinds = {
            "正式服务（UART保持）": "service",
            "演示图片": "file",
            "本地文件": "file",
            "USB摄像头": "camera",
            "图传快照": "snapshot",
        }
        return SourceSpec(kinds[self.source_kind.get()], self.source_value.get().strip())

    def browse(self) -> None:
        from tkinter import filedialog

        path = filedialog.askopenfilename(title="选择图片或视频", filetypes=(("图像和视频", "*.png *.jpg *.jpeg *.mp4 *.avi *.webm"), ("所有文件", "*.*")))
        if path:
            self.source_kind.set("本地文件")
            self.source_value.set(path)

    def toggle(self) -> None:
        if self.worker is not None and self.worker.is_alive():
            self.stop()
        else:
            self.start()

    def start(self) -> None:
        self.stop_event = threading.Event()
        self.received_frames = 0
        self.started_at = time.perf_counter()
        self.frame_times.clear()
        self.worker = ProcessingWorker(self._source_spec(), self.events, self.stop_event)
        self.worker.start()
        self.start_button.configure(text="停止", state="normal")
        self.status_text.set("正在启动…")

    def stop(self) -> None:
        self.stop_event.set()
        self.start_button.configure(text="正在停止…", state="disabled")
        self.status_text.set("正在停止连接…")
        self.status_dot.configure(fg=self.BAD)

    def _poll_events(self) -> None:
        try:
            while True:
                owner, kind, payload = self.events.get_nowait()
                if owner is not self.worker:
                    continue
                if kind == "frame":
                    frame, result, fetch_ms, loop_ms = payload
                    self._show_frame(frame, result, fetch_ms, loop_ms)
                elif kind == "status":
                    self.status_text.set(str(payload))
                elif kind == "error":
                    self.status_text.set("错误：" + str(payload))
                elif kind == "stopped":
                    self.worker = None
                    self.start_button.configure(text="开始", state="normal")
                    self.status_text.set("连接已停止 · 正式识别与 UART 不受影响")
                    self.status_dot.configure(fg=self.BAD)
        except queue.Empty:
            pass
        self.root.after(20, self._poll_events)

    def _show_frame(
        self,
        source: np.ndarray,
        result: DebugFrameResult | PublishedResult,
        fetch_ms: float,
        loop_ms: float,
    ) -> None:
        self.last_source, self.last_result = source, result
        self.received_frames += 1
        now = time.perf_counter()
        self.frame_times.append(now)
        while len(self.frame_times) > 2 and now - self.frame_times[0] > 2.0:
            self.frame_times.popleft()
        display_fps = (
            (len(self.frame_times) - 1)
            / max(self.frame_times[-1] - self.frame_times[0], 1e-6)
            if len(self.frame_times) >= 2
            else 0.0
        )
        processing_fps = 1000.0 / max(loop_ms, 1e-6)
        source_is_tube = source.shape[1] / max(source.shape[0], 1) > 4.0
        source_with_lines = draw_horizontal_reference_lines(
            source, tube_local=source_is_tube
        )
        self._source_photo = self._draw_image(
            self.source_canvas, source_with_lines
        )
        self._roi_photo = self._draw_image(self.roi_canvas, result.annotated_frame)

        ball = result.ball
        if ball is None:
            ball_text = "不可用\n" + (result.ball_error or "已禁用")
        elif not ball.found:
            ball_text = "未发现小球\n候选 0"
        else:
            assert ball.center_local_xy is not None
            confidence = 0.0 if ball.confidence is None else ball.confidence
            ball_text = f"已发现  置信度 {confidence:.3f}\n球心 ({ball.center_local_xy[0]:.1f}, {ball.center_local_xy[1]:.1f})\n候选 {ball.candidate_count}"
        self.ball_text.set(ball_text)

        markers = result.markers
        marker_state = "有效" if markers.valid else "无效"
        self.marker_text.set(f"{marker_state}  色块 {len(markers.runs)}\n{markers.reason}\n间距 CV " + ("--" if markers.spacing_cv is None else f"{markers.spacing_cv:.3f}"))
        ball_ms = "--" if result.ball_ms is None else f"{result.ball_ms:.1f} ms"
        marker_ms = (
            "--" if result.marker_ms is None else f"{result.marker_ms:.1f} ms"
        )
        self.performance_text.set(
            f"处理 FPS {processing_fps:.1f}\n"
            f"显示 FPS {display_fps:.1f}\n"
            f"取帧 {fetch_ms:.1f} ms\n"
            f"视觉 {result.total_ms:.1f} ms\n"
            f"小球 {ball_ms}\n"
            f"色块 {marker_ms}"
        )
        state = getattr(result, "state", {})
        uart = getattr(result, "uart", {})
        if state:
            self.detail_text.set(
                f"位置 {float(state.get('position_at_capture_cm', 0.0)):+.3f} cm\n"
                f"速度 {float(state.get('velocity_at_capture_cm_s', 0.0)):+.2f} cm/s\n"
                f"VALID {int(bool(state.get('valid', False)))} · "
                f"AGE {int(uart.get('age_ms', 0))} ms\n"
                "ROI 640 × 100"
            )
        else:
            self.detail_text.set(
                f"ROI 640 × 100\n输入 {source.shape[1]} × {source.shape[0]}"
            )
        self.status_text.set(
            "运行中 · 正式识别与 UART 正常发送"
            if self.source_kind.get() == "正式服务（UART保持）"
            else "运行中 · 独立本地处理 · 不操作 UART"
        )
        self.status_dot.configure(fg=self.OK)

    def _draw_image(self, canvas, bgr: np.ndarray):
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        canvas.update_idletasks()
        width = max(canvas.winfo_width(), 2)
        height = max(canvas.winfo_height(), 2)
        scale = min(width / rgb.shape[1], height / rgb.shape[0])
        size = (max(1, int(rgb.shape[1] * scale)), max(1, int(rgb.shape[0] * scale)))
        resized = cv2.resize(rgb, size, interpolation=cv2.INTER_AREA if scale < 1 else cv2.INTER_NEAREST)
        ok, encoded = cv2.imencode(".png", cv2.cvtColor(resized, cv2.COLOR_RGB2BGR))
        if not ok:
            return None
        photo = self.tk.PhotoImage(data=encoded.tobytes())
        canvas.delete("all")
        canvas.create_image(width // 2, height // 2, image=photo, anchor="center")
        return photo

    def snapshot(self) -> None:
        if self.last_result is None:
            self.status_text.set("当前没有可保存的识别画面")
            return
        output = ROOT / "tmp" / "debug_ui"
        output.mkdir(parents=True, exist_ok=True)
        name = datetime.now().strftime("vision-debug-%Y%m%d-%H%M%S.png")
        path = output / name
        cv2.imwrite(str(path), self.last_result.annotated_frame)
        self.status_text.set(f"截图已保存：{path}")

    def close(self) -> None:
        self.stop_event.set()
        self.root.destroy()


def run_headless(input_path: Path, output_path: Path | None, no_ball: bool) -> int:
    frame = cv2.imread(str(input_path), cv2.IMREAD_COLOR)
    if frame is None:
        raise SystemExit(f"无法读取输入图片：{input_path}")
    processor = DebugProcessor(enable_ball=not no_ball)
    result = processor.process(frame, sequence=0, capture_timestamp_s=time.time())
    if output_path is not None:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(output_path), result.annotated_frame):
            raise SystemExit(f"无法写入输出图片：{output_path}")
    ball_state = "unavailable" if result.ball is None else ("found" if result.ball.found else "missing")
    print(f"ball={ball_state} markers={'valid' if result.markers.valid else 'invalid'} runs={len(result.markers.runs)} total_ms={result.total_ms:.2f}")
    if result.ball_error:
        print(f"ball_backend={result.ball_error}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", type=Path, metavar="IMAGE", help="无窗口处理一张本地图片")
    parser.add_argument("--output", type=Path, help="无窗口模式的叠加图输出路径")
    parser.add_argument("--no-ball", action="store_true", help="不加载小球模型（用于快速检查界面/色块链路）")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.headless is not None:
        return run_headless(args.headless, args.output, args.no_ball)
    import tkinter as tk

    root = tk.Tk()
    VisionDebugApp(root)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
