#!/usr/bin/env python3
"""H3 静止守球 Debug_Ex 实时遥测可视化与 CSV 记录。

固件在 ``H3 Ball Static`` 中经 Debug_Ex/UART1（115200 8N1）输出 ``[BALL]``。
窗口显示在线五次参考、终端约束残差、模型加速度，以及水管角和控制分量。

用法：
  python ball_balance_viz.py --list
  python ball_balance_viz.py --port COM7
  python ball_balance_viz.py --port COM7 --csv ball.csv --log ball_raw.txt
  python ball_balance_viz.py --port COM7 --capture-only --duration 20 --csv ball.csv --log ball_raw.txt

热键：空格=暂停/继续，c=清空，q=退出。
依赖：pip install pyserial matplotlib
"""
from __future__ import annotations

import argparse
import sys
import threading
import time
from collections import deque

import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.animation import FuncAnimation
import serial
from serial.tools import list_ports

MAX_SAMPLES = 7200
DEFAULT_WINDOW_S = 15.0
FLOAT_FIELDS = ("xr", "x", "v", "age", "et", "xref", "vref", "aref", "T",
                "look", "pv", "pa", "a0", "ia", "iv", "rx", "rv", "ra", "ex",
                "ev", "aff", "fbp", "fbv", "fb", "rate", "u", "bias", "beam",
                "motor", "mspd", "mkp", "fps")
INT_FIELDS = ("st", "ok", "cnt", "tgt", "sf", "arr", "cc", "ctr", "sdt",
              "smax", "lim", "sat", "rl", "brk", "stuck", "set", "mv", "vv",
              "px", "edge", "deg", "hold", "q", "guard", "gap", "inv", "crc",
              "drop")
FIELDS = FLOAT_FIELDS + INT_FIELDS
CSV_COLS = ("pc_time", "t", *FIELDS)

_CJK_CANDIDATES = ("Microsoft YaHei", "SimHei", "SimSun", "Noto Sans CJK SC",
                   "Source Han Sans SC", "WenQuanYi Zen Hei", "Arial Unicode MS")
_AVAILABLE = {font.name for font in font_manager.fontManager.ttflist}
_CJK = next((font for font in _CJK_CANDIDATES if font in _AVAILABLE), None)
plt.rcParams["font.sans-serif"] = [_CJK, "sans-serif"] if _CJK else ["sans-serif"]
plt.rcParams["axes.unicode_minus"] = False


def list_serial_ports():
    ports = list(list_ports.comports())
    if not ports:
        print("未发现串口。")
    for port in ports:
        print(f"{port.device}\t{port.description}\t{port.hwid}")


def parse_ball_line(line: str):
    """解析单行 [BALL] key=value 遥测；非数据行或残缺行返回 None。"""
    line = line.strip()
    if not line.startswith("[BALL] t="):
        return None
    kv = {}
    for token in line[7:].split():
        if "=" in token:
            key, value = token.split("=", 1)
            kv[key] = value
    try:
        sample = {"t": int(kv["t"])}
        sample.update({key: float(kv[key]) for key in FLOAT_FIELDS})
        sample.update({key: int(kv[key]) for key in INT_FIELDS})
        return sample
    except (KeyError, ValueError):
        return None


class BallTelemetry:
    def __init__(self, csv_file=None):
        self.lock = threading.Lock()
        self.csv_file = csv_file
        self.t = deque(maxlen=MAX_SAMPLES)
        self.data = {key: deque(maxlen=MAX_SAMPLES) for key in FIELDS}
        self.latest = {}
        self.bad_lines = 0

    def clear(self):
        with self.lock:
            self.t.clear()
            for values in self.data.values():
                values.clear()
            self.latest = {}

    def feed_line(self, line: str):
        if not line.strip().startswith("[BALL] t="):
            return
        sample = parse_ball_line(line)
        if sample is None:
            self.bad_lines += 1
            return
        with self.lock:
            self.t.append(sample["t"])
            for key in FIELDS:
                self.data[key].append(sample[key])
            self.latest = sample
        if self.csv_file is not None:
            row = [f"{time.time():.3f}", str(sample["t"])]
            row.extend(str(sample[key]) for key in FIELDS)
            self.csv_file.write(",".join(row) + "\n")
            self.csv_file.flush()

    def snapshot(self):
        with self.lock:
            return (list(self.t), {key: list(value) for key, value in self.data.items()},
                    dict(self.latest))


def serial_reader(port, baud, telemetry, log_file, stop_event):
    while not stop_event.is_set():
        try:
            uart = serial.Serial(port, baud, timeout=0.1)
        except (serial.SerialException, OSError) as exc:
            print(f"[系统] 打开 {port} 失败：{exc}", file=sys.stderr)
            if stop_event.wait(1.0):
                break
            continue
        print(f"[系统] 已连接 {port} @ {baud}")
        buffer = bytearray()
        try:
            while not stop_event.is_set():
                buffer += uart.read(512)
                while b"\n" in buffer:
                    raw, _, buffer = buffer.partition(b"\n")
                    line = raw.decode("utf-8", "replace").strip()
                    telemetry.feed_line(line)
                    if log_file is not None:
                        log_file.write(f"{time.time():.3f} {line}\n")
                        log_file.flush()
        except (serial.SerialException, OSError):
            print("[系统] 串口断开，正在重连……", file=sys.stderr)
        finally:
            uart.close()
        stop_event.wait(0.5)


def build_figure():
    figure, (position_ax, velocity_ax, constraint_ax, acceleration_ax,
             angle_ax) = plt.subplots(5, 1, figsize=(12, 12), sharex=True)
    try:
        figure.canvas.manager.set_window_title("H3 静止守球遥测")
    except Exception:
        pass
    figure.subplots_adjust(left=0.08, right=0.97, top=0.89, bottom=0.08, hspace=0.17)

    lines = {}
    lines["xr"], = position_ax.plot(
        [], [], color="#7f7f7f", lw=1.1, alpha=0.8, label="树莓派原始位置")
    lines["x"], = position_ax.plot([], [], color="#1f77b4", lw=1.6, label="控制位置")
    lines["xref"], = position_ax.plot(
        [], [], color="#ff7f0e", lw=1.2, ls="--", label="预瞄参考位置")
    position_ax.axhline(0.0, color="black", lw=0.8)
    position_ax.axhspan(-2.0, 2.0, color="#2ca02c", alpha=0.12, label="稳定区 ±2 mm")
    position_ax.set_ylabel("位置 x (mm)")

    lines["v"], = velocity_ax.plot([], [], color="#d62728", lw=1.5, label="球速度")
    lines["vref"], = velocity_ax.plot(
        [], [], color="#1f77b4", lw=1.3, ls="--", label="目标速度")
    velocity_ax.axhline(0.0, color="black", lw=0.8)
    velocity_ax.axhspan(-5.0, 5.0, color="#2ca02c", alpha=0.12, label="稳定区 ±5 mm/s")
    velocity_ax.set_ylabel("速度 v (mm/s)")

    lines["rx"], = constraint_ax.plot(
        [], [], color="#1f77b4", lw=1.3, label="终端位置残差 rx (mm)")
    lines["rv"], = constraint_ax.plot(
        [], [], color="#d62728", lw=1.2, ls="--", label="终端速度残差 rv (mm/s)")
    lines["ra"], = constraint_ax.plot(
        [], [], color="#2ca02c", lw=1.1, ls=":", label="终端加速度残差 ra (mm/s²)")
    constraint_ax.axhline(0.0, color="black", lw=0.8)
    constraint_ax.set_ylabel("终端残差")

    lines["a0"], = acceleration_ax.plot(
        [], [], color="#1f77b4", lw=1.3, label="由实际水管角得到的初始加速度")
    lines["aref"], = acceleration_ax.plot(
        [], [], color="#d62728", lw=1.4, ls="--", label="预瞄参考加速度")
    acceleration_ax.axhline(0.0, color="black", lw=0.8)
    acceleration_ax.set_ylabel("加速度 (mm/s²)")

    for key, label, style, color in (
            ("beam", "实际水管角", "-", "#1f77b4"),
            ("u", "命令角", "-", "#d62728"),
            ("aff", "模型前馈角", "--", "#ff7f0e"),
            ("fbp", "位置反馈", ":", "#8c564b"),
            ("fbv", "速度反馈", ":", "#2ca02c"),
            ("bias", "零偏", ":", "#9467bd")):
        lines[key], = angle_ax.plot([], [], style, color=color, lw=1.4, label=label)
    angle_ax.axhline(0.0, color="black", lw=0.8)
    angle_ax.axhline(4.0, color="gray", ls=":", lw=0.8)
    angle_ax.axhline(-4.0, color="gray", ls=":", lw=0.8, label="基础控制限角 ±4.0°")
    angle_ax.set_ylabel("水管角 (deg)")
    angle_ax.set_xlabel("设备运行时间 (s)")

    for axis in (position_ax, velocity_ax, constraint_ax, acceleration_ax, angle_ax):
        axis.grid(True, alpha=0.3)
        axis.legend(loc="upper right", fontsize=8, ncol=2)
    return figure, (position_ax, velocity_ax, constraint_ax, acceleration_ax,
                    angle_ax), lines


def main(argv=None):
    parser = argparse.ArgumentParser(description="H3 静止守球 Debug_Ex 遥测可视化")
    parser.add_argument("--port", help="串口，例如 COM7 或 /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--window", type=float, default=DEFAULT_WINDOW_S,
                        help="滚动显示窗口，单位秒")
    parser.add_argument("--csv", help="保存解析后的 CSV")
    parser.add_argument("--log", help="保存全部原始串口行")
    parser.add_argument("--list", action="store_true", help="列出串口后退出")
    parser.add_argument("--capture-only", action="store_true",
                        help="不打开窗口，只做有界串口记录")
    parser.add_argument("--duration", type=float, default=20.0,
                        help="--capture-only 的记录时长，单位秒")
    args = parser.parse_args(argv)

    if args.list:
        list_serial_ports()
        return 0
    if not args.port:
        print("需要 --port（或使用 --list 查看串口）。", file=sys.stderr)
        return 2
    if args.window <= 0.0:
        print("--window 必须大于 0。", file=sys.stderr)
        return 2
    if args.duration <= 0.0:
        print("--duration 必须大于 0。", file=sys.stderr)
        return 2

    log_file = open(args.log, "w", encoding="utf-8") if args.log else None
    csv_file = open(args.csv, "w", encoding="utf-8", newline="") if args.csv else None
    if csv_file is not None:
        csv_file.write(",".join(CSV_COLS) + "\n")

    telemetry = BallTelemetry(csv_file)
    stop_event = threading.Event()
    reader = threading.Thread(target=serial_reader,
                              args=(args.port, args.baud, telemetry, log_file, stop_event),
                              daemon=True)
    reader.start()
    if args.capture_only:
        try:
            stop_event.wait(args.duration)
        finally:
            stop_event.set()
            reader.join(timeout=1.5)
            if log_file is not None:
                log_file.close()
            if csv_file is not None:
                csv_file.close()
        t_ms, _, latest = telemetry.snapshot()
        if latest:
            print(
                f"[系统] 已记录 {len(t_ms)} 帧，设备跨度 "
                f"{(t_ms[-1] - t_ms[0]) * 0.001:.2f} s；"
                f"末帧 x={latest['x']:+.2f} mm v={latest['v']:+.2f} mm/s "
                f"beam={latest['beam']:+.3f} deg bias={latest['bias']:+.3f} deg")
            return 0
        print(f"[系统] {args.duration:.1f} s 内没有收到完整 [BALL] 数据帧。",
              file=sys.stderr)
        return 1

    figure, axes, lines = build_figure()
    state = {"paused": False}

    def on_key(event):
        if event.key == " ":
            state["paused"] = not state["paused"]
        elif event.key == "c":
            telemetry.clear()
        elif event.key == "q":
            plt.close(figure)

    figure.canvas.mpl_connect("key_press_event", on_key)

    def update(_frame):
        if state["paused"]:
            figure.suptitle("[已暂停] 空格继续", color="red")
            return list(lines.values())
        t_ms, data, latest = telemetry.snapshot()
        t_s = [value * 0.001 for value in t_ms]
        for key, line in lines.items():
            line.set_data(t_s, data[key])
        if t_s:
            right = t_s[-1]
            axes[0].set_xlim(max(0.0, right - args.window), right + 0.25)
        for axis in axes:
            axis.relim()
            axis.autoscale_view(scalex=False, scaley=True)
        if latest:
            state_name = ("WAIT", "ACTIVE", "HOLD ANGLE", "FAULT")
            st = latest["st"]
            st_text = state_name[st] if 0 <= st < len(state_name) else str(st)
            figure.suptitle(
                f"{st_text}  x={latest['x']:+.2f} mm  v={latest['v']:+.1f} mm/s  "
                f"vref={latest['vref']:+.1f} aref={latest['aref']:+.0f} "
                f"T/look={latest['T']:.2f}/{latest['look']:.2f}s "
                f"u={latest['u']:+.2f}° beam={latest['beam']:+.2f}°  "
                f"motor={latest['mspd']:+.0f}°/s sf={latest['sf']}Hz "
                f"ARR/CC={latest['arr']}/{latest['cc']} tick={latest['sdt']}/{latest['smax']}ms  "
                f"∫a={latest['ia']:+.1f}/ {-latest['v']:+.1f} "
                f"∫v={latest['iv']:+.1f}/{latest['et']:+.1f} mm "
                f"lim={latest['lim']} rate={latest['rate']:.0f}°/s "
                f"brake={latest['brk']} stuck={latest['stuck']} stable={latest['set']}  |  "
                f"age={latest['age']:.0f} ms fps={latest['fps']:.1f} Q={latest['q']} "
                f"mv={latest['mv']} px={latest['px']} gap={latest['gap']} inv={latest['inv']} "
                f"crc={latest['crc']} drop={latest['drop']}", fontsize=9)
        else:
            figure.suptitle("等待 [BALL] 遥测……请在板上进入 H3 Ball Static", fontsize=10)
        return list(lines.values())

    animation = FuncAnimation(figure, update, interval=80, blit=False,
                              cache_frame_data=False)
    _ = animation
    try:
        plt.show()
    finally:
        stop_event.set()
        reader.join(timeout=1.5)
        if log_file is not None:
            log_file.close()
        if csv_file is not None:
            csv_file.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
