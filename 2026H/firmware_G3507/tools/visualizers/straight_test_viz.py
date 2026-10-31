#!/usr/bin/env python3
"""直行测试 Debug_Ex 遥测实时可视化。

固件在 Main Menu -> Straight Test 的九种任务中，通过 UART1/Debug_Ex
(115200 8N1) 每 20 ms 输出：

  [STR] t=<ms> m=<0..8> imu=<0|1> cmd=<target> act=<applied> phase=<0|1>
        dl=<%> dr=<%>
        vl=<m/s> vr=<m/s> xl=<m> xr=<m> yaw=<deg> gz=<deg/s>
        iyaw=<deg> corr=<%>

图表从上到下依次是左右轮占空比、速度、距离、yaw/积分yaw/gz。IMU 未就绪时
姿态数据不画入曲线，避免把占位零值当成真实姿态。

用法：
  python tools/visualizers/straight_test_viz.py --list
  python tools/visualizers/straight_test_viz.py --port COM7
  python tools/visualizers/straight_test_viz.py --port COM7 --csv straight.csv --log straight_raw.txt

依赖：
  pip install pyserial matplotlib

热键：空格=暂停/继续，c=清空，q=退出。
"""
from __future__ import annotations

import argparse
import math
import sys
import threading
import time
from collections import deque

import serial
from serial.tools import list_ports

import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.animation import FuncAnimation


DEFAULT_BAUD = 115200
DEFAULT_WINDOW_S = 15.0
MAX_SAMPLES = 6000
FLOAT_FIELDS = ("cmd", "act", "dl", "dr", "vl", "vr", "xl", "xr", "yaw", "gz", "iyaw", "corr")
CSV_COLUMNS = ("pc_time", "t", "m", "imu", "phase", *FLOAT_FIELDS)
MODE_NAMES = {
    0: "Duty Open",
    1: "Speed Closed",
    2: "Duty+Gyro Rate",
    3: "Duty+Yaw Hold",
    4: "Ramp Yaw Hold",
    5: "80 Rate->Yaw",
    6: "80 Enc->Yaw",
    7: "80 Int->Yaw",
    8: "100 Int->Yaw",
}

# 选择已安装的中文字体，避免 Matplotlib 标签显示成方块。
_CJK_CANDIDATES = (
    "Microsoft YaHei",
    "SimHei",
    "SimSun",
    "Noto Sans CJK SC",
    "Source Han Sans SC",
    "WenQuanYi Zen Hei",
    "Arial Unicode MS",
)
_AVAILABLE_FONTS = {font.name for font in font_manager.fontManager.ttflist}
_CJK_FONT = next((name for name in _CJK_CANDIDATES if name in _AVAILABLE_FONTS), None)
plt.rcParams["font.sans-serif"] = [_CJK_FONT, "sans-serif"] if _CJK_FONT else ["sans-serif"]
plt.rcParams["axes.unicode_minus"] = False


def list_serial_ports() -> None:
    """列出当前可见串口。"""
    ports = list(list_ports.comports())
    if not ports:
        print("未发现串口。")
        return
    for port in ports:
        print(f"{port.device}\t{port.description}\t{port.hwid}")


class StraightTelemetry:
    """线程安全的 [STR] 遥测缓存。"""

    def __init__(self, csv_file=None):
        self._lock = threading.Lock()
        self._csv_file = csv_file
        self._time_ms = deque(maxlen=MAX_SAMPLES)
        self._mode = deque(maxlen=MAX_SAMPLES)
        self._imu_valid = deque(maxlen=MAX_SAMPLES)
        self._data = {field: deque(maxlen=MAX_SAMPLES) for field in FLOAT_FIELDS}
        self._latest = {}
        self.bad_lines = 0

    def clear(self) -> None:
        with self._lock:
            self._time_ms.clear()
            self._mode.clear()
            self._imu_valid.clear()
            for values in self._data.values():
                values.clear()
            self._latest = {}

    def feed_line(self, line: str) -> None:
        line = line.strip()
        if not line.startswith("[STR]"):
            return

        fields = {}
        for token in line[5:].split():
            if "=" in token:
                key, value = token.split("=", 1)
                fields[key] = value

        try:
            timestamp = int(fields["t"])
            mode = int(fields["m"])
            imu_valid = int(fields["imu"])
            phase = int(fields["phase"])
            values = {key: float(fields[key]) for key in FLOAT_FIELDS}
        except (KeyError, ValueError):
            self.bad_lines += 1
            return

        latest = {
            "t": timestamp,
            "m": mode,
            "imu": imu_valid,
            "phase": phase,
            **values,
        }
        with self._lock:
            self._time_ms.append(timestamp)
            self._mode.append(mode)
            self._imu_valid.append(imu_valid)
            for key, value in values.items():
                self._data[key].append(value)
            self._latest = latest

        if self._csv_file is not None:
            row = [
                f"{time.time():.3f}",
                str(timestamp),
                str(mode),
                str(imu_valid),
                str(phase),
                *(f"{values[key]:.5f}" for key in FLOAT_FIELDS),
            ]
            self._csv_file.write(",".join(row) + "\n")
            self._csv_file.flush()

    def snapshot(self):
        with self._lock:
            return (
                list(self._time_ms),
                list(self._mode),
                list(self._imu_valid),
                {key: list(values) for key, values in self._data.items()},
                dict(self._latest),
            )


def serial_reader(port: str, baud: int, telemetry: StraightTelemetry,
                  log_file, stop_event: threading.Event) -> None:
    """带自动重连的串口读取线程。"""
    while not stop_event.is_set():
        try:
            serial_port = serial.Serial(port, baud, timeout=0.1)
        except (serial.SerialException, OSError) as exc:
            print(f"[系统] 打开 {port} 失败: {exc}", file=sys.stderr)
            if stop_event.wait(1.0):
                return
            continue

        print(f"[系统] 已连接 {port} @ {baud}")
        buffer = bytearray()
        try:
            while not stop_event.is_set():
                chunk = serial_port.read(512)
                if not chunk:
                    continue
                buffer.extend(chunk)
                while b"\n" in buffer:
                    raw, _, buffer = buffer.partition(b"\n")
                    line = raw.decode("utf-8", "replace").strip()
                    telemetry.feed_line(line)
                    if log_file is not None:
                        log_file.write(f"{time.time():.3f} {line}\n")
                        log_file.flush()
        except (serial.SerialException, OSError) as exc:
            print(f"[系统] 串口断开: {exc}; 正在重连…", file=sys.stderr)
        finally:
            try:
                serial_port.close()
            except Exception:
                pass

        if stop_event.wait(0.5):
            return


def rebase_time(time_ms):
    if not time_ms:
        return []
    origin = time_ms[0]
    return [(timestamp - origin) / 1000.0 for timestamp in time_ms]


def build_figure():
    figure, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    axis_duty, axis_speed, axis_distance, axis_yaw = axes
    axis_rate = axis_yaw.twinx()

    try:
        figure.canvas.manager.set_window_title("直行测试遥测")
    except Exception:
        pass
    figure.subplots_adjust(hspace=0.18, top=0.91, bottom=0.07, left=0.08, right=0.90)

    lines = {}
    lines["dl"], = axis_duty.plot([], [], color="#1f77b4", label="左轮")
    lines["dr"], = axis_duty.plot([], [], color="#d62728", label="右轮")
    axis_duty.set_ylabel("占空比 (%)")

    lines["vl"], = axis_speed.plot([], [], color="#1f77b4", label="左轮")
    lines["vr"], = axis_speed.plot([], [], color="#d62728", label="右轮")
    axis_speed.set_ylabel("速度 (m/s)")

    lines["xl"], = axis_distance.plot([], [], color="#1f77b4", label="左轮")
    lines["xr"], = axis_distance.plot([], [], color="#d62728", label="右轮")
    axis_distance.set_ylabel("距离 (m)")

    lines["yaw"], = axis_yaw.plot([], [], color="#2ca02c", label="yaw")
    lines["iyaw"], = axis_yaw.plot(
        [], [], color="#ff7f0e", linestyle="--", label="integrated yaw")
    lines["gz"], = axis_rate.plot([], [], color="#9467bd", label="gz")
    axis_yaw.set_ylabel("yaw (deg)", color="#2ca02c")
    axis_rate.set_ylabel("gz (deg/s)", color="#9467bd")
    axis_yaw.set_xlabel("时间 (秒，相对首样本)")

    for axis in axes:
        axis.axhline(0.0, color="gray", linewidth=0.5)
        axis.grid(True, alpha=0.3)
    for axis in (axis_duty, axis_speed, axis_distance):
        axis.legend(loc="upper right", ncol=2, fontsize=8)
    axis_yaw.legend(loc="upper left", fontsize=8)
    axis_rate.legend(loc="upper right", fontsize=8)

    return figure, axes, axis_rate, lines


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="直行测试 [STR] 遥测可视化")
    parser.add_argument("--port", help="串口，如 COM7 / /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--window", type=float, default=DEFAULT_WINDOW_S,
                        help="滚动显示时间窗（秒）")
    parser.add_argument("--csv", help="将解析后的数据保存为 CSV")
    parser.add_argument("--log", help="保存带 PC 时间戳的原始遥测行")
    parser.add_argument("--list", action="store_true", help="列出串口后退出")
    args = parser.parse_args(argv)

    if args.list:
        list_serial_ports()
        return 0
    if not args.port:
        print("需要 --port（可先用 --list 查看）。", file=sys.stderr)
        return 2
    if args.window <= 0.0:
        print("--window 必须大于 0。", file=sys.stderr)
        return 2
    if not _CJK_FONT:
        print("警告: 未发现中文字体，图表中文可能显示为方块。", file=sys.stderr)

    log_file = open(args.log, "w", encoding="utf-8") if args.log else None
    csv_file = open(args.csv, "w", encoding="utf-8", newline="") if args.csv else None
    if csv_file is not None:
        csv_file.write(",".join(CSV_COLUMNS) + "\n")

    telemetry = StraightTelemetry(csv_file=csv_file)
    stop_event = threading.Event()
    reader = threading.Thread(
        target=serial_reader,
        args=(args.port, args.baud, telemetry, log_file, stop_event),
        daemon=True,
    )
    reader.start()

    print(f"监听 {args.port} @ {args.baud}。空格=暂停 c=清空 q=退出")
    figure, axes, axis_rate, lines = build_figure()
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
            figure.suptitle("[已暂停] 空格继续", color="red", fontsize=10)
            return list(lines.values())

        time_ms, _modes, imu_valid, data, latest = telemetry.snapshot()
        timeline = rebase_time(time_ms)

        for field in ("dl", "dr", "vl", "vr", "xl", "xr"):
            count = min(len(timeline), len(data[field]))
            lines[field].set_data(timeline[:count], data[field][:count])

        imu_count = min(len(timeline), len(imu_valid), len(data["yaw"]),
                        len(data["iyaw"]), len(data["gz"]))
        yaw = [data["yaw"][i] if imu_valid[i] else math.nan for i in range(imu_count)]
        iyaw = [data["iyaw"][i] if imu_valid[i] else math.nan for i in range(imu_count)]
        gz = [data["gz"][i] if imu_valid[i] else math.nan for i in range(imu_count)]
        lines["yaw"].set_data(timeline[:imu_count], yaw)
        lines["iyaw"].set_data(timeline[:imu_count], iyaw)
        lines["gz"].set_data(timeline[:imu_count], gz)

        if timeline:
            end = timeline[-1]
            start = max(0.0, end - args.window)
            for axis in axes:
                axis.set_xlim(start, end + 0.3)

        for axis in (*axes, axis_rate):
            axis.relim()
            axis.autoscale_view(scalex=False, scaley=True)

        if latest:
            mode_name = MODE_NAMES.get(latest["m"], f"Unknown({latest['m']})")
            imu_text = "IMU OK" if latest["imu"] else "IMU WAIT/ERR"
            phase_text = ""
            if latest["m"] == 4:
                phase_text = " | CRUISE" if latest["phase"] else " | RAMP"
            elif latest["m"] == 5:
                phase_text = " | YAW" if latest["phase"] else " | RATE"
            elif latest["m"] == 6:
                phase_text = " | YAW" if latest["phase"] else " | ENCODER"
            elif latest["m"] == 7:
                phase_text = " | FUSED YAW" if latest["phase"] else " | INTEGRATED YAW"
            elif latest["m"] == 8:
                phase_text = " | FUSED YAW" if latest["phase"] else " | INTEGRATED YAW"
            figure.suptitle(
                f"{mode_name} | cmd/act={latest['cmd']:+.3f}/{latest['act']:+.3f} | "
                f"duty L/R={latest['dl']:+.1f}/{latest['dr']:+.1f}% | "
                f"speed L/R={latest['vl']:+.3f}/{latest['vr']:+.3f} m/s | "
                f"corr={latest['corr']:+.2f}% | {imu_text}{phase_text}",
                fontsize=9,
            )
        else:
            figure.suptitle("等待 [STR] 遥测…请在板上进入 Straight Test 的任一模式", fontsize=10)

        return list(lines.values())

    animation = FuncAnimation(
        figure,
        update,
        interval=80,
        blit=False,
        cache_frame_data=False,
    )
    # 保持引用，避免 FuncAnimation 在窗口运行期间被回收。
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
