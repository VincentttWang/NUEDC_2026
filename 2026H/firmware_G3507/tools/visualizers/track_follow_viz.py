#!/usr/bin/env python3
"""H 题循迹任务 [TRK] 遥测实时可视化上位机。

固件经 Debug_Ex/UART1（115200 8N1）每 50 ms 输出：

  [TRK] t=<ms> run=<id> req=<2|4|5|6> seg=<S1|S2|S3|S4>
        st=<FOLLOW|XLINE|OFFSET|STOP> fs=<NONE|LINE|ODOM|END> gm=<0|1>
        sen=<0|1> mask=<hex> n=<0..8>
        err=<...> cor=<...> vc=<m/s> ac=<m/s2> vs=<m/s>
        wref=<deg/s> wz=<deg/s> yaw=<deg> href=<deg> herr=<deg> hs=<0|1>
        vl=<m/s> vr=<m/s>
        dl=<%> dr=<%> sl=<m> sr=<m> s=<m> rem=<m>
        bx=<mm> be=<mm> ba=<deg> bvff=<deg> bfb=<deg> bbrk=<deg> bint=<deg> bfl=<flags>
        drop=<bytes>

窗口从上到下显示八路灰度、目标/实测轮速、航向角、指令/实测纵向加速度、
左右轮占空比、双轮里程与终点余量。默认以 0.12 m/s^2 为指令加速度告警线。

用法：
  python tools/visualizers/track_follow_viz.py --list
  python tools/visualizers/track_follow_viz.py --port COM7
  python tools/visualizers/track_follow_viz.py --port COM7 --csv track.csv --log track_raw.txt

依赖：pip install pyserial matplotlib
热键：空格=暂停/继续，c=清空，f=满屏，q=退出。
"""
from __future__ import annotations

import argparse
import sys
import threading
import time
from collections import deque

import serial
from serial.tools import list_ports

import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle


DEFAULT_BAUD = 115200
DEFAULT_WINDOW_S = 15.0
DEFAULT_ACCEL_LIMIT_MPS2 = 0.12
REQUIREMENT_ACCEL_LIMITS = {2: 0.30, 4: 0.12, 5: 0.12, 6: 0.12}
MAX_SAMPLES = 6000
FLOAT_FIELDS = (
    "err", "cor", "cff", "vc", "ac", "vs", "wref", "wz", "yaw", "href", "herr", "turn", "vl", "vr", "dl", "dr",
    "sl", "sr", "s", "rem",
)
OPTIONAL_FLOAT_FIELDS = ("cff", "vs", "wref", "wz", "yaw", "href", "herr", "turn")
BALL_FLOAT_FIELDS = (
    "bx", "be", "ba", "bvff", "bfb", "bbrk", "bint",
)
BALL_INT_FIELDS = ("bfl",)
CSV_COLUMNS = (
    "pc_time", "t", "run", "req", "seg", "ph", "st", "fs", "gm", "hs", "sen", "mask", "n",
    *FLOAT_FIELDS, *BALL_FLOAT_FIELDS, *BALL_INT_FIELDS, "drop",
)

_CJK_CANDIDATES = (
    "Microsoft YaHei", "SimHei", "SimSun", "Noto Sans CJK SC",
    "Source Han Sans SC", "WenQuanYi Zen Hei", "Arial Unicode MS",
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


def parse_track_line(line: str):
    """解析一条数据型 [TRK] 行；事件/其他前缀返回 None，坏数据抛 ValueError。"""
    line = line.strip()
    if not line.startswith("[TRK] t="):
        return None

    fields = {}
    for token in line[5:].split():
        if "=" in token:
            key, value = token.split("=", 1)
            fields[key] = value

    try:
        parsed = {
            "t": int(fields["t"]),
            "run": int(fields.get("run", "0")),
            "req": int(fields.get("req", "0")),
            "seg": fields.get("seg", "?"),
            "ph": fields.get("ph", "?"),
            "st": fields["st"],
            "fs": fields.get("fs", "NONE"),
            "gm": int(fields.get("gm", "0")),
            "hs": int(fields.get("hs", "0")),
            "sen": int(fields["sen"]),
            "mask": int(fields["mask"], 16),
            "n": int(fields["n"]),
            **{
                key: float(fields.get(key, "0.0"))
                if key in OPTIONAL_FLOAT_FIELDS else float(fields[key])
                for key in FLOAT_FIELDS
            },
            **{
                key: float(fields.get(key, "0.0"))
                for key in BALL_FLOAT_FIELDS
            },
            **{
                key: int(fields.get(key, "0"))
                for key in BALL_INT_FIELDS
            },
            "drop": int(fields["drop"]),
        }
    except (KeyError, ValueError) as exc:
        raise ValueError(f"无效 [TRK] 遥测: {line}") from exc

    if parsed["sen"] not in (0, 1):
        raise ValueError(f"sen 超出范围: {parsed['sen']}")
    if parsed["gm"] not in (0, 1):
        raise ValueError(f"gm 超出范围: {parsed['gm']}")
    if parsed["hs"] not in (0, 1):
        raise ValueError(f"hs must be 0 or 1: {parsed['hs']}")
    if not 0 <= parsed["mask"] <= 0xFF:
        raise ValueError(f"mask 超出范围: {parsed['mask']}")
    if not 0 <= parsed["n"] <= 8:
        raise ValueError(f"n 超出范围: {parsed['n']}")
    return parsed


class TrackTelemetry:
    """线程安全的 [TRK] 缓存（串口线程写，绘图线程读）。"""

    def __init__(self, csv_file=None):
        self._lock = threading.Lock()
        self._csv_file = csv_file
        self._time_ms = deque(maxlen=MAX_SAMPLES)
        self._run = deque(maxlen=MAX_SAMPLES)
        self._state = deque(maxlen=MAX_SAMPLES)
        self._finish_source = deque(maxlen=MAX_SAMPLES)
        self._gyro_only = deque(maxlen=MAX_SAMPLES)
        self._requirement = deque(maxlen=MAX_SAMPLES)
        self._segment = deque(maxlen=MAX_SAMPLES)
        self._sensor_ready = deque(maxlen=MAX_SAMPLES)
        self._mask = deque(maxlen=MAX_SAMPLES)
        self._black_count = deque(maxlen=MAX_SAMPLES)
        self._mcu_drop = deque(maxlen=MAX_SAMPLES)
        self._data = {field: deque(maxlen=MAX_SAMPLES) for field in FLOAT_FIELDS}
        self._latest = {}
        self.bad_lines = 0
        self.connected = False

    def clear(self) -> None:
        with self._lock:
            self._time_ms.clear()
            self._run.clear()
            self._state.clear()
            self._finish_source.clear()
            self._gyro_only.clear()
            self._requirement.clear()
            self._segment.clear()
            self._sensor_ready.clear()
            self._mask.clear()
            self._black_count.clear()
            self._mcu_drop.clear()
            for values in self._data.values():
                values.clear()
            self._latest = {}
            self.bad_lines = 0

    def set_connected(self, connected: bool) -> None:
        with self._lock:
            self.connected = connected

    def feed_line(self, line: str) -> None:
        if line.strip().startswith("[TRK] --- enter"):
            # 新一轮任务的设备时间戳会归零，自动切开两次试跑的数据。
            self.clear()
            return
        try:
            parsed = parse_track_line(line)
        except ValueError:
            with self._lock:
                self.bad_lines += 1
            return
        if parsed is None:
            return

        with self._lock:
            self._time_ms.append(parsed["t"])
            self._run.append(parsed["run"])
            self._requirement.append(parsed["req"])
            self._segment.append(parsed["seg"])
            self._state.append(parsed["st"])
            self._finish_source.append(parsed["fs"])
            self._gyro_only.append(parsed["gm"])
            self._sensor_ready.append(parsed["sen"])
            self._mask.append(parsed["mask"])
            self._black_count.append(parsed["n"])
            self._mcu_drop.append(parsed["drop"])
            for field in FLOAT_FIELDS:
                self._data[field].append(parsed[field])
            self._latest = dict(parsed)

        if self._csv_file is not None:
            row = [
                f"{time.time():.3f}", str(parsed["t"]), str(parsed["run"]),
                str(parsed["req"]), parsed["seg"], parsed["ph"], parsed["st"],
                parsed["fs"], str(parsed["gm"]),
                str(parsed["sen"]), f"{parsed['mask']:02X}", str(parsed["n"]),
                *(f"{parsed[field]:.6f}" for field in FLOAT_FIELDS),
                str(parsed["drop"]),
            ]
            self._csv_file.write(",".join(row) + "\n")
            self._csv_file.flush()

    def snapshot(self):
        with self._lock:
            return {
                "t": list(self._time_ms),
                "run": list(self._run),
                "req": list(self._requirement),
                "seg": list(self._segment),
                "state": list(self._state),
                "fs": list(self._finish_source),
                "gm": list(self._gyro_only),
                "sen": list(self._sensor_ready),
                "mask": list(self._mask),
                "n": list(self._black_count),
                "drop": list(self._mcu_drop),
                "data": {key: list(values) for key, values in self._data.items()},
                "latest": dict(self._latest),
                "bad_lines": self.bad_lines,
                "connected": self.connected,
            }


def serial_reader(port: str, baud: int, telemetry: TrackTelemetry,
                  log_file, stop_event: threading.Event) -> None:
    """带自动重连的串口读取线程。"""
    while not stop_event.is_set():
        try:
            serial_port = serial.Serial(port, baud, timeout=0.1)
        except (serial.SerialException, OSError) as exc:
            telemetry.set_connected(False)
            print(f"[系统] 打开 {port} 失败: {exc}", file=sys.stderr)
            if stop_event.wait(1.0):
                return
            continue

        telemetry.set_connected(True)
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
            telemetry.set_connected(False)
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


def filtered_measured_accel(time_ms, left_speed, right_speed, alpha=0.25):
    """由平均实测轮速差分并 EMA，供检查真实加速度趋势（不参与控制）。"""
    count = min(len(time_ms), len(left_speed), len(right_speed))
    if count == 0:
        return []
    result = [0.0]
    previous_speed = (left_speed[0] + right_speed[0]) * 0.5
    filtered = 0.0
    for index in range(1, count):
        speed = (left_speed[index] + right_speed[index]) * 0.5
        dt = (time_ms[index] - time_ms[index - 1]) / 1000.0
        raw = (speed - previous_speed) / dt if 0.001 <= dt <= 1.0 else filtered
        filtered += alpha * (raw - filtered)
        result.append(filtered)
        previous_speed = speed
    return result


def build_figure(accel_limit):
    figure = plt.figure(figsize=(13, 11.5))
    grid = figure.add_gridspec(
        6, 1, height_ratios=(0.65, 2.0, 1.25, 1.6, 1.4, 1.9)
    )
    axis_sensor = figure.add_subplot(grid[0])
    axis_speed = figure.add_subplot(grid[1])
    axis_yaw = figure.add_subplot(grid[2], sharex=axis_speed)
    axis_accel = figure.add_subplot(grid[3], sharex=axis_speed)
    axis_duty = figure.add_subplot(grid[4], sharex=axis_speed)
    axis_distance = figure.add_subplot(grid[5], sharex=axis_speed)
    axis_remaining = axis_distance.twinx()

    try:
        figure.canvas.manager.set_window_title("H 题循迹遥测")
    except Exception:
        pass
    figure.subplots_adjust(hspace=0.22, top=0.90, bottom=0.07, left=0.08, right=0.90)

    axis_sensor.set_xlim(0, 8)
    axis_sensor.set_ylim(0, 1)
    axis_sensor.set_yticks([])
    axis_sensor.set_xticks([])
    axis_sensor.set_frame_on(False)
    sensor_boxes = []
    sensor_labels = []
    for index in range(8):
        box = Rectangle((index + 0.08, 0.12), 0.84, 0.70,
                        facecolor="#d9d9d9", edgecolor="#777777")
        axis_sensor.add_patch(box)
        label = axis_sensor.text(index + 0.5, 0.47, f"X{index + 1}",
                                 ha="center", va="center")
        sensor_boxes.append(box)
        sensor_labels.append(label)
    sensor_caption = axis_sensor.text(4.0, 0.96, "等待灰度遥测", ha="center", va="top")

    lines = {}
    lines["vc"], = axis_speed.plot([], [], "--", color="#111111", lw=1.4, label="纵向指令")
    lines["vlt"], = axis_speed.plot([], [], "--", color="#1f77b4", lw=1.0, label="左轮目标")
    lines["vrt"], = axis_speed.plot([], [], "--", color="#d62728", lw=1.0, label="右轮目标")
    lines["vl"], = axis_speed.plot([], [], color="#1f77b4", lw=1.2, label="左轮实测")
    lines["vr"], = axis_speed.plot([], [], color="#d62728", lw=1.2, label="右轮实测")
    lines["vavg"], = axis_speed.plot([], [], color="#2ca02c", lw=1.6, label="平均实测")
    axis_speed.set_ylabel("速度 (m/s)")
    axis_speed.legend(loc="upper right", ncol=3, fontsize=8)

    lines["yaw"], = axis_yaw.plot(
        [], [], color="#9467bd", lw=1.5, label="JY61P yaw"
    )
    axis_yaw.set_ylabel("航向角 (°)")
    axis_yaw.legend(loc="upper right", fontsize=8)

    lines["ac"], = axis_accel.plot([], [], color="#111111", lw=1.5, label="指令 ac")
    lines["ameas"], = axis_accel.plot([], [], color="#ff7f0e", lw=1.2, label="实测估计 EMA")
    lines["alim_pos"] = axis_accel.axhline(
        accel_limit, color="#d62728", ls=":", lw=1.0, label="任务加速度限值")
    lines["alim_neg"] = axis_accel.axhline(
        -accel_limit, color="#d62728", ls=":", lw=1.0)
    axis_accel.set_ylabel("纵向加速度 (m/s²)")
    axis_accel.legend(loc="upper right", ncol=3, fontsize=8)

    lines["dl"], = axis_duty.plot([], [], color="#1f77b4", label="左轮")
    lines["dr"], = axis_duty.plot([], [], color="#d62728", label="右轮")
    axis_duty.set_ylabel("占空比 (%)")
    axis_duty.legend(loc="upper right", ncol=2, fontsize=8)

    lines["sl"], = axis_distance.plot([], [], color="#1f77b4", lw=1.0, label="左轮里程")
    lines["sr"], = axis_distance.plot([], [], color="#d62728", lw=1.0, label="右轮里程")
    lines["s"], = axis_distance.plot([], [], color="#2ca02c", lw=1.7, label="平均里程")
    lines["rem"], = axis_remaining.plot([], [], color="#9467bd", lw=1.5, label="终点余量")
    axis_distance.set_ylabel("里程 (m)")
    axis_remaining.set_ylabel("终点余量 (m)", color="#9467bd")
    axis_distance.set_xlabel("时间 (秒，相对首样本)")
    axis_distance.legend(loc="upper left", ncol=3, fontsize=8)
    axis_remaining.legend(loc="upper right", fontsize=8)

    for axis in (axis_speed, axis_yaw, axis_accel, axis_duty, axis_distance):
        axis.axhline(0.0, color="gray", linewidth=0.5)
        axis.grid(True, alpha=0.3)

    axes = (axis_speed, axis_yaw, axis_accel, axis_duty, axis_distance)
    return (figure, axes, axis_remaining, lines, sensor_boxes,
            sensor_labels, sensor_caption)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="H 题循迹 [TRK] 遥测可视化")
    parser.add_argument("--port", help="串口，如 COM7 / /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--window", type=float, default=DEFAULT_WINDOW_S,
                        help="滚动显示时间窗（秒）")
    parser.add_argument("--accel-limit", type=float,
                        help="覆盖任务自带的指令加速度告警绝对值（m/s^2）")
    parser.add_argument("--csv", help="将解析后的数据保存为 CSV")
    parser.add_argument("--log", help="保存带 PC 时间戳的原始串口行")
    parser.add_argument("--list", action="store_true", help="列出串口后退出")
    args = parser.parse_args(argv)

    if args.list:
        list_serial_ports()
        return 0
    if not args.port:
        print("需要 --port（可先用 --list 查看）。", file=sys.stderr)
        return 2
    if args.window <= 0.0 or (args.accel_limit is not None and args.accel_limit <= 0.0):
        print("--window 和 --accel-limit 必须大于 0。", file=sys.stderr)
        return 2
    if not _CJK_FONT:
        print("警告: 未发现中文字体，图表中文可能显示为方块。", file=sys.stderr)

    log_file = open(args.log, "w", encoding="utf-8") if args.log else None
    csv_file = open(args.csv, "w", encoding="utf-8", newline="") if args.csv else None
    if csv_file is not None:
        csv_file.write(",".join(CSV_COLUMNS) + "\n")

    telemetry = TrackTelemetry(csv_file=csv_file)
    stop_event = threading.Event()
    reader = threading.Thread(
        target=serial_reader,
        args=(args.port, args.baud, telemetry, log_file, stop_event),
        daemon=True,
    )
    reader.start()

    print(f"监听 {args.port} @ {args.baud}。空格=暂停 c=清空 q=退出")
    (figure, axes, axis_remaining, lines, sensor_boxes,
     sensor_labels, sensor_caption) = build_figure(
         args.accel_limit or DEFAULT_ACCEL_LIMIT_MPS2)
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
            return list(lines.values()) + sensor_boxes + sensor_labels

        snapshot = telemetry.snapshot()
        time_ms = snapshot["t"]
        timeline = rebase_time(time_ms)
        data = snapshot["data"]
        latest = snapshot["latest"]

        for field in FLOAT_FIELDS:
            if field in lines:
                count = min(len(timeline), len(data[field]))
                lines[field].set_data(timeline[:count], data[field][:count])

        speed_count = min(len(timeline), len(data["vl"]), len(data["vr"]))
        target_count = min(len(timeline), len(data["vc"]), len(data["vs"]))
        left_target = [
            data["vc"][index] + data["vs"][index]
            for index in range(target_count)
        ]
        right_target = [
            data["vc"][index] - data["vs"][index]
            for index in range(target_count)
        ]
        lines["vlt"].set_data(timeline[:target_count], left_target)
        lines["vrt"].set_data(timeline[:target_count], right_target)
        average_speed = [
            (data["vl"][index] + data["vr"][index]) * 0.5
            for index in range(speed_count)
        ]
        lines["vavg"].set_data(timeline[:speed_count], average_speed)
        measured_accel = filtered_measured_accel(time_ms, data["vl"], data["vr"])
        lines["ameas"].set_data(timeline[:len(measured_accel)], measured_accel)

        if timeline:
            end = timeline[-1]
            start = max(0.0, end - args.window)
            for axis in axes:
                axis.set_xlim(start, end + 0.3)

        for axis in (*axes, axis_remaining):
            axis.relim()
            axis.autoscale_view(scalex=False, scaley=True)

        if latest:
            active_accel_limit = args.accel_limit or REQUIREMENT_ACCEL_LIMITS.get(
                latest["req"], DEFAULT_ACCEL_LIMIT_MPS2)
            lines["alim_pos"].set_ydata([active_accel_limit, active_accel_limit])
            lines["alim_neg"].set_ydata([-active_accel_limit, -active_accel_limit])
            mask = latest["mask"]
            for index, (box, label) in enumerate(zip(sensor_boxes, sensor_labels)):
                detected = bool(mask & (1 << index))
                box.set_facecolor("#202020" if detected else "#d9d9d9")
                label.set_color("white" if detected else "black")
            sensor_caption.set_text(
                f"X1 → X8（黑色=检测到黑线）  mask={mask:02X}  命中={latest['n']}  "
                f"传感器={'OK' if latest['sen'] else 'WAIT/STALE'}")

            actual_speed = (latest["vl"] + latest["vr"]) * 0.5
            alerts = []
            if not latest["sen"]:
                alerts.append("灰度无效")
            if abs(latest["ac"]) > active_accel_limit + 1e-6:
                alerts.append("加速度指令超限")
            if latest["drop"]:
                alerts.append(f"MCU TX丢{latest['drop']}B")
            if latest["gm"]:
                alerts.append("弯道纯陀螺保持")
            if snapshot["bad_lines"]:
                alerts.append(f"解析坏行{snapshot['bad_lines']}")
            connection = "已连接" if snapshot["connected"] else "已断开/重连中"
            alert_text = " | ⚠ " + ", ".join(alerts) if alerts else ""
            figure.suptitle(
                f"{connection} | run={latest['run']} H{latest['req']} "
                f"{latest['seg']} {latest['st']} fs={latest['fs']} | "
                f"t={latest['t'] / 1000.0:.2f}s | "
                f"v 指令/实测={latest['vc']:.3f}/{actual_speed:.3f}m/s | "
                f"半差速={latest['vs']:+.3f}m/s | "
                f"ω参考/实测={latest['wref']:+.1f}/{latest['wz']:+.1f}°/s | "
                f"航向={latest['yaw']:+.1f}° | "
                f"ac={latest['ac']:+.3f}m/s² | err={latest['err']:+.1f}mm "
                f"cor={latest['cor']:+.2f} | s={latest['s']:.3f}m "
                f"rem={latest['rem']:.3f}m{alert_text}",
                color="red" if alerts else "black", fontsize=9,
            )
        else:
            sensor_caption.set_text("等待 [TRK] 遥测")
            figure.suptitle(
                "等待 [TRK] 遥测…请在板上进入 Main Menu -> Line Follow",
                fontsize=10,
            )
        return list(lines.values()) + sensor_boxes + sensor_labels

    animation = FuncAnimation(
        figure, update, interval=80, blit=False, cache_frame_data=False,
    )
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
