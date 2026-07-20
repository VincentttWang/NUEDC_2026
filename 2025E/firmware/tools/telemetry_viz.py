#!/usr/bin/env python3
"""2025E debug 串口遥测可视化上位机。

解析 MCU 经 debug 串口 (Debug_Ex=UART1, TX=PA8, 115200) 输出的任务遥测并实时绘图:
  Aim track:  [AIM] t=<ms> st ex ey cy cp pit yaw vf drp   (更新同步, 带时间戳)
  E1 循迹:    [E1]  t=<ms> gs=0x?? st lap edge
  事件行:      [AIM] LOCK/LOST/start/stop, [E1] corner#/edge/LINE LOST/FINISH ...

窗口内容:
  - Aim 误差、实际/候选 PID 指令、角度、E1 灰度 8 路位图, 横轴=设备时间戳
  - PID 调参侧栏: 直接输入 yaw/pitch Kp/Ki/Kd、限幅、死区与 yaw 最小驱动速度
  - 顶部显示 RMS/IAE/超调(过零)/限幅比例等闭环指标，可标记一段试验并导出报告
热键: 空格=暂停/继续, c=清空, m=开始/结束试验片段,
       s=导出 PID 报告, q=退出

用法:
  python telemetry_viz.py --port COM7 [--baud 115200] [--window 20]
  python telemetry_viz.py --port COM7 --log raw.txt --csv tel.csv
  python telemetry_viz.py --list          # 列出串口

依赖: pip install pyserial pyqtgraph PyQt5 matplotlib

性能说明: 默认入口使用 PyQtGraph，每 50 ms 仅更新滚动时间窗内的数据；
Matplotlib 代码仅保留为旧实现兼容辅助，不参与默认界面渲染。
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from collections import deque
from pathlib import Path

import serial
from serial.tools import list_ports

import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import TextBox
import numpy as np
import pyqtgraph as pg
from PyQt5 import QtCore, QtGui, QtWidgets

# 启动时从 Matplotlib 已发现的字体中选择，不能只假定 Windows 字体名存在。
_CJK_FONT_CANDIDATES = ("Microsoft YaHei", "SimHei", "SimSun", "Noto Sans CJK SC",
                        "Source Han Sans SC", "WenQuanYi Zen Hei", "Arial Unicode MS")
_AVAILABLE_FONT_NAMES = {font.name for font in font_manager.fontManager.ttflist}
ACTIVE_CJK_FONT = next((font for font in _CJK_FONT_CANDIDATES
                        if font in _AVAILABLE_FONT_NAMES), None)
if ACTIVE_CJK_FONT:
    plt.rcParams["font.sans-serif"] = [ACTIVE_CJK_FONT, "sans-serif"]
else:
    plt.rcParams["font.sans-serif"] = ["sans-serif"]
plt.rcParams["axes.unicode_minus"] = False   # 负号用 ASCII, 避免字体缺字

GRAYSCALE_LANES = 8          # E1 灰度传感器路数
DEFAULT_WINDOW_S = 20.0      # 滚动时间窗 (秒)
MAX_SAMPLES = 6000           # 每条曲线最多缓存点数
EVENT_SHOW = 14              # 事件面板显示条数
CSV_COLS = ["pc_time", "stream", "t", "st", "ex", "ey", "cy", "cp",
            "pit", "yaw", "vf", "drp", "gs", "lap", "edge"]

# 固件 core/gimbal_tracking 当前默认值。这些值仅用于候选曲线仿真与导出；
# 调试串口的 RX 尚未定义参数协议，界面输入不会向 MCU 写入数据。
PID_DEFAULTS = {
    "yaw_kp": 5.0, "yaw_ki": 0.0, "yaw_kd": 0.0,
    "pitch_kp": 0.20, "pitch_ki": 0.0, "pitch_kd": 0.0,
    "limit": 120.0, "deadband": 4.0, "yaw_min_speed": 6.0,
}


def list_serial_ports():
    ports = list(list_ports.comports())
    if not ports:
        print("未发现串口。")
        return
    for p in ports:
        print(f"{p.device}\t{p.description}\t{p.hwid}")


class Telemetry:
    """线程安全的遥测数据缓存 (串口线程写, 绘图线程读)。"""

    def __init__(self, csvf=None):
        self.lock = threading.Lock()
        self.csvf = csvf
        # AIM 绘图时间序列
        self.aim = {k: deque(maxlen=MAX_SAMPLES)
                    for k in ("t", "ex", "ey", "cy", "cp", "pit", "yaw")}
        self.aim_vf = 0
        self.aim_drp = 0
        # E1 绘图时间序列
        self.e1_t = deque(maxlen=MAX_SAMPLES)
        self.e1_gs = deque(maxlen=MAX_SAMPLES)   # 每项 8bit 整数
        self.e1_lap = 0
        self.e1_edge = 0
        # 事件 (pc_time, 文本)
        self.events = deque(maxlen=400)
        # 最新一条各字段 (供数值读出)
        self.latest = {"AIM": {}, "E1": {}}
        # 帧率估计: (pc_time, vf) 采样
        self.rate_samples = deque(maxlen=200)
        self.last_line = ""
        # 下行命令: 串口写通道 + MCU 回显的当前 PID 配置
        self.ser = None
        self._ser_lock = threading.Lock()
        self.cfg = {}            # MCU [CFG] 回显的当前参数 (mcu_key -> float)
        self.cfg_version = 0     # 每收到一条 [CFG] +1, 供上位机同步输入框

    def set_serial(self, ser):
        with self._ser_lock:
            self.ser = ser

    def send_cmd(self, text):
        """向 MCU 下发一行命令 (如 's ykp 0.15\\n'); 返回是否成功。"""
        with self._ser_lock:
            if self.ser is None:
                return False
            try:
                self.ser.write(text.encode("ascii", "ignore"))
                return True
            except (serial.SerialException, OSError):
                return False

    # ---- 串口线程侧 ----
    def add_event(self, text: str):
        with self.lock:
            self.events.append((time.time(), text))

    def clear(self):
        with self.lock:
            for d in self.aim.values():
                d.clear()
            self.e1_t.clear()
            self.e1_gs.clear()
            self.events.clear()
            self.rate_samples.clear()
            self.aim_vf = self.aim_drp = self.e1_lap = self.e1_edge = 0

    def feed_line(self, line: str):
        line = line.strip()
        if not line:
            return
        self.last_line = line
        if line.startswith("[AIM]"):
            self._parse(line[5:].strip(), "AIM")
        elif line.startswith("[E1]"):
            self._parse(line[4:].strip(), "E1")
        elif line.startswith("[CFG]"):
            self._parse_cfg(line[5:].strip())

    def _parse_cfg(self, rest):
        cfg = {}
        for tok in rest.split():
            if "=" in tok:
                k, v = tok.split("=", 1)
                try:
                    cfg[k] = float(v)
                except ValueError:
                    pass
        with self.lock:
            self.cfg = cfg
            self.cfg_version += 1
        self.add_event("[CFG] " + rest)   # 事件面板显示 MCU 应用确认

    def _parse(self, rest: str, tag: str):
        toks = rest.split()
        # 遥测行 t= 在行首; 事件行(如 "corner#1 enter t=3200") t 在行尾, 据此区分。
        if toks and toks[0].startswith("t="):
            kv = {}
            for tok in toks:
                if "=" in tok:
                    k, v = tok.split("=", 1)
                    kv[k] = v
            self._parse_telemetry(kv, tag)
        else:                               # 事件行 (自由文本)
            self.add_event(f"[{tag}] {rest}")

    @staticmethod
    def _to_int(s):
        try:
            return int(s, 0)                # 支持 0x 十六进制
        except (ValueError, TypeError):
            return None

    def _write_csv(self, stream, kv):
        if self.csvf is None:
            return
        row = {"pc_time": f"{time.time():.3f}", "stream": stream}
        for k in ("t", "st", "ex", "ey", "cy", "cp", "pit", "yaw",
                  "vf", "drp", "gs", "lap", "edge"):
            v = self._to_int(kv.get(k))
            row[k] = "" if v is None else v
        self.csvf.write(",".join(str(row.get(c, "")) for c in CSV_COLS) + "\n")

    def _parse_telemetry(self, kv, tag):
        t = self._to_int(kv.get("t"))
        if t is None:
            return
        vals = {k: self._to_int(kv.get(k)) for k in kv}
        with self.lock:
            self.latest[tag] = {k: v for k, v in vals.items() if v is not None}
            if tag == "AIM":
                self.aim["t"].append(t)
                for k in ("ex", "ey", "cy", "cp", "pit", "yaw"):
                    self.aim[k].append(self._to_int(kv.get(k)) or 0)
                vf = self._to_int(kv.get("vf"))
                if vf is not None:
                    self.aim_vf = vf
                    self.rate_samples.append((time.time(), vf))
                self.aim_drp = self._to_int(kv.get("drp")) or 0
            else:  # E1
                self.e1_t.append(t)
                self.e1_gs.append(self._to_int(kv.get("gs")) or 0)
                self.e1_lap = self._to_int(kv.get("lap")) or 0
                self.e1_edge = self._to_int(kv.get("edge")) or 0
        self._write_csv(tag, kv)

    def frame_rate_hz(self):
        """由 AIM vf 增量估计视觉帧接收率 (Hz)。"""
        with self.lock:
            rs = list(self.rate_samples)
        if len(rs) < 2:
            return 0.0
        dt = rs[-1][0] - rs[0][0]
        dvf = rs[-1][1] - rs[0][1]
        return (dvf / dt) if dt > 0.1 else 0.0

    # ---- 绘图线程侧 ----
    def snapshot(self):
        with self.lock:
            aim = {k: list(v) for k, v in self.aim.items()}
            e1_t = list(self.e1_t)
            e1_gs = list(self.e1_gs)
            events = list(self.events)
            latest = {k: dict(v) for k, v in self.latest.items()}
            stats = (self.aim_vf, self.aim_drp, self.e1_lap, self.e1_edge)
        return aim, e1_t, e1_gs, events, latest, stats


def serial_reader(port, baud, tel, logf, stop_evt):
    """带自动重连的串口读取线程。"""
    while not stop_evt.is_set():
        try:
            ser = serial.Serial(port, baud, timeout=0.1)
        except (serial.SerialException, OSError) as exc:
            tel.add_event(f"[系统] 打开失败: {exc}")
            if stop_evt.wait(1.0):
                break
            continue
        tel.add_event(f"[系统] 已连接 {port} @ {baud}")
        tel.set_serial(ser)
        tel.send_cmd("g\n")            # 查询 MCU 当前 PID, 同步输入框
        buf = bytearray()
        try:
            while not stop_evt.is_set():
                chunk = ser.read(256)
                if not chunk:
                    continue
                buf += chunk
                while b"\n" in buf:
                    raw, _, buf = buf.partition(b"\n")
                    line = raw.decode("utf-8", "replace")
                    tel.feed_line(line)
                    if logf is not None:
                        logf.write(f"{time.time():.3f} {line.strip()}\n")
        except (serial.SerialException, OSError):
            tel.add_event("[系统] 连接断开, 重试中...")
        finally:
            tel.set_serial(None)
            try:
                ser.close()
            except Exception:
                pass
        if logf is not None:
            logf.flush()
        if stop_evt.wait(0.5):
            break


def _rebase(t_ms):
    """device ms 列表 → 相对首样本的秒。"""
    if not t_ms:
        return []
    t0 = t_ms[0]
    return [(t - t0) / 1000.0 for t in t_ms]


def _pid_preview(t_ms, error, kp, ki, kd, limit, deadband, min_speed, sign):
    """按固件的位置式 PID 和死区/最小速度规则生成候选指令。"""
    output, integral, last_error = [], 0.0, 0.0
    for i, err in enumerate(error):
        dt = ((t_ms[i] - t_ms[i - 1]) / 1000.0) if i else 0.0
        if not 0.0001 <= dt <= 0.2:  # 任务重启/时间跳变不参与积分
            dt = 0.0
        integral = max(-1000.0, min(1000.0, integral + err * dt))
        derivative = (err - last_error) / dt if dt else 0.0
        raw = max(-limit, min(limit, kp * err + ki * integral + kd * derivative))
        cmd = sign * raw
        if abs(err) <= deadband:
            cmd = 0.0
        elif min_speed and 0.0 < abs(cmd) < min_speed:
            cmd = min_speed if cmd > 0 else -min_speed
        output.append(cmd)
        last_error = err
    return output


def _axis_metrics(t_ms, error, command, limit):
    """供同条件下不同参数组横向比较的闭环质量指标。"""
    n = min(len(t_ms), len(error), len(command))
    if n < 2:
        return {"samples": n, "rms_px": 0.0, "iae_px_s": 0.0,
                "peak_px": 0.0, "zero_crossings": 0, "saturated_pct": 0.0}
    e, c = error[:n], command[:n]
    iae = sum((abs(e[i - 1]) + abs(e[i])) * (t_ms[i] - t_ms[i - 1]) / 2000.0
              for i in range(1, n) if 0.0001 <= (t_ms[i] - t_ms[i - 1]) / 1000.0 <= 0.2)
    return {
        "samples": n, "rms_px": (sum(v * v for v in e) / n) ** 0.5,
        "iae_px_s": iae, "peak_px": max(abs(v) for v in e),
        "zero_crossings": sum(1 for a, b in zip(e, e[1:]) if a * b < 0),
        "saturated_pct": 100.0 * sum(1 for v in c if abs(v) >= limit - 0.5) / n,
    }


def build_figure():
    fig = plt.figure(figsize=(15.5, 9.5))
    try:
        fig.canvas.manager.set_window_title("2025E 遥测")
    except Exception:
        pass
    gs = fig.add_gridspec(5, 1, height_ratios=[2, 2, 2, 2, 2],
                          hspace=0.5, right=0.72)
    ax_yaw = fig.add_subplot(gs[0])
    ax_pitch = fig.add_subplot(gs[1], sharex=ax_yaw)
    ax_ang = fig.add_subplot(gs[2], sharex=ax_yaw)
    ax_gs = fig.add_subplot(gs[3], sharex=ax_yaw)
    ax_ev = fig.add_subplot(gs[4])
    ax_ev.axis("off")
    ax_yaw_cmd = ax_yaw.twinx()
    ax_pitch_cmd = ax_pitch.twinx()

    lines = {}
    lines["ex"], = ax_yaw.plot([], [], color="#1f77b4", label="偏航误差 (px)")
    lines["cy"], = ax_yaw_cmd.plot([], [], color="#d62728", label="偏航实际指令")
    lines["cy_preview"], = ax_yaw_cmd.plot([], [], "--", color="#ff7f0e",
                                        alpha=0.75, label="偏航候选 PID")
    ax_yaw.set_ylabel("Yaw 误差 (px)", color="#1f77b4")
    ax_yaw_cmd.set_ylabel("Yaw 指令 (°/s)", color="#d62728")
    ax_yaw.axhline(0, color="gray", lw=0.5)
    ax_yaw.grid(True, alpha=0.3)
    ax_yaw.legend([lines["ex"], lines["cy"], lines["cy_preview"]],
                  ["偏航误差 (px)", "偏航实际指令", "偏航候选 PID"],
                  loc="upper right", fontsize=8)

    lines["ey"], = ax_pitch.plot([], [], color="#2ca02c", label="俯仰误差 (px)")
    lines["cp"], = ax_pitch_cmd.plot([], [], color="#9467bd", label="俯仰实际指令")
    lines["cp_preview"], = ax_pitch_cmd.plot([], [], "--", color="#8c564b",
                                        alpha=0.75, label="俯仰候选 PID")
    ax_pitch.set_ylabel("Pitch 误差 (px)", color="#2ca02c")
    ax_pitch_cmd.set_ylabel("Pitch 指令 (°/s)", color="#9467bd")
    ax_pitch.axhline(0, color="gray", lw=0.5)
    ax_pitch.grid(True, alpha=0.3)
    ax_pitch.legend([lines["ey"], lines["cp"], lines["cp_preview"]],
                    ["俯仰误差 (px)", "俯仰实际指令", "俯仰候选 PID"],
                    loc="upper right", fontsize=8)

    lines["pit"], = ax_ang.plot([], [], label="俯仰角(度)")
    lines["yaw"], = ax_ang.plot([], [], label="偏航角(度)")
    ax_ang.set_ylabel("瞄准角度")
    ax_ang.legend(loc="upper right", fontsize=8)
    ax_ang.grid(True, alpha=0.3)

    gs_lines = []
    for i in range(GRAYSCALE_LANES):
        ln, = ax_gs.step([], [], where="post", lw=1.2)
        gs_lines.append(ln)
    ax_gs.set_ylabel("循迹灰度")
    ax_gs.set_xlabel("时间 (秒, 相对首样本)")
    ax_gs.set_ylim(-0.5, GRAYSCALE_LANES - 0.5)
    ax_gs.set_yticks(range(GRAYSCALE_LANES))
    ax_gs.grid(True, alpha=0.3)

    ev_text = ax_ev.text(0.0, 1.0, "", va="top", ha="left",
                         family=ACTIVE_CJK_FONT, fontsize=8,
                         transform=ax_ev.transAxes)

    return (fig, (ax_yaw, ax_pitch, ax_ang, ax_gs), (ax_yaw_cmd, ax_pitch_cmd),
            lines, gs_lines, ev_text)


_AIM_CN = {"st": "状态", "ex": "水平误差", "ey": "垂直误差", "cy": "偏航指令",
           "cp": "俯仰指令", "pit": "俯仰角", "yaw": "偏航角"}
_E1_CN = {"st": "状态", "gs": "灰度", "lap": "圈", "edge": "边"}


def _fmt_latest(latest):
    a = latest.get("AIM", {})
    e = latest.get("E1", {})
    parts = []
    if a:
        parts.append("瞄准 " + " ".join(f"{_AIM_CN[k]}={a[k]}"
                     for k in _AIM_CN if k in a))
    if e:
        parts.append("循迹 " + " ".join(f"{_E1_CN[k]}={e[k]}"
                     for k in _E1_CN if k in e))
    return "   |   ".join(parts)


def add_pid_controls(fig, initial):
    """创建直接数值输入的 PID 候选参数侧栏。"""
    specs = [("yaw_kp", "Yaw Kp"), ("yaw_ki", "Yaw Ki"), ("yaw_kd", "Yaw Kd"),
             ("pitch_kp", "Pitch Kp"), ("pitch_ki", "Pitch Ki"),
             ("pitch_kd", "Pitch Kd"), ("limit", "输出限幅"),
             ("deadband", "死区 px"), ("yaw_min_speed", "Yaw 最低")]
    state = {"params": dict(initial), "capture_start": None, "capture_end": None}
    inputs = {}
    for i, (key, label) in enumerate(specs):
        ax = fig.add_axes([0.84, 0.87 - i * 0.064, 0.13, 0.030])
        box = TextBox(ax, label, initial=f"{initial[key]:.6g}")
        inputs[key] = box  # 持有引用，防止控件被垃圾回收。

        def submit(text, name=key, widget=box):
            try:
                value = float(text.strip())
                if not math.isfinite(value):
                    raise ValueError
            except ValueError:
                widget.ax.set_facecolor("#ffd9d9")
                fig.canvas.draw_idle()
                return
            state["params"][name] = value
            widget.ax.set_facecolor("white")
            fig.canvas.draw_idle()

        box.on_submit(submit)
    metrics = fig.text(0.745, 0.25, "等待 AIM 遥测…", va="top", fontsize=8.5,
                       family=ACTIVE_CJK_FONT)
    fig.text(0.745, 0.96, "PID 候选参数（直接输入，回车生效；不下发 MCU）", fontsize=10,
             fontweight="bold")
    return state, metrics


def _smooth(y, win):
    """居中移动平均平滑; win<=1 或数据不足时原样返回。"""
    y = np.asarray(y, dtype=float)
    if win is None or int(win) <= 1 or len(y) < 2:
        return y
    win = min(int(win), len(y))
    return np.convolve(y, np.ones(win) / win, mode="same")


def _report_payload(aim, params, start_ms, end_ms, e1=None, events=None):
    """完整参数日志: PID 参数 + 逐样本全量数据 + 闭环指标 (+E1/事件)。"""
    selected = _select_capture(aim, start_ms, end_ms)
    n = len(selected["t"])
    samples = [
        {"t": selected["t"][i], "ex": selected["ex"][i], "ey": selected["ey"][i],
         "cy": selected["cy"][i], "cp": selected["cp"][i],
         "pit": selected["pit"][i], "yaw": selected["yaw"][i]}
        for i in range(n)
    ]
    payload = {
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "note": "完整参数日志; candidate_pid 为上位机候选值, 需手写回 gimbal_tracking.c/h 固化。",
        "candidate_pid": params,
        "capture_device_ms": {"start": start_ms, "end": end_ms},
        "sample_count": n,
        "yaw_metrics": _axis_metrics(selected["t"], selected["ex"], selected["cy"], params["limit"]),
        "pitch_metrics": _axis_metrics(selected["t"], selected["ey"], selected["cp"], params["limit"]),
        "aim_samples": samples,
    }
    if e1 is not None:
        et, egs = e1
        payload["e1_samples"] = [{"t": et[i], "gs": egs[i]}
                                 for i in range(min(len(et), len(egs)))]
    if events is not None:
        payload["events"] = [{"pc_time": round(pt, 3), "text": tx} for pt, tx in events]
    return payload


def _select_capture(aim, start_ms, end_ms):
    """取得标记片段；没有标记时使用当前全部缓存。"""
    indices = range(len(aim["t"]))
    if start_ms is not None:
        final_ms = end_ms if end_ms is not None else (aim["t"][-1] if aim["t"] else start_ms)
        indices = [i for i, t in enumerate(aim["t"]) if start_ms <= t <= final_ms]
    return {key: [values[i] for i in indices] for key, values in aim.items()}


def main(argv=None):
    ap = argparse.ArgumentParser(description="2025E debug 串口遥测可视化")
    ap.add_argument("--port", help="串口, 如 COM7 / /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--window", type=float, default=DEFAULT_WINDOW_S, help="滚动时间窗(秒)")
    ap.add_argument("--log", help="原始行存到文件")
    ap.add_argument("--csv", help="解析后的遥测存为 CSV (供离线绘图)")
    ap.add_argument("--report", default="pid_tuning_report.json",
                    help="按 s 导出的 PID 试验报告路径 (默认: pid_tuning_report.json)")
    ap.add_argument("--yaw-kp", type=float, default=PID_DEFAULTS["yaw_kp"])
    ap.add_argument("--pitch-kp", type=float, default=PID_DEFAULTS["pitch_kp"])
    ap.add_argument("--pid-limit", type=float, default=PID_DEFAULTS["limit"])
    ap.add_argument("--deadband", type=float, default=PID_DEFAULTS["deadband"])
    ap.add_argument("--yaw-min-speed", type=float, default=PID_DEFAULTS["yaw_min_speed"])
    ap.add_argument("--list", action="store_true", help="列出串口后退出")
    args = ap.parse_args(argv)

    if args.list:
        list_serial_ports()
        return 0
    if not args.port:
        print("需要 --port (或 --list 查看)。", file=sys.stderr)
        return 2
    if ACTIVE_CJK_FONT:
        print(f"中文字体: {ACTIVE_CJK_FONT}")
    else:
        print("警告: 未发现可用中文字体，界面中文可能显示为方块。", file=sys.stderr)

    logf = open(args.log, "w", encoding="utf-8") if args.log else None
    csvf = open(args.csv, "w", encoding="utf-8") if args.csv else None
    if csvf is not None:
        csvf.write(",".join(CSV_COLS) + "\n")

    tel = Telemetry(csvf=csvf)
    stop_evt = threading.Event()
    reader = threading.Thread(target=serial_reader,
                              args=(args.port, args.baud, tel, logf, stop_evt),
                              daemon=True)
    reader.start()
    print(f"监听 {args.port} @ {args.baud}, 关闭窗口结束。空格=暂停 c=清空 m=标记试验 s=导出报告")

    fig, axes, command_axes, lines, gs_lines, ev_text = build_figure()
    ax_time = axes[0]
    initial_pid = dict(PID_DEFAULTS)
    initial_pid.update(yaw_kp=args.yaw_kp, pitch_kp=args.pitch_kp,
                       limit=args.pid_limit, deadband=args.deadband,
                       yaw_min_speed=args.yaw_min_speed)
    pid_state, metrics_text = add_pid_controls(fig, initial_pid)
    state = {"paused": False}

    def on_key(event):
        if event.key == " ":
            state["paused"] = not state["paused"]
        elif event.key == "c":
            tel.clear()
            pid_state["capture_start"] = pid_state["capture_end"] = None
        elif event.key == "m":
            aim, *_ = tel.snapshot()
            if not aim["t"]:
                return
            if pid_state["capture_start"] is None or pid_state["capture_end"] is not None:
                pid_state["capture_start"] = aim["t"][-1]
                pid_state["capture_end"] = None
                tel.add_event(f"[PID] 开始试验片段 t={aim['t'][-1]}")
            else:
                pid_state["capture_end"] = aim["t"][-1]
                tel.add_event(f"[PID] 结束试验片段 t={aim['t'][-1]}")
        elif event.key == "s":
            aim, *_ = tel.snapshot()
            payload = _report_payload(aim, dict(pid_state["params"]),
                                      pid_state["capture_start"], pid_state["capture_end"])
            try:
                Path(args.report).write_text(json.dumps(payload, ensure_ascii=False, indent=2),
                                             encoding="utf-8")
                tel.add_event(f"[PID] 已导出报告: {args.report}")
            except OSError as exc:
                tel.add_event(f"[PID] 报告导出失败: {exc}")
        elif event.key == "q":
            plt.close(fig)

    fig.canvas.mpl_connect("key_press_event", on_key)

    def update(_frame):
        if state["paused"]:
            fig.suptitle("[已暂停] 空格继续", fontsize=10, color="red")
            return list(lines.values()) + gs_lines + [ev_text]

        aim, e1_t, e1_gs, events, latest, stats = tel.snapshot()

        t_aim = _rebase(aim["t"])
        for k in ("ex", "ey", "cy", "cp", "pit", "yaw"):
            n = min(len(t_aim), len(aim[k]))
            lines[k].set_data(t_aim[:n], aim[k][:n])

        p = pid_state["params"]
        yaw_preview = _pid_preview(aim["t"], aim["ex"], p["yaw_kp"], p["yaw_ki"],
                                   p["yaw_kd"], p["limit"], p["deadband"],
                                   p["yaw_min_speed"], -1.0)
        pitch_preview = _pid_preview(aim["t"], aim["ey"], p["pitch_kp"], p["pitch_ki"],
                                     p["pitch_kd"], p["limit"], p["deadband"], 0.0, 1.0)
        lines["cy_preview"].set_data(t_aim[:len(yaw_preview)], yaw_preview)
        lines["cp_preview"].set_data(t_aim[:len(pitch_preview)], pitch_preview)

        t_e1 = _rebase(e1_t)
        for i, ln in enumerate(gs_lines):
            n = min(len(t_e1), len(e1_gs))
            ys = [i + (0.7 if (e1_gs[j] >> i) & 1 else 0.0) for j in range(n)]
            ln.set_data(t_e1[:n], ys)

        t_max = 0.0
        if t_aim:
            t_max = max(t_max, t_aim[-1])
        if t_e1:
            t_max = max(t_max, t_e1[-1])
        if t_max > 0:
            ax_time.set_xlim(max(0.0, t_max - args.window), t_max + 0.5)
        for ax in (*axes[:3], *command_axes):
            ax.relim()
            ax.autoscale_view(scalex=False, scaley=True)

        # 事件面板 (最近若干条, 新的在下)
        lines_txt = []
        for pc_t, txt in list(events)[-EVENT_SHOW:]:
            lines_txt.append(f"{time.strftime('%H:%M:%S', time.localtime(pc_t))} {txt}")
        ev_text.set_text("\n".join(lines_txt))

        vf, drp, lap, edge = stats
        metric_aim = _select_capture(aim, pid_state["capture_start"],
                                     pid_state["capture_end"])
        yaw_metrics = _axis_metrics(metric_aim["t"], metric_aim["ex"],
                                    metric_aim["cy"], p["limit"])
        pitch_metrics = _axis_metrics(metric_aim["t"], metric_aim["ey"],
                                      metric_aim["cp"], p["limit"])
        capture = ("未标记" if pid_state["capture_start"] is None else
                   f"{pid_state['capture_start']}→{pid_state['capture_end'] or '进行中'} ms")
        metrics_text.set_text(
            "实际闭环指标\n"
            f"Yaw  RMS {yaw_metrics['rms_px']:.1f}px\n"
            f"     IAE {yaw_metrics['iae_px_s']:.1f}px·s\n"
            f"     峰值 {yaw_metrics['peak_px']:.0f}px  过零 {yaw_metrics['zero_crossings']}\n"
            f"     限幅 {yaw_metrics['saturated_pct']:.1f}%\n"
            f"Pitch RMS {pitch_metrics['rms_px']:.1f}px\n"
            f"      IAE {pitch_metrics['iae_px_s']:.1f}px·s\n"
            f"      峰值 {pitch_metrics['peak_px']:.0f}px  过零 {pitch_metrics['zero_crossings']}\n"
            f"      限幅 {pitch_metrics['saturated_pct']:.1f}%\n\n"
            f"试验片段: {capture}\n"
            "m 标记起止；s 导出报告")
        fig.suptitle(
            f"帧率≈{tel.frame_rate_hz():.1f}赫兹  有效帧={vf} 丢帧={drp}  |  "
            f"循迹 圈={lap} 边={edge}\n{_fmt_latest(latest)}",
            fontsize=9)
        return list(lines.values()) + gs_lines + [ev_text]

    _anim = FuncAnimation(fig, update, interval=100, blit=False,
                          cache_frame_data=False)
    try:
        plt.show()
    finally:
        stop_evt.set()
        reader.join(timeout=1.5)
        if logf is not None:
            logf.close()
        if csvf is not None:
            csvf.close()
    return 0


class QtTelemetryWindow(QtWidgets.QMainWindow):
    """PyQtGraph 增量绘图界面；仅重绘可见时间窗内的数据。"""

    # 上位机参数键 -> MCU 命令键 (可实时下发的参数)。
    _MCU_KEY = {"yaw_kp": "ykp", "yaw_ki": "yki", "yaw_kd": "ykd",
                "pitch_kp": "pkp", "pitch_ki": "pki", "pitch_kd": "pkd",
                "limit": "olim"}

    def __init__(self, tel, args):
        super().__init__()
        self.tel, self.args = tel, args
        self.params = dict(PID_DEFAULTS)
        self.params.update(yaw_kp=args.yaw_kp, pitch_kp=args.pitch_kp,
                           limit=args.pid_limit, deadband=args.deadband,
                           yaw_min_speed=args.yaw_min_speed)
        self.capture_start = self.capture_end = None
        self.paused = False
        self._cfg_seen = 0       # 已同步的 MCU 配置版本
        self.setWindowTitle("2025E PID 遥测调参台 (PyQtGraph)")
        self.resize(1500, 960)
        self._build_ui()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(50)  # 20 Hz GUI；串口采集线程不受此频率限制

    @staticmethod
    def _pen(color, width=1, style=None):
        return pg.mkPen(color, width=width, style=style or QtCore.Qt.SolidLine)

    def _plot_with_command_axis(self, layout, row, title, error_name, command_name):
        plot = layout.addPlot(row=row, col=0, title=title)
        plot.showGrid(x=True, y=True, alpha=0.25)
        plot.setLabel("left", "误差", units="px")
        plot.showAxis("right")
        plot.getAxis("right").setLabel("指令", units="°/s")
        view = pg.ViewBox()
        plot.scene().addItem(view)
        plot.getAxis("right").linkToView(view)
        view.setXLink(plot)
        plot.vb.sigResized.connect(lambda: view.setGeometry(plot.vb.sceneBoundingRect()))
        error = plot.plot(pen=self._pen("#42a5f5", 2), name=error_name)
        actual = pg.PlotCurveItem(pen=self._pen("#ef5350", 2), name=command_name)
        preview = pg.PlotCurveItem(pen=self._pen("#ffb300", 1, QtCore.Qt.DashLine),
                                   name="候选 PID")
        view.addItem(actual)
        view.addItem(preview)
        plot.addLegend(offset=(10, 10))
        plot.legend.addItem(error, error_name)
        plot.legend.addItem(actual, command_name)
        plot.legend.addItem(preview, "候选 PID")
        return plot, view, error, actual, preview

    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QHBoxLayout(central)
        graph = pg.GraphicsLayoutWidget()
        root.addWidget(graph, 4)
        self.yaw_plot, self.yaw_vb, self.ex_curve, self.cy_curve, self.cy_preview = \
            self._plot_with_command_axis(graph.ci, 0, "偏航闭环：误差与指令", "偏航误差", "实际指令")
        self.pitch_plot, self.pitch_vb, self.ey_curve, self.cp_curve, self.cp_preview = \
            self._plot_with_command_axis(graph.ci, 1, "俯仰闭环：误差与指令", "俯仰误差", "实际指令")
        self.pitch_plot.setXLink(self.yaw_plot)
        self.angle_plot = graph.addPlot(row=2, col=0, title="云台角度")
        self.angle_plot.showGrid(x=True, y=True, alpha=0.25)
        self.angle_plot.addLegend()
        self.pit_curve = self.angle_plot.plot(pen=self._pen("#ab47bc", 2), name="俯仰角")
        self.yaw_curve = self.angle_plot.plot(pen=self._pen("#26a69a", 2), name="偏航角")
        self.angle_plot.setXLink(self.yaw_plot)
        self.gray_plot = graph.addPlot(row=3, col=0, title="E1 灰度传感器")
        self.gray_img = pg.ImageItem()
        self.gray_plot.addItem(self.gray_img)
        self.gray_plot.setYRange(0, 8, padding=0)
        self.gray_plot.setLabel("left", "通道")
        self.gray_plot.setLabel("bottom", "时间", units="s")
        self.gray_plot.setXLink(self.yaw_plot)

        panel = QtWidgets.QWidget()
        root.addWidget(panel, 1)
        side = QtWidgets.QVBoxLayout(panel)
        side.addWidget(QtWidgets.QLabel(
            "PID 参数（回车生效）\n勾选后 Kp/Ki/Kd/限幅 实时下发 MCU；死区/最低仅预览"))
        self.push_chk = QtWidgets.QCheckBox("实时下发到 MCU")
        self.push_chk.setChecked(True)
        side.addWidget(self.push_chk)
        form = QtWidgets.QFormLayout()
        self.inputs = {}
        labels = {"yaw_kp":"Yaw Kp", "yaw_ki":"Yaw Ki", "yaw_kd":"Yaw Kd",
                  "pitch_kp":"Pitch Kp", "pitch_ki":"Pitch Ki", "pitch_kd":"Pitch Kd",
                  "limit":"输出限幅", "deadband":"死区 px", "yaw_min_speed":"Yaw 最低"}
        for key, label in labels.items():
            edit = QtWidgets.QLineEdit(f"{self.params[key]:.6g}")
            edit.editingFinished.connect(lambda name=key, field=edit: self._set_param(name, field))
            self.inputs[key] = edit
            form.addRow(label, edit)
        side.addLayout(form)
        self.metrics = QtWidgets.QLabel("等待 AIM 遥测…")
        self.metrics.setWordWrap(True)
        side.addWidget(self.metrics)
        side.addWidget(QtWidgets.QLabel("事件"))
        self.events = QtWidgets.QPlainTextEdit()
        self.events.setReadOnly(True)
        side.addWidget(self.events, 1)
        side.addWidget(QtWidgets.QLabel("空格 暂停  |  C 清空  |  M 标记片段  |  S 导出报告  |  Q 退出"))

    def _set_param(self, name, field):
        try:
            value = float(field.text())
            if not math.isfinite(value):
                raise ValueError
        except ValueError:
            field.setStyleSheet("background:#ffd9d9")
            return
        self.params[name] = value
        field.setStyleSheet("")
        field.setText(f"{value:.6g}")
        # 实时下发到 MCU (仅可下发的 PID 键)。
        mcu_key = self._MCU_KEY.get(name)
        if mcu_key and self.push_chk.isChecked():
            ok = self.tel.send_cmd(f"s {mcu_key} {value:.6g}\n")
            if not ok:
                self.tel.add_event("[系统] 下发失败(串口未连接)")

    @staticmethod
    def _visible(t_ms, values, window_s):
        if not t_ms:
            return np.array([]), np.array([])
        t = (np.asarray(t_ms) - t_ms[0]) / 1000.0
        begin = max(0.0, t[-1] - window_s)
        mask = t >= begin
        return t[mask], np.asarray(values)[mask]

    @staticmethod
    def _set_y_range(view, values):
        """显式设定可见数据的 Y 轴范围，避免 Qt 自动范围在双 ViewBox 下失效。"""
        if len(values) == 0:
            return
        low, high = float(np.min(values)), float(np.max(values))
        pad = max(1.0, (high - low) * 0.12)
        view.setYRange(low - pad, high + pad, padding=0)

    def _sync_command_view(self, plot, command_view, error, command):
        command_view.setGeometry(plot.vb.sceneBoundingRect())
        command_view.linkedViewChanged(plot.vb, command_view.XAxis)
        self._set_y_range(plot.vb, error)
        self._set_y_range(command_view, command)

    def refresh(self):
        if self.paused:
            return
        aim, e1_t, e1_gs, events, latest, stats = self.tel.snapshot()
        # MCU 回显 [CFG] 到达时, 同步实际 PID 到未聚焦的输入框
        if self.tel.cfg_version != self._cfg_seen:
            self._cfg_seen = self.tel.cfg_version
            with self.tel.lock:
                cfg = dict(self.tel.cfg)
            inv = {v: k for k, v in self._MCU_KEY.items()}
            for mk, val in cfg.items():
                pk = inv.get(mk)
                if pk and pk in self.inputs and not self.inputs[pk].hasFocus():
                    self.params[pk] = val
                    self.inputs[pk].setText(f"{val:.6g}")
        t, ex = self._visible(aim["t"], aim["ex"], self.args.window)
        _, ey = self._visible(aim["t"], aim["ey"], self.args.window)
        _, cy = self._visible(aim["t"], aim["cy"], self.args.window)
        _, cp = self._visible(aim["t"], aim["cp"], self.args.window)
        _, pit = self._visible(aim["t"], aim["pit"], self.args.window)
        _, yaw = self._visible(aim["t"], aim["yaw"], self.args.window)
        sw = self.args.smooth
        ex, ey = _smooth(ex, sw), _smooth(ey, sw)
        cy, cp = _smooth(cy, sw), _smooth(cp, sw)
        pit, yaw = _smooth(pit, sw), _smooth(yaw, sw)
        self.ex_curve.setData(t, ex); self.ey_curve.setData(t, ey)
        self.cy_curve.setData(t, cy); self.cp_curve.setData(t, cp)
        self.pit_curve.setData(t, pit); self.yaw_curve.setData(t, yaw)
        yp = _pid_preview(aim["t"], aim["ex"], self.params["yaw_kp"], self.params["yaw_ki"],
                          self.params["yaw_kd"], self.params["limit"], self.params["deadband"],
                          self.params["yaw_min_speed"], -1.0)
        pp = _pid_preview(aim["t"], aim["ey"], self.params["pitch_kp"], self.params["pitch_ki"],
                          self.params["pitch_kd"], self.params["limit"], self.params["deadband"], 0, 1.0)
        _, yp = self._visible(aim["t"], yp, self.args.window); _, pp = self._visible(aim["t"], pp, self.args.window)
        yp, pp = _smooth(yp, sw), _smooth(pp, sw)
        self.cy_preview.setData(t, yp); self.cp_preview.setData(t, pp)
        if len(t):
            self.yaw_plot.setXRange(max(0, t[-1] - self.args.window), t[-1], padding=0)
            self._sync_command_view(self.yaw_plot, self.yaw_vb, ex, np.concatenate((cy, yp)))
            self._sync_command_view(self.pitch_plot, self.pitch_vb, ey, np.concatenate((cp, pp)))
            self._set_y_range(self.angle_plot.vb, np.concatenate((pit, yaw)))
        if e1_t:
            te, ge = self._visible(e1_t, e1_gs, self.args.window)
            if len(te):
                bits = np.array([[(v >> ch) & 1 for v in ge] for ch in range(8)], dtype=np.uint8)
                self.gray_img.setImage(bits, autoLevels=False, levels=(0, 1))
                self.gray_img.setRect(QtCore.QRectF(float(te[0]), 0, max(0.001, float(te[-1]-te[0])), 8))
        self.events.setPlainText("\n".join(f"{time.strftime('%H:%M:%S', time.localtime(t0))} {txt}"
                                             for t0, txt in events[-EVENT_SHOW:]))
        selected = _select_capture(aim, self.capture_start, self.capture_end)
        ym = _axis_metrics(selected["t"], selected["ex"], selected["cy"], self.params["limit"])
        pm = _axis_metrics(selected["t"], selected["ey"], selected["cp"], self.params["limit"])
        self.metrics.setText(f"AIM 样本 {len(aim['t'])}  |  帧率 {self.tel.frame_rate_hz():.1f} Hz  丢帧 {stats[1]}\n"
            f"Yaw: RMS {ym['rms_px']:.1f}px | IAE {ym['iae_px_s']:.1f} | 过零 {ym['zero_crossings']} | 限幅 {ym['saturated_pct']:.1f}%\n"
            f"Pitch: RMS {pm['rms_px']:.1f}px | IAE {pm['iae_px_s']:.1f} | 过零 {pm['zero_crossings']} | 限幅 {pm['saturated_pct']:.1f}%")

    def keyPressEvent(self, event):
        key = event.key()
        if key == QtCore.Qt.Key_Space: self.paused = not self.paused
        elif key == QtCore.Qt.Key_C: self.tel.clear(); self.capture_start = self.capture_end = None
        elif key == QtCore.Qt.Key_M:
            aim, *_ = self.tel.snapshot()
            if aim["t"]:
                if self.capture_start is None or self.capture_end is not None:
                    self.capture_start, self.capture_end = aim["t"][-1], None
                else: self.capture_end = aim["t"][-1]
        elif key == QtCore.Qt.Key_S:
            aim, e1_t, e1_gs, events, *_ = self.tel.snapshot()
            payload = _report_payload(aim, self.params, self.capture_start, self.capture_end,
                                      e1=(e1_t, e1_gs), events=events)
            Path(self.args.report).write_text(
                json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
            self.tel.add_event(
                f"[PID] 已导出完整日志: {self.args.report} ({payload['sample_count']}样本)")
        elif key == QtCore.Qt.Key_Q: self.close()
        else: super().keyPressEvent(event)


def qt_main(argv=None):
    ap = argparse.ArgumentParser(description="2025E PyQtGraph PID 遥测调参台")
    ap.add_argument("--port", help="串口，如 COM7")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--window", type=float, default=DEFAULT_WINDOW_S)
    ap.add_argument("--smooth", type=int, default=5,
                    help="曲线移动平均窗口(样本), 1=不平滑 (默认5)")
    ap.add_argument("--log"); ap.add_argument("--csv")
    ap.add_argument("--report", default="pid_tuning_report.json")
    ap.add_argument("--yaw-kp", type=float, default=PID_DEFAULTS["yaw_kp"])
    ap.add_argument("--pitch-kp", type=float, default=PID_DEFAULTS["pitch_kp"])
    ap.add_argument("--pid-limit", type=float, default=PID_DEFAULTS["limit"])
    ap.add_argument("--deadband", type=float, default=PID_DEFAULTS["deadband"])
    ap.add_argument("--yaw-min-speed", type=float, default=PID_DEFAULTS["yaw_min_speed"])
    ap.add_argument("--list", action="store_true")
    args = ap.parse_args(argv)
    if args.list: list_serial_ports(); return 0
    if not args.port: ap.error("需要 --port（或 --list）")
    logf = open(args.log, "w", encoding="utf-8") if args.log else None
    csvf = open(args.csv, "w", encoding="utf-8") if args.csv else None
    if csvf: csvf.write(",".join(CSV_COLS) + "\n")
    tel, stop = Telemetry(csvf), threading.Event()
    reader = threading.Thread(target=serial_reader, args=(args.port,args.baud,tel,logf,stop), daemon=True)
    reader.start()
    app = QtWidgets.QApplication(sys.argv[:1])
    qt_fonts = set(QtGui.QFontDatabase().families())
    qt_cjk_font = next((name for name in _CJK_FONT_CANDIDATES if name in qt_fonts), None)
    if qt_cjk_font:
        app.setFont(QtGui.QFont(qt_cjk_font))
        print(f"Qt 中文字体: {qt_cjk_font}")
    else:
        # 某些 Qt 平台插件（例如 offscreen）不会枚举系统字体；仍尝试用
        # Matplotlib 已验证存在的字体名，Qt 会在正常桌面插件下解析它。
        fallback = ACTIVE_CJK_FONT
        if fallback:
            app.setFont(QtGui.QFont(fallback))
            print(f"Qt 未枚举中文字体，尝试系统字体: {fallback}", file=sys.stderr)
        else:
            print("警告: Qt 未发现可用中文字体，界面中文可能显示为方块。", file=sys.stderr)
    window = QtTelemetryWindow(tel, args); window.show()
    try: return app.exec_()
    finally:
        stop.set(); reader.join(1.5)
        if logf: logf.close()
        if csvf: csvf.close()


if __name__ == "__main__":
    raise SystemExit(qt_main())
