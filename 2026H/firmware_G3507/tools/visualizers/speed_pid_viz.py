#!/usr/bin/env python3
"""双轮速度闭环 (Speed PID) 遥测可视化上位机。

配合固件 Device Check「Speed PID」自检: MCU 经 debug 串口 (Debug_Ex=UART1, 115200)
每控制拍 (20ms/50Hz) 输出一行遥测:

  [SPD] t=<ms> tl=<目标左> tr=<目标右> l=<实测左> r=<实测右> dl=<占空左> dr=<占空右>
        il=<积分左> ir=<积分右>

  - t            设备时间戳 (ms), 取自每拍 on_tick 入口; 相邻行的差即实际控制周期
  - tl/tr        左/右轮目标线速度 (m/s)
  - l/r          左/右轮实测线速度 (m/s)
  - dl/dr        左/右轮应用占空比 (%, 为上一控制拍值)
  - il/ir        左/右轮速度环 PID 积分累加值 (输出中的积分项为 KI*本值); 旧固件无此
                 字段时按 0 处理

窗口:
  - 上图: 左右轮 目标(虚线) vs 实测(实线) 线速度, 横轴=设备时间戳
  - 中图: 左右轮 应用占空比
  - 下图: 左右轮 积分累加值, 附 output_limit/ki 参考线 (越线即积分项已单独占满输出,
          之上属过度累积, 退绕时会拖长恢复时间)
  - 标题: 每轮 目标/实测/误差 + 滚动窗口内跟踪误差 RMS, 便于判断 收敛/振荡/稳态偏差

热键: 空格=暂停/继续, c=清空, q=退出

用法:
  python speed_pid_viz.py --port COM7 [--baud 115200] [--window 12]
  python speed_pid_viz.py --port COM7 --csv spd.csv --log raw.txt
  python speed_pid_viz.py --port COM7 --ki 17 --output-limit 100   # 标注积分饱和参考线
  python speed_pid_viz.py --list          # 列出串口

依赖: pip install pyserial matplotlib

整定流程 (增益在固件 chassis.h 的 CHASSIS_SPEED_* 宏, 改后需重新烧录):
  1. 抬轮, 进入 Device Check -> Speed PID, 运行本脚本连接串口。
  2. 板上 UP/DOWN 给目标速度, 观察实测能否跟上目标:
       - 跟不上/太慢/有稳态偏差 -> 加 KI (消稳态) / KP (提响应)
       - 抖动/超调/振荡          -> 减 KP/KI, 或加一点 KD
  3. 一组增益满意后回填 chassis.h 的 CHASSIS_SPEED_KP/KI/KD 并重新烧录。
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

# 选一个存在的中文字体, 避免标签显示成方块。
_CJK_CANDIDATES = ("Microsoft YaHei", "SimHei", "SimSun", "Noto Sans CJK SC",
                   "Source Han Sans SC", "WenQuanYi Zen Hei", "Arial Unicode MS")
_AVAILABLE = {f.name for f in font_manager.fontManager.ttflist}
_CJK = next((f for f in _CJK_CANDIDATES if f in _AVAILABLE), None)
plt.rcParams["font.sans-serif"] = [_CJK, "sans-serif"] if _CJK else ["sans-serif"]
plt.rcParams["axes.unicode_minus"] = False

MAX_SAMPLES = 6000            # 每条曲线最多缓存点数
DEFAULT_WINDOW_S = 12.0       # 滚动时间窗 (秒)
FIELDS = ("tl", "tr", "l", "r", "dl", "dr", "il", "ir")
# 旧固件的 [SPD] 行没有积分字段; 缺省补 0 以便老日志/老板子仍能解析。
OPTIONAL_FIELDS = ("il", "ir")
CSV_COLS = ["pc_time", "t", *FIELDS]


def list_serial_ports():
    ports = list(list_ports.comports())
    if not ports:
        print("未发现串口。")
        return
    for p in ports:
        print(f"{p.device}\t{p.description}\t{p.hwid}")


class SpeedTelemetry:
    """线程安全的 [SPD] 遥测缓存 (串口线程写, 绘图线程读)。"""

    def __init__(self, csvf=None):
        self.lock = threading.Lock()
        self.csvf = csvf
        self.t = deque(maxlen=MAX_SAMPLES)                      # 设备 ms
        self.data = {k: deque(maxlen=MAX_SAMPLES) for k in FIELDS}
        self.latest = {}
        self.dropped_lines = 0

    def clear(self):
        with self.lock:
            self.t.clear()
            for d in self.data.values():
                d.clear()
            self.latest = {}

    def feed_line(self, line: str):
        line = line.strip()
        if not line.startswith("[SPD]"):
            return
        kv = {}
        for tok in line[5:].split():
            if "=" in tok:
                k, v = tok.split("=", 1)
                kv[k] = v
        try:
            t = int(kv["t"])
            vals = {k: float(kv[k]) if k in kv
                    else (0.0 if k in OPTIONAL_FIELDS else float(kv[k]))
                    for k in FIELDS}
        except (KeyError, ValueError):
            self.dropped_lines += 1
            return
        with self.lock:
            self.t.append(t)
            for k in FIELDS:
                self.data[k].append(vals[k])
            self.latest = {"t": t, **vals}
        if self.csvf is not None:
            self.csvf.write(",".join([f"{time.time():.3f}", str(t),
                                      *(f"{vals[k]:.4f}" for k in FIELDS)]) + "\n")

    def snapshot(self):
        with self.lock:
            t = list(self.t)
            data = {k: list(v) for k, v in self.data.items()}
            latest = dict(self.latest)
        return t, data, latest


def serial_reader(port, baud, tel, logf, stop_evt):
    """带自动重连的串口读取线程。"""
    while not stop_evt.is_set():
        try:
            ser = serial.Serial(port, baud, timeout=0.1)
        except (serial.SerialException, OSError) as exc:
            print(f"[系统] 打开 {port} 失败: {exc}", file=sys.stderr)
            if stop_evt.wait(1.0):
                break
            continue
        print(f"[系统] 已连接 {port} @ {baud}")
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
            print("[系统] 连接断开, 重试中...", file=sys.stderr)
        finally:
            try:
                ser.close()
            except Exception:
                pass
        if stop_evt.wait(0.5):
            break


def _rebase(t_ms):
    if not t_ms:
        return []
    t0 = t_ms[0]
    return [(t - t0) / 1000.0 for t in t_ms]


def _rms(seq):
    seq = [v for v in seq if v is not None]
    if not seq:
        return 0.0
    return (sum(v * v for v in seq) / len(seq)) ** 0.5


def build_figure(ki, output_limit):
    fig, (ax_spd, ax_duty, ax_integ) = plt.subplots(3, 1, figsize=(12, 9.5), sharex=True)
    try:
        fig.canvas.manager.set_window_title("Speed PID 遥测")
    except Exception:
        pass
    fig.subplots_adjust(hspace=0.18, top=0.90, right=0.97, left=0.08)

    lines = {}
    lines["tl"], = ax_spd.plot([], [], "--", color="#1f77b4", lw=1.2, label="左 目标")
    lines["l"], = ax_spd.plot([], [], "-", color="#1f77b4", lw=1.6, label="左 实测")
    lines["tr"], = ax_spd.plot([], [], "--", color="#d62728", lw=1.2, label="右 目标")
    lines["r"], = ax_spd.plot([], [], "-", color="#d62728", lw=1.6, label="右 实测")
    ax_spd.set_ylabel("轮线速度 (m/s)")
    ax_spd.axhline(0, color="gray", lw=0.5)
    ax_spd.grid(True, alpha=0.3)
    ax_spd.legend(loc="upper right", ncol=2, fontsize=8)

    lines["dl"], = ax_duty.plot([], [], "-", color="#1f77b4", lw=1.3, label="左 占空比")
    lines["dr"], = ax_duty.plot([], [], "-", color="#d62728", lw=1.3, label="右 占空比")
    ax_duty.set_ylabel("占空比 (%)")
    ax_duty.axhline(0, color="gray", lw=0.5)
    ax_duty.grid(True, alpha=0.3)
    ax_duty.legend(loc="upper right", ncol=2, fontsize=8)

    lines["il"], = ax_integ.plot([], [], "-", color="#1f77b4", lw=1.3, label="左 积分")
    lines["ir"], = ax_integ.plot([], [], "-", color="#d62728", lw=1.3, label="右 积分")
    # 积分项在输出中是 ki*integral, 故越过这条线时积分项已单独占满输出 -> 之上全是过度累积。
    if ki > 0.0:
        ax_integ.axhline(output_limit / ki, color="#ff7f0e", lw=1.0, ls=":",
                         label=f"积分项占满输出 ({output_limit / ki:.2f})")
        ax_integ.axhline(-output_limit / ki, color="#ff7f0e", lw=1.0, ls=":")
    ax_integ.set_ylabel("速度环积分累加值")
    ax_integ.set_xlabel("时间 (秒, 相对首样本)")
    ax_integ.axhline(0, color="gray", lw=0.5)
    ax_integ.grid(True, alpha=0.3)
    ax_integ.legend(loc="upper right", ncol=3, fontsize=8)

    return fig, (ax_spd, ax_duty, ax_integ), lines


def main(argv=None):
    ap = argparse.ArgumentParser(description="双轮速度闭环遥测可视化")
    ap.add_argument("--port", help="串口, 如 COM7 / /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--window", type=float, default=DEFAULT_WINDOW_S, help="滚动时间窗(秒)")
    ap.add_argument("--log", help="原始行存到文件")
    ap.add_argument("--csv", help="解析后的遥测存为 CSV")
    ap.add_argument("--list", action="store_true", help="列出串口后退出")
    ap.add_argument("--ki", type=float, default=17.0,
                    help="固件 CHASSIS_SPEED_KI, 用于在积分图上标注饱和参考线")
    ap.add_argument("--output-limit", type=float, default=100.0,
                    help="固件 CHASSIS_SPEED_OUTPUT_LIMIT")
    args = ap.parse_args(argv)

    if args.list:
        list_serial_ports()
        return 0
    if not args.port:
        print("需要 --port (或 --list 查看)。", file=sys.stderr)
        return 2
    if not _CJK:
        print("警告: 未发现可用中文字体, 界面中文可能显示为方块。", file=sys.stderr)

    logf = open(args.log, "w", encoding="utf-8") if args.log else None
    csvf = open(args.csv, "w", encoding="utf-8") if args.csv else None
    if csvf is not None:
        csvf.write(",".join(CSV_COLS) + "\n")

    tel = SpeedTelemetry(csvf=csvf)
    stop_evt = threading.Event()
    reader = threading.Thread(target=serial_reader,
                              args=(args.port, args.baud, tel, logf, stop_evt),
                              daemon=True)
    reader.start()
    print(f"监听 {args.port} @ {args.baud}。空格=暂停 c=清空 q=退出")

    fig, (ax_spd, ax_duty, ax_integ), lines = build_figure(args.ki, args.output_limit)
    state = {"paused": False}

    def on_key(event):
        if event.key == " ":
            state["paused"] = not state["paused"]
        elif event.key == "c":
            tel.clear()
        elif event.key == "q":
            plt.close(fig)

    fig.canvas.mpl_connect("key_press_event", on_key)

    def update(_frame):
        if state["paused"]:
            fig.suptitle("[已暂停] 空格继续", color="red", fontsize=10)
            return list(lines.values())

        t_ms, data, latest = tel.snapshot()
        t = _rebase(t_ms)
        for k in FIELDS:
            n = min(len(t), len(data[k]))
            lines[k].set_data(t[:n], data[k][:n])

        if t:
            t_max = t[-1]
            ax_spd.set_xlim(max(0.0, t_max - args.window), t_max + 0.3)
        for ax in (ax_spd, ax_duty, ax_integ):
            ax.relim()
            ax.autoscale_view(scalex=False, scaley=True)

        # 滚动窗口内跟踪误差 (实测-目标) 的 RMS, 判断收敛质量。
        if t:
            begin = t[-1] - args.window
            idx = [i for i, tv in enumerate(t) if tv >= begin]
            errL = [data["l"][i] - data["tl"][i] for i in idx]
            errR = [data["r"][i] - data["tr"][i] for i in idx]
            rmsL, rmsR = _rms(errL), _rms(errR)
        else:
            rmsL = rmsR = 0.0

        if latest:
            eL = latest["l"] - latest["tl"]
            eR = latest["r"] - latest["tr"]
            fig.suptitle(
                f"左  目标{latest['tl']:+.2f} 实测{latest['l']:+.2f} 误差{eL:+.2f} "
                f"占空{latest['dl']:+.0f}%  RMS{rmsL:.3f}    |    "
                f"右  目标{latest['tr']:+.2f} 实测{latest['r']:+.2f} 误差{eR:+.2f} "
                f"占空{latest['dr']:+.0f}%  RMS{rmsR:.3f}    (m/s)",
                fontsize=9)
        else:
            fig.suptitle("等待 [SPD] 遥测…  (板上进入 Device Check -> Speed PID, 抬轮给目标)",
                         fontsize=10)
        return list(lines.values())

    _anim = FuncAnimation(fig, update, interval=80, blit=False, cache_frame_data=False)
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


if __name__ == "__main__":
    raise SystemExit(main())
