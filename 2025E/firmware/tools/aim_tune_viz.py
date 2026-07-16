#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
aim_tune_viz.py — 实时可视化云台前馈整定 (读 MCU Debug_Ex 遥测)。

用途: 改参数(位置环KP / lead / slew / 循迹平滑) → 烧录 → 跑 F1 Line+aim,
      本脚本实时画出跟踪好坏, 用于"看着调参"。

遥测格式 (app_e_task.c 每 ~20ms 一行, 115200):
    A,t_ms,line_state,gz,vCenter,heading,yawCmd,yawRateFF,yawActX10[,yawBias,visErrY,
      startupBias,steadyBias,visValid,visStatus,visionFrame,bldcAngleFrame,yawOmegaCmd,yawRpmCmd]
      line_state: 0=直线 1=角前冲 2=角弧转
      yawActX10 : F32C 上报 yaw 多圈角(0.1°); 逻辑角 = yaw_dir * yawActX10/10
      yawBias   : 视觉慢校正当前 yaw bias (deg); 旧固件无此列, 兼容解析
      visErrY   : 最近有效视觉帧 yaw 误差 (deg)
      后续诊断列由脚本兼容忽略，原始值仍完整写入 --save 文件。

四张图:
  1) yawCmd vs yawAct  —— 云台实际有没有追上指令 (看滞后/过冲)
  2) 跟踪误差 cmd-act   —— 激光方位偏差 (=激光在靶上抖多少); 目标进 ±1° 绿线内
  3) gz                —— 车蛇形晃动 (循迹平滑效果)
  4) yawBias / visErrY —— 视觉收敛曲线 (bias 应快收敛后平缓, visErr 应趋近 0)
背景按 line_state 着色: 黄=角前冲, 粉=角弧转, 白=直线。
标题实时统计: 直线段 gz/err 的 RMS, 转弯段 err 的 RMS/峰值。

用法:
    python aim_tune_viz.py                 # 默认 COM16 @115200
    python aim_tune_viz.py --port COM16 --window 15 --save run.csv
注意:
  - COM16 是 MCU 的 Debug_Ex; 若你在用 WHEELTEC 上位机(也占 COM16), 先关掉它,
    并把 USB-UART 接回 MCU 调试口, 再跑本脚本。
  - yaw_dir 默认 -1 (对应固件 GIMBAL_YAW_DIR=-1); 若固件改了符号, 用 --yaw-dir。
"""
import argparse
import collections
import math
import threading

import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


def norm180(a):
    while a >= 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


def rms(xs):
    return math.sqrt(sum(x * x for x in xs) / len(xs)) if xs else 0.0


def maxabs(xs):
    return max((abs(x) for x in xs), default=0.0)


class Reader(threading.Thread):
    """后台线程: 读串口, 解析遥测行, 存入线程安全 deque。"""

    def __init__(self, port, baud, yaw_dir, save_path=None):
        super().__init__(daemon=True)
        self.port, self.baud, self.yaw_dir = port, baud, yaw_dir
        self.lock = threading.Lock()
        # (t, state, gz, v, head, ycmd_uw, yact_plot, err, ybias, viserr)
        self.data = collections.deque(maxlen=60000)
        self.t0 = None
        self.prev_uw = None
        self.error = None
        self.running = True
        self.save = open(save_path, "w", encoding="utf-8") if save_path else None
        self.ser = None

    def run(self):
        try:
            ser = serial.Serial(self.port, self.baud, timeout=0.2)
            self.ser = ser
        except Exception as e:  # noqa
            self.error = f"打开 {self.port} 失败: {e} (WHEELTEC 上位机是否占用? 先关闭)"
            self._close_save()
            return
        while self.running:
            try:
                raw = ser.readline()
            except Exception as e:  # noqa
                self.error = f"读取 {self.port} 出错: {e}"
                break
            if not raw:
                continue
            if self.save:
                try:
                    self.save.write(raw.decode("ascii", "replace"))
                    self.save.flush()
                except Exception:  # noqa
                    pass
            s = raw.decode("ascii", "replace").strip()
            if not s.startswith("A,"):
                continue
            p = s.split(",")
            if len(p) < 9:
                continue
            try:
                t_ms = int(p[1]); state = int(p[2]); gz = float(p[3]); v = float(p[4])
                head = float(p[5]); ycmd = float(p[6]); _yff = float(p[7]); yactx10 = int(p[8])
            except ValueError:
                continue
            ybias = viserr = 0.0
            if len(p) >= 11:   # 新固件: 视觉 bias / 视觉误差列 (旧格式兼容)
                try:
                    ybias = float(p[9]); viserr = float(p[10])
                except ValueError:
                    pass
            yact = self.yaw_dir * yactx10 / 10.0
            err = norm180(ycmd - yact)
            # 脚本侧对 yawCmd 解缠, 使叠加图连续无 ±180 跳变
            ycmd_uw = ycmd if self.prev_uw is None else self.prev_uw + norm180(ycmd - self.prev_uw)
            self.prev_uw = ycmd_uw
            yact_plot = ycmd_uw - err  # 把 yawAct 对齐到解缠后的 cmd 帧, 可见间隙=误差
            if self.t0 is None:
                self.t0 = t_ms
            t = (t_ms - self.t0) / 1000.0
            with self.lock:
                self.data.append((t, state, gz, v, head, ycmd_uw, yact_plot, err,
                                  ybias, viserr))
        try:
            ser.close()
        except Exception:  # noqa
            pass
        self.ser = None
        self._close_save()

    def stop(self):
        """请求采集线程退出，并关闭串口以立即唤醒阻塞读取。"""
        self.running = False
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:  # noqa
                pass

    def _close_save(self):
        if self.save is not None:
            try:
                self.save.flush()
                self.save.close()
            except Exception:  # noqa
                pass
            self.save = None

    def snapshot(self):
        with self.lock:
            return list(self.data)


def shade_states(axes, ts, states):
    """按 line_state 给背景着色 (角前冲=黄, 角弧转=粉, 直线=白)。"""
    n = len(states)
    start = 0
    for i in range(1, n + 1):
        if i == n or states[i] != states[start]:
            st = states[start]
            if st != 0:
                color = "lightyellow" if st == 1 else "mistyrose"
                for ax in axes:
                    ax.axvspan(ts[start], ts[i - 1], color=color, alpha=0.6, zorder=0)
            start = i


def main():
    ap = argparse.ArgumentParser(description="云台前馈整定实时可视化")
    ap.add_argument("--port", default="COM16")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--window", type=float, default=15.0, help="显示窗口秒数")
    ap.add_argument("--yaw-dir", type=float, default=-1.0, help="对应固件 GIMBAL_YAW_DIR")
    ap.add_argument("--save", default=None, help="同时把原始行存到文件")
    args = ap.parse_args()

    reader = Reader(args.port, args.baud, args.yaw_dir, args.save)
    reader.start()

    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(11, 9))
    fig.canvas.manager.set_window_title(f"aim tune viz — {args.port}")

    def update(_frame):
        if reader.error:
            fig.suptitle(reader.error, color="red")
            return
        data = reader.snapshot()
        if not data:
            fig.suptitle("waiting for telemetry... (enter F1 Line+aim)", color="gray")
            return
        t_now = data[-1][0]
        win = [d for d in data if d[0] >= t_now - args.window]
        if len(win) < 2:
            return
        ts = [d[0] for d in win]
        states = [d[1] for d in win]
        gz = [d[2] for d in win]
        ycmd = [d[5] for d in win]
        yact = [d[6] for d in win]
        err = [d[7] for d in win]

        for ax in axes:
            ax.clear()
        shade_states(axes, ts, states)

        axes[0].plot(ts, ycmd, color="tab:blue", lw=1.2, label="yawCmd")
        axes[0].plot(ts, yact, color="tab:orange", lw=1.2, label="yawAct (F32C)")
        axes[0].set_ylabel("yaw (deg)")
        axes[0].legend(loc="upper left", fontsize=8)
        axes[0].grid(True, alpha=0.3)

        axes[1].plot(ts, err, color="tab:red", lw=1.2)
        axes[1].axhline(1.0, ls="--", color="green", lw=0.8)
        axes[1].axhline(-1.0, ls="--", color="green", lw=0.8)
        axes[1].axhline(0.0, color="k", lw=0.5)
        axes[1].set_ylabel("track err = cmd-act (deg)")
        axes[1].grid(True, alpha=0.3)

        axes[2].plot(ts, gz, color="tab:green", lw=1.0)
        axes[2].axhline(0.0, color="k", lw=0.5)
        axes[2].set_ylabel("gz car yaw-rate (deg/s)")
        axes[2].grid(True, alpha=0.3)

        ybias = [d[8] for d in win]
        viserr = [d[9] for d in win]
        axes[3].plot(ts, ybias, color="tab:blue", lw=1.2, label="yawBias")
        axes[3].plot(ts, viserr, color="tab:gray", lw=0.8, label="visErrY")
        axes[3].axhline(0.0, color="k", lw=0.5)
        axes[3].set_ylabel("vision (deg)")
        axes[3].set_xlabel("t (s)")
        axes[3].legend(loc="upper left", fontsize=8)
        axes[3].grid(True, alpha=0.3)

        s_gz = [d[2] for d in win if d[1] == 0]
        s_err = [d[7] for d in win if d[1] == 0]
        c_err = [d[7] for d in win if d[1] in (1, 2)]
        title = (
            f"STRAIGHT  shimmy gz_rms={rms(s_gz):.1f} deg/s | "
            f"track err rms={rms(s_err):.2f} peak={maxabs(s_err):.2f} deg    ||    "
            f"CORNER  track err rms={rms(c_err):.2f} peak={maxabs(c_err):.2f} deg"
        )
        fig.suptitle(title, fontsize=10)

    _anim = FuncAnimation(fig, update, interval=150, cache_frame_data=False)
    try:
        plt.tight_layout(rect=(0, 0, 1, 0.97))
        plt.show()
    finally:
        reader.stop()
        reader.join(timeout=1.0)


if __name__ == "__main__":
    main()
