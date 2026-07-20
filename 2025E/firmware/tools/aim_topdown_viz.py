#!/usr/bin/env python3
"""2025E 云台/车体朝向 2D 俯视回放可视化。

读取 MCU debug 串口导出的 CSV 遥测 (每行以 `A,` 开头的 25 字段瞄准遥测), 在世界系
俯视图里回放:
  - 轨道 (1m 正方形 A/C/D/B) 与标靶位置;
  - 车辆轨迹 (poseX,poseY) 与当前车位置;
  - 车体朝向箭头 (heading);
  - 云台【指令】朝向箭头 (世界指向 = heading + yawCmd);
  - 云台【实际】朝向箭头 (世界指向 = heading + GIMBAL_YAW_DIR*yawActX10/10, 多圈解缠);
  - 车->靶视线 (细虚线; 瞄得准时应与云台指令箭头重合)。
交互: 底部时间滑条拖动; Play 按钮播放/暂停; 键盘 空格=播放/暂停, 左右=单步, q=退出。

CSV 字段 (A 行, 与 app_e_task.c 遥测一致):
  A,t_ms,lineState,gz,vCenter,heading,yawCmd,yawRateFF,yawActX10,yawBias,visErrY,
    startupBias,steadyBias,visValid,visStatus,visionFrame,bldcAngleFrame,yawOmegaCmd,
    yawRpmCmd,pitchBias,visErrP,poseX,poseY,bodyPitch,bodyRoll

用法:
  python aim_topdown_viz.py --csv ../../run.csv
  python aim_topdown_viz.py --csv run.csv --target 0,0.5 --side 1.0

依赖: pip install matplotlib numpy
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.widgets import Slider, Button

# ---- 中文字体 (从已安装字体里挑一个存在的, 缺失则退回默认) ----
_CJK = ("Microsoft YaHei", "SimHei", "SimSun", "Noto Sans CJK SC",
        "Source Han Sans SC", "WenQuanYi Zen Hei", "Arial Unicode MS")
_HAVE = {f.name for f in font_manager.fontManager.ttflist}
_FONT = next((f for f in _CJK if f in _HAVE), None)
plt.rcParams["font.sans-serif"] = [_FONT, "sans-serif"] if _FONT else ["sans-serif"]
plt.rcParams["axes.unicode_minus"] = False

STATE_NAME = {0: "FOLLOW", 1: "CORNER_FWD", 2: "CORNER_ARC"}


def norm180(a):
    """归一化角度到 [-180,180)。支持标量或 numpy 数组。"""
    return (np.asarray(a) + 180.0) % 360.0 - 180.0


# A 行 'A' 之后的数值列名。旧格式=前 24 列; 新格式尾部追加 5 列(pitch 环+转弯占空比)。
# 兼容两种: 不足的尾列补 NaN, 超出截断。
CSV_COLS = ["t_ms", "state", "gz", "v", "heading", "yawCmd", "yawRateFF",
            "yawActX10", "yawBias", "visErrY", "startupBias", "steadyBias",
            "visValid", "visStatus", "visionFrame", "bldcAngleFrame",
            "yawOmegaCmd", "yawRpmCmd", "pitchBias", "visErrP",
            "poseX", "poseY", "bodyPitch", "bodyRoll",
            "pitchCmd", "pitchActX10", "pitchRateFF", "dutyL", "dutyR"]
CSV_MIN_COLS = 24   # 至少要有的基础(yaw)数值列数


def load_csv(path: Path):
    """鲁棒解析: 忽略二进制噪声, 只取 A 行, 逐字段 float 容错; 兼容 24/29 值两种列数。"""
    rows = []
    text = path.read_text(encoding="utf-8", errors="ignore")
    for line in text.splitlines():
        line = line.strip().strip("\x00")
        if not line.startswith("A,"):
            continue
        parts = line.split(",")[1:1 + len(CSV_COLS)]   # 跳过 'A', 最多取 len(CSV_COLS) 个
        try:
            vals = [float(p) for p in parts]           # float() 自动去尾部 \r 等空白
        except ValueError:
            continue
        if len(vals) < CSV_MIN_COLS:
            continue
        vals += [float("nan")] * (len(CSV_COLS) - len(vals))   # 尾列补 NaN
        rows.append(vals)
    if not rows:
        raise SystemExit(f"未在 {path} 中找到有效的 'A,' 遥测行")
    a = np.array(rows, dtype=float)
    return {c: a[:, i] for i, c in enumerate(CSV_COLS)}


def main():
    ap = argparse.ArgumentParser(description="2025E 云台/车体朝向 2D 俯视回放")
    ap.add_argument("--csv", default="run.csv", help="遥测 CSV 路径 (默认 run.csv)")
    ap.add_argument("--target", default="0,0.5", help="靶心世界坐标 x,y (默认 0,0.5)")
    ap.add_argument("--side", type=float, default=1.0, help="轨道边长 m (默认 1.0)")
    ap.add_argument("--paper-w", type=float, default=0.297,
                    help="靶纸(A4)俯视水平宽度 m: 横放 0.297(默认) / 竖放 0.210")
    ap.add_argument("--ext", type=float, default=6.0, help="箭头延长线长度 m (默认 6)")
    ap.add_argument("--mount-x", type=float, default=0.120,
                    help="云台支点相对轮轴前移 m (车体前正; 标定 mount_x_m, 默认 0.12)")
    ap.add_argument("--mount-y", type=float, default=0.0,
                    help="云台支点横向偏移 m (车体左正; 标定 mount_y_m, 默认 0)")
    ap.add_argument("--lat", type=float, default=0.035,
                    help="激光束相对 yaw 轴横向偏移 m (左正; 标定 laser_lateral_offset, 默认 0.035)")
    ap.add_argument("--gimbal-yaw-dir", type=float, default=-1.0,
                    help="GIMBAL_YAW_DIR: 逻辑角=该值*yawActX10/10 (默认 -1)")
    ap.add_argument("--arrow", type=float, default=0.30, help="云台箭头长度 m")
    ap.add_argument("--interval", type=int, default=60, help="播放帧间隔 ms")
    args = ap.parse_args()

    tx, ty = (float(v) for v in args.target.split(","))
    half = args.side / 2.0
    # 角点 (与 localization.c 一致): A(-half,0) B(half,0) C(-half,-side) D(half,-side)
    A = (-half, 0.0); B = (half, 0.0); C = (-half, -args.side); D = (half, -args.side)

    d = load_csv(Path(args.csv))
    n = len(d["t_ms"])
    t0 = d["t_ms"][0]
    ts = (d["t_ms"] - t0) / 1000.0     # 相对秒
    px, py = d["poseX"], d["poseY"]
    heading = d["heading"]
    # 世界系朝向 (deg)
    body_w = heading
    gim_cmd_w = norm180(heading + d["yawCmd"])
    gim_act_body = norm180(args.gimbal_yaw_dir * d["yawActX10"] / 10.0)
    gim_act_w = norm180(heading + gim_act_body)
    # 云台支点世界坐标 = 轮轴中心 + mount 偏移经航向旋转 (与固件 aim_solver 一致)。
    # 固件从【支点】算方位角瞄准, 故云台箭头/延长线/视线都须从支点画; 否则车头与视线
    # 近垂直的边(如顶边)上, mount_x 前移会造成十几度的"假脱靶"(实为建模缺失)。
    hr = np.radians(heading)
    gx = px + args.mount_x * np.cos(hr) - args.mount_y * np.sin(hr)
    gy = py + args.mount_x * np.sin(hr) + args.mount_y * np.cos(hr)
    sight_w = np.degrees(np.arctan2(ty - gy, tx - gx))   # 支点->靶真实视线

    # ---------------- 画布 ----------------
    fig = plt.figure(figsize=(9.5, 9.0))
    ax = fig.add_axes([0.07, 0.16, 0.72, 0.78])
    ax.set_aspect("equal", adjustable="box")
    ax.set_title("云台/车体朝向 2D 俯视回放", fontsize=13)
    ax.set_xlabel("世界 X (m)"); ax.set_ylabel("世界 Y (m)")
    ax.grid(True, ls=":", alpha=0.4)

    # 静态: 轨道方框 (路径 A->C->D->B->A), 角点, 靶
    sq = [A, C, D, B, A]
    ax.plot([p[0] for p in sq], [p[1] for p in sq], "-", color="#888", lw=1.6, label="轨道")
    for name, p in (("A", A), ("B", B), ("C", C), ("D", D)):
        ax.plot(*p, "s", color="#555", ms=5)
        ax.annotate(name, p, textcoords="offset points", xytext=(6, 6), color="#555")
    # 标靶: A4 靶纸是竖直平面 (平行于 AB), 俯视图里投影成一条水平线段, 长度=纸张水平宽度。
    pw = args.paper_w
    ax.plot([tx - pw / 2, tx + pw / 2], [ty, ty], "-", color="#d62728", lw=4,
            solid_capstyle="butt", label="标靶(A4)", zorder=5)
    ax.plot([tx], [ty], "+", color="#d62728", ms=8, zorder=6)   # 靶心
    ax.annotate("靶 A4 %.0fmm" % (pw * 1000.0), (tx + pw / 2, ty),
                textcoords="offset points", xytext=(6, 4), color="#d62728")
    # 全程轨迹 (浅灰背景)
    ax.plot(px, py, "-", color="#cfcfcf", lw=1.0, zorder=1, label="轨迹")

    # 视图范围: 覆盖轨道+靶+轨迹, 留边
    xs = np.concatenate([px, [tx, A[0], B[0]]]); ys = np.concatenate([py, [ty, A[1], C[1]]])
    m = 0.25
    ax.set_xlim(xs.min() - m, xs.max() + m)
    ax.set_ylim(ys.min() - m, ys.max() + m)

    # 动态艺术家 (index i)
    tail, = ax.plot([], [], "-", color="#1f77b4", lw=2.2, zorder=3, label="已走轨迹")
    car, = ax.plot([], [], "o", color="#111", ms=9, zorder=6)
    pivot, = ax.plot([], [], "x", color="#555", ms=8, mew=1.6, zorder=6, label="云台支点")
    sight, = ax.plot([], [], "--", color="#d62728", lw=1.0, alpha=0.7, zorder=4, label="支点->靶视线")

    def new_quiver(color, label):
        return ax.quiver([px[0]], [py[0]], [0.0], [0.0], color=color, angles="xy",
                         scale_units="xy", scale=1.0, width=0.008, zorder=7, label=label)
    q_body = new_quiver("#111", "车体朝向")
    q_cmd = new_quiver("#1f77b4", "云台指令")
    q_act = new_quiver("#2ca02c", "云台实际")

    # 箭头延长线 (同色虚线): 直观看各朝向的指向是否命中靶纸线段
    ext_body, = ax.plot([], [], ":", color="#111", lw=0.9, alpha=0.5, zorder=2)
    ext_cmd, = ax.plot([], [], ":", color="#1f77b4", lw=1.0, alpha=0.6, zorder=2)
    ext_act, = ax.plot([], [], ":", color="#2ca02c", lw=1.0, alpha=0.6, zorder=2)

    ax.legend(loc="upper left", fontsize=8, framealpha=0.9)
    info = fig.text(0.81, 0.90, "", fontsize=9, va="top", family="monospace")

    L = args.arrow

    def update(i):
        i = int(i)
        x, y = px[i], py[i]        # 轮轴中心 (车体)
        gxv, gyv = gx[i], gy[i]    # 云台支点
        tail.set_data(px[:i + 1], py[:i + 1])
        car.set_data([x], [y])
        pivot.set_data([gxv], [gyv])
        sight.set_data([gxv, tx], [gyv, ty])   # 从支点看靶
        # 车体箭头/延长线: 从轮轴中心
        rb = math.radians(body_w[i]); cb, sb = math.cos(rb), math.sin(rb)
        q_body.set_offsets([[x, y]]); q_body.set_UVC([L * 0.7 * cb], [L * 0.7 * sb])
        ext_body.set_data([x, x + args.ext * cb], [y, y + args.ext * sb])
        # 云台指令/实际: 箭头与延长线同为一条【真实激光束】线(共线), 从光束出射点(支点侧移
        # lat, 左正)沿指向出射; 落在靶面 = 真实命中点(对准正确应落在 A4 线段内)。支点(×)标 yaw
        # 轴, 光束出射点相对轴侧移 lat(≈3.5cm), 故光束起点与支点有一小段间距(物理真实)。
        for q, ext, ang, ln in ((q_cmd, ext_cmd, gim_cmd_w[i], L),
                                (q_act, ext_act, gim_act_w[i], L * 0.85)):
            r = math.radians(ang)
            c, s = math.cos(r), math.sin(r)
            bx, by = gxv - args.lat * s, gyv + args.lat * c   # 光束出射点=支点+lat*左法向
            q.set_offsets([[bx, by]])                         # 箭头也从出射点 -> 与延长线共线
            q.set_UVC([ln * c], [ln * s])
            ext.set_data([bx, bx + args.ext * c], [by, by + args.ext * s])
        st = int(d["state"][i])
        # 等宽面板用 ASCII 标签: 保住数字列对齐, 且避免等宽字体缺中文字形的告警。
        info.set_text(
            f"frame {i+1}/{n}\n"
            f"t     = {ts[i]:7.2f} s\n"
            f"state = {STATE_NAME.get(st, st)}\n"
            f"pos   = ({x:+.3f},{y:+.3f})\n"
            f"body heading = {heading[i]:+6.1f}\n"
            f"gim  yawCmd  = {d['yawCmd'][i]:+6.1f}\n"
            f"cmd ->world  = {gim_cmd_w[i]:+6.1f}\n"
            f"act ->world  = {gim_act_w[i]:+6.1f}\n"
            f"sightline    = {sight_w[i]:+6.1f}\n"
            f"cmd - sight  = {norm180(gim_cmd_w[i]-sight_w[i]):+5.1f}\n"
            f"act - cmd    = {norm180(gim_act_w[i]-gim_cmd_w[i]):+5.1f}\n"
            f"omega gz = {d['gz'][i]:+6.1f} deg/s\n"
            f"v = {d['v'][i]:+.2f} m/s\n"
            f"visValid={int(d['visValid'][i])} visErrY={d['visErrY'][i]:+.2f}\n"
            f"laser {'ON ' if (d['visValid'][i] and abs(d['visErrY'][i]) < 3.0) else 'OFF'} target (|visErrY|<3)"
        )
        fig.canvas.draw_idle()

    # 滑条
    ax_sl = fig.add_axes([0.10, 0.06, 0.66, 0.03])
    sl = Slider(ax_sl, "帧", 0, n - 1, valinit=0, valstep=1)
    sl.on_changed(update)

    # 播放按钮 + 定时器
    ax_btn = fig.add_axes([0.80, 0.055, 0.10, 0.04])
    btn = Button(ax_btn, "Play")
    state = {"playing": False}
    timer = fig.canvas.new_timer(interval=args.interval)

    def tick():
        i = int(sl.val)
        if i >= n - 1:
            stop()
            return
        sl.set_val(i + 1)   # 触发 update

    def start():
        state["playing"] = True
        btn.label.set_text("Pause")
        timer.start()

    def stop():
        state["playing"] = False
        btn.label.set_text("Play")
        timer.stop()

    timer.add_callback(tick)

    def toggle(_evt=None):
        stop() if state["playing"] else start()

    btn.on_clicked(toggle)

    def on_key(evt):
        if evt.key == " ":
            toggle()
        elif evt.key == "right":
            sl.set_val(min(n - 1, int(sl.val) + 1))
        elif evt.key == "left":
            sl.set_val(max(0, int(sl.val) - 1))
        elif evt.key == "q":
            plt.close(fig)

    fig.canvas.mpl_connect("key_press_event", on_key)

    update(0)
    print(f"载入 {n} 帧, 时长 {ts[-1]:.1f}s. 空格=播放/暂停, 左右=单步, q=退出.")
    plt.show()


if __name__ == "__main__":
    main()
