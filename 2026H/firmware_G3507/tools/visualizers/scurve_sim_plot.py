#!/usr/bin/env python3
"""纯 S 曲线滚球控制仿真结果作图。

数据来自 ``tools/checks/sim_ball_scurve.c``——它链接的是固件里真实的
``middleware/ball_scurve``，因此图上的行为就是上板会跑的控制律。

用法：
  python scurve_sim_plot.py --build --out ../../../output/scurve
  python scurve_sim_plot.py --csv-dir ../../../output/scurve

依赖：pip install matplotlib
"""
from __future__ import annotations

import argparse
import csv
import math
import subprocess
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager

_CJK_CANDIDATES = ("Microsoft YaHei", "SimHei", "SimSun", "Noto Sans CJK SC")
_AVAILABLE = {f.name for f in font_manager.fontManager.ttflist}
_CJK = next((f for f in _CJK_CANDIDATES if f in _AVAILABLE), None)
if _CJK:
    plt.rcParams["font.sans-serif"] = [_CJK, "sans-serif"]
plt.rcParams["axes.unicode_minus"] = False

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
SIM_DIR = REPO / "tools" / "checks"

SCENARIOS = [
    ("ideal", "理想对象", "只有计数量化与步进伺服"),
    ("gain", "换算不准", "K_G −25%、查表角 +15%"),
    ("vision", "测量链", "40fps + 55ms 时延 + 差分测速"),
    ("bow", "水管弯曲", "中部拱起 1.5 mm"),
    ("stick", "静摩擦", "脱离角 0.30° + 粗糙度"),
    ("level", "水平点偏", "真实水平比名义高 15 cnt"),
    ("real", "全部叠加", "上面所有因素同时存在"),
]
TARGET_MM = -50.0


def load(path: Path) -> dict[str, list[float]]:
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise SystemExit(f"空文件：{path}")
    return {key: [float(row[key]) for row in rows] for key in rows[0]}


def build_sim() -> None:
    source = SIM_DIR / "sim_ball_scurve.c"
    module = REPO / "middleware" / "ball_scurve" / "ball_scurve.c"
    binary = SIM_DIR / "sim_ball_scurve.exe"
    command = [
        "gcc", "-O2", "-std=c11", "-Wall",
        f"-I{REPO / 'middleware' / 'ball_scurve'}",
        "-o", str(binary), str(source), str(module), "-lm",
    ]
    subprocess.run(command, check=True)


def run_sim(scenario: str, out_dir: Path, servo_kp: float) -> Path:
    binary = SIM_DIR / "sim_ball_scurve.exe"
    target = out_dir / f"{scenario}_kp{servo_kp:g}.csv"
    out_dir.mkdir(parents=True, exist_ok=True)
    with target.open("w", newline="", encoding="utf-8") as handle:
        subprocess.run([str(binary), scenario, str(servo_kp), "120"],
                       stdout=handle, check=True)
    return target


def transit_rms(data: dict[str, list[float]]) -> float:
    """巡航段（|v_ref| > 30 mm/s）的剖面跟踪误差 RMS。"""
    errors = [xr - x for xr, x, vr in
              zip(data["x_ref"], data["x"], data["v_ref"]) if abs(vr) > 30.0]
    if not errors:
        return float("nan")
    return math.sqrt(sum(e * e for e in errors) / len(errors))


def final_error(data: dict[str, list[float]]) -> float:
    tail = [x for t, x in zip(data["t"], data["x"]) if t >= 9.0]
    return sum(tail) / len(tail) - TARGET_MM if tail else float("nan")


def identify_gain(data: dict[str, list[float]]) -> tuple[float, float]:
    """对 (查表水管角 beam, 球加速度) 做最小二乘，返回 (真实 K_G, 真实水平角)。

    这正是上板时该做的辨识：S 曲线移动本身已经在 ±3° 内扫过倾角，
    不需要另外设计标定动作。只取球确实在滚动的样本，避开静摩擦死区。

    ⚠ 必须用 ``beam``（编码器经查表算出的角，遥测里叫 beam）而不是对象真值：
      水平点标定误差恰恰体现为二者之差，用真值做回归等于假装误差不存在。
    """
    pairs = [(th, a) for th, a, v, stuck in
             zip(data["beam"], data["a"], data["v"], data["stuck"])
             if abs(v) > 20.0 and stuck < 0.5]
    if len(pairs) < 20:
        return float("nan"), float("nan")
    n = len(pairs)
    sx = sum(p[0] for p in pairs)
    sy = sum(p[1] for p in pairs)
    sxx = sum(p[0] * p[0] for p in pairs)
    sxy = sum(p[0] * p[1] for p in pairs)
    denominator = n * sxx - sx * sx
    if abs(denominator) < 1e-9:
        return float("nan"), float("nan")
    slope = (n * sxy - sx * sy) / denominator
    intercept = (sy - slope * sx) / n
    # a = slope*theta + intercept -> 零交点 theta0 = -intercept/slope
    return slope * 57.29577951308232, (-intercept / slope if slope else float("nan"))


def plot(csv_dir: Path, servo_kp: float, out_png: Path) -> None:
    figure, axes = plt.subplots(3, 1, figsize=(13, 11), sharex=True,
                                gridspec_kw={"height_ratios": [3, 2, 2]})
    reference_drawn = False
    summary = []

    for scenario, label, note in SCENARIOS:
        path = csv_dir / f"{scenario}_kp{servo_kp:g}.csv"
        if not path.exists():
            continue
        data = load(path)
        if not reference_drawn:
            axes[0].plot(data["t"], data["x_ref"], "k--", linewidth=2.0,
                         label="S 曲线参考 x_ref", zorder=5)
            axes[1].plot(data["t"], data["v_ref"], "k--", linewidth=2.0,
                         label="参考速度 v_ref", zorder=5)
            reference_drawn = True
        axes[0].plot(data["t"], data["x"], linewidth=1.3, label=f"{label}")
        axes[1].plot(data["t"], data["v"], linewidth=1.0, label=label)
        axes[2].plot(data["t"], data["theta_actual"], linewidth=1.0, label=label)
        summary.append((label, note, transit_rms(data), final_error(data),
                        *identify_gain(data)))

    for waypoint in (0.0, 50.0, -50.0):
        axes[0].axhline(waypoint, color="0.75", linewidth=0.8, zorder=0)
    axes[0].axhspan(-60.0, -40.0, color="green", alpha=0.08, zorder=0)
    axes[0].text(11.6, -50.0, "±10 mm 判据", va="center", ha="right",
                 fontsize=9, color="green")

    axes[0].set_ylabel("球位置 (mm)")
    axes[0].set_title(f"纯 S 曲线滚球控制 · O→+5cm→−5cm · 步进位置环 KP={servo_kp:g}")
    axes[0].legend(loc="upper left", fontsize=8, ncol=2)
    axes[0].grid(alpha=0.3)

    axes[1].set_ylabel("球速度 (mm/s)")
    axes[1].axhline(30.0, color="red", linestyle=":", linewidth=1.0)
    axes[1].axhline(-30.0, color="red", linestyle=":", linewidth=1.0)
    axes[1].text(11.6, 33.0, "±30 mm/s 逃逸速度", ha="right", fontsize=8, color="red")
    axes[1].grid(alpha=0.3)

    axes[2].set_ylabel("实际水管角 (deg)")
    axes[2].set_xlabel("时间 (s)")
    axes[2].grid(alpha=0.3)

    figure.tight_layout()
    figure.savefig(out_png, dpi=130)
    print(f"图已保存：{out_png}")

    print()
    print(f"{'场景':<10} {'说明':<26} {'巡航RMS':>9} {'末端误差':>10} "
          f"{'辨识K_G':>10} {'辨识水平角':>11}")
    print("-" * 82)
    for label, note, rms, final, gain, level in summary:
        print(f"{label:<10} {note:<26} {rms:>7.2f}mm {final:>8.2f}mm "
              f"{gain:>9.0f} {level:>9.3f}°")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build", action="store_true", help="先用 gcc 编译仿真器")
    parser.add_argument("--run", action="store_true", help="重新跑全部场景")
    parser.add_argument("--servo-kp", type=float, default=3.0,
                        help="步进位置环 KP（step_motor.h 的 STEP_MOTOR_SERVO_KP）")
    parser.add_argument("--csv-dir", type=Path,
                        default=REPO.parent / "output" / "scurve")
    parser.add_argument("--out", type=Path, default=None)
    arguments = parser.parse_args()

    if arguments.build:
        build_sim()
    if arguments.build or arguments.run:
        for scenario, _, _ in SCENARIOS:
            run_sim(scenario, arguments.csv_dir, arguments.servo_kp)

    out_png = arguments.out or (arguments.csv_dir /
                                f"scurve_kp{arguments.servo_kp:g}.png")
    plot(arguments.csv_dir, arguments.servo_kp, out_png)


if __name__ == "__main__":
    main()

