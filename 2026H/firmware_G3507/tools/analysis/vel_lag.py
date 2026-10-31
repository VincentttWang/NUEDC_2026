#!/usr/bin/env python3
"""从实机遥测直接测量「树莓派速度信号」相对「位置数值微分」的时延。

方法：极限环段是一个频率已知（约 6.4 rad/s）的准正弦，因此可以在该频率上
同时测相位差和幅值比，用一阶滞后模型 1/(1+jωτ) 两路独立反解 τ 并交叉验证：

    相位法  phase = atan(ωτ)
    幅值法  gain  = 1/sqrt(1+(ωτ)^2)

两者一致 => 一阶模型成立，τ 可信；不一致 => 滤波不是一阶，需要换模型。
"""
from __future__ import annotations

import math
import re
import sys
from pathlib import Path

import numpy as np

FIELD = re.compile(r"(\w+)=(-?[\d.]+)")


def load(path: Path) -> list[dict[str, float]]:
    rows = []
    for line in path.read_text(encoding="ascii", errors="ignore").splitlines():
        if not line.startswith("[SCV]") or not line.rstrip().endswith(tuple("0123456789")):
            continue
        if "drop=" not in line:
            continue
        rows.append({m.group(1): float(m.group(2)) for m in FIELD.finditer(line)})
    if not rows:
        raise SystemExit(f"没有解析到 [SCV] 行：{path}")
    t0 = rows[0]["t"]
    for r in rows:
        r["t"] = (r["t"] - t0) / 1000.0
    return rows


def fit_sine(t: np.ndarray, y: np.ndarray, omega: float) -> tuple[float, float]:
    """在给定角频率上做最小二乘拟合，返回 (幅值, 相位 rad)。"""
    design = np.column_stack([np.cos(omega * t), np.sin(omega * t), np.ones_like(t)])
    coefficients, *_ = np.linalg.lstsq(design, y, rcond=None)
    a, b = coefficients[0], coefficients[1]
    return math.hypot(a, b), math.atan2(-b, a)


def dominant_omega(t: np.ndarray, x: np.ndarray) -> float:
    """用均值上行过零估计主频。"""
    centred = x - x.mean()
    crossings = [
        t[i] for i in range(1, len(t))
        if centred[i - 1] < 0.0 <= centred[i]
    ]
    if len(crossings) < 3:
        raise SystemExit("过零次数不足，无法估主频")
    period = float(np.mean(np.diff(crossings)))
    return 2.0 * math.pi / period


def main() -> None:
    path = Path(sys.argv[1])
    lo = float(sys.argv[2]) if len(sys.argv) > 2 else 40.0
    hi = float(sys.argv[3]) if len(sys.argv) > 3 else 90.0

    rows = [r for r in load(path) if lo <= r["t"] <= hi]
    t = np.array([r["t"] for r in rows])
    xr = np.array([r["xr"] for r in rows])
    vr = np.array([r.get("vr", r["v"]) for r in rows])
    v = np.array([r["v"] for r in rows])

    omega = dominant_omega(t, xr)
    print(f"数据段 {lo:.0f}~{hi:.0f} s，{len(rows)} 个样本，"
          f"平均采样 {np.mean(np.diff(t)) * 1000:.0f} ms")
    print(f"主频 ω = {omega:.2f} rad/s（周期 {2 * math.pi / omega:.2f} s）")
    print()

    # 位置的数值微分作为「无滤波速度」的参照。
    # 中心差分对一阶滞后没有额外相位，只有 O(dt²) 的幅值误差，量级可忽略。
    dxdt = np.gradient(xr, t)

    amp_ref, ph_ref = fit_sine(t, dxdt, omega)
    print(f"{'信号':<22}{'幅值':>10}{'相位(deg)':>12}{'相对参照':>12}")
    print("-" * 58)
    print(f"{'d(xr)/dt 数值微分':<22}{amp_ref:>10.1f}{math.degrees(ph_ref):>12.1f}"
          f"{'(参照)':>12}")

    for name, sig in (("vr 树莓派原始测速", vr), ("v  控制器实际用值", v)):
        amp, ph = fit_sine(t, sig, omega)
        lag_deg = math.degrees((ph_ref - ph + math.pi) % (2 * math.pi) - math.pi)
        print(f"{name:<22}{amp:>10.1f}{math.degrees(ph):>12.1f}{lag_deg:>11.1f}°")

        gain = amp / amp_ref
        tau_phase = math.tan(math.radians(lag_deg)) / omega if lag_deg > 0 else float("nan")
        tau_gain = (math.sqrt(max(1.0 / gain**2 - 1.0, 0.0)) / omega) if gain > 0 else float("nan")
        print(f"{'':<22}幅值比 {gain:.3f}   "
              f"τ(相位法) {tau_phase * 1000:6.0f} ms   "
              f"τ(幅值法) {tau_gain * 1000:6.0f} ms")
        if math.isfinite(tau_phase) and math.isfinite(tau_gain):
            consistent = abs(tau_phase - tau_gain) < 0.4 * max(tau_phase, tau_gain)
            print(f"{'':<22}两法一致性：{'一致 → 一阶模型成立' if consistent else '不一致 → 不是一阶滤波'}")
            alpha = 0.025 / (tau_phase + 0.025)   # 40 fps 下等效的一阶系数
            print(f"{'':<22}折算 40 fps 一阶系数 alpha ≈ {alpha:.3f}"
                  f"（仿真里此前硬编码 0.35）")
        print()


if __name__ == "__main__":
    main()
