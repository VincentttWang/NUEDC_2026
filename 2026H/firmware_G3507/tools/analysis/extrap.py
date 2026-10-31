#!/usr/bin/env python3
"""核对 MSPM0 侧的位置外推：控制器实际使用的 x 是不是 xr + v*age。

这条路径此前在仿真里**完全没有建模**（仿真直接把测量位置喂给控制器）。
它把一个带滞后的速度信号乘上测量龄期后加回位置，等于在环路里插入一条
额外的、相位可疑的反馈通道——是极限环的候选成因。
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

import numpy as np

FIELD = re.compile(r"(\w+)=(-?[\d.]+)")


def load(path: Path, lo: float, hi: float) -> list[dict[str, float]]:
    rows = []
    for line in path.read_text(encoding="ascii", errors="ignore").splitlines():
        if not line.startswith("[SCV]") or "drop=" not in line:
            continue
        rows.append({m.group(1): float(m.group(2)) for m in FIELD.finditer(line)})
    t0 = rows[0]["t"]
    for r in rows:
        r["t"] = (r["t"] - t0) / 1000.0
    return [r for r in rows if lo <= r["t"] <= hi]


def main() -> None:
    base = Path(sys.argv[1])
    cases = (
        ("极限环段", "com16_dither.txt", 40.0, 90.0),
        ("稳定段", "com16_holdsched.txt", 20.0, 110.0),
        ("KP=3 段", "com16_long.txt", 40.0, 60.0),
    )
    for name, filename, lo, hi in cases:
        rows = load(base / filename, lo, hi)
        xr = np.array([r["xr"] for r in rows])
        x = np.array([r["x"] for r in rows])
        v = np.array([r["v"] for r in rows])
        age = np.array([r["age"] for r in rows]) / 1000.0

        extrapolation = v * age
        residual = x - (xr + extrapolation)
        rms = float(np.sqrt(np.mean(residual**2)))
        verdict = "确认" if rms < 0.15 else "不完全吻合"

        print(f"--- {name}（{len(rows)} 样本）---")
        print(f"  x − xr              均值{np.mean(x - xr):+7.3f}  "
              f"RMS{np.sqrt(np.mean((x - xr)**2)):6.3f}  "
              f"最大{np.max(np.abs(x - xr)):6.3f} mm")
        print(f"  x − (xr + v·age)    均值{np.mean(residual):+7.3f}  "
              f"RMS{rms:6.3f}  最大{np.max(np.abs(residual)):6.3f} mm")
        print(f"  => 外推公式 x = xr + v·age：{verdict}")
        print(f"  外推项 v·age 幅值 RMS {np.sqrt(np.mean(extrapolation**2)):.3f} mm，"
              f"为位置标准差的 {100 * np.sqrt(np.mean(extrapolation**2)) / max(np.std(xr), 1e-9):.0f}%")
        print(f"  age 均值 {np.mean(age) * 1000:.0f} ms，最大 {np.max(age) * 1000:.0f} ms")
        print()


if __name__ == "__main__":
    main()
