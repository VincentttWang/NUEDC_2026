#!/usr/bin/env python3
"""从 [SCV] 遥测中提取落点，统计分布。

回答的问题：**落点误差有多少是可重复的偏置，多少是随机的？**

这件事重要，是因为纯 PD 的静摩擦残差上界是 θ_stick/Kp = 13.2 mm，而我们只
测到过一次 1.84 mm。若散布确实很小，说明捕获点由可重复的物理量（局部平衡角）
决定，那么加一个标定偏置就够；若散布接近 13 mm，说明捕获点近乎随机，
就必须上主动解困（HOLD 积分或抖动）。

## 落点怎么定义

**不用固件的 `[SCV] arrive` 行**——它在剖面时钟走完时触发，那时球往往还在动。
真正的落点是"静摩擦把球钉住之后"的位置，所以按下面的规则从遥测里找：

    在每个航点段内，取最后一段连续满足 |v| < still_speed 的样本，
    对其 x 求均值。若该段短于 still_time，判为"未稳定"，单独计数。

## 用法

    python landing_stats.py capture1.txt [capture2.txt ...]
    python landing_stats.py --still-speed 3 --still-time 1.0 cap.txt
"""
from __future__ import annotations

import argparse
import math
import re
import statistics
from pathlib import Path

FIELD = re.compile(r"(\w+)=(-?[\d.]+)")
# 一整行遥测的形状：以 [SCV] t=<数字> 开头、以 drop=<数字> 收尾。串口偶尔会把
# 两行搅在一起（缓冲区瞬时满），那种行里会出现 `-1.37.0353` 这类拼接出来的
# 伪数字——不能让它把整次分析带崩，直接整行丢弃。
WELL_FORMED = re.compile(r"^\[SCV\] t=\d+ .*drop=\d+\s*$")


def load(path: Path) -> tuple[list[dict[str, float]], int]:
    rows: list[dict[str, float]] = []
    dropped = 0
    for line in path.read_text(encoding="ascii", errors="ignore").splitlines():
        if not line.startswith("[SCV]") or "drop=" not in line:
            continue                      # 空行与 plan/resume/arrive 日志行
        if not WELL_FORMED.match(line):
            dropped += 1
            continue
        try:
            row = {m.group(1): float(m.group(2)) for m in FIELD.finditer(line)}
        except ValueError:
            dropped += 1
            continue
        if "x" in row and "t" in row:
            rows.append(row)
    if not rows:
        return [], dropped
    t0 = rows[0]["t"]
    for row in rows:
        row["t"] = (row["t"] - t0) / 1000.0
    return rows, dropped


def segments(rows: list[dict[str, float]]) -> list[tuple[float, list[dict]]]:
    """按目标值切段。目标一变就是新的一次落点尝试。"""
    out: list[tuple[float, list[dict]]] = []
    current: list[dict] = []
    target = None
    for row in rows:
        tgt = row.get("tgt", 0.0)
        if target is None or abs(tgt - target) > 0.5:
            if current:
                out.append((target, current))
            target, current = tgt, []
        current.append(row)
    if current:
        out.append((target, current))
    return out


def landing(seg: list[dict], still_speed: float, still_time: float,
            max_gap: float = 0.30):
    """段末连续静止区间的位置均值；未静止则返回 None。

    两个必须排除的伪静止：

    1. **视觉丢失**。DEGRADED 期间固件打的是 `ok=0 x=0.00 v=0.00`，
       若照单全收，会被判成"在 0 mm 处纹丝不动"——凭空造出一个落点。
    2. **跨黑屏拼接**。丢掉 ok=0 的行之后，两侧 ok=1 的样本在时间上不连续；
       若直接相邻处理，等于假设黑屏期间球没动过。超过 max_gap 就断开。
    """
    still: list[dict] = []
    for row in reversed(seg):
        if row.get("ok", 1.0) < 0.5:
            break                                   # 视觉丢失，静止区间到此为止
        if abs(row.get("v", 0.0)) >= still_speed:
            break
        if still and (still[-1]["t"] - row["t"]) > max_gap:
            break                                   # 中间有黑屏，不能跨过去
        still.append(row)
    if not still:
        return None
    span = still[0]["t"] - still[-1]["t"]
    if span < still_time:
        return None
    return statistics.fmean(r["x"] for r in still), span, len(still)


def report(name: str, values: list[float], target: float) -> None:
    errors = [v - target for v in values]
    n = len(errors)
    mean = statistics.fmean(errors)
    spread = statistics.pstdev(errors) if n > 1 else 0.0
    inside = sum(1 for e in errors if abs(e) < 10.0)
    print(f"  {name:<22} n={n:<3} "
          f"均值{mean:+7.2f}  标准差{spread:6.2f}  "
          f"范围[{min(errors):+7.2f},{max(errors):+7.2f}]  "
          f"±10mm {inside}/{n}")
    if n > 1:
        # 偏置 vs 随机：均值是可标定掉的部分，标准差是标定不掉的部分。
        share = abs(mean) / (abs(mean) + spread) if (abs(mean) + spread) > 0 else 0.0
        print(f"  {'':<22}     └ 可标定的偏置占 {share:.0%}，"
              f"标定后残余约 ±{spread:.2f} mm")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("files", nargs="+", type=Path)
    parser.add_argument("--still-speed", type=float, default=3.0,
                        help="判静止的速度阈值 mm/s（默认 3）")
    parser.add_argument("--still-time", type=float, default=0.8,
                        help="静止需持续多久才算落点 s（默认 0.8）")
    arguments = parser.parse_args()

    by_target: dict[float, list[float]] = {}
    unsettled = 0
    total = 0

    for path in arguments.files:
        rows, dropped = load(path)
        if not rows:
            print(f"⚠ {path.name}：没有可用的 [SCV] 行")
            continue
        blind = sum(1 for r in rows if r.get("ok", 1.0) < 0.5)
        print(f"\n=== {path.name}（{len(rows)} 样本，{rows[-1]['t']:.0f} s）===")
        avail = 1.0 - blind / len(rows)
        print(f"  视觉可用率 {avail:.0%}（{blind} 行 ok=0），坏行丢弃 {dropped}")
        if avail < 0.90:
            print(f"  ⚠ 视觉可用率低于 90%，落点统计的样本量与代表性都不足，"
                  f"结论仅供参考")
        print(f"  {'段':<4}{'目标':>8}{'落点':>9}{'误差':>9}{'静止时长':>10}")
        for index, (target, seg) in enumerate(segments(rows)):
            total += 1
            result = landing(seg, arguments.still_speed, arguments.still_time)
            if result is None:
                unsettled += 1
                print(f"  {index:<4}{target:>8.1f}{'未稳定':>11}"
                      f"{'':>9}{'':>10}")
                continue
            x, span, count = result
            by_target.setdefault(target, []).append(x)
            print(f"  {index:<4}{target:>8.1f}{x:>9.2f}{x - target:>+9.2f}"
                  f"{span:>9.1f}s")

    print(f"\n=== 按航点汇总（共 {total} 段，未稳定 {unsettled} 段）===")
    for target in sorted(by_target, reverse=True):
        report(f"目标 {target:+.0f} mm", by_target[target], target)

    every = [v - t for t, vs in by_target.items() for v in vs]
    if len(every) > 1:
        print(f"\n=== 全部落点 ===")
        report("合计", [e for e in every], 0.0)
        print(f"\n  静摩擦残差理论上界 θ_stick/Kp = 0.62/0.04711 = 13.2 mm")
        print(f"  实测最大 |误差| = {max(abs(e) for e in every):.2f} mm "
              f"（占上界 {max(abs(e) for e in every) / 13.2:.0%}）")
        if statistics.pstdev(every) < 2.0:
            print("  → 散布很小：捕获点由可重复的物理量决定，加标定偏置即可")
        else:
            print("  → 散布较大：捕获点近乎随机，需要主动解困（HOLD 积分 / 抖动）")


if __name__ == "__main__":
    main()
