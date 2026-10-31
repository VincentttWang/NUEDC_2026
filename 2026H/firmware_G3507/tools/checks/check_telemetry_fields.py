#!/usr/bin/env python3
"""核对固件遥测字段、上位机用到的字段、以及 UART 带宽预算。

## 为什么需要这个检查

遥测拆成 [SCV]/[SCVD] 两行分频之后，最容易出的错是**上位机去读一个已经
搬家的字段**——不会报错，只会静默显示 0，看图的人根本察觉不到。

第二类错是**带宽悄悄超限**：字段是一次加一个的，每次看都"只多了一点"，
累积到 797 字符/60 ms = 115% 时 DebugUart 环形缓冲开始丢字节，遥测出现
随机截断。这正是本项目真实踩过的坑。

## 用法

    python tools/checks/check_telemetry_fields.py

退出码非 0 表示有问题。
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
TASK = ROOT / "app" / "app_ball_scurve_task.c"
TUNE_HTML = ROOT / "tools" / "visualizers" / "ball_tune.html"

UART_BPS = 115200 / 10.0        # 8N1：每字节 10 位
# 预算随遥测模式变化：
#   拆分版（[SCV]+[SCVD]）：[TUNE] 热更回显最多占 ~5%，加状态日志约 5%，预留 20% 余量 → 70%
#   单行版（无 [TUNE]）  ：偶发 plan/arrive 日志行 <2%，物理极限 ~95%，宽松限 90%
#   这个常量在 main() 里会按是否有 [SCVD] 行动态选择。
BUDGET_SPLIT  = 0.70    # 拆分模式（有 [TUNE]）
BUDGET_SINGLE = 0.90    # 单行模式（无 [TUNE]，只有偶发日志行）
CTRL_TICK_MS = 20.0

# 各分析脚本依赖的字段，必须全在 [SCV]（高频行）里。
ANALYSIS_NEEDS = {
    "landing_stats.py": {"t", "x", "v", "tgt"},
    "vel_lag.py": {"t", "xr", "v"},
    "extrap.py": {"t", "xr", "x", "v", "age"},
}

# ball_tune.html 在解析一行 [SCV] 后生成的前端派生字段，不要求固件直接发送。
HTML_DERIVED_FIELDS = {"ff_nom", "ff_pv"}


def format_string(src: str, anchor: str) -> str | None:
    """把 printf 里连续的字符串字面量拼起来，直到以 \\r\\n 结尾的那一段。

    找不到锚点返回 None——遥测是单行还是拆成两行分频，取决于当前采用的
    控制方案，不是错误。回退到纯 S 曲线基线时就只有 [SCV] 一行。
    """
    if anchor not in src:
        return None
    index = src.index(anchor)
    parts, cursor = [], index
    while True:
        start = src.index('"', cursor)
        end = src.index('"', start + 1)
        literal = src[start + 1:end]
        parts.append(literal)
        cursor = end + 1
        if literal.endswith("\\r\\n"):
            break
    return "".join(parts)


def measure(fmt: str, period_ms: float) -> tuple[set[str], int, int, float]:
    fields = set(re.findall(r"(\w+)=%", fmt))
    floats = len(re.findall(r"%\.?\d*f", fmt))
    ints = len(re.findall(r"%l?[ud]", fmt))
    literal = len(re.sub(r"%[.0-9]*l?[fud]", "", fmt).replace("\\r\\n", ""))
    # 浮点按 6 字符、整数按 4 字符估计输出宽度
    length = literal + floats * 6 + ints * 4
    return fields, floats, length, length / (period_ms / 1000.0)


def html_fields(html: str) -> dict[str, set[str]]:
    """只取真正会去遥测里查的字段，避开参数表等同形结构。"""
    # 前面必须是 { 或 ,，否则 `kileak:"积分泄漏"` 这种也会被当成 `k:` 命中。
    plots = set(re.findall(r'[{,]\s*k:\s*"(\w+)"', html))
    plots |= set(re.findall(r'[{,]\s*key:\s*"(\w+)"', html))
    tiles = set(re.findall(r"last\.(\w+)", html))
    # DIAG 表的形状是 ["名字", "标签", "单位", 判据函数]
    diag_block = re.search(r"const DIAG = \[(.*?)\];", html, re.S)
    diag = set(re.findall(r'\["(\w+)"', diag_block.group(1))) if diag_block else set()
    return {"画图": plots, "瓦片": tiles, "诊断": diag}


def main() -> int:
    src = TASK.read_text(encoding="utf-8")
    html = TUNE_HTML.read_text(encoding="utf-8")

    # 周期取自任务里的 H3S_TELEMETRY_PERIOD_MS，别写死；正式 H3 会按串口预算调整。
    period = re.search(r"H3S_TELEMETRY_PERIOD_MS\s+(\d+)U", src)
    scv_period = float(period.group(1)) if period else 60.0

    scv_fmt = format_string(src, '"[SCV] t=%lu st=')
    scv, scv_f, scv_len, scv_rate = measure(scv_fmt, scv_period)
    scvd_fmt = format_string(src, '"[SCVD] t=%lu vr=')

    ok = True
    print(f"{'行':<8}{'字段':>5}{'%f':>5}{'行长':>7}{'周期':>8}{'B/s':>8}{'占UART':>8}")
    print("-" * 49)
    print(f"[SCV] {'':<2}{len(scv):>5}{scv_f:>5}{scv_len:>7}{scv_period:>6.0f}ms"
          f"{scv_rate:>8.0f}{scv_rate / UART_BPS:>8.0%}")
    total = scv_rate
    if scvd_fmt is None:
        scvd, scvd_f = set(), 0
        print(f"[SCVD]{'':<2}{'—':>5}{'—':>5}{'—':>7}{'—':>8}{'未拆分':>8}")
    else:
        scvd, scvd_f, scvd_len, scvd_rate = measure(scvd_fmt, 500.0)
        total += scvd_rate
        print(f"[SCVD]{'':<2}{len(scvd):>5}{scvd_f:>5}{scvd_len:>7}{500:>6}ms"
              f"{scvd_rate:>8.0f}{scvd_rate / UART_BPS:>8.0%}")
    print("-" * 49)
    print(f"{'合计':<8}{'':>17}{total:>16.0f}{total / UART_BPS:>8.0%}")

    budget = BUDGET_SPLIT if scvd_fmt is not None else BUDGET_SINGLE
    if total / UART_BPS > budget:
        print(f"\n★ 带宽超预算：{total / UART_BPS:.0%} > {budget:.0%}。"
              f"DebugUart 会丢字节、遥测随机截断。")
        ok = False

    overlap = (scv & scvd) - {"t"}
    if overlap:
        print(f"\n★ 两行重复发送：{sorted(overlap)}（浪费带宽）")
        ok = False

    # printf 格式化开销：Cortex-M0+ 无 FPU，每个 %f 约 1200~3000 周期
    lo = scv_f * 1200 / 32e6
    hi = scv_f * 3000 / 32e6
    print(f"\n[SCV] printf 开销 {lo * 1e6:.0f}~{hi * 1e6:.0f} us，"
          f"占 {CTRL_TICK_MS:.0f} ms 控制拍 {lo / (CTRL_TICK_MS / 1000):.0%}~"
          f"{hi / (CTRL_TICK_MS / 1000):.0%}")
    if hi / (CTRL_TICK_MS / 1000) > 0.25:
        print("★ 格式化开销过大，会推迟同一轮的 StepMotor_Tick")
        ok = False

    known = scv | scvd
    print()
    for label, used in html_fields(html).items():
        # 单行遥测模式仍应核对画图和瓦片；诊断条兼容历史 [SCVD] 字段，
        # 当前没有 [SCVD] 时只检查其中确实放进 [SCV] 的字段。
        if (scvd_fmt is None) and (label == "诊断"):
            used = used & known
        if used:
            missing = sorted(f for f in used
                             if (f not in known) and (f not in HTML_DERIVED_FIELDS))
            if missing:
                print(f"★ ball_tune.html {label}用到固件没有的字段：{missing}")
                ok = False
            else:
                print(f"  ball_tune.html {label}：{len(used)} 个字段全部存在 ✓")

    print()
    for name, need in ANALYSIS_NEEDS.items():
        lack = sorted(need - scv)
        if lack:
            print(f"★ {name} 需要的 {lack} 不在高频行 [SCV] 里")
            ok = False
        else:
            print(f"  {name:<20}依赖字段全在 [SCV] ✓")

    print("\n" + ("遥测字段与带宽检查通过。" if ok else "检查未通过。"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
