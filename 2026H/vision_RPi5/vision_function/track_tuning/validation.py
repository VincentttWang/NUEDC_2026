"""Safety and consistency checks for track-tuning values."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping

from .parameters import PARAMETERS


@dataclass(frozen=True)
class ValidationIssue:
    severity: str
    message: str


def validate_values(values: Mapping[str, int]) -> list[ValidationIssue]:
    issues: list[ValidationIssue] = []
    missing = [spec.key for spec in PARAMETERS if spec.key not in values]
    if missing:
        return [ValidationIssue("error", "缺少参数：" + "、".join(missing))]

    normalized: dict[str, int] = {}
    for spec in PARAMETERS:
        try:
            value = int(values[spec.key])
        except (TypeError, ValueError):
            issues.append(ValidationIssue("error", f"{spec.label} 必须是整数"))
            continue
        normalized[spec.key] = value
        if not 0 <= value <= 0xFFFF:
            issues.append(ValidationIssue("error", f"{spec.label} 超出协议范围 0..65535"))
        if (
            spec.recommended_min is not None
            and spec.recommended_max is not None
            and not spec.recommended_min <= value <= spec.recommended_max
        ):
            issues.append(
                ValidationIssue(
                    "warning",
                    f"{spec.label}={value} 超出建议范围 {spec.recommended_text}",
                )
            )

    if len(normalized) != len(PARAMETERS):
        return issues

    s1 = normalized["s1_end_mm"]
    s2 = normalized["s2_end_mm"]
    s3 = normalized["s3_end_mm"]
    s4 = normalized["s4_heading_end_mm"]
    if not s1 < s2 < s3 < s4:
        issues.append(ValidationIssue("error", "分段里程必须满足 S1 < S2 < S3 < S4"))

    if s2 + normalized["s3_gyro_recover_mm"] >= s3:
        issues.append(
            ValidationIssue(
                "warning",
                "S2 结束里程 + 回正距离应小于 S3 结束里程，否则回正窗口会覆盖下一弯",
            )
        )

    lap_stop = normalized["lap_stop_mm"]
    if normalized["finish_arm_margin_mm"] > lap_stop:
        issues.append(ValidationIssue("error", "横线武装提前量不得大于 A 点停车参考里程"))
    if normalized["loaded_decel_warning_mm"] > lap_stop:
        issues.append(ValidationIssue("error", "载球减速提前量不得大于 A 点停车参考里程"))

    if s4 > lap_stop:
        issues.append(ValidationIssue("warning", "建议 S4 航向结束里程不大于 A 点停车参考里程"))
    if lap_stop > normalized["loaded_odom_arrival_mm"]:
        issues.append(ValidationIssue("warning", "建议 A 点停车参考里程不大于载球编码器到达里程"))
    if lap_stop > normalized["h2_odom_fallback_mm"]:
        issues.append(ValidationIssue("warning", "建议 A 点停车参考里程不大于 H2 兜底停车里程"))
    return issues
