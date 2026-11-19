"""Authoritative metadata for the MSPM0 track-tuning parameters."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ParameterSpec:
    param_id: int
    key: str
    label: str
    priority: int
    default: int
    recommended_min: int | None
    recommended_max: int | None
    unit: str
    purpose: str

    @property
    def recommended_text(self) -> str:
        if self.recommended_min is None or self.recommended_max is None:
            return "现场确定"
        return f"{self.recommended_min}～{self.recommended_max}"


PARAMETERS = (
    ParameterSpec(0x0101, "s1_end_mm", "S1 结束里程", 1, 1500, 1300, 1700, "mm", "S1→S2 的切换里程。"),
    ParameterSpec(0x0102, "s2_end_mm", "S2 结束里程", 1, 3071, 2750, 3400, "mm", "S2→S3 的切换里程。"),
    ParameterSpec(0x0103, "s3_end_mm", "S3 结束里程", 1, 4641, 4200, 5000, "mm", "S3→S4 的切换里程。"),
    ParameterSpec(0x0104, "s4_heading_end_mm", "S4 航向结束里程", 1, 6212, 5600, 6800, "mm", "S4 航向参考完成 360° 的绝对里程。"),
    ParameterSpec(0x0105, "s3_gyro_recover_mm", "S3 陀螺仪回正距离", 1, 300, 0, 600, "mm", "S2 结束后使用陀螺仪直行回正的距离。"),
    ParameterSpec(0x0110, "lap_stop_mm", "A 点停车参考里程", 2, 6242, None, None, "mm", "A 点减速和横线识别使用的绝对里程参考。"),
    ParameterSpec(0x0111, "finish_arm_margin_mm", "横线武装提前量", 2, 400, None, None, "mm", "横线识别从 lap_stop_mm 减去本值的位置开始武装。"),
    ParameterSpec(0x0112, "loaded_decel_warning_mm", "载球减速提前量", 2, 250, None, None, "mm", "H5/H6 从 lap_stop_mm 减去本值的位置开始减速。"),
    ParameterSpec(0x0113, "loaded_odom_arrival_mm", "载球编码器到达里程", 2, 6342, None, None, "mm", "H5/H6 判断到达 A 点的编码器绝对里程。"),
    ParameterSpec(0x0114, "h2_odom_fallback_mm", "H2 兜底停车里程", 2, 6442, None, None, "mm", "H2 横线漏检时使用的编码器兜底停车绝对里程。"),
    ParameterSpec(0x0120, "h2_s2_ff_x1000", "H2 第一弯前馈", 3, 1100, 850, 1300, "×0.001", "H2 第一弯的前馈倍率；1100 表示 1.100 倍。"),
    ParameterSpec(0x0121, "loaded_s2_ff_x1000", "H5/H6 第一弯前馈", 3, 1000, 850, 1300, "×0.001", "H5/H6 第一弯的前馈倍率；1000 表示 1.000 倍。"),
    ParameterSpec(0x0122, "s4_ff_x1000", "第二弯前馈", 3, 1000, 850, 1300, "×0.001", "H2/H5/H6 第二弯的前馈倍率；1000 表示 1.000 倍。"),
)

PARAMETER_BY_ID = {spec.param_id: spec for spec in PARAMETERS}
PARAMETER_BY_KEY = {spec.key: spec for spec in PARAMETERS}

if len(PARAMETER_BY_ID) != len(PARAMETERS) or len(PARAMETER_BY_KEY) != len(PARAMETERS):
    raise RuntimeError("Track-tuning parameter IDs and keys must be unique")
