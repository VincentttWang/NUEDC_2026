"""Shared preprocessing for the production steel-ball detector."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path

import cv2
import numpy as np

from vision_function.fisheye import FisheyeUndistorter


@dataclass(frozen=True)
class TubeRectificationConfig:
    """A tube-local crop on the rectified plane, before model letterboxing."""

    reference_size: tuple[int, int]
    decode_scale: int
    balance: float
    reference_origin_xy: tuple[int, int]
    reference_output_size: tuple[int, int]
    coordinate_space: str

    @property
    def input_size(self) -> tuple[int, int]:
        return tuple(value // self.decode_scale for value in self.reference_size)

    @property
    def origin_xy(self) -> tuple[int, int]:
        return tuple(value // self.decode_scale for value in self.reference_origin_xy)

    @property
    def output_size(self) -> tuple[int, int]:
        return tuple(value // self.decode_scale for value in self.reference_output_size)

    def region_for_scale(
        self, decode_scale: int
    ) -> tuple[tuple[int, int], tuple[int, int]]:
        values = (*self.reference_origin_xy, *self.reference_output_size)
        if decode_scale <= 0 or any(value % decode_scale for value in values):
            raise ValueError(
                f"局部校正区域 {values} 不能被 decode_scale={decode_scale} 精确缩放"
            )
        origin = tuple(value // decode_scale for value in self.reference_origin_xy)
        size = tuple(value // decode_scale for value in self.reference_output_size)
        return origin, size


def _positive_pair(payload: object, name: str) -> tuple[int, int]:
    if not isinstance(payload, (list, tuple)) or len(payload) != 2:
        raise ValueError(f"{name} 必须包含两个整数")
    pair = int(payload[0]), int(payload[1])
    if any(value <= 0 for value in pair):
        raise ValueError(f"{name} 必须为正数")
    return pair


def load_tube_rectification(path: Path) -> TubeRectificationConfig:
    """Load the new tube-local config, with legacy ROI compatibility."""
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    reference = payload["reference_image_size"]
    reference_size = int(reference["width"]), int(reference["height"])
    decode_scale = int(payload["production_decode_scale"])
    if decode_scale <= 0 or any(value % decode_scale for value in reference_size):
        raise ValueError("production_decode_scale 与参考图像尺寸不兼容")

    balance = float(payload.get("fisheye_balance", 0.0))
    if not 0.0 <= balance <= 1.0:
        raise ValueError("fisheye_balance 必须位于 0～1")

    region = payload.get("reference_rectified_region")
    if region is None:
        region = payload.get("reference_roi", payload.get("roi"))
    if region is None:
        raise ValueError("配置缺少 reference_rectified_region 或 reference_roi")

    left, top = int(region["left"]), int(region["top"])
    if "width" in region and "height" in region:
        output_size = int(region["width"]), int(region["height"])
    else:
        output_size = int(region["right"]) - left, int(region["bottom"]) - top
    _positive_pair(output_size, "reference output size")
    values = (left, top, *output_size)
    if any(value % decode_scale for value in values):
        raise ValueError("局部区域参数必须能被 production_decode_scale 精确缩放")

    return TubeRectificationConfig(
        reference_size=reference_size,
        decode_scale=decode_scale,
        balance=balance,
        reference_origin_xy=(left, top),
        reference_output_size=output_size,
        coordinate_space=str(payload.get("coordinate_space", "legacy_rectified_roi")),
    )


class TubePreprocessor:
    """Resize/decode-normalize and directly remap the tube-local rectified image."""

    def __init__(self, config: TubeRectificationConfig) -> None:
        self.config = config
        self.undistorter = FisheyeUndistorter(balance=config.balance)

    def __call__(self, frame: np.ndarray) -> np.ndarray:
        if frame is None or frame.size == 0:
            raise ValueError("Input frame is empty")
        size = frame.shape[1], frame.shape[0]
        if size == self.config.reference_size:
            frame = cv2.resize(
                frame, self.config.input_size, interpolation=cv2.INTER_AREA
            )
        elif size != self.config.input_size:
            raise ValueError(
                f"Input must be {self.config.reference_size} or "
                f"{self.config.input_size}, got {size}"
            )
        return self.undistorter.undistort_rectified_region(
            frame,
            self.config.origin_xy,
            self.config.output_size,
        )
