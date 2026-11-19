"""树莓派钢球识别使用的 OpenCV 鱼眼去畸变层。"""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path

import cv2
import numpy as np


DEFAULT_CALIBRATION_PATH = (
    Path(__file__).resolve().parent / "calibration" / "fisheye_1280x720.json"
)


@dataclass(frozen=True)
class FisheyeCalibration:
    camera_matrix: np.ndarray
    distortion: np.ndarray
    image_size: tuple[int, int]
    output_matrix_crop: np.ndarray
    output_matrix_full: np.ndarray
    rms_px: float


def _matrix(payload: object, name: str) -> np.ndarray:
    matrix = np.asarray(payload, dtype=np.float64)
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise ValueError(f"{name} 必须是有限的 3x3 矩阵")
    return matrix


def load_fisheye_calibration(
    path: Path = DEFAULT_CALIBRATION_PATH,
) -> FisheyeCalibration:
    """加载本摄像头的 OpenCV fisheye 标定参数。"""
    path = Path(path)
    payload = json.loads(path.read_text(encoding="utf-8"))
    if payload.get("model") != "opencv_fisheye":
        raise ValueError(f"不支持的畸变模型：{payload.get('model')!r}")

    size = payload["image_size"]
    image_size = int(size["width"]), int(size["height"])
    if image_size[0] <= 0 or image_size[1] <= 0:
        raise ValueError("标定图像尺寸必须为正数")

    distortion = np.asarray(
        payload["distortion_coefficients"], dtype=np.float64
    ).reshape(-1)
    if distortion.shape != (4,) or not np.all(np.isfinite(distortion)):
        raise ValueError("OpenCV fisheye 畸变参数必须包含四个有限数值")

    output = payload["output_camera_matrices"]
    return FisheyeCalibration(
        camera_matrix=_matrix(payload["camera_matrix"], "camera_matrix"),
        distortion=distortion.reshape(4, 1),
        image_size=image_size,
        output_matrix_crop=_matrix(output["balance_0"], "balance_0"),
        output_matrix_full=_matrix(output["balance_1"], "balance_1"),
        rms_px=float(payload["rms_px"]),
    )


class FisheyeUndistorter:
    """缓存 remap 映射，为钢球识别持续输出最新的去畸变帧。

    映射只在首次遇到某个分辨率时生成。后续每帧只执行 ``cv2.remap``，
    不在识别循环里重复计算相机矩阵。
    """

    def __init__(
        self,
        calibration: FisheyeCalibration | None = None,
        *,
        balance: float = 0.0,
        zoom: float = 1.0,
        center_offset_px: tuple[float, float] = (0.0, 0.0),
    ) -> None:
        if not 0.0 <= balance <= 1.0:
            raise ValueError("balance 必须位于 0～1")
        if zoom <= 0.0:
            raise ValueError("zoom 必须大于 0")
        if len(center_offset_px) != 2 or not all(
            np.isfinite(value) for value in center_offset_px
        ):
            raise ValueError("center_offset_px 必须包含两个有限数值")

        self.calibration = calibration or load_fisheye_calibration()
        self.balance = float(balance)
        self.zoom = float(zoom)
        self.center_offset_px = tuple(float(value) for value in center_offset_px)
        self._map_cache: dict[
            tuple[int, int], tuple[np.ndarray, np.ndarray, np.ndarray]
        ] = {}
        self._roi_map_cache: dict[
            tuple[tuple[int, int], tuple[int, int, int, int]],
            tuple[np.ndarray, np.ndarray, np.ndarray],
        ] = {}
        self._region_map_cache: dict[
            tuple[
                tuple[int, int],
                tuple[float, float],
                tuple[int, int],
            ],
            tuple[np.ndarray, np.ndarray, np.ndarray],
        ] = {}

    def _matrices(self, image_size: tuple[int, int]) -> tuple[np.ndarray, np.ndarray]:
        width, height = image_size
        if width <= 0 or height <= 0:
            raise ValueError("图像尺寸必须为正数")

        source_width, source_height = self.calibration.image_size
        scale_x = width / source_width
        scale_y = height / source_height
        if abs(scale_x - scale_y) > 1e-6:
            raise ValueError(
                f"输入宽高比必须与标定分辨率 {source_width}x{source_height} 一致"
            )

        source = self.calibration.camera_matrix.copy()
        source[0, 0] *= scale_x
        source[1, 1] *= scale_y
        source[0, 2] = (
            source[0, 2] + self.center_offset_px[0]
        ) * scale_x
        source[1, 2] = (
            source[1, 2] + self.center_offset_px[1]
        ) * scale_y

        output = (
            self.calibration.output_matrix_crop * (1.0 - self.balance)
            + self.calibration.output_matrix_full * self.balance
        )
        output[0, 0] *= scale_x * self.zoom
        output[1, 1] *= scale_y * self.zoom
        output[0, 2] *= scale_x
        output[1, 2] *= scale_y
        output[2, 2] = 1.0
        return source, output

    def maps(
        self,
        image_size: tuple[int, int],
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """返回固定点 remap 映射和去畸变后的相机矩阵。"""
        cached = self._map_cache.get(image_size)
        if cached is not None:
            return cached

        source, output = self._matrices(image_size)
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(
            source,
            self.calibration.distortion,
            np.eye(3),
            output,
            image_size,
            cv2.CV_16SC2,
        )
        cached = map1, map2, output
        self._map_cache[image_size] = cached
        return cached

    @staticmethod
    def _validate_roi(
        image_size: tuple[int, int],
        roi_xyxy: tuple[int, int, int, int],
    ) -> tuple[int, int, int, int]:
        if len(roi_xyxy) != 4:
            raise ValueError("ROI 必须是 (left, top, right, bottom)")
        left, top, right, bottom = (int(value) for value in roi_xyxy)
        width, height = image_size
        if not (0 <= left < right <= width and 0 <= top < bottom <= height):
            raise ValueError(f"ROI {(left, top, right, bottom)} 超出图像范围 {image_size}")
        return left, top, right, bottom

    def roi_maps(
        self,
        image_size: tuple[int, int],
        roi_xyxy: tuple[int, int, int, int],
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """返回校正后坐标系中指定 ROI 的固定点 remap 映射。

        输出与先调用 :meth:`undistort` 再裁剪同一 ROI 等价，但映射表和
        ``cv2.remap`` 输出都只有 ROI 大小。返回的相机矩阵以 ROI 左上角为
        新坐标原点；需要恢复全图坐标时，再加 ``(left, top)`` 偏移。
        """
        roi = self._validate_roi(image_size, roi_xyxy)
        key = image_size, roi
        cached = self._roi_map_cache.get(key)
        if cached is not None:
            return cached

        left, top, right, bottom = roi
        cached = self.rectified_region_maps(
            image_size,
            (float(left), float(top)),
            (right - left, bottom - top),
        )
        self._roi_map_cache[key] = cached
        return cached

    def rectified_region_maps(
        self,
        image_size: tuple[int, int],
        origin_xy: tuple[float, float],
        output_size: tuple[int, int],
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """生成独立长条形局部去畸变画布的映射。

        ``origin_xy`` 是该局部画布左上角在无限延伸的全图校正平面中的
        坐标。它不要求位于名义全图尺寸内，因此可以保留原本落在 1280x720
        输出画布之外的水管边沿，而无需先生成完整去畸变帧。
        """
        if len(origin_xy) != 2 or not all(np.isfinite(value) for value in origin_xy):
            raise ValueError("origin_xy 必须包含两个有限数值")
        if len(output_size) != 2:
            raise ValueError("output_size 必须是 (width, height)")
        width, height = (int(value) for value in output_size)
        if width <= 0 or height <= 0:
            raise ValueError("局部去畸变输出尺寸必须为正数")

        origin = float(origin_xy[0]), float(origin_xy[1])
        size = width, height
        key = image_size, origin, size
        cached = self._region_map_cache.get(key)
        if cached is not None:
            return cached

        source, output = self._matrices(image_size)
        region_output = output.copy()
        region_output[0, 2] -= origin[0]
        region_output[1, 2] -= origin[1]
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(
            source,
            self.calibration.distortion,
            np.eye(3),
            region_output,
            size,
            cv2.CV_16SC2,
        )
        cached = map1, map2, region_output
        self._region_map_cache[key] = cached
        return cached

    def undistort(self, frame: np.ndarray) -> np.ndarray:
        """去畸变一帧 BGR/灰度图，供后续钢球检测使用。"""
        if frame is None or frame.size == 0 or frame.ndim not in (2, 3):
            raise ValueError("输入帧为空或维度无效")
        height, width = frame.shape[:2]
        map1, map2, _ = self.maps((width, height))
        return cv2.remap(
            frame,
            map1,
            map2,
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
        )

    def undistort_roi(
        self,
        frame: np.ndarray,
        roi_xyxy: tuple[int, int, int, int],
    ) -> np.ndarray:
        """从原始帧直接生成校正后坐标系中的指定 ROI。"""
        if frame is None or frame.size == 0 or frame.ndim not in (2, 3):
            raise ValueError("输入帧为空或维度无效")
        height, width = frame.shape[:2]
        map1, map2, _ = self.roi_maps((width, height), roi_xyxy)
        return cv2.remap(
            frame,
            map1,
            map2,
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
        )

    def undistort_rectified_region(
        self,
        frame: np.ndarray,
        origin_xy: tuple[float, float],
        output_size: tuple[int, int],
    ) -> np.ndarray:
        """从原始帧直接生成独立坐标系中的局部去畸变图。"""
        if frame is None or frame.size == 0 or frame.ndim not in (2, 3):
            raise ValueError("输入帧为空或维度无效")
        height, width = frame.shape[:2]
        map1, map2, _ = self.rectified_region_maps(
            (width, height), origin_xy, output_size
        )
        return cv2.remap(
            frame,
            map1,
            map2,
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
        )

    def undistort_points(
        self,
        points: np.ndarray,
        image_size: tuple[int, int],
    ) -> np.ndarray:
        """仅校正点坐标；后续需要降低整帧 remap 开销时可使用。"""
        values = np.asarray(points, dtype=np.float64)
        if values.size == 0 or values.shape[-1] != 2:
            raise ValueError("points 的末维必须为 2 且不能为空")
        original_shape = values.shape
        source, output = self._matrices(image_size)
        corrected = cv2.fisheye.undistortPoints(
            values.reshape(-1, 1, 2),
            source,
            self.calibration.distortion,
            R=np.eye(3),
            P=output,
        )
        return corrected.reshape(original_shape)
