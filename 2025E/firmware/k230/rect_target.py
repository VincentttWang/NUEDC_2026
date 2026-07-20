# ------------------------------------------------------------------------
# rect_target.py  —  2025E 靶标矩形高帧率识别 (K230 / CanMV)
#
# 设计要点 (基于实景调参):
#   * 320x240 灰度 + cv_lite.grayscale_find_rectangles_with_corners (C 层 Canny+
#     approxPolyDP+角点) 全帧检测, 再按 ROI 在 Python 侧过滤; 比 find_rects 更快更稳。
#   * 候选通过完整可见、四边形几何、内部亮度和黑边对比验证后按质量评分。
#   * 搜索阶段连续多帧确认, 锁定阶段拒绝异常跳变, 防止背景矩形瞬时误锁。
#   * 锁定后仅在局部 ROI 内搜索, 大幅提帧率; 丢失走 coasting -> 扩 ROI -> 回全屏。
#   * 靶心 = 矩形对角线交点; 通过 UART1(pin3 TX/pin4 RX) 发 0x12/0x5B 27 字节帧,
#     与 MSPM0 bsp/canmv 解析器一致; GPIO2 控激光。
#   * DISPLAY_MODE 可选 none/ide/lcd, 默认 none 以获得最高帧率。
# ------------------------------------------------------------------------
import time, os, gc, math
import image
import cv_lite                 # C 加速矩形检测 (Canny+approxPolyDP+角点), 替代 find_rects
from media.sensor import *
from media.display import *
from media.media import *
from machine import Pin, FPIOA, UART

# Optional semantic detector. The traditional CV path remains fully functional
# when the custom model is absent, so an incomplete AI deployment cannot brick
# target acquisition.
AI_MODEL_PATH = "/sdcard/target_yolov8n_224.kmodel"
AI_MODEL_INPUT_SIZE = [224, 224]
AI_CONFIDENCE_THRESHOLD = 0.35
AI_NMS_THRESHOLD = 0.45
AI_RUN_INTERVAL = 4
AI_ROI_HOLD_FRAMES = 8
AI_ROI_MARGIN = 18
AI_FULL_CV_FALLBACK_INTERVAL = 5
AI_FIND_RECTS_THRESHOLD = 3500
AI_FALLBACK_CONFIDENCE = 0.55
AI_FALLBACK_CONFIRMATIONS = 3
AI_FALLBACK_MAX_SHIFT_PX = 40

try:
    os.stat(AI_MODEL_PATH)
    AI_MODEL_AVAILABLE = True
except Exception:
    AI_MODEL_AVAILABLE = False

if AI_MODEL_AVAILABLE:
    import aidemo
    import nncase_runtime as nn
    import ulab.numpy as np
    from libs.AI2D import Ai2d
    from libs.AIBase import AIBase
    from libs.Utils import letterbox_pad_param

    class TargetDetector(AIBase):
        def __init__(self):
            super().__init__(AI_MODEL_PATH, AI_MODEL_INPUT_SIZE, AI_MODEL_INPUT_SIZE, 0)
            self.ai2d = Ai2d(0)
            self.ai2d.set_ai2d_dtype(
                nn.ai2d_format.NCHW_FMT,
                nn.ai2d_format.NCHW_FMT,
                np.uint8,
                np.uint8,
            )

        def config_preprocess(self):
            top, bottom, left, right, _ = letterbox_pad_param(
                AI_MODEL_INPUT_SIZE, AI_MODEL_INPUT_SIZE)
            self.ai2d.pad([0, 0, 0, 0, top, bottom, left, right], 0, [128, 128, 128])
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            self.ai2d.build([1, 3, 224, 224], [1, 3, 224, 224])

        def preprocess(self, input_np):
            return [nn.from_numpy(input_np)]

        def postprocess(self, results):
            output = results[0][0].transpose()
            return aidemo.yolov8_det_postprocess(
                output.copy(), [224, 224], [224, 224],
                [DETECT_HEIGHT, DETECT_WIDTH], 1,
                AI_CONFIDENCE_THRESHOLD, AI_NMS_THRESHOLD, 4)

# ===== 显示后端: "none" (最快) / "ide" (CanMV IDE) / "lcd" (ST7701) =====
DISPLAY_MODE = "lcd"
DISPLAY_INTERVAL = 3
LOG_INTERVAL = 10
GC_INTERVAL = 30

# ===== 图像尺寸 =====
DETECT_WIDTH = ALIGN_UP(320, 16)
DETECT_HEIGHT = 240
IMG_CENTER_X = DETECT_WIDTH // 2
IMG_CENTER_Y = DETECT_HEIGHT // 2
LCD_WIDTH, LCD_HEIGHT = 800, 480

# ===== 相机 FOV (估计值, 用于把像素偏差换算为角度, 供参考/调试) =====
H_FOV_DEG, V_FOV_DEG = 60.0, 35.0
H_TAN_HALF_FOV = math.tan(math.radians(H_FOV_DEG / 2.0))
V_TAN_HALF_FOV = math.tan(math.radians(V_FOV_DEG / 2.0))

# ===== 矩形识别与筛选 (实景调参) =====
# 实测: 靶 area~16500 / asp~1.3 / mag~105k; 背景噪声候选 area 全部 <700。
# 面积是最强判据 (靶比噪声大 20 倍以上), 故只用 面积+长宽比 + 取最大面积。
# 阈值取 10000: 靶仍轻松通过, 弱边缘/轻遮挡时更易闭合矩形。
# 注: stdev 内部方差不作判据 —— 实测靶 stdev(~30) 反而低于部分噪声(47/51), 会误杀。
# 注: 曝光/增益锁定在本固件(GC2093)不支持 (set_auto_exposure 抛异常), 暂不使用。
FIND_RECTS_THRESHOLD = 10000   # (保留常量, cv_lite 路径不再使用; 参见 CVLITE_* 参数)
MIN_ASPECT_RATIO = 0.5
MAX_ASPECT_RATIO = 2.0
MIN_AREA = 2000                # 放低以容纳更远(更小)的靶; 仍 3x 于观测到的最大噪声
MAX_AREA = 60000
IMAGE_BORDER_MARGIN = 3
MIN_EDGE_LENGTH = 16
MIN_QUAD_FILL_RATIO = 0.50
MAX_SIDE_LENGTH_RATIO = 4.0
MIN_INTERIOR_MEAN = 80
MIN_EDGE_CONTRAST = 24
MAX_EDGE_DARK_LEVEL = 165
MIN_VALID_DARK_EDGES = 3
MIN_RECT_MAGNITUDE = 50000
# Relaxed geometry is only enabled inside a recent, temporally confirmed AI ROI.
AI_MIN_AREA = 800
AI_MIN_EDGE_LENGTH = 10
AI_MIN_QUAD_FILL_RATIO = 0.40
AI_MAX_SIDE_LENGTH_RATIO = 5.0
AI_MIN_INTERIOR_MEAN = 55
AI_MIN_EDGE_CONTRAST = 12
AI_MAX_EDGE_DARK_LEVEL = 205
AI_MIN_VALID_DARK_EDGES = 2
AI_MIN_RECT_MAGNITUDE = 16000
SEARCH_CONFIRM_FRAMES = 3
SEARCH_CONFIRM_MAX_MISSES = 4
CONFIRM_MAX_SHIFT_PX = 24
LOCK_MAX_SHIFT_PX = 50
NESTED_MIN_AREA_RATIO = 0.45
NESTED_MAX_AREA_RATIO = 0.90
NESTED_CENTER_RATIO = 0.12

# ===== cv_lite 矩形检测参数 (C 层预筛, 精细判据仍交给 validate_target) =====
# grayscale_find_rectangles_with_corners(shape[h,w], img_np, canny_lo, canny_hi,
#   approx_eps, area_min_ratio, max_angle_cos, gauss) -> [[x,y,w,h, c1x..c4y], ...]
# 这里参数取偏宽松, 让远/弱边框也能进候选; MIN_AREA/几何/亮度由下游精筛。
CVLITE_IMAGE_SHAPE = [DETECT_HEIGHT, DETECT_WIDTH]  # [高, 宽], 需与 to_numpy_ref 一致
CVLITE_CANNY_LO = 50
CVLITE_CANNY_HI = 150
CVLITE_APPROX_EPS = 0.04       # 多边形逼近精度比例 (越小越精确)
CVLITE_AREA_MIN_RATIO = 0.002  # 最小面积占比 (~150px @320x240), 下游再按 MIN_AREA 精筛
CVLITE_MAX_ANGLE_COS = 0.3     # 邻边夹角余弦上限 (越小越接近直角)
CVLITE_GAUSS = 5               # 高斯核 (奇数)

# ===== 局部 ROI 跟踪 =====
ROI_MARGIN = 30            # 锁定后 ROI 相对靶框四周外扩
MAX_COASTING_FRAMES = 3    # 短暂丢失维持帧数
LOCK_HOLD_MISS_FRAMES = 5  # 已锁定时容忍 find_rects 的连续漏检帧数
ROI_EXPAND_MARGIN = 60     # 重捕时每步扩张量
MAX_ROI_EXPAND_STEPS = 2   # 扩张次数上限, 超限退回全屏

# ===== UART (对齐 uart1_comm_test.py: UART1, pin3 TX / pin4 RX) =====
K230_UART = UART.UART1
K230_UART_TX_PIN = 3
K230_UART_RX_PIN = 4
K230_UART_BAUDRATE = 115200
FRAME_START = 0x12
FRAME_END = 0x5B

# ===== 激光: 误差足够小才点亮 (当前以相机中心近似激光点) =====
LASER_PIN = 2
LASER_ENABLE_ERROR_PX = 8
CAMERA_ROTATED_180 = True

# ===== 状态机 =====
STATE_SEARCHING, STATE_LOCKED, STATE_COASTING = 0, 1, 2

sensor = None
uart = None
laser_pin = None
display_img = None
ai_detector = None


def uart_init():
    global uart
    fpioa = FPIOA()
    fpioa.set_function(K230_UART_TX_PIN, FPIOA.UART1_TXD)
    fpioa.set_function(K230_UART_RX_PIN, FPIOA.UART1_RXD)
    uart = UART(K230_UART, baudrate=K230_UART_BAUDRATE,
                bits=UART.EIGHTBITS, parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)


def laser_init():
    global laser_pin
    fpioa = FPIOA()
    fpioa.set_function(LASER_PIN, FPIOA.GPIO2)
    laser_pin = Pin(LASER_PIN, Pin.OUT, pull=Pin.PULL_NONE, drive=7)
    laser_pin.value(0)


def set_laser(on):
    if laser_pin:
        laser_pin.value(1 if on else 0)


def append_u16_be(frame, value):
    value = int(value)
    if value < 0:
        value = 0
    elif value > 65535:
        value = 65535
    frame.append((value >> 8) & 0xFF)
    frame.append(value & 0xFF)


def normalize_rect_corners(corners):
    if not corners or len(corners) < 4:
        return [(0, 0), (0, 0), (0, 0), (0, 0)]
    by_y = sorted(corners, key=lambda p: p[1])
    top = sorted(by_y[:2], key=lambda p: p[0])
    bottom = sorted(by_y[2:], key=lambda p: p[0])
    return [bottom[0], bottom[1], top[1], top[0]]  # BL, BR, TR, TL


def mirror_point(point):
    if not CAMERA_ROTATED_180:
        return int(point[0]), int(point[1])
    x = (2 * IMG_CENTER_X) - int(point[0])
    y = (2 * IMG_CENTER_Y) - int(point[1])
    return (
        max(0, min(DETECT_WIDTH - 1, x)),
        max(0, min(DETECT_HEIGHT - 1, y)),
    )


_tx_seq = 0


def send_vision_frame(target_x=0, target_y=0, laser_x=0, laser_y=0, corners=None):
    # byte1 = 滚动帧序号 (0..255)。MSPM0 bsp/canmv 解析器忽略该字节, 兼容无损;
    # 供上位机/MSPM0 后续做延迟感知融合、丢帧检测使用。
    global _tx_seq
    if uart is None:
        return
    frame = bytearray()
    frame.append(FRAME_START)
    frame.append(_tx_seq & 0xFF)
    _tx_seq += 1
    append_u16_be(frame, target_x)
    append_u16_be(frame, target_y)
    append_u16_be(frame, laser_x)
    append_u16_be(frame, laser_y)
    for x, y in normalize_rect_corners(corners):
        append_u16_be(frame, x)
        append_u16_be(frame, y)
    frame.append(FRAME_END)
    uart.write(frame)


def get_target_center(corners):
    # 对角线交点法求四边形中心
    x1, y1 = corners[0]; x2, y2 = corners[2]
    x3, y3 = corners[1]; x4, y4 = corners[3]
    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(denom) < 0.0001:
        return (x1 + x2 + x3 + x4) // 4, (y1 + y2 + y3 + y4) // 4
    ix = ((x1*y2 - y1*x2)*(x3 - x4) - (x1 - x2)*(x3*y4 - y3*x4)) // denom
    iy = ((x1*y2 - y1*x2)*(y3 - y4) - (y1 - y2)*(x3*y4 - y3*x4)) // denom
    return int(ix), int(iy)


def distance_sq(p1, p2):
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
    return dx * dx + dy * dy


def polygon_area(corners):
    twice_area = 0
    for i in range(4):
        x1, y1 = corners[i]
        x2, y2 = corners[(i + 1) & 3]
        twice_area += x1 * y2 - y1 * x2
    return abs(twice_area) * 0.5


def is_convex_quad(corners):
    sign = 0
    for i in range(4):
        x1, y1 = corners[i]
        x2, y2 = corners[(i + 1) & 3]
        x3, y3 = corners[(i + 2) & 3]
        cross = (x2 - x1) * (y3 - y2) - (y2 - y1) * (x3 - x2)
        if abs(cross) < 4:
            return False
        current = 1 if cross > 0 else -1
        if sign and current != sign:
            return False
        sign = current
    return True


def pixel_value(img, x, y):
    x = max(0, min(DETECT_WIDTH - 1, int(x)))
    y = max(0, min(DETECT_HEIGHT - 1, int(y)))
    value = img.get_pixel(x, y)
    return value if isinstance(value, int) else value[0]


def interior_mean(img, corners, center):
    total = pixel_value(img, center[0], center[1])
    count = 1
    for corner in corners:
        # Stay well inside the black tape while still sampling the paper face.
        x = center[0] + (corner[0] - center[0]) * 0.45
        y = center[1] + (corner[1] - center[1]) * 0.45
        total += pixel_value(img, x, y)
        count += 1
    return total / count


def edge_signature(img, p1, p2, center, sample_radius):
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    length = math.sqrt(dx * dx + dy * dy)
    if length < 1.0:
        return 0, 255

    nx = -dy / length
    ny = dx / length
    mx = (p1[0] + p2[0]) * 0.5
    my = (p1[1] + p2[1]) * 0.5
    if nx * (center[0] - mx) + ny * (center[1] - my) < 0:
        nx = -nx
        ny = -ny

    contrast_sum = 0
    dark_sum = 0
    samples = 0
    half_radius = max(1, sample_radius // 2)
    offsets = (-sample_radius, -half_radius, 0, half_radius, sample_radius)
    for fraction in (0.33, 0.67):
        bx = p1[0] + dx * fraction
        by = p1[1] + dy * fraction
        low = 255
        high = 0
        for offset in offsets:
            value = pixel_value(img, bx + nx * offset, by + ny * offset)
            low = min(low, value)
            high = max(high, value)
        contrast_sum += high - low
        dark_sum += low
        samples += 1
    return contrast_sum / samples, dark_sum / samples


def validate_target(img, r, ai_guided=False):
    min_area = AI_MIN_AREA if ai_guided else MIN_AREA
    min_edge_length = AI_MIN_EDGE_LENGTH if ai_guided else MIN_EDGE_LENGTH
    min_quad_fill = AI_MIN_QUAD_FILL_RATIO if ai_guided else MIN_QUAD_FILL_RATIO
    max_side_ratio = AI_MAX_SIDE_LENGTH_RATIO if ai_guided else MAX_SIDE_LENGTH_RATIO
    min_interior = AI_MIN_INTERIOR_MEAN if ai_guided else MIN_INTERIOR_MEAN
    min_contrast = AI_MIN_EDGE_CONTRAST if ai_guided else MIN_EDGE_CONTRAST
    max_dark = AI_MAX_EDGE_DARK_LEVEL if ai_guided else MAX_EDGE_DARK_LEVEL
    min_dark_edges = AI_MIN_VALID_DARK_EDGES if ai_guided else MIN_VALID_DARK_EDGES
    min_magnitude = AI_MIN_RECT_MAGNITUDE if ai_guided else MIN_RECT_MAGNITUDE
    w, h = r.w(), r.h()
    if w <= 0 or h <= 0:
        return None
    aspect = float(w) / h
    area = w * h
    if not (MIN_ASPECT_RATIO <= aspect <= MAX_ASPECT_RATIO):
        return None
    if not (min_area <= area <= MAX_AREA):
        return None
    if r.magnitude() < min_magnitude:
        return None

    rx, ry, rw, rh = r.rect()
    if (rx <= IMAGE_BORDER_MARGIN or ry <= IMAGE_BORDER_MARGIN or
            rx + rw >= DETECT_WIDTH - IMAGE_BORDER_MARGIN or
            ry + rh >= DETECT_HEIGHT - IMAGE_BORDER_MARGIN):
        return None

    corners = normalize_rect_corners(r.corners())
    for x, y in corners:
        if (x < IMAGE_BORDER_MARGIN or y < IMAGE_BORDER_MARGIN or
                x >= DETECT_WIDTH - IMAGE_BORDER_MARGIN or
                y >= DETECT_HEIGHT - IMAGE_BORDER_MARGIN):
            return None
    if not is_convex_quad(corners):
        return None

    side_lengths = []
    for i in range(4):
        side_lengths.append(math.sqrt(distance_sq(corners[i], corners[(i + 1) & 3])))
    min_side = min(side_lengths)
    max_side = max(side_lengths)
    if min_side < min_edge_length or max_side / min_side > max_side_ratio:
        return None

    quad_area = polygon_area(corners)
    if quad_area / area < min_quad_fill:
        return None

    center = get_target_center(corners)
    cx, cy = center
    if not (rx < cx < rx + rw and ry < cy < ry + rh):
        return None
    if interior_mean(img, corners, center) < min_interior:
        return None

    sample_radius = max(2, min(6, int(min_side * 0.06)))
    valid_edges = 0
    contrast_total = 0
    for i in range(4):
        contrast, dark = edge_signature(
            img, corners[i], corners[(i + 1) & 3], center, sample_radius)
        contrast_total += contrast
        if contrast >= min_contrast and dark <= max_dark:
            valid_edges += 1
    if valid_edges < min_dark_edges:
        return None

    quality = valid_edges * 100000 + int(contrast_total * 100) + min(area, 50000)
    return corners, center, quality


def rect_contains(outer, inner, margin=5):
    ox, oy, ow, oh = outer.rect()
    ix, iy, iw, ih = inner.rect()
    return (ix >= ox - margin and iy >= oy - margin and
            ix + iw <= ox + ow + margin and iy + ih <= oy + oh + margin)


def select_best_rect(img, rects, require_nested, ai_guided=False):
    valid = []
    for r in rects:
        result = validate_target(img, r, ai_guided)
        if result is not None:
            valid.append((r, result[0], result[1], result[2], r.w() * r.h()))

    if not valid:
        return None
    if not require_nested:
        best = max(valid, key=lambda item: item[3])
        return best[0], best[1], best[2]

    best_pair = None
    best_quality = -1
    for i in range(len(valid)):
        for j in range(i + 1, len(valid)):
            first = valid[i]
            second = valid[j]
            outer, inner = (first, second) if first[4] >= second[4] else (second, first)
            area_ratio = inner[4] / outer[4]
            if not (NESTED_MIN_AREA_RATIO <= area_ratio <= NESTED_MAX_AREA_RATIO):
                continue
            if not rect_contains(outer[0], inner[0]):
                continue
            max_center_distance = max(6, int(math.sqrt(outer[4]) * NESTED_CENTER_RATIO))
            if distance_sq(outer[2], inner[2]) > max_center_distance * max_center_distance:
                continue
            quality = outer[3] + inner[3]
            if quality > best_quality:
                best_pair = (outer[0], outer[1], outer[2])
                best_quality = quality
    if best_pair is not None:
        return best_pair

    # The thick tape often produces only one quad at this threshold. A strong
    # single quad still has to pass magnitude, geometry, border and paper-face
    # checks in validate_target(), then temporal confirmation below.
    best = max(valid, key=lambda item: item[3])
    return best[0], best[1], best[2]


def clamp_roi(x, y, w, h):
    x1 = max(0, x); y1 = max(0, y)
    x2 = min(DETECT_WIDTH, x + w); y2 = min(DETECT_HEIGHT, y + h)
    return [x1, y1, x2 - x1, y2 - y1]


class AiRect:
    """Minimal rectangle interface used only after confirmed AI detections."""
    def __init__(self, box):
        self._rect = clamp_roi(int(box[0]), int(box[1]), int(box[2]), int(box[3]))

    def rect(self):
        return self._rect

    def w(self):
        return self._rect[2]

    def h(self):
        return self._rect[3]

    def corners(self):
        x, y, w, h = self._rect
        return [(x, y + h), (x + w, y + h), (x + w, y), (x, y)]


class CvLiteRect:
    """把 cv_lite 行 [x,y,w,h, c1x,c1y, c2x,c2y, c3x,c3y, c4x,c4y] 适配成
    validate_target/select_best_rect 期望的 rect 接口 (与旧 find_rects 对象一致)。"""
    def __init__(self, row):
        self._rect = (int(row[0]), int(row[1]), int(row[2]), int(row[3]))
        self._corners = [
            (int(row[4]), int(row[5])), (int(row[6]), int(row[7])),
            (int(row[8]), int(row[9])), (int(row[10]), int(row[11])),
        ]

    def rect(self):
        return self._rect

    def w(self):
        return self._rect[2]

    def h(self):
        return self._rect[3]

    def corners(self):
        return self._corners

    def magnitude(self):
        # cv_lite 无边缘梯度幅值概念; 其内部 Canny+夹角+面积过滤已充当质量门,
        # 故返回极大值使 validate_target 的 magnitude 判据恒过 (交由几何/亮度精筛)。
        return 1000000000


def detect_rects(img):
    # 全帧灰度检测一次 (cv_lite 无 ROI 参数), 返回适配后的 rect 列表。
    img_np = img.to_numpy_ref()
    raw = cv_lite.grayscale_find_rectangles_with_corners(
        CVLITE_IMAGE_SHAPE, img_np,
        CVLITE_CANNY_LO, CVLITE_CANNY_HI,
        CVLITE_APPROX_EPS, CVLITE_AREA_MIN_RATIO,
        CVLITE_MAX_ANGLE_COS, CVLITE_GAUSS)
    return [CvLiteRect(r) for r in raw] if raw else []


def filter_rects_roi(rects, roi):
    # 保留 bbox 中心落在 roi 内的候选, 复刻旧 find_rects(roi=...) 的局部搜索语义。
    rx, ry, rw, rh = roi
    out = []
    for r in rects:
        x, y, w, h = r.rect()
        cx = x + w // 2
        cy = y + h // 2
        if rx <= cx < rx + rw and ry <= cy < ry + rh:
            out.append(r)
    return out


def roi_from_rect(r):
    rx, ry, rw, rh = r.rect()
    return clamp_roi(rx - ROI_MARGIN, ry - ROI_MARGIN, rw + 2*ROI_MARGIN, rh + 2*ROI_MARGIN)


def expand_roi(roi, margin):
    rx, ry, rw, rh = roi
    return clamp_roi(rx - margin, ry - margin, rw + 2*margin, rh + 2*margin)


def is_fullscreen(roi):
    return roi[0] == 0 and roi[1] == 0 and roi[2] >= DETECT_WIDTH and roi[3] >= DETECT_HEIGHT


def camera_init():
    global sensor, display_img, ai_detector
    sensor = Sensor(width=DETECT_WIDTH, height=DETECT_HEIGHT)
    sensor.reset()
    sensor.set_framesize(width=DETECT_WIDTH, height=DETECT_HEIGHT)
    sensor.set_pixformat(Sensor.GRAYSCALE)
    if AI_MODEL_AVAILABLE:
        sensor.set_framesize(width=224, height=224, chn=CAM_CHN_ID_2)
        sensor.set_pixformat(Sensor.RGBP888, chn=CAM_CHN_ID_2)
    if DISPLAY_MODE == "ide":
        Display.init(Display.VIRT, width=DETECT_WIDTH, height=DETECT_HEIGHT, fps=100, to_ide=True)
    elif DISPLAY_MODE == "lcd":
        # ST7701 实测须 to_ide=True 才显示 (见 memory k230-canmv); to_ide=False 会卡死 show()。
        Display.init(Display.ST7701, width=LCD_WIDTH, height=LCD_HEIGHT, to_ide=True)
        display_img = image.Image(LCD_WIDTH, LCD_HEIGHT, image.RGB565)
    MediaManager.init()
    sensor.run()
    if AI_MODEL_AVAILABLE:
        ai_detector = TargetDetector()
        ai_detector.config_preprocess()
    laser_init()
    uart_init()


def camera_deinit():
    # 每步独立 try, 保证即使初始化中途失败也能尽量释放, 避免 sensor 残留导致
    # 下次 "sensor already inited"。
    try: set_laser(False)
    except Exception: pass
    try:
        if ai_detector:
            ai_detector.deinit()
    except Exception: pass
    try:
        if sensor:
            sensor.stop()
    except Exception: pass
    try:
        if DISPLAY_MODE != "none":
            Display.deinit()
    except Exception: pass
    try: os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    except Exception: pass
    time.sleep_ms(100)
    try: MediaManager.deinit()
    except Exception: pass


def show(img):
    if DISPLAY_MODE == "ide":
        Display.show_image(img)
    elif DISPLAY_MODE == "lcd":
        display_img.clear()
        display_img.draw_image(img, 0, 0, x_scale=LCD_WIDTH/DETECT_WIDTH, y_scale=LCD_HEIGHT/DETECT_HEIGHT)
        Display.show_image(display_img)


def draw_lock(img, r, cx, cy, roi):
    img.draw_rectangle([v for v in r.rect()], color=255, thickness=2)
    img.draw_cross(cx, cy, color=255, size=15)
    img.draw_rectangle(roi, color=255, thickness=1)
    img.draw_cross(IMG_CENTER_X, IMG_CENTER_Y, color=255, size=10)
    img.draw_line(IMG_CENTER_X, IMG_CENTER_Y, cx, cy, color=255)


def run_ai_detector():
    if not AI_MODEL_AVAILABLE or ai_detector is None:
        return None, 0.0, None
    frame = sensor.snapshot(chn=CAM_CHN_ID_2)
    result = ai_detector.run(frame.to_numpy_ref())
    if not result or not len(result[0]):
        return None, 0.0, None

    best_index = 0
    for index in range(1, len(result[0])):
        if result[2][index] > result[2][best_index]:
            best_index = index
    x, y, w, h = result[0][best_index]
    box = clamp_roi(int(x), int(y), int(w), int(h))
    roi = clamp_roi(
        int(x) - AI_ROI_MARGIN,
        int(y) - AI_ROI_MARGIN,
        int(w) + 2 * AI_ROI_MARGIN,
        int(h) + 2 * AI_ROI_MARGIN,
    )
    return roi, float(result[2][best_index]), box


def capture_loop():
    tracking_state = STATE_SEARCHING
    coast_counter = 0
    roi_expand_steps = 0
    search_roi = [0, 0, DETECT_WIDTH, DETECT_HEIGHT]
    last_rect = None
    last_cx = last_cy = 0
    last_corners = None
    lock_miss_count = 0
    pending_center = None
    confirm_count = 0
    confirm_misses = 0
    frame_counter = 0
    last_log_state = -1
    ai_roi = None
    ai_box = None
    ai_roi_age = AI_ROI_HOLD_FRAMES + 1
    ai_confidence = 0.0
    ai_confirm_count = 0
    ai_pending_center = None

    frame_times = []
    fps_val = 0.0

    while True:
        frame_counter += 1
        now = time.ticks_ms()
        frame_times.append(now)
        while frame_times and time.ticks_diff(now, frame_times[0]) > 1000:
            frame_times.pop(0)
        if len(frame_times) > 1:
            fps_val = (len(frame_times) - 1) * 1000.0 / time.ticks_diff(now, frame_times[0])

        try:
            os.exitpoint()
            img = sensor.snapshot()

            if AI_MODEL_AVAILABLE and frame_counter % AI_RUN_INTERVAL == 1:
                detected_roi, detected_confidence, detected_box = run_ai_detector()
                if detected_roi is not None:
                    ai_roi = detected_roi
                    ai_box = detected_box
                    ai_confidence = detected_confidence
                    ai_roi_age = 0
                    box_center = (
                        detected_box[0] + detected_box[2] // 2,
                        detected_box[1] + detected_box[3] // 2)
                    if (detected_confidence >= AI_FALLBACK_CONFIDENCE and
                            ai_pending_center is not None and
                            distance_sq(box_center, ai_pending_center) <=
                            AI_FALLBACK_MAX_SHIFT_PX * AI_FALLBACK_MAX_SHIFT_PX):
                        ai_confirm_count += 1
                    elif detected_confidence >= AI_FALLBACK_CONFIDENCE:
                        ai_confirm_count = 1
                    else:
                        ai_confirm_count = 0
                    ai_pending_center = box_center
                else:
                    ai_roi_age += AI_RUN_INTERVAL
                    ai_confirm_count = max(0, ai_confirm_count - 1)
            elif AI_MODEL_AVAILABLE:
                ai_roi_age += 1

            rect_search_roi = search_roi
            using_ai_roi = False
            if (AI_MODEL_AVAILABLE and is_fullscreen(search_roi) and ai_roi is not None and
                    ai_roi_age <= AI_ROI_HOLD_FRAMES):
                rect_search_roi = ai_roi
                using_ai_roi = True

            ai_guided = (ai_box is not None and ai_roi_age <= AI_ROI_HOLD_FRAMES and
                         ai_confirm_count >= AI_FALLBACK_CONFIRMATIONS)
            # cv_lite 全帧检测一次, 再按 ROI 在 Python 侧过滤 (C 层无 ROI 参数);
            # 比旧路径一帧调两次 find_rects 更省 —— 两处 ROI 复用同一次检测结果。
            all_rects = detect_rects(img)
            rects = filter_rects_roi(all_rects, rect_search_roi)
            candidate = select_best_rect(
                img, rects, tracking_state != STATE_LOCKED, ai_guided) if rects else None
            if (candidate is None and using_ai_roi and
                    frame_counter % AI_FULL_CV_FALLBACK_INTERVAL == 0):
                rects = filter_rects_roi(all_rects, search_roi)
                candidate = select_best_rect(
                    img, rects, tracking_state != STATE_LOCKED) if rects else None

            # A stable semantic detection may provide a coarse lock when glare,
            # blur or broken tape prevents find_rects from closing a contour.
            # Traditional CV remains first priority and replaces these corners
            # as soon as a valid quadrilateral is available.
            if candidate is None and ai_guided:
                best = AiRect(ai_box)
                corners = best.corners()
                center = get_target_center(corners)
                candidate = (best, corners, center)

            if candidate is not None:
                best, corners, center = candidate
                if tracking_state == STATE_LOCKED:
                    if distance_sq(center, (last_cx, last_cy)) > LOCK_MAX_SHIFT_PX * LOCK_MAX_SHIFT_PX:
                        candidate = None
                else:
                    if (pending_center is not None and
                            distance_sq(center, pending_center) <= CONFIRM_MAX_SHIFT_PX * CONFIRM_MAX_SHIFT_PX):
                        confirm_count += 1
                    else:
                        confirm_count = 1
                    pending_center = center
                    confirm_misses = 0
                    if confirm_count < SEARCH_CONFIRM_FRAMES:
                        candidate = None
            else:
                confirm_misses += 1
                if confirm_misses > SEARCH_CONFIRM_MAX_MISSES:
                    pending_center = None
                    confirm_count = 0
                    confirm_misses = 0

            if candidate is not None:
                tracking_state = STATE_LOCKED
                coast_counter = 0
                lock_miss_count = 0
                roi_expand_steps = 0
                confirm_count = SEARCH_CONFIRM_FRAMES
                confirm_misses = 0
                pending_center = center
                search_roi = roi_from_rect(best)

                cx, cy = center
                tx_cx, tx_cy = mirror_point(center)
                tx_corners = [mirror_point(point) for point in corners]
                dx, dy = tx_cx - IMG_CENTER_X, tx_cy - IMG_CENTER_Y

                laser_on = (abs(dx) <= LASER_ENABLE_ERROR_PX and abs(dy) <= LASER_ENABLE_ERROR_PX)
                set_laser(laser_on)
                send_vision_frame(tx_cx, tx_cy, IMG_CENTER_X, IMG_CENTER_Y, tx_corners)

                last_rect = [v for v in best.rect()]
                last_cx, last_cy = cx, cy
                last_corners = corners

                if DISPLAY_MODE != "none" and frame_counter % DISPLAY_INTERVAL == 0:
                    draw_lock(img, best, cx, cy, search_roi)
                    img.draw_string_advanced(10, 10, 20, "FPS:%.1f L:%d" % (fps_val, 1 if laser_on else 0), color=255)
                    show(img)
                if frame_counter % LOG_INTERVAL == 0 or last_log_state != STATE_LOCKED:
                    print("LOCK raw=(%d,%d) tx=(%d,%d) dx=%d dy=%d roi=%s ai=%.2f fps=%.1f" % (
                        cx, cy, tx_cx, tx_cy, dx, dy, search_roi,
                        ai_confidence, fps_val))
                last_log_state = STATE_LOCKED
            else:
                set_laser(False)
                hold_last_target = False
                if tracking_state == STATE_LOCKED:
                    lock_miss_count += 1
                    if lock_miss_count <= LOCK_HOLD_MISS_FRAMES and last_corners is not None:
                        hold_last_target = True
                    else:
                        tracking_state = STATE_COASTING
                        coast_counter = MAX_COASTING_FRAMES
                        lock_miss_count = 0
                elif tracking_state == STATE_COASTING:
                    coast_counter -= 1
                    if coast_counter <= 0:
                        if (not is_fullscreen(search_roi)) and roi_expand_steps < MAX_ROI_EXPAND_STEPS:
                            roi_expand_steps += 1
                            search_roi = expand_roi(search_roi, ROI_EXPAND_MARGIN)
                            coast_counter = MAX_COASTING_FRAMES
                        else:
                            tracking_state = STATE_SEARCHING
                            roi_expand_steps = 0
                            search_roi = [0, 0, DETECT_WIDTH, DETECT_HEIGHT]

                if hold_last_target:
                    held_center = mirror_point((last_cx, last_cy))
                    held_corners = [mirror_point(point) for point in last_corners]
                    send_vision_frame(
                        held_center[0], held_center[1],
                        IMG_CENTER_X, IMG_CENTER_Y, held_corners)
                else:
                    send_vision_frame()  # 全零帧 = 未找到目标
                if DISPLAY_MODE != "none" and frame_counter % DISPLAY_INTERVAL == 0:
                    if tracking_state == STATE_COASTING and last_rect:
                        img.draw_rectangle(last_rect, color=255, thickness=1)
                        img.draw_cross(last_cx, last_cy, color=255, size=10)
                    img.draw_string_advanced(10, 10, 20, "FPS:%.1f SEARCH" % fps_val, color=255)
                    show(img)
                if frame_counter % LOG_INTERVAL == 0 or last_log_state != tracking_state:
                    print("MISS state=%d hold=%d confirm=%d roi=%s ai=%.2f fps=%.1f" % (
                        tracking_state, 1 if hold_last_target else 0,
                        confirm_count, search_roi, ai_confidence, fps_val))
                last_log_state = tracking_state

            img = None
            if frame_counter % GC_INTERVAL == 0:
                gc.collect()
        except KeyboardInterrupt:
            print("user stop")
            break
        except MemoryError:
            tracking_state = STATE_SEARCHING
            coast_counter = 0
            roi_expand_steps = 0
            search_roi = [0, 0, DETECT_WIDTH, DETECT_HEIGHT]
            gc.collect()
            continue
        except BaseException as e:
            import sys
            sys.print_exception(e)
            break


def main():
    os.exitpoint(os.EXITPOINT_ENABLE)
    try:
        print("--- rect_target 启动 (CV + optional YOLO ROI, AI=%d) ---" % (
            1 if AI_MODEL_AVAILABLE else 0))
        camera_init()
        capture_loop()
    except Exception as e:
        import sys
        sys.print_exception(e)
    finally:
        # 无论初始化是否成功都释放, 防止 sensor/MediaManager 状态残留。
        camera_deinit()


if __name__ == "__main__":
    main()
