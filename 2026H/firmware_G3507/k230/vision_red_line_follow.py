# ------------------------------------------------------------------------
# vision_red_line_follow.py -- CanMV K230 v1.8 红色竖线循迹
#
# GC2093 以 1280x720@90fps 工作：通道0输出 800x480 YUV420SP 并直绑 LCD，
# 通道2输出 320x192 RGB888 供 OpenCV。两通道共享同一组 5:3 中心裁剪参数。
#
# 识别链：RGB->HSV 双红色掩膜->形态学->多水平 ROI 轮廓->连续路径->地面投影拟合。
# 黑线由 S/V 阈值排除；红色横线产生的宽轮廓被拒绝，不参与轨迹拟合。
# ------------------------------------------------------------------------
import gc
import math
import os
import time

import cv2
import image
from machine import FPIOA, UART
from media.display import Display
from media.sensor import CAM_CHN_ID_0, CAM_CHN_ID_2, Sensor


# ===== 传感器、识别通道与方向 =====
SENSOR_WIDTH = 1280
SENSOR_HEIGHT = 720
SENSOR_FPS = 90
SENSOR_CROP_X = 40
SENSOR_CROP_Y = 0
SENSOR_CROP_WIDTH = 1200
SENSOR_CROP_HEIGHT = 720

IMAGE_WIDTH = 320
IMAGE_HEIGHT = 192
LCD_WIDTH = 800
LCD_HEIGHT = 480
CAMERA_ROTATED_180 = False
AUTO_FOCUS_ENABLED = True

# ===== HSV 红色阈值（OpenCV: H 0..179, S/V 0..255） =====
# 红色跨越色相首尾，因此使用两个区间。S 下限排除白/灰，V 下限排除黑线。
HSV_RED_LOW_1 = (0, 70, 45)
HSV_RED_HIGH_1 = (12, 255, 255)
HSV_RED_LOW_2 = (165, 70, 45)
HSV_RED_HIGH_2 = (179, 255, 255)

MORPH_OPEN_ENABLED = True
MORPH_KERNEL_WIDTH = 3
MORPH_KERNEL_HEIGHT = 3

# ===== 多水平 ROI 与红色竖线几何约束 =====
SCAN_ROWS = (58, 76, 94, 112, 130, 148, 166, 182)
SCAN_BAND_HEIGHT = 12
MIN_CONTOUR_AREA = 3.0
MIN_TRACK_WIDTH = 2
MAX_TRACK_WIDTH = 42
MIN_TRACK_HEIGHT = 2
MAX_CHOICES_PER_ROW = 6
MIN_FIT_POINTS = 5
MAX_FIT_RESIDUAL_PX = 10.0
MAX_VERTICAL_HEADING_DEG = 60.0

NEAR_REFERENCE_Y = 176.0
FAR_REFERENCE_Y = 72.0
TRACK_REFERENCE_X = IMAGE_WIDTH * 0.5

# ===== K230 安装几何（米、度；以下数值只是待上车测量的可运行初值） =====
# 车体坐标原点为轮轴中心地面投影：右为 +X，前为 +Y，上为 +Z。
# CAMERA_PIVOT_DISTANCE_M 是从转轴沿摄像头光轴向镜头方向的有符号距离。
K230_PIVOT_AXLE_FORWARD_M = 0.12
K230_PIVOT_HEIGHT_M = 0.24
K230_PIVOT_PITCH_DEG = 45.0
K230_CAMERA_PIVOT_DISTANCE_M = 0.03

# 检测通道镜头内参也必须实测标定；主点和焦距均使用 320x192 检测图坐标。
CAMERA_FOCAL_X_PX = 228.5
CAMERA_FOCAL_Y_PX = 228.5
CAMERA_PRINCIPAL_X_PX = IMAGE_WIDTH * 0.5
CAMERA_PRINCIPAL_Y_PX = IMAGE_HEIGHT * 0.5

TRACK_REFERENCE_LATERAL_M = 0.0
TRACK_REFERENCE_HEADING_DEG = 0.0
POSITION_NORMALIZATION_M = 0.20
MIN_GROUND_FORWARD_M = 0.02
MAX_GROUND_FORWARD_M = 3.00
MAX_GROUND_FIT_RESIDUAL_M = 0.030

# LOCK 时只在预测轨迹附近搜索；连续丢失后恢复全宽 SEARCH。
LOCK_SEARCH_HALF_WIDTH = 72
LOCK_MAX_POINT_JUMP_PX = 46.0
LOST_TO_SEARCH_FRAMES = 4

# ===== 输出滤波与协议 =====
OUTPUT_EMA_ALPHA = 0.40
FRAME_START0 = 0xA5
FRAME_START1 = 0x5A
FLAG_TRACK_VALID = 0x01

# 用户尚未确认接入 MSPM0 实际串口，当前只在 USB 控制台输出诊断。
UART_ENABLED = False
UART_TX_PIN = 3
UART_RX_PIN = 4
UART_BAUDRATE = 115200

# ===== LCD 与调试 =====
DISPLAY_LCD = True
DISPLAY_INTERVAL = 5
LOG_INTERVAL = 30
RUN_DURATION_MS = 0


sensor = None
uart = None
display_img = None
morph_kernel = None
tx_sequence = 0

track_locked = False
track_lost_count = 0
last_slope = 0.0
last_intercept = TRACK_REFERENCE_X
last_near_x = TRACK_REFERENCE_X

filter_ready = False
filtered_position = 0.0
filtered_heading = 0.0

camera_pitch_rad = math.radians(K230_PIVOT_PITCH_DEG)
camera_sin_pitch = math.sin(camera_pitch_rad)
camera_cos_pitch = math.cos(camera_pitch_rad)
camera_forward_m = (K230_PIVOT_AXLE_FORWARD_M +
                    K230_CAMERA_PIVOT_DISTANCE_M * camera_cos_pitch)
camera_height_m = (K230_PIVOT_HEIGHT_M -
                   K230_CAMERA_PIVOT_DISTANCE_M * camera_sin_pitch)
focus_status = "AF OFF"


def clamp(value, low, high):
    return low if value < low else high if value > high else value


def append_i16_be(frame, value):
    value = max(-32768, min(32767, int(round(value)))) & 0xFFFF
    frame.append((value >> 8) & 0xFF)
    frame.append(value & 0xFF)


def send_frame(valid, position, heading, confidence):
    global tx_sequence
    if uart is None:
        return
    flags = FLAG_TRACK_VALID if valid else 0
    frame = bytearray((FRAME_START0, FRAME_START1,
                       tx_sequence & 0xFF, flags))
    append_i16_be(frame, position * 1000.0 if valid else 0.0)
    append_i16_be(frame, heading * 100.0 if valid else 0.0)
    frame.append(confidence if valid else 0)
    frame.append(sum(frame) & 0xFF)
    uart.write(frame)
    tx_sequence = (tx_sequence + 1) & 0xFF


def normalize_find_contours(result):
    # OpenCV 4 返回 (contours, hierarchy)，部分兼容层返回三项。
    return result[0] if len(result) == 2 else result[1]


def contour_center(contour, x_offset, y_offset, rect):
    moments = cv2.moments(contour)
    if abs(moments["m00"]) > 1.0e-6:
        return (x_offset + moments["m10"] / moments["m00"],
                y_offset + moments["m01"] / moments["m00"])
    x, y, width, height = rect
    return (x_offset + x + 0.5 * width,
            y_offset + y + 0.5 * height)


def predicted_x(y):
    return last_slope * y + last_intercept


def find_band_choices(mask, center_y):
    half_height = SCAN_BAND_HEIGHT // 2
    y0 = max(0, int(center_y) - half_height)
    y1 = min(IMAGE_HEIGHT, y0 + SCAN_BAND_HEIGHT)

    if track_locked:
        center_x = predicted_x(center_y)
        x0 = max(0, int(center_x - LOCK_SEARCH_HALF_WIDTH))
        x1 = min(IMAGE_WIDTH, int(center_x + LOCK_SEARCH_HALF_WIDTH))
    else:
        x0 = 0
        x1 = IMAGE_WIDTH
    if x1 <= x0 or y1 <= y0:
        return []

    roi = mask[y0:y1, x0:x1]
    result = cv2.findContours(roi, cv2.RETR_EXTERNAL,
                              cv2.CHAIN_APPROX_SIMPLE)
    choices = []
    for contour in normalize_find_contours(result):
        area = float(cv2.contourArea(contour))
        if area < MIN_CONTOUR_AREA:
            continue
        rect = cv2.boundingRect(contour)
        _, _, width, height = rect
        if (width < MIN_TRACK_WIDTH or width > MAX_TRACK_WIDTH or
                height < MIN_TRACK_HEIGHT):
            continue
        cx, cy = contour_center(contour, x0, y0, rect)
        if track_locked and abs(cx - predicted_x(cy)) > LOCK_MAX_POINT_JUMP_PX:
            continue
        choices.append((cx, cy, width, area))

    reference_x = predicted_x(center_y) if track_locked else last_near_x
    choices.sort(key=lambda item: abs(item[0] - reference_x))
    return choices[:MAX_CHOICES_PER_ROW]


def select_track_points(mask):
    rows = []
    for center_y in reversed(SCAN_ROWS):
        choices = find_band_choices(mask, center_y)
        if choices:
            rows.append((float(center_y), choices))
    if not rows:
        return []

    states = []
    first_y, first_choices = rows[0]
    first_reference = predicted_x(first_y) if track_locked else last_near_x
    for cx, cy, width, area in first_choices:
        states.append((abs(cx - first_reference), [(cy, cx)]))

    for row_y, choices in rows[1:]:
        next_states = []
        for cx, cy, width, area in choices:
            best = None
            for cost, points in states:
                previous_y, previous_x = points[-1]
                dy = abs(previous_y - cy)
                slope = abs(previous_x - cx) / dy if dy > 0.0 else 0.0
                new_cost = cost + abs(previous_x - cx) + 0.08 * width
                if slope > 1.2:
                    new_cost += 12.0 * (slope - 1.2)
                if best is None or new_cost < best[0]:
                    best = (new_cost, points)
            if best is not None:
                next_states.append((best[0], best[1] + [(cy, cx)]))
        if next_states:
            states = next_states
    return min(states, key=lambda state: state[0])[1] if states else []


def fit_x_from_y(points):
    count = len(points)
    if count < 2:
        return None
    sum_y = sum_x = sum_yy = sum_yx = 0.0
    for y, x in points:
        sum_y += y
        sum_x += x
        sum_yy += y * y
        sum_yx += y * x
    denominator = count * sum_yy - sum_y * sum_y
    if abs(denominator) < 1.0e-6:
        return None
    slope = (count * sum_yx - sum_y * sum_x) / denominator
    intercept = (sum_x - slope * sum_y) / count
    residual = sum(abs(x - (slope * y + intercept))
                   for y, x in points) / count
    return slope, intercept, residual


def pixel_to_ground(y, x):
    """把检测图像点投影到轮轴中心地面坐标，返回 (forward_m, lateral_m)。"""
    normalized_x = (x - CAMERA_PRINCIPAL_X_PX) / CAMERA_FOCAL_X_PX
    normalized_y = (y - CAMERA_PRINCIPAL_Y_PX) / CAMERA_FOCAL_Y_PX
    ray_down = camera_sin_pitch + normalized_y * camera_cos_pitch
    if ray_down <= 1.0e-6:
        return None

    ray_forward = camera_cos_pitch - normalized_y * camera_sin_pitch
    distance_scale = camera_height_m / ray_down
    forward_m = camera_forward_m + distance_scale * ray_forward
    lateral_m = distance_scale * normalized_x
    if (forward_m < MIN_GROUND_FORWARD_M or
            forward_m > MAX_GROUND_FORWARD_M):
        return None
    return forward_m, lateral_m


def project_track_to_ground(points):
    ground_points = []
    for y, x in points:
        ground_point = pixel_to_ground(y, x)
        if ground_point is not None:
            ground_points.append(ground_point)
    return ground_points


def request_auto_focus():
    global focus_status
    if not AUTO_FOCUS_ENABLED:
        focus_status = "AF OFF"
        return

    try:
        enabled = sensor.auto_focus(True)
        focus_status = "AF SET" if enabled else "AF FAIL"
        print("vision_focus request=%s" % focus_status)
    except Exception as exc:
        focus_status = "AF N/A"
        print("vision_focus request unavailable: %s" % exc)


def check_auto_focus_capability():
    global focus_status
    if not AUTO_FOCUS_ENABLED or focus_status != "AF SET":
        return

    try:
        focus_caps = sensor.focus_caps()
        if focus_caps and focus_caps[0]:
            focus_status = "AF ON"
        else:
            focus_status = "AF FIXED"
        print("vision_focus status=%s caps=%s" %
              (focus_status, focus_caps))
    except Exception as exc:
        focus_status = "AF N/A"
        print("vision_focus capability unavailable: %s" % exc)


def make_red_mask(rgb_img):
    rgb = rgb_img.to_numpy_ref()
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    mask_low = cv2.inRange(hsv, HSV_RED_LOW_1, HSV_RED_HIGH_1)
    mask_high = cv2.inRange(hsv, HSV_RED_LOW_2, HSV_RED_HIGH_2)
    mask = cv2.bitwise_or(mask_low, mask_high)
    if MORPH_OPEN_ENABLED:
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, morph_kernel)
    return mask


def detect_track(mask):
    global track_locked, track_lost_count
    global last_slope, last_intercept, last_near_x

    points = select_track_points(mask)
    pixel_fit = fit_x_from_y(points) if len(points) >= MIN_FIT_POINTS else None
    if pixel_fit is None or pixel_fit[2] > MAX_FIT_RESIDUAL_PX:
        track_lost_count += 1
        if track_lost_count > LOST_TO_SEARCH_FRAMES:
            track_locked = False
            last_slope = 0.0
            last_intercept = last_near_x
        return False, 0.0, 0.0, 0, points, pixel_fit, None, 0.0

    slope, intercept, residual = pixel_fit
    near_x = slope * NEAR_REFERENCE_Y + intercept
    ground_points = project_track_to_ground(points)
    ground_fit = (fit_x_from_y(ground_points)
                  if len(ground_points) >= MIN_FIT_POINTS else None)
    if ground_fit is None or ground_fit[2] > MAX_GROUND_FIT_RESIDUAL_M:
        track_lost_count += 1
        return False, 0.0, 0.0, 0, points, pixel_fit, ground_fit, 0.0

    ground_slope, ground_intercept, ground_residual = ground_fit
    lateral_error_m = ground_intercept - TRACK_REFERENCE_LATERAL_M
    heading_raw = math.degrees(math.atan(ground_slope))
    if abs(heading_raw) > MAX_VERTICAL_HEADING_DEG:
        track_lost_count += 1
        return False, 0.0, 0.0, 0, points, pixel_fit, ground_fit, 0.0

    position = clamp(lateral_error_m / POSITION_NORMALIZATION_M, -1.0, 1.0)
    heading = clamp(heading_raw - TRACK_REFERENCE_HEADING_DEG, -90.0, 90.0)
    support = min(1.0, len(points) / float(len(SCAN_ROWS)))
    pixel_quality = clamp(1.0 - residual / MAX_FIT_RESIDUAL_PX, 0.0, 1.0)
    ground_quality = clamp(1.0 - ground_residual /
                           MAX_GROUND_FIT_RESIDUAL_M, 0.0, 1.0)
    quality = min(pixel_quality, ground_quality)
    confidence = int((0.72 * support + 0.28 * quality) * 255.0)

    last_slope = slope
    last_intercept = intercept
    last_near_x = clamp(near_x, 0.0, IMAGE_WIDTH - 1.0)
    track_locked = True
    track_lost_count = 0
    return (True, position, heading, confidence, points, pixel_fit,
            ground_fit, lateral_error_m)


def camera_init():
    global sensor, uart, display_img, morph_kernel

    if (CAMERA_FOCAL_X_PX <= 0.0 or CAMERA_FOCAL_Y_PX <= 0.0 or
            POSITION_NORMALIZATION_M <= 0.0 or camera_height_m <= 0.0 or
            K230_PIVOT_PITCH_DEG < 0.0 or K230_PIVOT_PITCH_DEG > 90.0):
        raise ValueError("invalid K230 mounting geometry or camera intrinsics")

    sensor = Sensor(width=SENSOR_WIDTH, height=SENSOR_HEIGHT, fps=SENSOR_FPS)
    sensor.reset()
    # 官方 API 要求硬件自动对焦在 sensor.run() 之前开启。
    request_auto_focus()
    if CAMERA_ROTATED_180:
        sensor.set_hmirror(True)
        sensor.set_vflip(True)

    # v1.8 的双通道 PipeLine 例程先初始化显示，再配置和绑定视频通道。
    if DISPLAY_LCD:
        Display.init(Display.ST7701, width=LCD_WIDTH, height=LCD_HEIGHT,
                     osd_num=1, to_ide=True)

    # 两路使用完全相同的传感器裁剪窗口，避免自动 crop 造成视场角偏差。
    sensor_crop = (SENSOR_CROP_X, SENSOR_CROP_Y,
                   SENSOR_CROP_WIDTH, SENSOR_CROP_HEIGHT)
    sensor.set_framesize(width=LCD_WIDTH, height=LCD_HEIGHT,
                         chn=CAM_CHN_ID_0, crop=sensor_crop)
    sensor.set_pixformat(Sensor.YUV420SP, chn=CAM_CHN_ID_0)
    sensor.set_framesize(width=IMAGE_WIDTH, height=IMAGE_HEIGHT,
                         chn=CAM_CHN_ID_2, crop=sensor_crop)
    sensor.set_pixformat(Sensor.RGB888, chn=CAM_CHN_ID_2)

    display_width = sensor.width(chn=CAM_CHN_ID_0)
    display_height = sensor.height(chn=CAM_CHN_ID_0)
    detect_width = sensor.width(chn=CAM_CHN_ID_2)
    detect_height = sensor.height(chn=CAM_CHN_ID_2)
    if display_width * detect_height != detect_width * display_height:
        raise RuntimeError("display and detection channel aspect ratios differ")

    if DISPLAY_LCD:
        bind_info = sensor.bind_info(x=0, y=0, chn=CAM_CHN_ID_0)
        # ST7701 使用面板默认方向；v1.8 PipeLine 例程不强制覆盖旋转标志。
        Display.bind_layer(**bind_info, layer=Display.LAYER_VIDEO1)
        display_img = image.Image(LCD_WIDTH, LCD_HEIGHT, image.ARGB8888)

    sensor.run()
    # v1.8 01Studio 实测 focus_caps() 需在 run() 后读取。
    check_auto_focus_capability()

    morph_kernel = cv2.getStructuringElement(
        cv2.MORPH_RECT, (MORPH_KERNEL_WIDTH, MORPH_KERNEL_HEIGHT))

    if UART_ENABLED:
        fpioa = FPIOA()
        fpioa.set_function(UART_TX_PIN, FPIOA.UART1_TXD)
        fpioa.set_function(UART_RX_PIN, FPIOA.UART1_RXD)
        uart = UART(UART.UART1, baudrate=UART_BAUDRATE,
                    bits=UART.EIGHTBITS, parity=UART.PARITY_NONE,
                    stop=UART.STOPBITS_ONE)

    print("vision_v18 sensor=%dx%d@%d display=%dx%d detect=%dx%d uart=%d" %
          (SENSOR_WIDTH, SENSOR_HEIGHT, SENSOR_FPS,
           display_width, display_height, detect_width, detect_height,
           UART_ENABLED))
    print("vision_geometry pivot_y=%.3f pivot_h=%.3f pitch=%.1f arm=%.3f camera_y=%.3f camera_h=%.3f" %
          (K230_PIVOT_AXLE_FORWARD_M, K230_PIVOT_HEIGHT_M,
           K230_PIVOT_PITCH_DEG, K230_CAMERA_PIVOT_DISTANCE_M,
           camera_forward_m, camera_height_m))


def camera_deinit():
    try:
        if sensor:
            sensor.stop()
    except Exception:
        pass
    # 按项目要求不关闭 Display，停止/异常时保留最后一帧和 OSD。


def draw_overlay(valid, position, heading, confidence, lateral_error_m,
                 points, fit,
                 fps, capture_ms, mask_ms, track_ms):
    if not DISPLAY_LCD or display_img is None:
        return
    display_img.clear()
    x_scale = LCD_WIDTH / IMAGE_WIDTH
    y_scale = LCD_HEIGHT / IMAGE_HEIGHT

    for y, x in points:
        display_img.draw_cross(int(x * x_scale), int(y * y_scale),
                               color=(0, 255, 0), size=8, thickness=2)
    if fit is not None:
        slope, intercept, residual = fit
        x_far = slope * FAR_REFERENCE_Y + intercept
        x_near = slope * NEAR_REFERENCE_Y + intercept
        display_img.draw_line(int(x_far * x_scale),
                              int(FAR_REFERENCE_Y * y_scale),
                              int(x_near * x_scale),
                              int(NEAR_REFERENCE_Y * y_scale),
                              color=(0, 255, 0), thickness=3)

    if valid:
        status = ("LOCK e%+.3fm p%+.2f h%+.1f c%d" %
                  (lateral_error_m, position, heading, confidence))
    else:
        status = "SEARCH RED VERTICAL"
    display_img.draw_string_advanced(12, 12, 27, status,
                                     color=(255, 255, 255))
    display_img.draw_string_advanced(
        12, 48, 22, "%s FPS %.1f cap%d mask%d track%d" %
        (focus_status, fps, capture_ms, mask_ms, track_ms),
        color=(255, 255, 0))
    # v1.8 PipeLine 的单 OSD 配置对应 OSD3。
    Display.show_image(display_img, 0, 0, Display.LAYER_OSD3)


def main():
    global filter_ready, filtered_position, filtered_heading

    os.exitpoint(os.EXITPOINT_ENABLE)
    camera_init()
    start_ms = time.ticks_ms()
    frame_count = 0
    fps = 0.0
    fps_window_start_ms = start_ms
    fps_window_count = 0

    try:
        while (RUN_DURATION_MS <= 0 or
               time.ticks_diff(time.ticks_ms(), start_ms) < RUN_DURATION_MS):
            os.exitpoint()

            capture_start_ms = time.ticks_ms()
            img = sensor.snapshot(chn=CAM_CHN_ID_2)
            capture_ms = time.ticks_diff(time.ticks_ms(), capture_start_ms)

            mask_start_ms = time.ticks_ms()
            mask = make_red_mask(img)
            mask_ms = time.ticks_diff(time.ticks_ms(), mask_start_ms)

            track_start_ms = time.ticks_ms()
            (valid, position, heading, confidence, points, fit,
             ground_fit, lateral_error_m) = detect_track(mask)
            track_ms = time.ticks_diff(time.ticks_ms(), track_start_ms)

            if valid:
                if not filter_ready:
                    filtered_position = position
                    filtered_heading = heading
                    filter_ready = True
                else:
                    filtered_position += OUTPUT_EMA_ALPHA * (position - filtered_position)
                    filtered_heading += OUTPUT_EMA_ALPHA * (heading - filtered_heading)
            else:
                filter_ready = False
                filtered_position = 0.0
                filtered_heading = 0.0

            send_frame(valid, filtered_position, filtered_heading, confidence)
            filtered_lateral_error_m = (
                filtered_position * POSITION_NORMALIZATION_M +
                TRACK_REFERENCE_LATERAL_M) if valid else 0.0

            frame_count += 1
            fps_window_count += 1
            now_ms = time.ticks_ms()
            fps_elapsed_ms = time.ticks_diff(now_ms, fps_window_start_ms)
            if fps_elapsed_ms >= 1000:
                fps = fps_window_count * 1000.0 / fps_elapsed_ms
                fps_window_start_ms = now_ms
                fps_window_count = 0

            if frame_count % DISPLAY_INTERVAL == 0:
                draw_overlay(valid, filtered_position, filtered_heading,
                             confidence, filtered_lateral_error_m,
                             points, fit, fps,
                             capture_ms, mask_ms, track_ms)

            if frame_count % LOG_INTERVAL == 0:
                residual = fit[2] if fit is not None else -1.0
                ground_residual = ground_fit[2] if ground_fit is not None else -1.0
                print("vision_v18 valid=%d lateral_m=%+.3f pos=%+.3f heading=%+.2f conf=%d points=%d px_res=%.2f ground_res=%.3f fps=%.1f cap=%d mask=%d track=%d" %
                      (valid, filtered_lateral_error_m, filtered_position,
                       filtered_heading, confidence, len(points), residual,
                       ground_residual, fps,
                       capture_ms, mask_ms, track_ms))
                gc.collect()

            img = None
            mask = None
    except KeyboardInterrupt:
        pass
    except BaseException as exc:
        import sys
        sys.print_exception(exc)
    finally:
        camera_deinit()


if __name__ == "__main__":
    main()
