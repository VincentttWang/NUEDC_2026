# ------------------------------------------------------------------------
# aim_track_angle.py -- K230 角度误差追踪
#
# 矩形检测恢复为 aim_track.py 的灰度 cv_lite 路径，
# 面积/长宽比筛选、最大候选优先。
# 角度与标定目标点计算采用 rect11_lcd.py 的 FOV/靶高/偏置参数模型。
# 协议：A5 5A status seq yaw_i16_be pitch_i16_be checksum，单位 0.01 deg。
# ------------------------------------------------------------------------
import time, os, gc, math
import image
import cv_lite
from media.sensor import *
from media.display import *
from media.media import *
from machine import Pin, FPIOA, UART

# ===== 图像与矩形检测：恢复为 aim_track.py 的灰度参数 =====
DETECT_WIDTH = 320
DETECT_HEIGHT = 240
IMG_CX = DETECT_WIDTH // 2
IMG_CY = DETECT_HEIGHT // 2
CVLITE_SHAPE = [DETECT_HEIGHT, DETECT_WIDTH]

CANNY_LO, CANNY_HI = 50, 150
APPROX_EPS = 0.04
AREA_MIN_RATIO = 0.002
MAX_ANGLE_COS = 0.3
GAUSS = 5

# 灰度矩形筛选阈值：保留供现场标定。
MIN_AREA = 1500
MAX_AREA = 60000
MIN_ASPECT = 0.4
MAX_ASPECT = 2.5

# 摄像头倒装补偿：与 aim_track.py 相同，只影响发给 MSPM0 的控制坐标。
MIRROR_180 = True

# ===== 角度模型：参考 rect11_lcd.py 的初始参数 =====
H_FOV_DEG = 60.0
V_FOV_DEG = 35.0
H_TAN_HALF_FOV = math.tan(math.radians(H_FOV_DEG / 2.0))
V_TAN_HALF_FOV = math.tan(math.radians(V_FOV_DEG / 2.0))

TARGET_HEIGHT_CM = 17.3
CALIB_OFFSET_X_CM = -0.607  # 横向平移偏差 (cm)，正数表示激光在摄像头右侧，负数表示在左侧
CALIB_OFFSET_Y_CM = 1.060  # 纵向平移偏差 (cm)，正数表示激光在摄像头下方，负数表示在上方
# 人工光点标定：原始画面 (173,105)，经 MIRROR_180 后控制坐标为 (147,135)。
# 按当前 H_FOV=60°、V_FOV=35° 换算得到下列理想角；若重新测得光点，请同步更新。
IDEAL_YAW_DEG = -0.649   #正数:激光笔右偏
IDEAL_PITCH_DEG = 1.359  #正数：激光笔下偏

# ===== 蓝紫色激光笔：暂不参与识别，仅保留待人工标定的阈值 =====
# 格式：(L_min, L_max, A_min, A_max, B_min, B_max)。当前版本不使用它。
LASER_PURPLE_THRESHOLD = (0, 100, -128, 127, -128, 127)

# ===== UART1（GPIO3/4，对接 MSPM0 UART2） =====
UART_TX_PIN, UART_RX_PIN = 3, 4
UART_BAUD = 115200
FRAME_SOF0, FRAME_SOF1 = 0xA5, 0x5A
ANGLE_SCALE = 100.0
ANGLE_STATUS_VALID = 0
ANGLE_STATUS_NOT_FOUND = 1
ANGLE_STATUS_LOST = 2
VISION_FRAME_START, VISION_FRAME_END = 0x12, 0x5B

# ===== 激光与 LCD =====
LASER_PIN = 2
DISPLAY_LCD = True
LCD_WIDTH, LCD_HEIGHT = 800, 480
DISPLAY_INTERVAL = 2

DEBUG = True
LOG_INTERVAL = 30

uart = None
laser = None
sensor = None
display_img = None
laser_requested = False
laser_command_buffer = bytearray()
angle_tx_seq = 0
target_seen = False


def mirror_pt(x, y):
    if not MIRROR_180:
        return int(x), int(y)
    mx = max(0, min(DETECT_WIDTH - 1, 2 * IMG_CX - int(x)))
    my = max(0, min(DETECT_HEIGHT - 1, 2 * IMG_CY - int(y)))
    return mx, my


def diag_center(corners):
    # 与 aim_track.py 相同：使用两条对角线交点，退化时使用四点平均。
    x1, y1 = corners[0]; x2, y2 = corners[2]
    x3, y3 = corners[1]; x4, y4 = corners[3]
    den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if -1e-3 < den < 1e-3:
        return ((x1 + x2 + x3 + x4) // 4,
                (y1 + y2 + y3 + y4) // 4)
    px = ((x1 * y2 - y1 * x2) * (x3 - x4) -
          (x1 - x2) * (x3 * y4 - y3 * x4)) / den
    py = ((x1 * y2 - y1 * x2) * (y3 - y4) -
          (y1 - y2) * (x3 * y4 - y3 * x4)) / den
    return int(px), int(py)


def pick_best(rects):
    # 与 aim_track.py 相同：通过面积/长宽比筛选后选择最大 bbox。
    best = None
    best_area = 0
    for r in rects:
        w, h = r[2], r[3]
        if w <= 0 or h <= 0:
            continue
        area = w * h
        if area < MIN_AREA or area > MAX_AREA:
            continue
        aspect = float(w) / float(h)
        if aspect < MIN_ASPECT or aspect > MAX_ASPECT:
            continue
        if area > best_area:
            best_area = area
            best = r
    if best is None:
        return None
    corners = [(best[4], best[5]), (best[6], best[7]),
               (best[8], best[9]), (best[10], best[11])]
    return best, corners, diag_center(corners)


def append_i16_be(frame, value):
    value = max(-32768, min(32767, int(round(value)))) & 0xFFFF
    frame.append((value >> 8) & 0xFF)
    frame.append(value & 0xFF)


def append_u16_be(frame, value):
    value = max(0, min(65535, int(value)))
    frame.append((value >> 8) & 0xFF)
    frame.append(value & 0xFF)


def normalize_rect_corners(corners):
    # MCU 几何层约定：[左下、右下、右上、左上]。
    ordered_y = sorted(corners, key=lambda point: point[1])
    top = sorted(ordered_y[:2], key=lambda point: point[0])
    bottom = sorted(ordered_y[2:], key=lambda point: point[0])
    return [bottom[0], bottom[1], top[1], top[0]]


def send_angle_frame(yaw_err_deg, pitch_err_deg, status):
    global angle_tx_seq
    if uart is None:
        return
    frame = bytearray()
    frame.append(FRAME_SOF0)
    frame.append(FRAME_SOF1)
    frame.append(status & 0xFF)
    frame.append(angle_tx_seq & 0xFF)
    angle_tx_seq = (angle_tx_seq + 1) & 0xFF
    append_i16_be(frame, yaw_err_deg * ANGLE_SCALE)
    append_i16_be(frame, pitch_err_deg * ANGLE_SCALE)
    frame.append(sum(frame) & 0xFF)
    uart.write(frame)


def send_vision_frame(target, corners):
    # 与角度帧配套发送：搜索/圆轨迹读取矩形角点，静态瞄准仍优先使用角度帧。
    if uart is None:
        return
    frame = bytearray()
    frame.append(VISION_FRAME_START)
    frame.append(0)
    append_u16_be(frame, target[0])
    append_u16_be(frame, target[1])
    append_u16_be(frame, IMG_CX)
    append_u16_be(frame, IMG_CY)
    for x, y in normalize_rect_corners(corners):
        append_u16_be(frame, x)
        append_u16_be(frame, y)
    frame.append(VISION_FRAME_END)
    uart.write(frame)


def angle_error(target_x, target_y):
    # 与 rect11_lcd.py 相同：目标点相对图像中心 -> 视场角 -> 相对理想角误差。
    dx = target_x - IMG_CX
    dy = target_y - IMG_CY
    yaw_deg = math.degrees(math.atan((dx / (DETECT_WIDTH / 2.0)) * H_TAN_HALF_FOV))
    pitch_deg = math.degrees(math.atan((dy / (DETECT_HEIGHT / 2.0)) * V_TAN_HALF_FOV))
    return yaw_deg, pitch_deg, yaw_deg - IDEAL_YAW_DEG, pitch_deg - IDEAL_PITCH_DEG


def set_laser(on):
    if laser is not None:
        laser.value(1 if on else 0)


def poll_laser_command():
    # MSPM0 仅发送 "LASER=0\\n" 或 "LASER=1\\n"；逐字节累积以兼容 UART 分段接收。
    global laser_requested, laser_command_buffer
    if uart is None or uart.any() <= 0:
        return
    data = uart.read()
    if not data:
        return
    for byte in data:
        if byte == 10 or byte == 13:
            if laser_command_buffer == b"LASER=1":
                laser_requested = True
            elif laser_command_buffer == b"LASER=0":
                laser_requested = False
            laser_command_buffer = bytearray()
        elif len(laser_command_buffer) < 16:
            laser_command_buffer.append(byte)
        else:
            laser_command_buffer = bytearray()


def camera_init():
    global sensor, uart, laser, display_img
    sensor = Sensor(width=DETECT_WIDTH, height=DETECT_HEIGHT)
    sensor.reset()
    sensor.set_framesize(width=DETECT_WIDTH, height=DETECT_HEIGHT)
    sensor.set_pixformat(Sensor.GRAYSCALE)
    if DISPLAY_LCD:
        # 与 aim_track.py 相同的 ST7701 配置。
        Display.init(Display.ST7701, width=LCD_WIDTH, height=LCD_HEIGHT, to_ide=True)
        display_img = image.Image(LCD_WIDTH, LCD_HEIGHT, image.RGB565)
    MediaManager.init()
    sensor.run()
    fpioa = FPIOA()
    fpioa.set_function(UART_TX_PIN, FPIOA.UART1_TXD)
    fpioa.set_function(UART_RX_PIN, FPIOA.UART1_RXD)
    uart = UART(UART.UART1, baudrate=UART_BAUD,
                bits=UART.EIGHTBITS, parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)
    fpioa.set_function(LASER_PIN, FPIOA.GPIO2)
    laser = Pin(LASER_PIN, Pin.OUT)
    set_laser(False)


def camera_deinit():
    set_laser(False)
    try:
        if sensor:
            sensor.stop()
    except Exception:
        pass
    try:
        if DISPLAY_LCD:
            Display.deinit()
    except Exception:
        pass
    try:
        MediaManager.deinit()
    except Exception:
        pass


def show(img):
    if not DISPLAY_LCD or display_img is None:
        return
    display_img.clear()
    display_img.draw_image(img, 0, 0,
                           x_scale=LCD_WIDTH / DETECT_WIDTH,
                           y_scale=LCD_HEIGHT / DETECT_HEIGHT)
    Display.show_image(display_img)


def draw_overlay(img, found, rect, corners, center, target, yaw_err, pitch_err, fps):
    img.draw_cross(IMG_CX, IMG_CY, color=255, size=10, thickness=1)
    if found:
        for i in range(4):
            x0, y0 = corners[i]
            x1, y1 = corners[(i + 1) & 3]
            img.draw_line(int(x0), int(y0), int(x1), int(y1), color=255, thickness=2)
        img.draw_cross(center[0], center[1], color=255, size=12, thickness=2)
        img.draw_circle(target[0], target[1], 4, color=255, fill=True)
        img.draw_line(IMG_CX, IMG_CY, target[0], target[1], color=255, thickness=1)
        img.draw_string_advanced(4, 4, 18, "LOCK", color=255)
        img.draw_string_advanced(4, 26, 18, "Y %.2f P %.2f" % (yaw_err, pitch_err), color=255)
    else:
        img.draw_string_advanced(4, 4, 18, "SEARCH", color=255)
    img.draw_string_advanced(4, 48, 18, "FPS:%.1f" % fps, color=255)


def main():
    global sensor, target_seen
    os.exitpoint(os.EXITPOINT_ENABLE)
    camera_init()
    frame_count = 0
    fps = 0.0
    fps_window_start_ms = time.ticks_ms()
    fps_window_count = 0
    try:
        while True:
            os.exitpoint()
            poll_laser_command()
            img = sensor.snapshot()
            fps_window_count += 1
            now_ms = time.ticks_ms()
            fps_elapsed_ms = time.ticks_diff(now_ms, fps_window_start_ms)
            if fps_elapsed_ms >= 1000:
                fps = fps_window_count * 1000.0 / fps_elapsed_ms
                fps_window_start_ms = now_ms
                fps_window_count = 0
            rects = cv_lite.grayscale_find_rectangles_with_corners(
                CVLITE_SHAPE, img.to_numpy_ref(), CANNY_LO, CANNY_HI,
                APPROX_EPS, AREA_MIN_RATIO, MAX_ANGLE_COS, GAUSS)
            best = pick_best(rects) if rects else None

            if best is not None:
                rect, corners, center = best
                # 根据横纵向物理偏置（不同心）动态计算像素补偿后的靶心目标点。
                pixel_per_cm = float(rect[3]) / TARGET_HEIGHT_CM
                raw_target = (int(center[0] - CALIB_OFFSET_X_CM * pixel_per_cm),
                              int(center[1] - CALIB_OFFSET_Y_CM * pixel_per_cm))
                target = mirror_pt(raw_target[0], raw_target[1])
                yaw_deg, pitch_deg, yaw_err, pitch_err = angle_error(target[0], target[1])
                send_angle_frame(yaw_err, pitch_err, ANGLE_STATUS_VALID)
                target_seen = True
                send_vision_frame(target, [mirror_pt(x, y) for x, y in corners])

                set_laser(laser_requested)
            else:
                corners = None
                center = None
                raw_target = None
                target = None
                yaw_err = 0.0
                pitch_err = 0.0
                send_angle_frame(0.0, 0.0,
                                 ANGLE_STATUS_LOST if target_seen else ANGLE_STATUS_NOT_FOUND)
                send_vision_frame((0, 0), [(0, 0), (0, 0), (0, 0), (0, 0)])
                set_laser(laser_requested)

            if DISPLAY_LCD and (frame_count % DISPLAY_INTERVAL == 0):
                # LCD 是原始相机图；叠加也使用原始（未镜像）矩形坐标。
                draw_overlay(img, best is not None, best[0] if best else None,
                             corners, center, raw_target, yaw_err, pitch_err, fps)
                show(img)

            frame_count += 1
            if DEBUG and (frame_count % LOG_INTERVAL == 0):
                if best is not None:
                    print("rect=(%d,%d) target=(%d,%d) err=(%.2f,%.2f)" %
                          (center[0], center[1], target[0], target[1], yaw_err, pitch_err))
                else:
                    print("no target")
            img = None
            if frame_count % 30 == 0:
                gc.collect()
    except KeyboardInterrupt:
        pass
    except BaseException as exc:
        import sys
        sys.print_exception(exc)
    finally:
        camera_deinit()


if __name__ == "__main__":
    main()
