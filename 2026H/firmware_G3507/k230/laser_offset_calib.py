# ------------------------------------------------------------------------
# laser_offset_calib.py -- S10 激光-相机光轴偏差标定 (K230 侧)
#
# 目的: 实测蓝紫激光落点与相机光轴的角度偏差, 得到主程序 aim_track_angle.py
#       所需的 IDEAL_YAW_DEG / IDEAL_PITCH_DEG (以及多距离下的视差判断)。
#
# 原理:
#   1) K230 自主闪烁激光 (亮/灭交替), 相邻帧做差分 -> 唯一亮度突变点 = 光斑。
#      差分法对环境亮度/白纸反光免疫; 感光纸痕迹是慢化学变色, 相邻帧(~50ms)
#      内变化远低于阈值, 不干扰。
#   2) 灭帧上跑 cv_lite 矩形检测 -> 靶心 (与主程序同一套三层筛选, 定义一致)。
#   3) 发送的 A5 5A 角度帧 = [靶心角 - 光斑角] = 激光系瞄准误差。
#      MCU 侧 Aim track (AppE_RunContinuousAim, 纯角度伺服) 不需要任何修改,
#      会自动把【激光】伺服到靶心 (光斑与相机同装云台, 图像中光斑位置近似
#      不动, 环路动力学与"目标居中"完全同构)。
#   4) 收敛(|err|<阈值)后, 屏幕/串口打印:
#         IDEAL_YAW_DEG  = 光斑的相机系 yaw 角
#         IDEAL_PITCH_DEG= 光斑的相机系 pitch 角
#      即"激光在相机图像里的理想指向", 抄进 aim_track_angle.py 顶部常量。
#
# 操作流程:
#   a. 部署本脚本到 K230 运行 (k230_tool.py run, 或临时替换 /sdcard/main.py);
#   b. MCU 进入「Aim track」页 (会发 LASER=1 并按角度帧伺服云台);
#   c. 车摆在 ~1m 处对靶, 等屏上 CONVERGED, 记录 IDEAL 两值;
#   d. 把车挪到最近(~0.5m)/最远(~1.6m) 各重复一次:
#        - 三个距离 IDEAL 基本一致  -> 纯安装偏角, 直接填 IDEAL_*;
#        - IDEAL 随距离单调漂移     -> 存在平移视差, 按屏上提示的
#          d(角度)/d(距离) 换算 CALIB_OFFSET_X/Y_CM (≈ tan(Δ角)·距离差)。
#   e. 抄值 -> 恢复主程序 aim_track_angle.py 自启。
# ------------------------------------------------------------------------
import time, os, gc, math
import image
import cv_lite
from ulab import numpy as np
from media.sensor import *
from media.display import *
from media.media import *
from machine import Pin, FPIOA, UART

# ===== 图像 (与主程序一致) =====
# 传感器工作在原生 1920x1080(16:9) 全阵列, 处理分辨率 480x270(÷4): 与 aim_track_angle.py
# 保持一致的 FOV 与缩放比。光轴偏置必须在与生产程序【相同 FOV】下标定, 否则像素↔角度
# 映射不一致会标错。Sensor(320,240) 会落到 1280x960 裁剪模式丢失约 1/3 水平 FOV。
SENSOR_WIDTH = 1920
SENSOR_HEIGHT = 1080
DETECT_WIDTH = 480
DETECT_HEIGHT = 270
IMG_CX = DETECT_WIDTH // 2
IMG_CY = DETECT_HEIGHT // 2
CVLITE_SHAPE = [DETECT_HEIGHT, DETECT_WIDTH]

CANNY_LO, CANNY_HI = 50, 150
APPROX_EPS = 0.04
AREA_MIN_RATIO = 0.002
MAX_ANGLE_COS = 0.3
GAUSS = 5
MIN_ASPECT, MAX_ASPECT = 0.4, 2.5
MIN_AREA, MAX_AREA = 1500, 60000

MIRROR_180 = True            # 与主程序一致: 只影响发给 MCU/显示的控制坐标

# ===== 角度模型 (与主程序一致) =====
H_FOV_DEG = 60.0
V_FOV_DEG = 35.0
H_TAN_HALF_FOV = math.tan(math.radians(H_FOV_DEG / 2.0))
V_TAN_HALF_FOV = math.tan(math.radians(V_FOV_DEG / 2.0))

# ===== 光斑差分检测 =====
DOT_MIN_DIFF = 40            # 差分峰值下限 (0~255): 低于此认为本周期没找到光斑
DOT_WIN = 7                  # 峰值邻域质心窗口半径 (px)
DOT_MAX_AREA_PX = 400        # 差分超阈像素数上限: 过大=整幅亮度变化(曝光波动), 丢弃
# 闪烁节奏: 激光切换后先等驱动亮度稳定(软启/余辉), 再丢过渡帧, 然后才取测量帧。
# 调低闪烁频率 = 加大这两个值; 当前 ≈ 每状态 80ms+3帧 -> 闪烁 ~3Hz, 测量 ~3 周期/s。
BLINK_SETTLE_MS = 80         # 切换激光电平后的驻留时间 (ms)
BLINK_SETTLE_FRAMES = 3      # 驻留后再丢弃的过渡帧数 (曝光跨切换沿)

# ===== 输出平滑与收敛判据 =====
OUT_EMA = 0.4                # 光斑/靶心角度 EMA (仅用于 IDEAL 读数显示)
CONVERGE_ERR_DEG = 0.5       # |靶心-光斑| 双轴小于此值判收敛
CONVERGE_CYCLES = 8          # 连续 N 个测量周期满足才报 CONVERGED
DISPLAY_INTERVAL = 2         # 每 N 个测量周期刷一次 LCD (刷屏是重操作, 降低测量延迟)

# ===== UART1 (与主程序一致的 A5 5A 角度帧) =====
UART_TX_PIN, UART_RX_PIN = 3, 4
UART_BAUD = 115200
FRAME_SOF0, FRAME_SOF1 = 0xA5, 0x5A
ANGLE_SCALE = 100.0
ANGLE_STATUS_VALID = 0
ANGLE_STATUS_NOT_FOUND = 1

LASER_PIN = 2
DISPLAY_LCD = True
LCD_WIDTH, LCD_HEIGHT = 800, 480

DEBUG_LOG_EVERY = 10         # 每 N 个测量周期打一行

uart = None
laser = None
sensor = None
display_img = None
laser_allowed = False        # MCU LASER=1/0 意愿: 0 时暂停闪烁与测量
laser_command_buffer = bytearray()
angle_tx_seq = 0


def mirror_pt(x, y):
    if not MIRROR_180:
        return int(x), int(y)
    mx = max(0, min(DETECT_WIDTH - 1, 2 * IMG_CX - int(x)))
    my = max(0, min(DETECT_HEIGHT - 1, 2 * IMG_CY - int(y)))
    return mx, my


def angle_of(px, py):
    # 像素 -> 相机系视场角 (deg); 输入为镜像补偿后的控制坐标。
    dx = px - IMG_CX
    dy = py - IMG_CY
    yaw = math.degrees(math.atan((dx / (DETECT_WIDTH / 2.0)) * H_TAN_HALF_FOV))
    pitch = math.degrees(math.atan((dy / (DETECT_HEIGHT / 2.0)) * V_TAN_HALF_FOV))
    return yaw, pitch


def diag_center(c):
    x1, y1 = c[0]; x2, y2 = c[2]
    x3, y3 = c[1]; x4, y4 = c[3]
    den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if -1e-3 < den < 1e-3:
        return ((x1 + x2 + x3 + x4) // 4, (y1 + y2 + y3 + y4) // 4)
    px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / den
    py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / den
    return int(px), int(py)


def find_target(img):
    # 灭帧上找靶: 与主程序同一套几何/尺寸筛选, 取最大面积。返回 (cx,cy) 或 None。
    rects = cv_lite.grayscale_find_rectangles_with_corners(
        CVLITE_SHAPE, img.to_numpy_ref(), CANNY_LO, CANNY_HI,
        APPROX_EPS, AREA_MIN_RATIO, MAX_ANGLE_COS, GAUSS)
    best = None
    best_area = 0
    best_corners = None
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
            best_corners = [(r[4], r[5]), (r[6], r[7]), (r[8], r[9]), (r[10], r[11])]
    if best is None:
        return None, None
    return diag_center(best_corners), best_corners


def find_dot(on_frame_i16, off_frame_i16):
    """帧间差分找光斑: 返回 (x, y) 亚像素质心, 或 None。
    on/off 均为 int16 ulab 数组 (激光亮帧 - 灭帧, 光斑处为大正值)。"""
    d = on_frame_i16 - off_frame_i16
    peak = int(np.max(d))
    if peak < DOT_MIN_DIFF:
        return None
    # 防"整幅变亮"(曝光/环境突变): 超阈像素过多则本周期作废。
    thr = max(DOT_MIN_DIFF, peak // 2)
    mask = np.where(d > thr, d, 0)
    n_hot = int(np.sum(np.where(d > thr, 1, 0)))
    if n_hot > DOT_MAX_AREA_PX or n_hot == 0:
        return None
    # 峰值位置 + 邻域加权质心 (亚像素)。
    flat = int(np.argmax(d))
    py = flat // DETECT_WIDTH
    px = flat % DETECT_WIDTH
    y0 = max(0, py - DOT_WIN); y1 = min(DETECT_HEIGHT, py + DOT_WIN + 1)
    x0 = max(0, px - DOT_WIN); x1 = min(DETECT_WIDTH, px + DOT_WIN + 1)
    w = mask[y0:y1, x0:x1]
    total = float(np.sum(w))
    if total <= 0.0:
        return None
    col_sum = np.sum(w, axis=0)
    row_sum = np.sum(w, axis=1)
    cx = x0 + float(np.sum(col_sum * np.arange(col_sum.size))) / total
    cy = y0 + float(np.sum(row_sum * np.arange(row_sum.size))) / total
    return cx, cy


def append_i16_be(frame, value):
    value = max(-32768, min(32767, int(round(value)))) & 0xFFFF
    frame.append((value >> 8) & 0xFF)
    frame.append(value & 0xFF)


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


def poll_laser_command():
    # MCU 的 LASER=1/0 表达"允许/禁止激光"; 标定期间由本脚本负责实际闪烁。
    global laser_allowed, laser_command_buffer
    if uart is None or uart.any() <= 0:
        return
    data = uart.read()
    if not data:
        return
    for byte in data:
        if byte == 10 or byte == 13:
            if laser_command_buffer == b"LASER=1":
                laser_allowed = True
            elif laser_command_buffer == b"LASER=0":
                laser_allowed = False
            laser_command_buffer = bytearray()
        elif len(laser_command_buffer) < 16:
            laser_command_buffer.append(byte)
        else:
            laser_command_buffer = bytearray()


def camera_init():
    global sensor, uart, laser, display_img
    # 用原生全阵列构造 (1920x1080 全 FOV), 再由 ISP 缩放到处理分辨率; 直接用
    # DETECT_WIDTH/HEIGHT 构造会落到 1280x960 裁剪模式而丢失水平 FOV。
    sensor = Sensor(width=SENSOR_WIDTH, height=SENSOR_HEIGHT)
    sensor.reset()
    sensor.set_framesize(width=DETECT_WIDTH, height=DETECT_HEIGHT)
    sensor.set_pixformat(Sensor.GRAYSCALE)
    if DISPLAY_LCD:
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
    laser.value(0)


def camera_deinit():
    try:
        if laser:
            laser.value(0)
    except Exception:
        pass
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


def grab_i16(skip):
    # 丢弃切换过渡帧后取一帧, 拷贝为 int16 (差分要带符号)。
    img = None
    for _ in range(skip):
        img = sensor.snapshot()
    img = sensor.snapshot()
    return img, np.array(img.to_numpy_ref(), dtype=np.int16)


# LCD 数值面板配色 (RGB565 缓冲用 RGB 元组)
COL_OK = (0, 255, 0)
COL_WARN = (255, 220, 0)
COL_BAD = (255, 60, 60)
COL_TXT = (255, 255, 255)


def show_overlay(img, target, corners, dot, laser_angle, target_angle,
                 err, conv_count, result, fps):
    if not DISPLAY_LCD or display_img is None:
        return
    # 小图上只画几何标记 (中心十字/靶框/光斑圈), 随图拉伸。
    img.draw_cross(IMG_CX, IMG_CY, color=255, size=10, thickness=1)
    if corners is not None:
        for i in range(4):
            x0, y0 = corners[i]
            x1, y1 = corners[(i + 1) & 3]
            img.draw_line(int(x0), int(y0), int(x1), int(y1), color=255, thickness=2)
    if target is not None:
        img.draw_cross(int(target[0]), int(target[1]), color=255, size=8, thickness=2)
    if dot is not None:
        img.draw_circle(int(dot[0]), int(dot[1]), 8, color=255, thickness=2)

    display_img.clear()
    display_img.draw_image(img, 0, 0,
                           x_scale=LCD_WIDTH / DETECT_WIDTH,
                           y_scale=LCD_HEIGHT / DETECT_HEIGHT)

    # 数值面板: 画在 800x480 原生分辨率上, 大字号不糊。
    converged = conv_count >= CONVERGE_CYCLES
    if laser_angle is not None:
        display_img.draw_string_advanced(
            10, 8, 48, "IDEAL Y %+.2f  P %+.2f" % laser_angle,
            color=(COL_OK if converged else COL_WARN))
    else:
        display_img.draw_string_advanced(10, 8, 48, "IDEAL  --.--   --.--",
                                         color=COL_BAD)
    if err is not None:
        display_img.draw_string_advanced(
            10, 62, 40, "err  Y %+.2f  P %+.2f" % err,
            color=(COL_OK if (abs(err[0]) < CONVERGE_ERR_DEG and
                              abs(err[1]) < CONVERGE_ERR_DEG) else COL_TXT))
    if target_angle is not None:
        display_img.draw_string_advanced(
            10, 108, 28, "target Y %+.2f P %+.2f" % target_angle, color=COL_TXT)
    display_img.draw_string_advanced(
        10, 142, 28,
        "dot:%s  tgt:%s  conv %d/%d  fps %.1f" % (
            "OK" if dot is not None else "--",
            "OK" if target is not None else "--",
            conv_count, CONVERGE_CYCLES, fps),
        color=(COL_TXT if (dot is not None and target is not None) else COL_BAD))
    if converged:
        display_img.draw_string_advanced(
            10, LCD_HEIGHT - 60, 40, "CONVERGED -> copy IDEAL", color=COL_OK)
    elif result is not None:
        # 曾经收敛过的最终值持续驻留 (换位姿后仍可回看上次结果)。
        display_img.draw_string_advanced(
            10, LCD_HEIGHT - 52, 32,
            "last result Y %+.2f P %+.2f" % result, color=COL_OK)
    Display.show_image(display_img)


def show_waiting():
    # MCU 未开激光 (未进 Aim track 页) 时的提示画面。
    if not DISPLAY_LCD or display_img is None:
        return
    display_img.clear()
    display_img.draw_string_advanced(10, 8, 48, "LASER CALIB", color=COL_WARN)
    display_img.draw_string_advanced(10, 70, 32,
                                     "waiting MCU: enter [Aim track]",
                                     color=COL_TXT)
    display_img.draw_string_advanced(10, 110, 28,
                                     "(needs LASER=1 from MCU)", color=COL_TXT)
    Display.show_image(display_img)


def main():
    os.exitpoint(os.EXITPOINT_ENABLE)
    camera_init()
    laser_ang = None            # (yaw, pitch) EMA: 光斑相机系角 = IDEAL 候选值
    target_ang = None
    result = None               # 最近一次收敛时的 IDEAL 值 (LCD 驻留显示)
    conv_count = 0
    cycle = 0
    fps = 0.0
    fps_t0 = time.ticks_ms()
    fps_n = 0
    waiting_shown = False
    try:
        while True:
            os.exitpoint()
            poll_laser_command()

            if not laser_allowed:
                # MCU 未开激光: 熄灭、只发 NOT_FOUND, 等待 Aim track 进入。
                laser.value(0)
                send_angle_frame(0.0, 0.0, ANGLE_STATUS_NOT_FOUND)
                if not waiting_shown:
                    show_waiting()
                    waiting_shown = True
                time.sleep_ms(50)
                continue
            waiting_shown = False

            # === 一个测量周期: 灭帧 -> 亮帧 -> 差分 (低频闪烁: 驻留+丢过渡帧) ===
            laser.value(0)
            time.sleep_ms(BLINK_SETTLE_MS)
            off_img, off_i16 = grab_i16(BLINK_SETTLE_FRAMES)
            laser.value(1)
            time.sleep_ms(BLINK_SETTLE_MS)
            on_img, on_i16 = grab_i16(BLINK_SETTLE_FRAMES)

            target_raw, corners_raw = find_target(off_img)   # 灭帧找靶(无光斑干扰)
            dot_raw = find_dot(on_i16, off_i16)
            off_i16 = None
            on_i16 = None

            fps_n += 1
            now = time.ticks_ms()
            el = time.ticks_diff(now, fps_t0)
            if el >= 1000:
                fps = fps_n * 1000.0 / el
                fps_t0 = now
                fps_n = 0

            err = None
            if (target_raw is not None) and (dot_raw is not None):
                tx, ty = mirror_pt(target_raw[0], target_raw[1])
                lx, ly = mirror_pt(dot_raw[0], dot_raw[1])
                t_yaw, t_pitch = angle_of(tx, ty)
                l_yaw, l_pitch = angle_of(lx, ly)
                if laser_ang is None:
                    laser_ang = (l_yaw, l_pitch)
                    target_ang = (t_yaw, t_pitch)
                else:
                    laser_ang = (laser_ang[0] + OUT_EMA * (l_yaw - laser_ang[0]),
                                 laser_ang[1] + OUT_EMA * (l_pitch - laser_ang[1]))
                    target_ang = (target_ang[0] + OUT_EMA * (t_yaw - target_ang[0]),
                                  target_ang[1] + OUT_EMA * (t_pitch - target_ang[1]))
                # 发给 MCU 的是【激光系误差】= 靶心 - 光斑; aim_track 会把激光伺服到靶心。
                # 靶心取本周期瞬时值(测量周期 ~150ms, EMA 再叠 1-2 周期滞后会加剧过冲);
                # 光斑在图像中准静止, 用 EMA 值降噪。IDEAL 读数(显示)仍用双 EMA。
                err = (t_yaw - laser_ang[0], t_pitch - laser_ang[1])
                send_angle_frame(err[0], err[1], ANGLE_STATUS_VALID)
                if abs(err[0]) < CONVERGE_ERR_DEG and abs(err[1]) < CONVERGE_ERR_DEG:
                    if conv_count < CONVERGE_CYCLES:
                        conv_count += 1
                        if conv_count == CONVERGE_CYCLES:
                            result = (laser_ang[0], laser_ang[1])
                            print("==== CONVERGED ====")
                            print("IDEAL_YAW_DEG   = %.2f" % laser_ang[0])
                            print("IDEAL_PITCH_DEG = %.2f" % laser_ang[1])
                            print("(抄进 aim_track_angle.py; 换距离重测对比判视差)")
                    else:
                        result = (laser_ang[0], laser_ang[1])   # 收敛期间跟随最新值
                else:
                    conv_count = 0
            else:
                send_angle_frame(0.0, 0.0, ANGLE_STATUS_NOT_FOUND)
                conv_count = 0

            if cycle % DISPLAY_INTERVAL == 0:
                show_overlay(on_img, target_raw, corners_raw, dot_raw,
                             laser_ang, target_ang, err, conv_count, result, fps)

            cycle += 1
            if cycle % DEBUG_LOG_EVERY == 0:
                if laser_ang is not None:
                    print("cyc%d laser=(%.2f,%.2f) target=(%.2f,%.2f) err=(%s) conv=%d fps=%.1f"
                          % (cycle, laser_ang[0], laser_ang[1],
                             target_ang[0], target_ang[1],
                             ("%.2f,%.2f" % err) if err else "-", conv_count, fps))
                else:
                    print("cyc%d no-dot/no-target (dot=%s target=%s) fps=%.1f"
                          % (cycle, dot_raw is not None, target_raw is not None, fps))
            if cycle % 20 == 0:
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
