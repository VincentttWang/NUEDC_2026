# ------------------------------------------------------------------------
# aim_track.py — 2025E 持续瞄准 K230 配套程序 (精简版)
#
# 与 MCU 端「Aim track」(AppE_RunContinuousAim / GimbalTracking_UpdateLaserCenter)
# 配套: 用 cv_lite 检测黑色矩形靶, 每帧把
#   [目标中心x, 目标中心y, 图像中心x, 图像中心y, 4个角点]
# 打成 27 字节定长帧发到 MSPM0 (UART1)。MCU 用 PID 把"目标中心"驱到"图像中心",
# 即把靶居中 = 激光瞄准。无靶时发全零帧, MCU 判为目标丢失并停云台。
#
# 帧格式 (大端, 严格对齐 bsp/canmv/canmv_uart.c, 共 27 字节):
#   [0]   0x12 帧头
#   [1]   滚动序号 (MCU 忽略)
#   [2:10]  laser 段 = 4×u16: 目标中心x,y, 图像中心x,y
#   [10:26] rect  段 = 8×u16: 角点1..4 的 x,y
#   [26]  0x5B 帧尾
# ------------------------------------------------------------------------
import time, os, gc
import image
import cv_lite
from media.sensor import *
from media.display import *
from media.media import *
from machine import Pin, FPIOA, UART

# ===== 图像 =====
DETECT_WIDTH = 320
DETECT_HEIGHT = 240
IMG_CX = DETECT_WIDTH // 2
IMG_CY = DETECT_HEIGHT // 2
CVLITE_SHAPE = [DETECT_HEIGHT, DETECT_WIDTH]   # [高, 宽]

# ===== cv_lite 矩形检测参数 (同 rect_target, 偏宽松让远/弱框也进候选) =====
CANNY_LO, CANNY_HI = 50, 150
APPROX_EPS = 0.04
AREA_MIN_RATIO = 0.002
MAX_ANGLE_COS = 0.3
GAUSS = 5

# ===== 候选精筛 =====
MIN_AREA = 1500        # bbox 面积下限 (px^2), 滤背景噪声
MAX_AREA = 60000
MIN_ASPECT = 0.4
MAX_ASPECT = 2.5

# ===== 相机 180° 倒装补偿 (与 rect_target 的 CAMERA_ROTATED_180 一致) =====
MIRROR_180 = True

# ===== LCD 显示 (板载 ST7701 800x480; 实测须 to_ide=True 才显示且不卡) =====
DISPLAY_LCD = True
LCD_WIDTH, LCD_HEIGHT = 800, 480
DISPLAY_INTERVAL = 2       # 每 N 帧刷一次屏, 兼顾流畅与帧率

# ===== UART1: pin3 TX / pin4 RX / 115200 (对接 MSPM0 UART2) =====
UART_TX_PIN, UART_RX_PIN = 3, 4
UART_BAUD = 115200
FRAME_START, FRAME_END = 0x12, 0x5B

# ===== 激光笔: GPIO2 高电平点亮 =====
LASER_PIN = 2
# 目标中心到图像中心的最大允许二维误差 (像素)。超过此值立即熄灭激光。
LASER_LOCK_ERROR_PX = 8

# ===== 调试打印 =====
DEBUG = True
LOG_INTERVAL = 30      # 每 N 帧打印一次

uart = None
laser = None
sensor = None
display_img = None
_seq = 0


def uart_init():
    global uart
    fpioa = FPIOA()
    fpioa.set_function(UART_TX_PIN, FPIOA.UART1_TXD)
    fpioa.set_function(UART_RX_PIN, FPIOA.UART1_RXD)
    uart = UART(UART.UART1, baudrate=UART_BAUD,
                bits=UART.EIGHTBITS, parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)


def laser_init():
    # 与 laser_on.py 保持一致：GPIO2 输出高电平时激光笔点亮。
    global laser
    fpioa = FPIOA()
    fpioa.set_function(LASER_PIN, FPIOA.GPIO2)
    laser = Pin(LASER_PIN, Pin.OUT)
    laser.value(0)             # 启动时始终关闭，防止上电误点亮


def set_laser_locked(cx, cy):
    # 仅当目标在图像中心附近时点亮；圆形阈值避免对角线方向误差偏大。
    if laser is None:
        return False
    dx = int(cx) - IMG_CX
    dy = int(cy) - IMG_CY
    locked = (dx * dx + dy * dy) <= (LASER_LOCK_ERROR_PX * LASER_LOCK_ERROR_PX)
    laser.value(1 if locked else 0)
    return locked


def laser_off():
    if laser is not None:
        laser.value(0)


def mirror_pt(x, y):
    if not MIRROR_180:
        return int(x), int(y)
    mx = (2 * IMG_CX) - int(x)
    my = (2 * IMG_CY) - int(y)
    mx = max(0, min(DETECT_WIDTH - 1, mx))
    my = max(0, min(DETECT_HEIGHT - 1, my))
    return mx, my


def _put_u16_be(buf, v):
    v = int(v)
    if v < 0:
        v = 0
    elif v > 65535:
        v = 65535
    buf.append((v >> 8) & 0xFF)
    buf.append(v & 0xFF)


def send_frame(cx, cy, corners):
    # corners: 4×(x,y); 全部已是最终发送坐标 (含镜像)。无靶时传 cx=cy=0 且角点全 0。
    global _seq
    if uart is None:
        return
    f = bytearray()
    f.append(FRAME_START)
    f.append(_seq & 0xFF)
    _seq += 1
    _put_u16_be(f, cx)          # 目标中心
    _put_u16_be(f, cy)
    _put_u16_be(f, IMG_CX)      # 激光点 = 图像中心
    _put_u16_be(f, IMG_CY)
    for i in range(4):
        if corners is not None:
            _put_u16_be(f, corners[i][0])
            _put_u16_be(f, corners[i][1])
        else:
            _put_u16_be(f, 0)
            _put_u16_be(f, 0)
    f.append(FRAME_END)
    uart.write(f)


def diag_center(c):
    # 四角点对角线交点; 退化时取平均。c 顺序无所谓 (取 0-2 与 1-3 两条对角线)。
    x1, y1 = c[0]; x2, y2 = c[2]
    x3, y3 = c[1]; x4, y4 = c[3]
    d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if -1e-3 < d < 1e-3:
        return (x1 + x2 + x3 + x4) // 4, (y1 + y2 + y3 + y4) // 4
    px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / d
    py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / d
    return int(px), int(py)


def pick_best(rects):
    # 面积最强判据: 选通过 面积/长宽比 的最大 bbox。返回 (corners, (cx,cy)) 或 None。
    best = None
    best_area = 0
    for r in rects:
        w, h = r[2], r[3]
        if w <= 0 or h <= 0:
            continue
        area = w * h
        if area < MIN_AREA or area > MAX_AREA:
            continue
        asp = float(w) / float(h)
        if asp < MIN_ASPECT or asp > MAX_ASPECT:
            continue
        if area > best_area:
            best_area = area
            best = r
    if best is None:
        return None
    corners = [(best[4], best[5]), (best[6], best[7]),
               (best[8], best[9]), (best[10], best[11])]
    return corners, diag_center(corners)


def camera_init():
    global sensor, display_img
    sensor = Sensor(width=DETECT_WIDTH, height=DETECT_HEIGHT)
    sensor.reset()
    sensor.set_framesize(width=DETECT_WIDTH, height=DETECT_HEIGHT)
    sensor.set_pixformat(Sensor.GRAYSCALE)
    if DISPLAY_LCD:
        # to_ide=True 为 ST7701 实测可用配方 (见 rect_target); 不要传 fps=。
        Display.init(Display.ST7701, width=LCD_WIDTH, height=LCD_HEIGHT, to_ide=True)
        display_img = image.Image(LCD_WIDTH, LCD_HEIGHT, image.RGB565)
    MediaManager.init()
    sensor.run()
    uart_init()
    laser_init()


def camera_deinit():
    laser_off()
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
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    except Exception:
        pass
    time.sleep_ms(100)
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


def draw_overlay(img, best, n):
    # 图像中心十字 (激光瞄准参考点)
    img.draw_cross(IMG_CX, IMG_CY, color=255, size=10, thickness=1)
    if best is not None:
        corners, (cx, cy) = best
        for i in range(4):
            x0, y0 = corners[i]
            x1, y1 = corners[(i + 1) % 4]
            img.draw_line(int(x0), int(y0), int(x1), int(y1), color=255, thickness=2)
        img.draw_cross(int(cx), int(cy), color=255, size=14, thickness=2)
        img.draw_line(IMG_CX, IMG_CY, int(cx), int(cy), color=255, thickness=1)
        tag = "LOCK n=%d" % n
    else:
        tag = "SEARCH n=%d" % n
    img.draw_string_advanced(4, 4, 20, tag, color=255)


def main():
    os.exitpoint(os.EXITPOINT_ENABLE)
    print("--- aim_track 启动 (cv_lite 持续瞄准, UART1@115200) ---")
    camera_init()
    frame_cnt = 0
    try:
        while True:
            os.exitpoint()
            img = sensor.snapshot()
            rects = cv_lite.grayscale_find_rectangles_with_corners(
                CVLITE_SHAPE, img.to_numpy_ref(),
                CANNY_LO, CANNY_HI, APPROX_EPS, AREA_MIN_RATIO, MAX_ANGLE_COS, GAUSS)
            best = pick_best(rects) if rects else None

            if best is not None:
                corners, (cx, cy) = best
                tx = mirror_pt(cx, cy)
                tcorners = [mirror_pt(px, py) for (px, py) in corners]
                # 保证中心非零 (MCU 用 0 判丢失)
                if tx[0] == 0:
                    tx = (1, tx[1])
                if tx[1] == 0:
                    tx = (tx[0], 1)
                send_frame(tx[0], tx[1], tcorners)
                set_laser_locked(tx[0], tx[1])
            else:
                send_frame(0, 0, None)   # 全零 = 丢失
                laser_off()

            if DISPLAY_LCD and (frame_cnt % DISPLAY_INTERVAL == 0):
                draw_overlay(img, best, len(rects) if rects else 0)
                show(img)

            frame_cnt += 1
            img = None
            if DEBUG and (frame_cnt % LOG_INTERVAL == 0):
                if best is not None:
                    print("tx center=(%d,%d) n=%d" % (tx[0], tx[1], len(rects)))
                else:
                    print("no target (n=%d)" % (len(rects) if rects else 0))
            if frame_cnt % 30 == 0:
                gc.collect()
    except KeyboardInterrupt:
        print("user stop")
    except BaseException as e:
        import sys
        sys.print_exception(e)
    finally:
        camera_deinit()


if __name__ == "__main__":
    main()
