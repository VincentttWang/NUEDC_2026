"""蓝紫色激光光点人工标定工具。

用途：让 GPIO2 激光笔在本脚本运行期间常亮，在彩色画面中检测蓝紫色光点，
并经 USB 控制台输出其坐标，供人工填写 aim_track_angle.py 的标定参数。

注意：本脚本不初始化 K230 UART1，避免向 G3507 发送非视觉协议数据。
"""
import gc
import os
import time

import image
from media.sensor import Sensor
from media.display import Display
from media.media import MediaManager
from machine import FPIOA, Pin

DETECT_WIDTH = 320
DETECT_HEIGHT = 240
LCD_WIDTH = 800
LCD_HEIGHT = 480
LASER_PIN = 2

# 手动标定阈值：LAB 格式为 (L_min, L_max, A_min, A_max, B_min, B_max)。
# 初值只作为蓝紫色光点搜索起点；请根据 LCD 画面和控制台结果现场调节。
LASER_PURPLE_THRESHOLD = (15, 100, 10, 100, -100, 35)
LASER_BLOB_MIN_PIXELS = 6
LASER_BLOB_MAX_PIXELS = 1000
# 人工确认的真实光点。选择候选时优先取此点附近的色块，避免墙面/反光误选。
MANUAL_SPOT_X = 173
MANUAL_SPOT_Y = 105
MANUAL_SPOT_SEARCH_RADIUS_PX = 35
DISPLAY_INTERVAL = 2
PRINT_INTERVAL_MS = 150

sensor = None
laser = None
display_img = None


def select_laser_blob(blobs):
    """选择离人工确认光点最近的候选，拒绝远处同色干扰。"""
    best = None
    best_distance_sq = None
    for blob in blobs:
        pixels = blob.pixels()
        if pixels < LASER_BLOB_MIN_PIXELS or pixels > LASER_BLOB_MAX_PIXELS:
            continue
        dx = blob.cx() - MANUAL_SPOT_X
        dy = blob.cy() - MANUAL_SPOT_Y
        distance_sq = dx * dx + dy * dy
        if distance_sq > MANUAL_SPOT_SEARCH_RADIUS_PX * MANUAL_SPOT_SEARCH_RADIUS_PX:
            continue
        if best_distance_sq is None or distance_sq < best_distance_sq:
            best = blob
            best_distance_sq = distance_sq
    return best


def init():
    global sensor, laser, display_img
    sensor = Sensor(width=DETECT_WIDTH, height=DETECT_HEIGHT)
    sensor.reset()
    sensor.set_framesize(width=DETECT_WIDTH, height=DETECT_HEIGHT)
    sensor.set_pixformat(Sensor.RGB565)

    Display.init(Display.ST7701, width=LCD_WIDTH, height=LCD_HEIGHT, to_ide=True)
    display_img = image.Image(LCD_WIDTH, LCD_HEIGHT, image.RGB565)
    MediaManager.init()
    sensor.run()

    fpioa = FPIOA()
    fpioa.set_function(LASER_PIN, FPIOA.GPIO2)
    laser = Pin(LASER_PIN, Pin.OUT)
    laser.value(1)  # 本标定脚本运行期间保持常亮。


def show(img):
    display_img.clear()
    display_img.draw_image(img, 0, 0,
                           x_scale=LCD_WIDTH / DETECT_WIDTH,
                           y_scale=LCD_HEIGHT / DETECT_HEIGHT)
    Display.show_image(display_img)


def deinit():
    if laser is not None:
        laser.value(0)
    try:
        if sensor is not None:
            sensor.stop()
    except Exception:
        pass
    try:
        Display.deinit()
    except Exception:
        pass
    try:
        MediaManager.deinit()
    except Exception:
        pass


def main():
    frame_count = 0
    last_print_ms = 0
    os.exitpoint(os.EXITPOINT_ENABLE)
    init()
    print("LASER_CAL_START threshold=%s" % (LASER_PURPLE_THRESHOLD,))
    try:
        while True:
            os.exitpoint()
            img = sensor.snapshot()
            blobs = img.find_blobs([LASER_PURPLE_THRESHOLD],
                                   pixels_threshold=LASER_BLOB_MIN_PIXELS,
                                   area_threshold=LASER_BLOB_MIN_PIXELS,
                                   merge=True)
            spot = select_laser_blob(blobs)
            now_ms = time.ticks_ms()

            if spot is not None:
                x, y = spot.cx(), spot.cy()
                img.draw_rectangle(spot.rect(), color=0xF81F, thickness=2)
                img.draw_cross(x, y, color=0xF81F, size=12, thickness=2)
                img.draw_string_advanced(4, 4, 18, "SPOT %d,%d" % (x, y), color=0xFFFF)
                if time.ticks_diff(now_ms, last_print_ms) >= PRINT_INTERVAL_MS:
                    print("LASER_SPOT x=%d y=%d px=%d rect=%s" %
                          (x, y, spot.pixels(), spot.rect()))
                    last_print_ms = now_ms
            else:
                img.draw_string_advanced(4, 4, 18, "SPOT NONE", color=0xFFFF)
                if time.ticks_diff(now_ms, last_print_ms) >= PRINT_INTERVAL_MS:
                    print("LASER_SPOT NONE")
                    last_print_ms = now_ms

            img.draw_cross(MANUAL_SPOT_X, MANUAL_SPOT_Y, color=0x07E0, size=10, thickness=1)

            if frame_count % DISPLAY_INTERVAL == 0:
                show(img)
            frame_count += 1
            img = None
            blobs = None
            if frame_count % 20 == 0:
                gc.collect()
    except KeyboardInterrupt:
        pass
    except BaseException as exc:
        import sys
        sys.print_exception(exc)
    finally:
        deinit()


if __name__ == "__main__":
    main()
