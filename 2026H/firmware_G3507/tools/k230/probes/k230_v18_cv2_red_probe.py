"""Finite cv2 red-mask/contour probe for CanMV K230 v1.8."""
import gc
import os
import time

import cv2
from media.media import MediaManager
from media.sensor import CAM_CHN_ID_1, Sensor


SENSOR_WIDTH = 1280
SENSOR_HEIGHT = 720
SENSOR_FPS = 90
IMAGE_WIDTH = 320
IMAGE_HEIGHT = 192

HSV_RED_LOW_1 = (0, 70, 45)
HSV_RED_HIGH_1 = (12, 255, 255)
HSV_RED_LOW_2 = (165, 70, 45)
HSV_RED_HIGH_2 = (179, 255, 255)


def main():
    os.exitpoint(os.EXITPOINT_ENABLE)
    sensor = Sensor(width=SENSOR_WIDTH, height=SENSOR_HEIGHT, fps=SENSOR_FPS)
    sensor.reset()
    sensor.set_framesize(width=IMAGE_WIDTH, height=IMAGE_HEIGHT,
                         chn=CAM_CHN_ID_1, crop=True)
    sensor.set_pixformat(Sensor.RGB888, chn=CAM_CHN_ID_1)
    MediaManager.init()
    sensor.run()
    try:
        for frame_count in range(20):
            begin_ms = time.ticks_ms()
            img = sensor.snapshot(chn=CAM_CHN_ID_1)
            capture_ms = time.ticks_diff(time.ticks_ms(), begin_ms)

            begin_ms = time.ticks_ms()
            rgb = img.to_numpy_ref()
            hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
            hsv_ms = time.ticks_diff(time.ticks_ms(), begin_ms)

            begin_ms = time.ticks_ms()
            mask1 = cv2.inRange(hsv, HSV_RED_LOW_1, HSV_RED_HIGH_1)
            mask2 = cv2.inRange(hsv, HSV_RED_LOW_2, HSV_RED_HIGH_2)
            mask = cv2.bitwise_or(mask1, mask2)
            mask_ms = time.ticks_diff(time.ticks_ms(), begin_ms)

            begin_ms = time.ticks_ms()
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                                    cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
            morph_ms = time.ticks_diff(time.ticks_ms(), begin_ms)

            begin_ms = time.ticks_ms()
            roi = mask[140:156, :]
            contour_result = cv2.findContours(
                roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = contour_result[0] if len(contour_result) == 2 else contour_result[1]
            contour_ms = time.ticks_diff(time.ticks_ms(), begin_ms)

            if frame_count == 0:
                print("CV2_PROBE rgb=%s hsv=%s mask=%s result_len=%d" %
                      (rgb.shape, hsv.shape, mask.shape, len(contour_result)))
            print("CV2_PROBE frame=%d capture=%d hsv=%d mask=%d morph=%d contour=%d n=%d" %
                  (frame_count, capture_ms, hsv_ms, mask_ms, morph_ms,
                   contour_ms, len(contours)))
            img = rgb = hsv = mask1 = mask2 = mask = roi = None
            if frame_count % 5 == 0:
                gc.collect()
    finally:
        try:
            sensor.stop()
        except Exception:
            pass
        try:
            MediaManager.deinit()
        except Exception:
            pass


main()
