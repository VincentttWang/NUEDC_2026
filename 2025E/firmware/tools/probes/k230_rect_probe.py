"""Finite on-device probe for raw find_rects candidates."""

import gc
import os
import time

from media.media import *
from media.sensor import *


sensor = None
try:
    sensor = Sensor(width=320, height=240)
    sensor.reset()
    sensor.set_framesize(width=320, height=240)
    sensor.set_pixformat(Sensor.GRAYSCALE)
    MediaManager.init()
    sensor.run()
    for _ in range(15):
        image = sensor.snapshot()

    for frame_index in range(20):
        image = sensor.snapshot()
        start = time.ticks_us()
        rects = image.find_rects(threshold=10000)
        elapsed_ms = time.ticks_diff(time.ticks_us(), start) / 1000.0
        strong = [rect for rect in rects if rect.magnitude() >= 40000]
        print("FRAME", frame_index, "MS", elapsed_ms, "COUNT", len(rects), "STRONG", len(strong))
        for index, rect in enumerate(rects):
            if rect.magnitude() >= 40000:
                print("RECT", index, rect.rect(), rect.magnitude(), rect.corners())
        gc.collect()
finally:
    try:
        if sensor:
            sensor.stop()
    except Exception:
        pass
    try:
        MediaManager.deinit()
    except Exception:
        pass
    print("RECT_PROBE_END")
