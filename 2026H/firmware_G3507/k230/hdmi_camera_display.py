"""Display the K230 camera on HDMI through the LT9611 video layer."""

import os
import sys
import time

from media.display import *
from media.media import *
from media.sensor import *


LOG_PATH = "/sdcard/hdmi_camera.log"
sensor = None


def stage(message):
    print(message)
    with open(LOG_PATH, "a") as log_file:
        log_file.write(message + "\n")


with open(LOG_PATH, "w") as log_file:
    log_file.write("HDMI_CAMERA_BEGIN\n")

os.exitpoint(os.EXITPOINT_ENABLE)

try:
    stage("SENSOR_CREATE")
    sensor = Sensor()
    sensor.reset()
    stage("SENSOR_RESET_OK")

    sensor.set_framesize(Sensor.FHD)
    sensor.set_pixformat(Sensor.YUV420SP)
    bind_info = sensor.bind_info()
    Display.bind_layer(**bind_info, layer=Display.LAYER_VIDEO1)
    stage("VIDEO_LAYER_BOUND")

    Display.init(Display.LT9611, to_ide=True)
    stage("HDMI_INIT_OK %dx%d" % (Display.width(), Display.height()))

    sensor.run()
    stage("CAMERA_RUNNING")

    heartbeat = time.ticks_ms()
    while True:
        os.exitpoint()
        time.sleep_ms(20)
        if time.ticks_diff(time.ticks_ms(), heartbeat) >= 5000:
            print("HDMI_CAMERA_ALIVE")
            heartbeat = time.ticks_ms()
except KeyboardInterrupt as error:
    print("HDMI_CAMERA_STOP", error)
except BaseException as error:
    sys.print_exception(error)
    try:
        stage("HDMI_CAMERA_ERROR %s" % repr(error))
    except BaseException:
        pass
finally:
    if isinstance(sensor, Sensor):
        sensor.stop()
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
