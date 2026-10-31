"""Read-only capability probe for the connected CanMV K230 firmware."""
import os
import sys


print("CAP uname=%s" % (os.uname(),))
print("CAP implementation=%s" % (sys.implementation,))
print("CAP os_exec_names=%s" %
      ([name for name in dir(os) if "exec" in name.lower()],))
try:
    print("CAP root_files=%s" % (os.listdir("/"),))
    print("CAP sdcard_files=%s" % (os.listdir("/sdcard"),))
except BaseException as exc:
    print("CAP listdir_error=%r" % (exc,))

try:
    import cv2
    names = dir(cv2)
    wanted = ("cvtColor", "inRange", "threshold", "findContours",
              "connectedComponentsWithStats", "fitLine", "HoughLinesP",
              "moments", "minAreaRect", "boundingRect", "contourArea",
              "resize", "erode", "dilate", "morphologyEx", "split",
              "bitwise_and", "bitwise_or", "subtract", "RETR_EXTERNAL",
              "CHAIN_APPROX_SIMPLE", "COLOR_RGB2HSV", "COLOR_RGB2LAB")
    print("CAP cv2=1")
    for name in wanted:
        print("CAP cv2.%s=%d" % (name, name in names))
except BaseException as exc:
    print("CAP cv2=0 error=%r" % (exc,))

try:
    import cv_lite
    names = dir(cv_lite)
    print("CAP cv_lite=1 names=%s" %
          ([name for name in names if not name.startswith("__")],))
except BaseException as exc:
    print("CAP cv_lite=0 error=%r" % (exc,))

try:
    from media.sensor import Sensor
    print("CAP sensor_modes=%s" % (Sensor.list_mode(),))
except BaseException as exc:
    print("CAP sensor_modes_error=%r" % (exc,))
