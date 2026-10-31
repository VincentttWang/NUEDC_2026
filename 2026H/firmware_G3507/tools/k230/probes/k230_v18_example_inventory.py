"""Locate v1.8 on-device LCD/sensor/PipeLine examples."""
import os


def walk(path, depth=0):
    if depth > 5:
        return
    try:
        names = os.listdir(path)
    except Exception:
        return
    for name in names:
        child = path + "/" + name
        try:
            mode = os.stat(child)[0]
        except Exception:
            continue
        if mode & 0x4000:
            walk(child, depth + 1)
            continue
        if not name.endswith(".py"):
            continue
        try:
            with open(child, "r") as stream:
                text = stream.read()
        except Exception:
            continue
        if ("Display.ST7701" in text or "Display.LT9611" in text or
                "Display.bind_layer" in text or
                "display_mode=\"lcd\"" in text or "display_mode='lcd'" in text):
            print("EXAMPLE %s lt9611=%d sensor=%d bind=%d" % (
                child,
                "Display.LT9611" in text,
                "Sensor(" in text,
                "Display.bind_layer" in text,
            ))


walk("/sdcard/examples")
walk("/sdcard/libs")
