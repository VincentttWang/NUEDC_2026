"""Inspect HDMI display support and search common on-device example paths."""

import os

import media.display as display_module
from media.display import Display


print("HDMI_INVENTORY_BEGIN")
print("DISPLAY_MODULE", getattr(display_module, "__file__", None))
print("DISPLAY_FIELDS", dir(Display))

for name in dir(Display):
    upper_name = name.upper()
    if "HDMI" in upper_name or "LT9611" in upper_name:
        try:
            print("DISPLAY_HDMI_FIELD", name, getattr(Display, name))
        except BaseException as error:
            print("DISPLAY_HDMI_FIELD_ERROR", name, repr(error))


scan_budget = 300


def scan(root, depth=0):
    global scan_budget
    if depth > 3 or scan_budget <= 0:
        return
    try:
        entries = os.listdir(root)
    except BaseException:
        return
    for entry in entries:
        if scan_budget <= 0:
            return
        scan_budget -= 1
        path = root.rstrip("/") + "/" + entry
        lower_path = path.lower()
        if "hdmi" in lower_path or "lt9611" in lower_path:
            print("HDMI_PATH", path)
        try:
            mode = os.stat(path)[0]
        except BaseException:
            continue
        if mode & 0x4000:
            scan(path, depth + 1)


for search_root in ("/sdcard", "/usr", "/data", "/root"):
    scan(search_root)

print("SCAN_REMAINING", scan_budget)
print("HDMI_INVENTORY_END")
