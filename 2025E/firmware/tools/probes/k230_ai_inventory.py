"""Read-only inventory of AI models and demo helpers on a CanMV K230."""

import os


def walk(path, depth=0, max_depth=4):
    try:
        names = sorted(os.listdir(path))
    except Exception as exc:
        print("LIST_ERROR", path, exc)
        return
    for name in names:
        child = path.rstrip("/") + "/" + name
        try:
            stat = os.stat(child)
            is_dir = bool(stat[0] & 0x4000)
            size = stat[6]
        except Exception:
            is_dir = False
            size = -1
        print("DIR" if is_dir else "FILE", child, size)
        if is_dir and depth < max_depth:
            walk(child, depth + 1, max_depth)


for module_name in ("aidemo", "nncase_runtime", "ulab.numpy"):
    try:
        module = __import__(module_name)
        print("MODULE", module_name, "OK", dir(module))
    except Exception as exc:
        print("MODULE", module_name, "ERROR", exc)

for root in ("/sdcard/res", "/sdcard/examples/kmodel", "/sdcard/examples/05-AI-Demo", "/sdcard/libs"):
    print("ROOT", root)
    walk(root)

print("AI_INVENTORY_END")
