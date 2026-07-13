"""Read-only CanMV K230 runtime and filesystem status probe."""

import gc
import os
import sys


def safe_call(label, callback):
    try:
        print("%s=%s" % (label, callback()))
    except Exception as exc:
        print("%s=<ERROR %s>" % (label, exc))


print("K230_STATUS_BEGIN")
safe_call("IMPLEMENTATION", lambda: sys.implementation)
safe_call("UNAME", lambda: os.uname())
safe_call("CWD", lambda: os.getcwd())
safe_call("MEM_FREE", lambda: gc.mem_free())
safe_call("MEM_ALLOC", lambda: gc.mem_alloc())
safe_call("SYS_PATH", lambda: sys.path)
safe_call("LOADED_MODULES", lambda: sorted(sys.modules.keys()))

interesting_globals = (
    "sensor",
    "tracking_state",
    "search_roi",
    "fps_val",
    "uart2",
    "gpio2_pin",
)
for name in interesting_globals:
    value = globals().get(name, "<ABSENT>")
    print("GLOBAL_%s=%s" % (name.upper(), value))

for path in ("/", "/sdcard"):
    safe_call("LIST_%s" % path.replace("/", "ROOT"), lambda path=path: sorted(os.listdir(path)))

for path in ("/sdcard/main.py", "/sdcard/boot.py"):
    safe_call("STAT_%s" % path.replace("/", "_"), lambda path=path: os.stat(path))
    try:
        with open(path, "r") as source:
            preview = source.read(240).replace("\r", "\\r").replace("\n", "\\n")
        print("PREVIEW_%s=%s" % (path.replace("/", "_"), preview))
    except Exception as exc:
        print("PREVIEW_%s=<ERROR %s>" % (path.replace("/", "_"), exc))

for module_name in ("image", "cv_lite", "media.sensor", "nncase_runtime"):
    try:
        __import__(module_name)
        print("IMPORT_%s=OK" % module_name)
    except Exception as exc:
        print("IMPORT_%s=<ERROR %s>" % (module_name, exc))

try:
    import nncase_runtime as nn
    print("NNCASE_RUNTIME_VERSION=%s" % nn.version())
except Exception as exc:
    print("NNCASE_RUNTIME_VERSION=<ERROR %s>" % exc)

print("K230_STATUS_END")
