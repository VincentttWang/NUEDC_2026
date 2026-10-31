"""Check whether the K230 firmware contains standard MicroPython WebREPL."""

for module_name in ("webrepl", "webrepl_setup", "_webrepl", "websocket"):
    try:
        module = __import__(module_name)
        print("WEBREPL_MODULE", module_name, "OK", dir(module))
    except BaseException as error:
        print("WEBREPL_MODULE", module_name, "MISSING", repr(error))
