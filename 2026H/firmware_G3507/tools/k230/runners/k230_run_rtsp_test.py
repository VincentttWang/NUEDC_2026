"""Run the uploaded WBC RTSP test in the foreground on the K230."""

path = "/sdcard/rtsp_test.py"
with open(path, "r") as source:
    code = compile(source.read(), path, "exec")
exec(code, {"__name__": "__main__"})
