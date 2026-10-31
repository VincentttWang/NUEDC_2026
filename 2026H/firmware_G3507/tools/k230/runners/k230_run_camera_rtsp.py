"""Run the uploaded camera-to-RTSP service in the K230 foreground."""

path = "/sdcard/rtsp_camera_stream.py"
with open(path, "r") as source:
    code = compile(source.read(), path, "exec")
exec(code, {"__name__": "__main__"})
