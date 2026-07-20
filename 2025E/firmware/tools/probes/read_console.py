#!/usr/bin/env python3
"""只读监听 K230 串口 N 秒, 打印设备 stdout (确认 main.py 持续运行/遥测)。用法:
   python _read_console.py COM15 [seconds]"""
import sys, time
import serial

port = sys.argv[1] if len(sys.argv) > 1 else "COM15"
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0

ser = serial.Serial()
ser.port = port
ser.baudrate = 115200
ser.timeout = 0.2
ser.dtr = True
ser.rts = False
ser.open()
ser.dtr = True

buf = bytearray()
t0 = time.monotonic()
try:
    while time.monotonic() - t0 < secs:
        chunk = ser.read(4096)
        if chunk:
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                sys.stdout.write(line.decode("utf-8", "replace").rstrip("\r") + "\n")
                sys.stdout.flush()
        else:
            time.sleep(0.02)
finally:
    ser.close()
    print("--- reader done (%.1fs) ---" % (time.monotonic() - t0))
