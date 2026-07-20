#!/usr/bin/env python3
"""Collect RGB target images with CV-generated YOLO labels over raw REPL."""

from __future__ import annotations

import base64
import re
import sys
import time
from pathlib import Path

import serial


CTRL_A = b"\x01"
CTRL_B = b"\x02"
CTRL_C = b"\x03"
CTRL_D = b"\x04"
PORT = sys.argv[1] if len(sys.argv) > 1 else "COM15"
ROOT = Path(__file__).resolve().parents[1] / "ai" / "target_dataset"
SAMPLE_COUNT = 80


DEVICE_CODE = r'''
import gc, time, ubinascii
from media.sensor import *
from media.media import *

W=320; H=240; COUNT=80
s=None
try:
    s=Sensor(width=W,height=H); s.reset()
    s.set_framesize(width=W,height=H,chn=CAM_CHN_ID_0)
    s.set_pixformat(Sensor.GRAYSCALE,chn=CAM_CHN_ID_0)
    s.set_framesize(width=W,height=H,chn=CAM_CHN_ID_2)
    s.set_pixformat(Sensor.RGB565,chn=CAM_CHN_ID_2)
    MediaManager.init(); s.run()
    for _ in range(20):
        s.snapshot(chn=CAM_CHN_ID_0)
    saved=0
    attempts=0
    while saved<COUNT and attempts<COUNT*5:
        attempts+=1
        gray=s.snapshot(chn=CAM_CHN_ID_0)
        rects=gray.find_rects(threshold=10000)
        best=None; best_mag=0
        for r in rects:
            x,y,w,h=r.rect(); mag=r.magnitude()
            if (mag>=80000 and w*h>=2000 and x>=3 and y>=3 and
                    x+w<W-3 and y+h<H-3 and 0.5<=w/h<=2.0 and mag>best_mag):
                best=r; best_mag=mag
        if best is None:
            continue
        rgb=s.snapshot(chn=CAM_CHN_ID_2)
        jpg=rgb.to_jpeg(quality=72).bytearray()
        x,y,w,h=best.rect()
        print("SAMPLE %d %d %d %d %d %d"%(saved,x,y,w,h,len(jpg)))
        print("B64_BEGIN")
        print(ubinascii.b2a_base64(jpg).decode().strip())
        print("B64_END")
        saved+=1
        gc.collect()
    print("COLLECT_DONE",saved,attempts)
finally:
    try:
        if s: s.stop()
    except Exception: pass
    try: MediaManager.deinit()
    except Exception: pass
    print("COLLECT_END")
'''


def read_until(ser: serial.Serial, markers: tuple[bytes, ...], timeout: float) -> bytes:
    deadline = time.monotonic() + timeout
    data = bytearray()
    while time.monotonic() < deadline:
        chunk = ser.read(max(1, min(4096, ser.in_waiting)))
        if chunk:
            data.extend(chunk)
            if any(marker in data for marker in markers):
                return bytes(data)
        else:
            time.sleep(0.005)
    return bytes(data)


def enter_raw(ser: serial.Serial) -> None:
    for _ in range(6):
        ser.write(CTRL_C)
        if b">>>" in read_until(ser, (b">>>",), 1.5):
            break
    ser.reset_input_buffer()
    ser.write(CTRL_A)
    if b"raw REPL" not in read_until(ser, (b"raw REPL; CTRL-B to exit",), 3.0):
        raise RuntimeError("failed to enter raw REPL")


def save_samples(output: str) -> int:
    saved = 0
    pattern = re.compile(
        r"SAMPLE (\d+) (\d+) (\d+) (\d+) (\d+) (\d+)\s+"
        r"B64_BEGIN\s+(.*?)\s+B64_END",
        re.S,
    )
    for match in pattern.finditer(output):
        index, x, y, width, height, expected_size = map(int, match.group(1, 2, 3, 4, 5, 6))
        image_data = base64.b64decode("".join(match.group(7).split()))
        if len(image_data) != expected_size:
            print("skip truncated sample", index, len(image_data), expected_size)
            continue
        split = "val" if index % 5 == 0 else "train"
        image_dir = ROOT / "images" / split
        label_dir = ROOT / "labels" / split
        image_dir.mkdir(parents=True, exist_ok=True)
        label_dir.mkdir(parents=True, exist_ok=True)
        stem = f"live_{index:04d}"
        (image_dir / f"{stem}.jpg").write_bytes(image_data)
        center_x = (x + width / 2.0) / 320.0
        center_y = (y + height / 2.0) / 240.0
        norm_width = width / 320.0
        norm_height = height / 240.0
        (label_dir / f"{stem}.txt").write_text(
            f"0 {center_x:.6f} {center_y:.6f} {norm_width:.6f} {norm_height:.6f}\n",
            encoding="ascii",
        )
        saved += 1
    return saved


def main() -> int:
    ser = serial.Serial()
    ser.port = PORT
    ser.baudrate = 115200
    ser.timeout = 0.2
    ser.write_timeout = 5.0
    ser.dtr = True
    ser.rts = False
    ser.open()
    try:
        enter_raw(ser)
        ser.write(DEVICE_CODE.encode("utf-8"))
        ser.write(CTRL_D)
        output = read_until(ser, (b"COLLECT_END", b"Traceback"), 240.0).decode("utf-8", "replace")
        if "Traceback" in output:
            print(output)
            return 1
        saved = save_samples(output)
        print("saved", saved, "samples to", ROOT)
        done = re.search(r"COLLECT_DONE\s+(\d+)\s+(\d+)", output)
        if done:
            print("device collected", done.group(1), "samples in", done.group(2), "attempts")
        return 0 if saved else 1
    finally:
        ser.write(CTRL_B)
        ser.close()


if __name__ == "__main__":
    raise SystemExit(main())
