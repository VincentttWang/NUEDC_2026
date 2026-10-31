#!/usr/bin/env python3
"""向 MSPM0 的 Rpi_UART/UART2 发送 11 字节滚球视觉测试帧。"""

from __future__ import annotations

import argparse
import math
import struct
import time

import serial


SYNC = b"\xA5\x5A"
VERSION = 1
FLAG_VALID = 1 << 0
FLAG_MOVING = 1 << 1
FLAG_V_VALID = 1 << 2
FLAG_EDGE = 1 << 3


def crc8(data: bytes) -> int:
    """CRC-8/0x07，初值 0xFF，不反转，无最终异或。"""
    value = 0xFF
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = ((value << 1) ^ 0x07) & 0xFF if value & 0x80 else (value << 1) & 0xFF
    return value


def build_frame(seq: int, age_ms: int, x_01mm: int, velocity_mm_s: int, flags: int) -> bytes:
    """按小端格式构帧；CRC 只覆盖 VER..FLAGS 共 8 字节。"""
    payload = struct.pack(
        "<BBBhhB",
        VERSION,
        seq & 0xFF,
        max(0, min(255, age_ms)),
        max(-32768, min(32767, x_01mm)),
        max(-32768, min(32767, velocity_mm_s)),
        flags & 0x3F,
    )
    return SYNC + payload + bytes((crc8(payload),))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Rpi_UART 11 字节协议通信测试发送端")
    parser.add_argument("port", help="树莓派串口，例如 /dev/serial0")
    parser.add_argument("--fps", type=float, default=27.0, help="发送帧率，默认 27")
    parser.add_argument("--amplitude-mm", type=float, default=80.0, help="正弦位置振幅，默认 80 mm")
    parser.add_argument("--period-s", type=float, default=4.0, help="正弦运动周期，默认 4 s")
    parser.add_argument("--quality", type=int, choices=range(4), default=3, help="QUALITY 0..3")
    parser.add_argument("--invalid-every", type=int, default=0, help="每 N 帧发一帧 VALID=0")
    parser.add_argument("--corrupt-every", type=int, default=0, help="每 N 帧故意破坏 CRC")
    parser.add_argument("--gap-every", type=int, default=0, help="每 N 帧额外跳过一个 SEQ")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.fps <= 0.0 or args.period_s <= 0.0:
        raise SystemExit("--fps 和 --period-s 必须大于 0")

    period = 1.0 / args.fps
    omega = 2.0 * math.pi / args.period_s
    seq = 0
    frame_count = 0
    next_deadline = time.monotonic()
    start = next_deadline

    with serial.Serial(args.port, 115200, bytesize=8, parity="N", stopbits=1, timeout=0) as uart:
        print(f"发送到 {args.port}: 115200 8N1, {args.fps:.1f} fps；Ctrl-C 停止")
        try:
            while True:
                capture_time = time.monotonic()
                phase = omega * (capture_time - start)
                x_mm = args.amplitude_mm * math.sin(phase)
                velocity_mm_s = args.amplitude_mm * omega * math.cos(phase)

                frame_count += 1
                if args.gap_every and frame_count % args.gap_every == 0:
                    seq = (seq + 1) & 0xFF

                flags = FLAG_VALID | FLAG_MOVING | FLAG_V_VALID | (args.quality << 4)
                if args.invalid_every and frame_count % args.invalid_every == 0:
                    flags &= ~FLAG_VALID

                # AGE 必须紧邻整帧 write() 前计算，不能提前缓存帧。
                age_ms = round((time.monotonic() - capture_time) * 1000.0)
                frame = build_frame(seq, age_ms, round(x_mm * 10.0), round(velocity_mm_s), flags)
                if args.corrupt_every and frame_count % args.corrupt_every == 0:
                    frame = frame[:-1] + bytes((frame[-1] ^ 0x01,))
                uart.write(frame)  # 单次 write() 写完整个 11 字节帧。

                seq = (seq + 1) & 0xFF
                next_deadline += period
                delay = next_deadline - time.monotonic()
                if delay > 0.0:
                    time.sleep(delay)
                else:
                    # 发送端落后时不追帧，重新以当前时刻建立节拍。
                    next_deadline = time.monotonic()
        except KeyboardInterrupt:
            print("\n已停止")


if __name__ == "__main__":
    main()
