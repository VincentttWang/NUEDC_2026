#!/usr/bin/env python3
"""MSPM0 UART2 赛道调参协议的树莓派命令行参考实现。"""

from __future__ import annotations

import argparse
import struct
import time

try:
    import serial
except ModuleNotFoundError:
    serial = None


SYNC = b"\xA5\x5A"
REQUEST_TYPE = 0x81
RESPONSE_TYPE = 0x82
OP_GET = 0x01
OP_SET = 0x02
OP_GET_ALL = 0x03
OP_RESET_DEFAULTS = 0x04
STATUS_NAMES = {0: "OK", 1: "UNKNOWN_ID", 2: "BUSY", 3: "BAD_OP"}
LIST_END = 0xFFFF

PARAMS = {
    0x0101: ("s1_end_mm", 1, 1300, 1700),
    0x0102: ("s2_end_mm", 1, 2750, 3400),
    0x0103: ("s3_end_mm", 1, 4200, 5000),
    0x0104: ("s4_heading_end_mm", 1, 5600, 6800),
    0x0105: ("s3_gyro_recover_mm", 1, 0, 600),
    0x0110: ("lap_stop_mm", 2, None, None),
    0x0111: ("finish_arm_margin_mm", 2, None, None),
    0x0112: ("loaded_decel_warning_mm", 2, None, None),
    0x0113: ("loaded_odom_arrival_mm", 2, None, None),
    0x0114: ("h2_odom_fallback_mm", 2, None, None),
    0x0120: ("h2_s2_ff_x1000", 3, 850, 1300),
    0x0121: ("loaded_s2_ff_x1000", 3, 850, 1300),
    0x0122: ("s4_ff_x1000", 3, 850, 1300),
}


def crc8(data: bytes) -> int:
    value = 0xFF
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = ((value << 1) ^ 0x07) & 0xFF if value & 0x80 else (value << 1) & 0xFF
    return value


def build_request(seq: int, op: int, param_id: int = 0, value: int = 0) -> bytes:
    payload = struct.pack("<BBBHHB", REQUEST_TYPE, seq & 0xFF, op, param_id, value, 0)
    return SYNC + payload + bytes((crc8(payload),))


def read_response(uart: object, expected_seq: int) -> tuple[int, int, int, int]:
    deadline = time.monotonic() + uart.timeout
    matched = 0
    while time.monotonic() < deadline:
        byte = uart.read(1)
        if not byte:
            continue
        if matched == 0:
            matched = 1 if byte == SYNC[:1] else 0
            continue
        if byte != SYNC[1:2]:
            matched = 1 if byte == SYNC[:1] else 0
            continue
        tail = uart.read(9)
        if len(tail) != 9:
            break
        payload, received_crc = tail[:8], tail[8]
        if crc8(payload) != received_crc:
            matched = 0
            continue
        kind, seq, status, param_id, value, priority = struct.unpack("<BBBHHB", payload)
        if kind == RESPONSE_TYPE and seq == expected_seq:
            return status, param_id, value, priority
        matched = 0
    raise TimeoutError("等待 MSPM0 调参响应超时")


def print_response(status: int, param_id: int, value: int, priority: int) -> None:
    status_name = STATUS_NAMES.get(status, f"UNKNOWN_STATUS_{status}")
    name = PARAMS.get(param_id, ("LIST_END" if param_id == LIST_END else "unknown",))[0]
    print(f"{status_name:10s} id=0x{param_id:04X} {name:24s} value={value} priority={priority}")


def parse_u16(text: str) -> int:
    value = int(text, 0)
    if not 0 <= value <= 0xFFFF:
        raise argparse.ArgumentTypeError("必须是 0..65535 的整数")
    return value


def main() -> None:
    parser = argparse.ArgumentParser(description="MSPM0 H2/H5/H6 赛道参数调试")
    parser.add_argument("port", help="树莓派串口，例如 /dev/serial0")
    parser.add_argument("--timeout", type=float, default=0.8, help="响应超时秒数")
    sub = parser.add_subparsers(dest="command", required=True)
    get_parser = sub.add_parser("get")
    get_parser.add_argument("param_id", type=parse_u16)
    set_parser = sub.add_parser("set")
    set_parser.add_argument("param_id", type=parse_u16)
    set_parser.add_argument("value", type=parse_u16)
    sub.add_parser("get-all")
    sub.add_parser("reset")
    args = parser.parse_args()

    if serial is None:
        raise SystemExit("缺少 pyserial，请先执行: python3 -m pip install pyserial")

    op = {"get": OP_GET, "set": OP_SET, "get-all": OP_GET_ALL,
          "reset": OP_RESET_DEFAULTS}[args.command]
    param_id = getattr(args, "param_id", 0)
    value = getattr(args, "value", 0)
    if args.command == "set" and param_id in PARAMS:
        name, _, low, high = PARAMS[param_id]
        if low is not None and not low <= value <= high:
            print(f"警告: {name}={value} 超出建议范围 {low}..{high}")

    seq = int(time.monotonic() * 1000.0) & 0xFF
    with serial.Serial(args.port, 115200, bytesize=8, parity="N", stopbits=1,
                       timeout=args.timeout) as uart:
        uart.reset_input_buffer()
        uart.write(build_request(seq, op, param_id, value))
        while True:
            status, response_id, response_value, priority = read_response(uart, seq)
            print_response(status, response_id, response_value, priority)
            if args.command != "get-all" or response_id == LIST_END or status != 0:
                break


if __name__ == "__main__":
    main()
