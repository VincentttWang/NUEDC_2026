#!/usr/bin/env python3
"""Small serial helper for CanMV K230 MicroPython scripts.

The CanMV IDE K230 installation on this machine exposes examples and docs, but
not a standalone command line uploader.  This tool uses the standard
MicroPython raw REPL protocol through pyserial, so it is useful when the K230
USB serial port is available as a COM port.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import serial
from serial.tools import list_ports


CTRL_A = b"\x01"
CTRL_B = b"\x02"
CTRL_C = b"\x03"
CTRL_D = b"\x04"


class K230ReplError(RuntimeError):
    pass


def list_serial_ports() -> int:
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return 1

    for port in ports:
        desc = port.description or ""
        hwid = port.hwid or ""
        print(f"{port.device}\t{desc}\t{hwid}")
    return 0


def read_until(ser: serial.Serial, expected: bytes, timeout_s: float) -> bytes:
    deadline = time.monotonic() + timeout_s
    data = bytearray()

    while time.monotonic() < deadline:
        chunk = ser.read(1)
        if chunk:
            data += chunk
            if expected in data:
                return bytes(data)
        else:
            time.sleep(0.01)

    raise K230ReplError(f"Timed out waiting for {expected!r}; got {bytes(data)!r}")


def enter_raw_repl(ser: serial.Serial) -> None:
    ser.reset_input_buffer()
    ser.write(CTRL_C)
    ser.write(CTRL_C)
    time.sleep(0.15)
    ser.reset_input_buffer()
    ser.write(CTRL_A)
    read_until(ser, b"raw REPL; CTRL-B to exit", 2.0)
    ser.write(CTRL_D)
    read_until(ser, b"raw REPL; CTRL-B to exit", 2.0)


def exit_raw_repl(ser: serial.Serial) -> None:
    ser.write(CTRL_B)


def raw_exec(ser: serial.Serial, code: str, timeout_s: float = 10.0) -> bytes:
    if not code.endswith("\n"):
        code += "\n"

    ser.write(code.encode("utf-8"))
    ser.write(CTRL_D)
    read_until(ser, b"OK", 2.0)
    output = read_until(ser, b"\x04", timeout_s)
    error = read_until(ser, b"\x04", 2.0)

    stdout = output.split(b"\x04", 1)[0]
    stderr = error.split(b"\x04", 1)[0]
    if stderr:
        raise K230ReplError(stderr.decode("utf-8", errors="replace"))

    return stdout


def connect(port: str, baudrate: int, timeout_s: float) -> serial.Serial:
    return serial.Serial(port=port, baudrate=baudrate, timeout=timeout_s, write_timeout=timeout_s)


def run_script(port: str, baudrate: int, script: Path) -> int:
    code = script.read_text(encoding="utf-8")
    with connect(port, baudrate, 0.2) as ser:
        enter_raw_repl(ser)
        try:
            output = raw_exec(ser, code, timeout_s=60.0)
            if output:
                sys.stdout.buffer.write(output)
        finally:
            exit_raw_repl(ser)
    return 0


def write_file(port: str, baudrate: int, local: Path, remote: str) -> int:
    content = local.read_bytes()
    code = (
        "data = " + repr(content) + "\n"
        f"with open({remote!r}, 'wb') as f:\n"
        "    f.write(data)\n"
        f"print('wrote {remote}', len(data))\n"
    )
    with connect(port, baudrate, 0.2) as ser:
        enter_raw_repl(ser)
        try:
            output = raw_exec(ser, code, timeout_s=15.0)
            if output:
                sys.stdout.buffer.write(output)
        finally:
            exit_raw_repl(ser)
    return 0


def soft_reset(port: str, baudrate: int) -> int:
    with connect(port, baudrate, 0.2) as ser:
        ser.write(CTRL_C)
        ser.write(CTRL_C)
        time.sleep(0.1)
        ser.write(CTRL_D)
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="CanMV K230 serial REPL helper")
    parser.add_argument("--port", help="Serial port, for example COM15")
    parser.add_argument("--baudrate", type=int, default=115200)

    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("list", help="List serial ports")

    run = sub.add_parser("run", help="Run a local script through raw REPL")
    run.add_argument("script", type=Path)

    put = sub.add_parser("put", help="Write a local file to the device filesystem")
    put.add_argument("local", type=Path)
    put.add_argument("remote", nargs="?", default="/sdcard/main.py")

    sub.add_parser("reset", help="Send Ctrl-D soft reset")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    if args.command == "list":
        return list_serial_ports()

    if not args.port:
        print("--port is required for this command", file=sys.stderr)
        return 2

    try:
        if args.command == "run":
            return run_script(args.port, args.baudrate, args.script)
        if args.command == "put":
            return write_file(args.port, args.baudrate, args.local, args.remote)
        if args.command == "reset":
            return soft_reset(args.port, args.baudrate)
    except (OSError, serial.SerialException, K230ReplError) as exc:
        print(f"k230_tool: {exc}", file=sys.stderr)
        return 1

    return 2


if __name__ == "__main__":
    raise SystemExit(main())
