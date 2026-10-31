#!/usr/bin/env python3
"""Small serial helper for CanMV K230 MicroPython scripts.

The CanMV IDE K230 installation on this machine exposes examples and docs, but
not a standalone command line uploader.  This tool uses the standard
MicroPython raw REPL protocol through pyserial, so it is useful when the K230
USB serial port is available as a COM port.
"""

from __future__ import annotations

import argparse
import base64
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
        chunk = ser.read(max(1, min(4096, ser.in_waiting)))
        if chunk:
            data += chunk
            if expected in data:
                return bytes(data)
        else:
            time.sleep(0.01)

    raise K230ReplError(f"Timed out waiting for {expected!r}; got {bytes(data)!r}")


def enter_raw_repl(ser: serial.Serial) -> None:
    ser.reset_input_buffer()
    stopped = False
    for _ in range(6):
        ser.write(CTRL_C)
        try:
            read_until(ser, b">>>", 1.5)
            stopped = True
            break
        except K230ReplError:
            continue

    # Some media loops stop on Ctrl-C but do not emit a visible friendly-REPL
    # prompt. Trying Ctrl-A is still safe and matches the proven camera probe.
    if not stopped:
        ser.reset_input_buffer()
    ser.write(CTRL_A)
    marker = b"raw REPL; CTRL-B to exit"
    banner = read_until(ser, marker, 2.0)
    if b">" not in banner.split(marker, 1)[1]:
        read_until(ser, b">", 2.0)


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
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baudrate
    ser.timeout = timeout_s
    ser.write_timeout = timeout_s
    ser.dtr = True
    ser.rts = False
    ser.open()
    ser.dtr = True
    return ser


def run_script(port: str, baudrate: int, script: Path, stream: bool = False) -> int:
    code = script.read_text(encoding="utf-8")
    with connect(port, baudrate, 0.2) as ser:
        enter_raw_repl(ser)
        try:
            if stream:
                if not code.endswith("\n"):
                    code += "\n"
                ser.write(code.encode("utf-8"))
                ser.write(CTRL_D)
                acknowledgement = read_until(ser, b"OK", 2.0)
                pending = acknowledgement.split(b"OK", 1)[1]
                eof_markers = 0

                def emit(chunk: bytes) -> bool:
                    nonlocal eof_markers
                    visible = bytearray()
                    for byte in chunk:
                        if byte == CTRL_D[0]:
                            eof_markers += 1
                            if eof_markers >= 2:
                                break
                        else:
                            visible.append(byte)
                    if visible:
                        sys.stdout.buffer.write(visible)
                        sys.stdout.buffer.flush()
                    return eof_markers >= 2

                if pending and emit(pending):
                    return 0
                try:
                    while True:
                        chunk = ser.read(max(1, min(4096, ser.in_waiting)))
                        if chunk:
                            if emit(chunk):
                                return 0
                        else:
                            time.sleep(0.01)
                except KeyboardInterrupt:
                    ser.write(CTRL_C)
                    time.sleep(0.2)
                return 0

            output = raw_exec(ser, code, timeout_s=60.0)
            if output:
                sys.stdout.buffer.write(output)
        finally:
            exit_raw_repl(ser)
    return 0


def write_file(port: str, baudrate: int, local: Path, remote: str) -> int:
    content = local.read_bytes()
    with connect(port, baudrate, 0.2) as ser:
        enter_raw_repl(ser)
        try:
            raw_exec(
                ser,
                "try:\n    f.close()\nexcept:\n    pass\n"
                f"import binascii\nf = open({remote!r}, 'wb')",
                timeout_s=5.0,
            )
            for offset in range(0, len(content), 1024):
                chunk = content[offset:offset + 1024]
                encoded = base64.b64encode(chunk)
                code = f"f.write(binascii.a2b_base64({encoded!r}))"
                raw_exec(ser, code, timeout_s=10.0)
            output = raw_exec(
                ser,
                f"f.close()\nimport os\nprint('wrote {remote}', os.stat({remote!r})[6])",
                timeout_s=5.0,
            )
            if output:
                sys.stdout.buffer.write(output)
        finally:
            exit_raw_repl(ser)
    return 0


def print_file(port: str, baudrate: int, remote: str) -> int:
    """Print a remote text file without modifying the K230 filesystem."""
    with connect(port, baudrate, 0.2) as ser:
        enter_raw_repl(ser)
        try:
            output = raw_exec(
                ser,
                f"with open({remote!r}, 'r') as source:\n"
                "    while True:\n"
                "        chunk = source.read(1024)\n"
                "        if not chunk:\n"
                "            break\n"
                "        print(chunk, end='')",
                timeout_s=15.0,
            )
            if output:
                sys.stdout.buffer.write(output)
        finally:
            exit_raw_repl(ser)
    return 0


def copy_file(port: str, baudrate: int, source: str, destination: str) -> int:
    """Copy a file on the K230 filesystem in bounded chunks."""
    with connect(port, baudrate, 0.2) as ser:
        enter_raw_repl(ser)
        try:
            output = raw_exec(
                ser,
                f"source = open({source!r}, 'rb')\n"
                f"destination = open({destination!r}, 'wb')\n"
                "while True:\n"
                "    chunk = source.read(1024)\n"
                "    if not chunk:\n"
                "        break\n"
                "    destination.write(chunk)\n"
                "source.close()\n"
                "destination.close()\n"
                f"import os\nprint('copied', {destination!r}, os.stat({destination!r})[6])",
                timeout_s=15.0,
            )
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


def hard_reset(port: str, baudrate: int) -> int:
    """Ask the K230 to perform a full SoC reset."""
    with connect(port, baudrate, 0.2) as ser:
        enter_raw_repl(ser)
        ser.write(b"import machine\nmachine.reset()\n")
        ser.write(CTRL_D)
        time.sleep(1.0)
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="CanMV K230 serial REPL helper")
    parser.add_argument("--port", help="Serial port, for example COM15")
    parser.add_argument("--baudrate", type=int, default=115200)

    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("list", help="List serial ports")

    run = sub.add_parser("run", help="Run a local script through raw REPL")
    run.add_argument("script", type=Path)
    run.add_argument(
        "--stream",
        action="store_true",
        help="Keep the REPL attached and stream output until Ctrl-C",
    )

    put = sub.add_parser("put", help="Write a local file to the device filesystem")
    put.add_argument("local", type=Path)
    put.add_argument("remote", nargs="?", default="/sdcard/main.py")

    cat = sub.add_parser("cat", help="Print a remote text file without changing it")
    cat.add_argument("remote")

    copy = sub.add_parser("copy", help="Copy a file within the device filesystem")
    copy.add_argument("source")
    copy.add_argument("destination")

    sub.add_parser("reset", help="Send Ctrl-D soft reset")
    sub.add_parser("hard-reset", help="Reset the K230 SoC with machine.reset()")
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
            return run_script(args.port, args.baudrate, args.script, args.stream)
        if args.command == "put":
            return write_file(args.port, args.baudrate, args.local, args.remote)
        if args.command == "cat":
            return print_file(args.port, args.baudrate, args.remote)
        if args.command == "copy":
            return copy_file(args.port, args.baudrate, args.source, args.destination)
        if args.command == "reset":
            return soft_reset(args.port, args.baudrate)
        if args.command == "hard-reset":
            return hard_reset(args.port, args.baudrate)
    except (OSError, serial.SerialException, K230ReplError) as exc:
        print(f"k230_tool: {exc}", file=sys.stderr)
        return 1

    return 2


if __name__ == "__main__":
    raise SystemExit(main())
