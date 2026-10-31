#!/usr/bin/env python3
"""Upload, run, and monitor programs through the K230 Wi-Fi dev agent."""

from __future__ import annotations

import argparse
import socket
import sys
import time
from pathlib import Path


def read_line(client: socket.socket) -> str:
    data = bytearray()
    while True:
        byte = client.recv(1)
        if not byte:
            raise ConnectionError("connection closed before response")
        if byte == b"\n":
            return data.decode(errors="replace").rstrip("\r")
        data += byte


def control(host: str, port: int, command: bytes,
            payload: bytes = b"") -> str:
    with socket.create_connection((host, port), timeout=10) as client:
        banner = read_line(client)
        if not banner.startswith("K230DEV 1"):
            raise RuntimeError(f"unexpected server banner: {banner}")
        client.sendall(command + b"\n")
        if payload:
            client.sendall(payload)
        return read_line(client)


def put_file(host: str, port: int, local: Path, remote: str) -> None:
    content = local.read_bytes()
    response = control(
        host,
        port,
        f"PUT {remote} {len(content)}".encode(),
        content,
    )
    print(response)
    if not response.startswith("OK "):
        raise RuntimeError(response)


def follow_console(host: str, port: int, reconnect_delay: float) -> None:
    while True:
        try:
            with socket.create_connection((host, port), timeout=10) as client:
                client.settimeout(None)
                print(f"connected to tcp://{host}:{port}", file=sys.stderr)
                while True:
                    payload = client.recv(4096)
                    if not payload:
                        raise ConnectionError("console disconnected")
                    sys.stdout.buffer.write(payload)
                    sys.stdout.buffer.flush()
        except KeyboardInterrupt:
            return
        except (OSError, ConnectionError) as error:
            print(f"console unavailable: {error}; retrying", file=sys.stderr)
            time.sleep(reconnect_delay)


def main() -> int:
    parser = argparse.ArgumentParser(description="K230 Wi-Fi remote developer")
    parser.add_argument("--host", default="10.190.177.220")
    parser.add_argument("--control-port", type=int, default=8266)
    parser.add_argument("--console-port", type=int, default=2323)
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("status")
    put = sub.add_parser("put")
    put.add_argument("local", type=Path)
    put.add_argument("remote", nargs="?", default="/sdcard/app.py")
    deploy = sub.add_parser("deploy")
    deploy.add_argument("local", type=Path)
    deploy.add_argument("remote", nargs="?", default="/sdcard/app.py")
    run = sub.add_parser("run")
    run.add_argument("remote", nargs="?", default="/sdcard/app.py")
    sub.add_parser("restart")
    console = sub.add_parser("console")
    console.add_argument("--reconnect-delay", type=float, default=2.0)
    args = parser.parse_args()

    if args.command == "console":
        follow_console(args.host, args.console_port, args.reconnect_delay)
        return 0
    if args.command in ("put", "deploy"):
        put_file(args.host, args.control_port, args.local, args.remote)
        if args.command == "deploy":
            print(control(args.host, args.control_port, b"RESTART"))
        return 0

    wire_command = {
        "status": b"STATUS",
        "run": f"RUN {getattr(args, 'remote', '/sdcard/app.py')}".encode(),
        "restart": b"RESTART",
    }[args.command]
    response = control(args.host, args.control_port, wire_command)
    print(response)
    return 0 if response.startswith("OK ") else 1


if __name__ == "__main__":
    raise SystemExit(main())
