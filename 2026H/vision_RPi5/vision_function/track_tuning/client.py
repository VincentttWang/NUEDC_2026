"""Single-request UART client for MSPM0 track tuning."""

from __future__ import annotations

import os
from pathlib import Path
import select
import threading
import time
from typing import Protocol

from .parameters import PARAMETER_BY_ID, PARAMETERS
from .protocol import (
    LIST_END,
    OP_GET,
    OP_GET_ALL,
    OP_RESET_DEFAULTS,
    OP_SET,
    PACKET_SIZE,
    STATUS_NAMES,
    STATUS_OK,
    SYNC,
    TuningProtocolError,
    TuningResponse,
    build_request,
    decode_response,
)


class TuningClientError(RuntimeError):
    """Base error for a tuning transaction."""


class TuningTimeoutError(TuningClientError):
    """Raised when the MCU does not return a complete matching response."""


class TuningStatusError(TuningClientError):
    def __init__(self, response: TuningResponse) -> None:
        self.response = response
        name = STATUS_NAMES.get(response.status, f"UNKNOWN_STATUS_{response.status}")
        super().__init__(f"MSPM0 返回 {name}（参数 0x{response.param_id:04X}）")


class SerialTransport(Protocol):
    def reset_input_buffer(self) -> None: ...
    def write(self, packet: bytes) -> int: ...
    def read(self, size: int, timeout: float) -> bytes: ...
    def close(self) -> None: ...


class PosixSerial:
    """Minimal Linux 115200 8N1 transport with no third-party dependency."""

    def __init__(self, device: str) -> None:
        if not device.startswith("/dev/"):
            raise ValueError("UART 设备必须是绝对 /dev 路径")
        self.device = device
        self._fd: int | None = None

    def open(self) -> None:
        if self._fd is not None:
            return
        try:
            import termios
        except ModuleNotFoundError as exc:
            raise RuntimeError("真实 UART 调参只能在 Linux/树莓派上运行") from exc
        flags = os.O_RDWR | os.O_NOCTTY | getattr(os, "O_CLOEXEC", 0) | os.O_NONBLOCK
        fd = os.open(self.device, flags)
        try:
            attributes = termios.tcgetattr(fd)
            attributes[0] = 0
            attributes[1] = 0
            attributes[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
            attributes[3] = 0
            attributes[4] = termios.B115200
            attributes[5] = termios.B115200
            attributes[6][termios.VMIN] = 0
            attributes[6][termios.VTIME] = 0
            termios.tcsetattr(fd, termios.TCSANOW, attributes)
            termios.tcflush(fd, termios.TCIOFLUSH)
        except Exception:
            os.close(fd)
            raise
        self._fd = fd

    def reset_input_buffer(self) -> None:
        if self._fd is None:
            raise RuntimeError("UART 尚未打开")
        import termios

        termios.tcflush(self._fd, termios.TCIFLUSH)

    def write(self, packet: bytes) -> int:
        if self._fd is None:
            raise RuntimeError("UART 尚未打开")
        _readable, writable, _exceptional = select.select([], [self._fd], [], 1.0)
        if not writable:
            raise TuningTimeoutError("等待 UART 可写超时")
        return os.write(self._fd, packet)

    def read(self, size: int, timeout: float) -> bytes:
        if self._fd is None:
            raise RuntimeError("UART 尚未打开")
        readable, _writable, _exceptional = select.select(
            [self._fd], [], [], max(0.0, float(timeout))
        )
        if not readable:
            return b""
        try:
            return os.read(self._fd, size)
        except BlockingIOError:
            return b""

    def close(self) -> None:
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None

    def __enter__(self) -> PosixSerial:
        self.open()
        return self

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        self.close()


class TuningClient:
    """Own sequence numbers and permit exactly one in-flight request."""

    def __init__(self, transport: SerialTransport, *, timeout: float = 0.8) -> None:
        if timeout <= 0:
            raise ValueError("timeout 必须大于 0")
        self.transport = transport
        self.timeout = float(timeout)
        self._sequence = int(time.monotonic() * 1000.0) & 0xFF
        self._transaction_lock = threading.Lock()

    def _next_sequence(self) -> int:
        sequence = self._sequence
        self._sequence = (self._sequence + 1) & 0xFF
        return sequence

    def _read_exact(self, size: int, deadline: float) -> bytes:
        result = bytearray()
        while len(result) < size:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            chunk = self.transport.read(size - len(result), remaining)
            if chunk:
                result.extend(chunk)
        return bytes(result)

    def _read_response(self, expected_sequence: int) -> TuningResponse:
        deadline = time.monotonic() + self.timeout
        matched = 0
        while time.monotonic() < deadline:
            byte = self._read_exact(1, deadline)
            if not byte:
                break
            if matched == 0:
                matched = 1 if byte == SYNC[:1] else 0
                continue
            if byte != SYNC[1:2]:
                matched = 1 if byte == SYNC[:1] else 0
                continue
            tail = self._read_exact(PACKET_SIZE - 2, deadline)
            if len(tail) != PACKET_SIZE - 2:
                break
            try:
                response = decode_response(SYNC + tail)
            except TuningProtocolError:
                matched = 0
                continue
            if response.sequence == expected_sequence:
                return response
            matched = 0
        raise TuningTimeoutError("等待 MSPM0 调参响应超时")

    def _start(self, operation: int, param_id: int = 0, value: int = 0) -> int:
        sequence = self._next_sequence()
        self.transport.reset_input_buffer()
        packet = build_request(sequence, operation, param_id, value)
        written = self.transport.write(packet)
        if written != PACKET_SIZE:
            raise OSError(f"UART 短写：{written}/{PACKET_SIZE} 字节")
        return sequence

    @staticmethod
    def _require_ok(response: TuningResponse) -> None:
        if response.status != STATUS_OK:
            raise TuningStatusError(response)

    def get(self, param_id: int) -> TuningResponse:
        with self._transaction_lock:
            sequence = self._start(OP_GET, param_id)
            response = self._read_response(sequence)
            self._require_ok(response)
            if response.param_id != param_id:
                raise TuningClientError(
                    f"GET 回传参数不匹配：期望 0x{param_id:04X}，收到 0x{response.param_id:04X}"
                )
            return response

    def set(self, param_id: int, value: int) -> TuningResponse:
        with self._transaction_lock:
            sequence = self._start(OP_SET, param_id, value)
            response = self._read_response(sequence)
            self._require_ok(response)
            if response.param_id != param_id:
                raise TuningClientError(
                    f"SET 回传参数不匹配：期望 0x{param_id:04X}，收到 0x{response.param_id:04X}"
                )
            return response

    def get_all(self) -> dict[int, TuningResponse]:
        with self._transaction_lock:
            sequence = self._start(OP_GET_ALL)
            responses: dict[int, TuningResponse] = {}
            while True:
                response = self._read_response(sequence)
                self._require_ok(response)
                if response.param_id == LIST_END:
                    if response.priority != 0 or response.value != len(PARAMETERS):
                        raise TuningClientError(
                            "GET_ALL 结束帧无效："
                            f"count={response.value} priority={response.priority}"
                        )
                    break
                spec = PARAMETER_BY_ID.get(response.param_id)
                if spec is None:
                    raise TuningClientError(
                        f"GET_ALL 收到未知参数 0x{response.param_id:04X}"
                    )
                if response.param_id in responses:
                    raise TuningClientError(
                        f"GET_ALL 重复收到参数 0x{response.param_id:04X}"
                    )
                if response.priority != spec.priority:
                    raise TuningClientError(
                        f"参数 {spec.key} 优先级不匹配：{response.priority} != {spec.priority}"
                    )
                responses[response.param_id] = response
            missing = [
                f"0x{spec.param_id:04X}" for spec in PARAMETERS if spec.param_id not in responses
            ]
            if missing:
                raise TuningClientError("GET_ALL 缺少参数：" + "、".join(missing))
            return responses

    def reset_defaults(self) -> TuningResponse:
        with self._transaction_lock:
            sequence = self._start(OP_RESET_DEFAULTS)
            response = self._read_response(sequence)
            self._require_ok(response)
            if response.param_id != LIST_END or response.value != len(PARAMETERS):
                raise TuningClientError("RESET_DEFAULTS 响应结束帧无效")
            return response


def load_uart_device(config_path: Path | str) -> str:
    import json

    payload = json.loads(Path(config_path).read_text(encoding="utf-8"))
    device = str(payload["device"])
    if not device.startswith("/dev/"):
        raise ValueError("uart.json 中的设备必须是绝对 /dev 路径")
    if int(payload.get("baudrate", 115200)) != 115200:
        raise ValueError("调参协议要求 UART 波特率为 115200")
    return device
