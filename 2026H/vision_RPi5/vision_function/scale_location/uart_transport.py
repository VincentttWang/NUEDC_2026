"""POSIX UART transport and frame-locked ball-state sender."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import termios
from time import time
from typing import Callable, Protocol

from .protocol import BallFrame, PACKET_SIZE, encode_frame


class PacketWriter(Protocol):
    def write(self, packet: bytes) -> int: ...


@dataclass(frozen=True)
class UartSettings:
    device: str
    baudrate: int = 115200
    position_sign: int = 1

    @classmethod
    def load(cls, path: Path | str) -> UartSettings:
        payload = json.loads(Path(path).read_text(encoding="utf-8"))
        settings = cls(
            str(payload["device"]),
            int(payload.get("baudrate", 115200)),
            int(payload.get("position_sign", 1)),
        )
        if settings.baudrate != 115200:
            raise ValueError("The current protocol requires 115200 baud")
        if not settings.device.startswith("/dev/"):
            raise ValueError("UART device must be an absolute /dev path")
        if settings.position_sign not in (-1, 1):
            raise ValueError("position_sign must be -1 or 1")
        return settings


class PosixUart:
    """A blocking 115200 8N1 UART that emits each packet with one os.write()."""

    def __init__(self, settings: UartSettings) -> None:
        self.settings = settings
        self._fd: int | None = None

    def open(self) -> None:
        if self._fd is not None:
            return
        fd = os.open(self.settings.device, os.O_RDWR | os.O_NOCTTY | os.O_CLOEXEC)
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

    def close(self) -> None:
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None

    def write(self, packet: bytes) -> int:
        if self._fd is None:
            raise RuntimeError("UART is not open")
        if len(packet) != PACKET_SIZE:
            raise ValueError(f"UART packet must contain {PACKET_SIZE} bytes")
        return os.write(self._fd, packet)

    def __enter__(self) -> PosixUart:
        self.open()
        return self

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        self.close()


@dataclass(frozen=True)
class SendResult:
    frame: BallFrame
    packet: bytes


class FrameLockedSender:
    """Compute AGE immediately before one packet write and own protocol SEQ."""

    def __init__(
        self,
        writer: PacketWriter,
        *,
        initial_sequence: int = 0,
        clock: Callable[[], float] = time,
        future_tolerance_ms: float = 5.0,
    ) -> None:
        if not 0 <= initial_sequence <= 0xFF:
            raise ValueError("initial_sequence must fit uint8")
        self.writer = writer
        self.sequence = initial_sequence
        self.clock = clock
        self.future_tolerance_s = float(future_tolerance_ms) / 1000.0
        if not math.isfinite(self.future_tolerance_s) or self.future_tolerance_s < 0:
            raise ValueError("future_tolerance_ms must be finite and non-negative")

    def send(
        self,
        *,
        capture_timestamp_s: float,
        position_cm: float,
        velocity_cm_s: float,
        valid: bool,
        moving: bool,
        velocity_valid: bool,
        edge: bool,
        quality: int,
    ) -> SendResult:
        age_s = self.clock() - float(capture_timestamp_s)
        if not math.isfinite(age_s) or age_s < -self.future_tolerance_s:
            raise ValueError("capture timestamp is in the future or invalid")
        age_s = max(0.0, age_s)
        age_ms_raw = int(math.ceil(age_s * 1000.0 - 1e-9))
        age_ms = min(age_ms_raw, 0xFF)
        if age_ms_raw > 0xFF:
            valid = False
            velocity_valid = False
            quality = 0

        frame = BallFrame.from_physical(
            sequence=self.sequence,
            age_ms=age_ms,
            position_cm=position_cm,
            velocity_cm_s=velocity_cm_s,
            valid=valid,
            moving=moving,
            velocity_valid=velocity_valid,
            edge=edge,
            quality=quality,
        )
        packet = encode_frame(frame)
        self.sequence = (self.sequence + 1) & 0xFF
        written = self.writer.write(packet)
        if written != PACKET_SIZE:
            raise OSError(f"Short UART write: {written}/{PACKET_SIZE} bytes")
        return SendResult(frame, packet)
