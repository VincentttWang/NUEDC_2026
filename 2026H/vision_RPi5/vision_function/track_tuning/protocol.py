"""Fixed 11-byte MSPM0 track-tuning protocol."""

from __future__ import annotations

from dataclasses import dataclass
import struct

SYNC = b"\xA5\x5A"
PACKET_SIZE = 11
REQUEST_TYPE = 0x81
RESPONSE_TYPE = 0x82
OP_GET = 0x01
OP_SET = 0x02
OP_GET_ALL = 0x03
OP_RESET_DEFAULTS = 0x04
STATUS_OK = 0x00
STATUS_UNKNOWN_ID = 0x01
STATUS_BUSY = 0x02
STATUS_BAD_OP = 0x03
LIST_END = 0xFFFF

STATUS_NAMES = {
    STATUS_OK: "OK",
    STATUS_UNKNOWN_ID: "UNKNOWN_ID",
    STATUS_BUSY: "BUSY",
    STATUS_BAD_OP: "BAD_OP",
}

_BODY = struct.Struct("<BBBHHB")


class TuningProtocolError(ValueError):
    """Raised when a tuning frame is malformed."""


@dataclass(frozen=True)
class TuningResponse:
    sequence: int
    status: int
    param_id: int
    value: int
    priority: int


def crc8(data: bytes) -> int:
    """CRC-8 poly=0x07, init=0xFF, refin=false, xorout=0."""
    value = 0xFF
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = ((value << 1) ^ 0x07) & 0xFF if value & 0x80 else (value << 1) & 0xFF
    return value


def _u8(value: int, name: str) -> int:
    value = int(value)
    if not 0 <= value <= 0xFF:
        raise TuningProtocolError(f"{name} 必须在 0..255 范围内")
    return value


def _u16(value: int, name: str) -> int:
    value = int(value)
    if not 0 <= value <= 0xFFFF:
        raise TuningProtocolError(f"{name} 必须在 0..65535 范围内")
    return value


def build_request(sequence: int, operation: int, param_id: int = 0, value: int = 0) -> bytes:
    body = _BODY.pack(
        REQUEST_TYPE,
        _u8(sequence, "sequence"),
        _u8(operation, "operation"),
        _u16(param_id, "param_id"),
        _u16(value, "value"),
        0,
    )
    return SYNC + body + bytes((crc8(body),))


def build_response(
    sequence: int,
    status: int,
    param_id: int,
    value: int,
    priority: int,
) -> bytes:
    """Build a response frame for tests and protocol simulators."""
    body = _BODY.pack(
        RESPONSE_TYPE,
        _u8(sequence, "sequence"),
        _u8(status, "status"),
        _u16(param_id, "param_id"),
        _u16(value, "value"),
        _u8(priority, "priority"),
    )
    return SYNC + body + bytes((crc8(body),))


def decode_response(packet: bytes) -> TuningResponse:
    if len(packet) != PACKET_SIZE:
        raise TuningProtocolError(f"响应帧必须为 {PACKET_SIZE} 字节")
    if packet[:2] != SYNC:
        raise TuningProtocolError("响应同步头错误")
    body = packet[2:10]
    if crc8(body) != packet[10]:
        raise TuningProtocolError("响应 CRC 错误")
    kind, sequence, status, param_id, value, priority = _BODY.unpack(body)
    if kind != RESPONSE_TYPE:
        raise TuningProtocolError(f"响应类型错误：0x{kind:02X}")
    return TuningResponse(sequence, status, param_id, value, priority)
