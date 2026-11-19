"""Fixed 11-byte Raspberry Pi to MSPM0 ball-state protocol."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntFlag
import math
import struct


SYNC = b"\xA5\x5A"
PROTOCOL_VERSION = 0x01
PACKET_SIZE = 11
_BODY = struct.Struct("<BBBhhB")


class ProtocolError(ValueError):
    """Raised when a packet cannot be encoded or decoded safely."""


class FrameFlags(IntFlag):
    VALID = 1 << 0
    MOVING = 1 << 1
    V_VALID = 1 << 2
    EDGE = 1 << 3


@dataclass(frozen=True)
class BallFrame:
    sequence: int
    age_ms: int
    position_01mm: int
    velocity_mm_s: int
    flags: int
    version: int = PROTOCOL_VERSION

    @property
    def position_cm(self) -> float:
        return self.position_01mm / 100.0

    @property
    def velocity_cm_s(self) -> float:
        return self.velocity_mm_s / 10.0

    @classmethod
    def from_physical(
        cls,
        *,
        sequence: int,
        age_ms: int,
        position_cm: float,
        velocity_cm_s: float,
        valid: bool,
        moving: bool,
        velocity_valid: bool,
        edge: bool,
        quality: int,
    ) -> BallFrame:
        if not math.isfinite(position_cm) or not math.isfinite(velocity_cm_s):
            raise ProtocolError("Position and velocity must be finite")
        flags = make_flags(
            valid=valid,
            moving=moving,
            velocity_valid=velocity_valid,
            edge=edge,
            quality=quality,
        )
        return cls(
            sequence=sequence,
            age_ms=age_ms,
            position_01mm=round(position_cm * 100.0),
            velocity_mm_s=round(velocity_cm_s * 10.0),
            flags=flags,
        )


def make_flags(
    *,
    valid: bool,
    moving: bool,
    velocity_valid: bool,
    edge: bool,
    quality: int,
) -> int:
    if not 0 <= quality <= 3:
        raise ProtocolError("quality must be in the range 0..3")
    flags = FrameFlags(0)
    if valid:
        flags |= FrameFlags.VALID
    if moving:
        flags |= FrameFlags.MOVING
    if velocity_valid:
        flags |= FrameFlags.V_VALID
    if edge:
        flags |= FrameFlags.EDGE
    return int(flags) | (quality << 4)


def crc8(data: bytes) -> int:
    """CRC-8 poly=0x07, init=0xFF, refin=false, refout=false, xorout=0."""
    crc = 0xFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def _uint8(value: int, name: str) -> int:
    value = int(value)
    if not 0 <= value <= 0xFF:
        raise ProtocolError(f"{name} does not fit uint8: {value}")
    return value


def _int16(value: int, name: str) -> int:
    value = int(value)
    if not -0x8000 <= value <= 0x7FFF:
        raise ProtocolError(f"{name} does not fit int16: {value}")
    return value


def encode_frame(frame: BallFrame) -> bytes:
    version = _uint8(frame.version, "version")
    if version != PROTOCOL_VERSION:
        raise ProtocolError(f"Unsupported protocol version {version}")
    sequence = _uint8(frame.sequence, "sequence")
    age_ms = _uint8(frame.age_ms, "age_ms")
    position = _int16(frame.position_01mm, "position_01mm")
    velocity = _int16(frame.velocity_mm_s, "velocity_mm_s")
    flags = _uint8(frame.flags, "flags")
    if flags & 0xC0:
        raise ProtocolError("Reserved flag bits 6..7 must be zero")
    body = _BODY.pack(version, sequence, age_ms, position, velocity, flags)
    return SYNC + body + bytes((crc8(body),))


def decode_frame(packet: bytes) -> BallFrame:
    if len(packet) != PACKET_SIZE:
        raise ProtocolError(f"Packet must contain {PACKET_SIZE} bytes")
    if packet[:2] != SYNC:
        raise ProtocolError("Invalid packet sync")
    body = packet[2:10]
    if crc8(body) != packet[10]:
        raise ProtocolError("CRC mismatch")
    version, sequence, age_ms, position, velocity, flags = _BODY.unpack(body)
    frame = BallFrame(sequence, age_ms, position, velocity, flags, version)
    # Apply the same semantic checks as the encoder after parsing.
    encode_frame(frame)
    return frame
