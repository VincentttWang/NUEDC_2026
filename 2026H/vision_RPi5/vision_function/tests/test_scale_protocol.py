from __future__ import annotations

import unittest

from vision_function.scale_location.protocol import (
    BallFrame,
    FrameFlags,
    ProtocolError,
    decode_frame,
    encode_frame,
)


class ScaleProtocolTests(unittest.TestCase):
    def test_fixed_zero_position_vector(self) -> None:
        frame = BallFrame.from_physical(
            sequence=0,
            age_ms=27,
            position_cm=0.0,
            velocity_cm_s=0.0,
            valid=True,
            moving=False,
            velocity_valid=True,
            edge=False,
            quality=3,
        )
        self.assertEqual(encode_frame(frame).hex(), "a55a01001b0000000035e9")

    def test_signed_little_endian_round_trip(self) -> None:
        frame = BallFrame.from_physical(
            sequence=255,
            age_ms=255,
            position_cm=-12.5,
            velocity_cm_s=-30.0,
            valid=True,
            moving=True,
            velocity_valid=True,
            edge=True,
            quality=2,
        )
        packet = encode_frame(frame)
        self.assertEqual(packet.hex(), "a55a01ffff1efbd4fe2f41")
        self.assertEqual(packet[5:7], bytes.fromhex("1efb"))
        self.assertEqual(packet[7:9], bytes.fromhex("d4fe"))
        self.assertEqual(decode_frame(packet), frame)

    def test_fixed_positive_invalid_vector(self) -> None:
        frame = BallFrame.from_physical(
            sequence=1,
            age_ms=0,
            position_cm=12.5,
            velocity_cm_s=30.0,
            valid=False,
            moving=True,
            velocity_valid=False,
            edge=True,
            quality=1,
        )
        self.assertEqual(encode_frame(frame).hex(), "a55a010100e2042c011a5a")

    def test_crc_failure_is_rejected(self) -> None:
        packet = bytearray(
            encode_frame(BallFrame(1, 2, 3, 4, int(FrameFlags.VALID)))
        )
        packet[5] ^= 0x01
        with self.assertRaisesRegex(ProtocolError, "CRC mismatch"):
            decode_frame(bytes(packet))

    def test_range_and_reserved_bits_are_rejected(self) -> None:
        with self.assertRaisesRegex(ProtocolError, "int16"):
            encode_frame(BallFrame(0, 0, 32768, 0, 0))
        with self.assertRaisesRegex(ProtocolError, "Reserved"):
            encode_frame(BallFrame(0, 0, 0, 0, 0x80))


if __name__ == "__main__":
    unittest.main()
