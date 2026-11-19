from __future__ import annotations

import os
import pty
import unittest

from vision_function.scale_location.protocol import FrameFlags, decode_frame
from vision_function.scale_location.uart_transport import (
    FrameLockedSender,
    PosixUart,
    UartSettings,
)


class FakeWriter:
    def __init__(self, result: int = 11) -> None:
        self.result = result
        self.packets: list[bytes] = []

    def write(self, packet: bytes) -> int:
        self.packets.append(packet)
        return self.result


class UartTransportTests(unittest.TestCase):
    def test_age_sequence_and_single_write(self) -> None:
        writer = FakeWriter()
        sender = FrameLockedSender(writer, initial_sequence=254, clock=lambda: 100.025)
        result = sender.send(
            capture_timestamp_s=100.0,
            position_cm=1.25,
            velocity_cm_s=-2.0,
            valid=True,
            moving=True,
            velocity_valid=True,
            edge=False,
            quality=3,
        )
        self.assertEqual(len(writer.packets), 1)
        self.assertEqual(writer.packets[0], result.packet)
        decoded = decode_frame(result.packet)
        self.assertEqual(decoded.sequence, 254)
        self.assertEqual(decoded.age_ms, 25)
        self.assertEqual(sender.sequence, 255)

    def test_overage_saturates_and_clears_validity(self) -> None:
        writer = FakeWriter()
        sender = FrameLockedSender(writer, clock=lambda: 10.3)
        result = sender.send(
            capture_timestamp_s=10.0,
            position_cm=0.0,
            velocity_cm_s=0.0,
            valid=True,
            moving=False,
            velocity_valid=True,
            edge=False,
            quality=3,
        )
        self.assertEqual(result.frame.age_ms, 255)
        self.assertFalse(result.frame.flags & FrameFlags.VALID)
        self.assertFalse(result.frame.flags & FrameFlags.V_VALID)
        self.assertEqual(result.frame.flags >> 4, 0)

    def test_submillisecond_future_timestamp_is_clamped_to_zero_age(self) -> None:
        writer = FakeWriter()
        sender = FrameLockedSender(writer, clock=lambda: 10.0)
        result = sender.send(
            capture_timestamp_s=10.0002,
            position_cm=0.0,
            velocity_cm_s=0.0,
            valid=True,
            moving=False,
            velocity_valid=False,
            edge=False,
            quality=1,
        )
        self.assertEqual(result.frame.age_ms, 0)

    def test_short_write_is_an_error_and_consumes_sequence(self) -> None:
        sender = FrameLockedSender(FakeWriter(5), clock=lambda: 1.0)
        with self.assertRaisesRegex(OSError, "Short UART write"):
            sender.send(
                capture_timestamp_s=1.0,
                position_cm=0.0,
                velocity_cm_s=0.0,
                valid=False,
                moving=False,
                velocity_valid=False,
                edge=False,
                quality=0,
            )
        self.assertEqual(sender.sequence, 1)

    def test_posix_uart_writes_one_packet_to_a_real_tty(self) -> None:
        master_fd, slave_fd = pty.openpty()
        device = os.ttyname(slave_fd)
        os.close(slave_fd)
        packet = bytes.fromhex("a55a01001b0000000035e9")
        try:
            with PosixUart(UartSettings(device)) as uart:
                self.assertEqual(uart.write(packet), len(packet))
                self.assertEqual(os.read(master_fd, len(packet)), packet)
        finally:
            os.close(master_fd)


if __name__ == "__main__":
    unittest.main()
