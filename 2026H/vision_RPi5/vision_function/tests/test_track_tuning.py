from __future__ import annotations

import json
from pathlib import Path
import struct
import tempfile
import unittest

from vision_function.track_tuning.client import (
    TuningClient,
    TuningStatusError,
    load_uart_device,
)
from vision_function.track_tuning.parameters import PARAMETERS
from vision_function.track_tuning.profiles import load_profile, save_profile
from vision_function.track_tuning.protocol import (
    LIST_END,
    OP_GET_ALL,
    OP_SET,
    STATUS_BUSY,
    TuningProtocolError,
    build_request,
    build_response,
    decode_response,
)
from vision_function.track_tuning.validation import validate_values


class FakeTransport:
    def __init__(self, responder) -> None:
        self.responder = responder
        self.buffer = bytearray()
        self.writes: list[bytes] = []

    def reset_input_buffer(self) -> None:
        self.buffer.clear()

    def write(self, packet: bytes) -> int:
        self.writes.append(packet)
        self.buffer.extend(self.responder(packet))
        return len(packet)

    def read(self, size: int, timeout: float) -> bytes:
        del timeout
        if not self.buffer:
            return b""
        chunk = bytes(self.buffer[:size])
        del self.buffer[:size]
        return chunk

    def close(self) -> None:
        pass


def unpack_request(packet: bytes) -> tuple[int, int, int, int, int, int]:
    return struct.unpack("<BBBHHB", packet[2:10])


class TrackTuningProtocolTests(unittest.TestCase):
    def test_request_matches_reference_frame(self) -> None:
        packet = build_request(0x34, OP_SET, 0x0120, 1050)
        self.assertEqual(packet.hex(), "a55a81340220011a040021")

    def test_response_crc_is_checked(self) -> None:
        packet = bytearray(build_response(7, 0, 0x0101, 1500, 1))
        packet[-1] ^= 0x01
        with self.assertRaisesRegex(TuningProtocolError, "CRC"):
            decode_response(bytes(packet))

    def test_get_all_requires_and_returns_all_parameters(self) -> None:
        def responder(request: bytes) -> bytes:
            kind, sequence, operation, _param_id, _value, reserved = unpack_request(request)
            self.assertEqual((kind, operation, reserved), (0x81, OP_GET_ALL, 0))
            frames = [b"noise"]
            frames.extend(
                build_response(sequence, 0, spec.param_id, spec.default, spec.priority)
                for spec in PARAMETERS
            )
            frames.append(build_response(sequence, 0, LIST_END, len(PARAMETERS), 0))
            return b"".join(frames)

        client = TuningClient(FakeTransport(responder), timeout=0.1)
        responses = client.get_all()
        self.assertEqual(len(responses), 13)
        self.assertEqual(responses[0x0120].value, 1100)

    def test_wrong_sequence_is_ignored(self) -> None:
        def responder(request: bytes) -> bytes:
            _kind, sequence, _operation, param_id, value, _reserved = unpack_request(request)
            return (
                build_response((sequence + 1) & 0xFF, 0, param_id, value, 3)
                + build_response(sequence, 0, param_id, value, 3)
            )

        client = TuningClient(FakeTransport(responder), timeout=0.1)
        response = client.set(0x0120, 1050)
        self.assertEqual(response.value, 1050)

    def test_busy_is_not_retried(self) -> None:
        def responder(request: bytes) -> bytes:
            _kind, sequence, operation, param_id, value, _reserved = unpack_request(request)
            self.assertEqual(operation, OP_SET)
            return build_response(sequence, STATUS_BUSY, param_id, value, 3)

        transport = FakeTransport(responder)
        client = TuningClient(transport, timeout=0.1)
        with self.assertRaises(TuningStatusError) as raised:
            client.set(0x0120, 1050)
        self.assertEqual(raised.exception.response.status, STATUS_BUSY)
        self.assertEqual(len(transport.writes), 1)


class TrackTuningValidationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.defaults = {spec.key: spec.default for spec in PARAMETERS}

    def test_defaults_are_valid_without_warnings(self) -> None:
        self.assertEqual(validate_values(self.defaults), [])

    def test_segment_order_is_blocking_error(self) -> None:
        values = dict(self.defaults, s2_end_mm=1400)
        issues = validate_values(values)
        self.assertTrue(any(issue.severity == "error" and "S1 < S2" in issue.message for issue in issues))

    def test_out_of_recommended_range_is_warning(self) -> None:
        values = dict(self.defaults, h2_s2_ff_x1000=1400)
        issues = validate_values(values)
        self.assertTrue(any(issue.severity == "warning" and "建议范围" in issue.message for issue in issues))

    def test_margin_larger_than_lap_is_error(self) -> None:
        values = dict(self.defaults, finish_arm_margin_mm=7000)
        issues = validate_values(values)
        self.assertTrue(any(issue.severity == "error" and "武装提前量" in issue.message for issue in issues))


class TrackTuningProfileTests(unittest.TestCase):
    def setUp(self) -> None:
        self.defaults = {spec.key: spec.default for spec in PARAMETERS}

    def test_profile_round_trip(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "赛场A.json"
            saved = save_profile(path, "赛场 A", self.defaults)
            loaded = load_profile(path)
        self.assertEqual(loaded, saved)

    def test_unknown_profile_version_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "future.json"
            path.write_text(
                json.dumps({"schema_version": 99, "name": "x", "saved_at": "x", "values": self.defaults}),
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "不支持"):
                load_profile(path)

    def test_uart_config_uses_real_mcu_device(self) -> None:
        root = Path(__file__).resolve().parents[2]
        config = root / "vision_function" / "scale_location" / "config" / "uart.json"
        self.assertEqual(load_uart_device(config), "/dev/ttyAMA0")

    def test_help_document_covers_every_parameter(self) -> None:
        root = Path(__file__).resolve().parents[2]
        help_text = (root / "vision_function" / "track_tuning" / "PARAMETER_HELP.md").read_text(
            encoding="utf-8"
        )
        for spec in PARAMETERS:
            self.assertIn(spec.key, help_text)

    def test_desktop_entry_uses_safe_takeover_launcher(self) -> None:
        root = Path(__file__).resolve().parents[2]
        desktop = (
            root
            / "vision_function"
            / "track_tuning"
            / "deploy"
            / "nuedc-track-tuning.desktop"
        ).read_text(encoding="utf-8")
        launcher = (root / "scripts" / "start_track_tuning.sh").read_text(encoding="utf-8")
        self.assertIn("scripts/start_track_tuning.sh", desktop)
        self.assertIn("systemctl stop", launcher)
        self.assertIn("trap restore_service EXIT", launcher)
        self.assertIn("/tmp/nuedc-vision-debug.lock", launcher)


if __name__ == "__main__":
    unittest.main()
