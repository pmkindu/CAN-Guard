"""Kodier-, Dekodier- und Transporttests (Roadmap v4.2, Phase 5.3 Paket A).

Die Sollbytes in can_guard_spec.py sind unabhängig von der DBC hergeleitet
(Little Endian, Temperatur/Feuchte Faktor 0.1).
"""
from uuid import uuid4
import math
import sys
import unittest
from pathlib import Path

import can
import cantools

sys.path.insert(0, str(Path(__file__).resolve().parent))
from can_guard_spec import DBC_PATH, MESSAGES  # noqa: E402

TEST_CASES = tuple(
    (m.name, m.frame_id, m.example, m.example_hex) for m in MESSAGES
)
DHT_MESSAGES = tuple(m for m in MESSAGES if m.name.startswith("SENSOR_NODE_3_"))


class TestGatewayParser(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        print(f"\nDBC-Datei: {DBC_PATH}", flush=True)
        if not DBC_PATH.is_file():
            raise FileNotFoundError(f"DBC-Datei fehlt: {DBC_PATH}")
        cls.db = cantools.database.load_file(str(DBC_PATH), strict=True)

    def assert_signals_equal(self, actual, expected):
        for name, value in expected.items():
            self.assertIn(name, actual)
            self.assertTrue(
                math.isclose(actual[name], value, rel_tol=0, abs_tol=1e-6),
                f"{name}: erwartet {value}, erhalten {actual[name]}",
            )

    def test_dbc_layout(self):
        for name, frame_id, signals, expected_hex in TEST_CASES:
            with self.subTest(message=name):
                definition = self.db.get_message_by_name(name)
                self.assertEqual(definition.frame_id, frame_id)
                self.assertFalse(definition.is_extended_frame)
                self.assertEqual(definition.length, len(bytes.fromhex(expected_hex)))
                self.assertEqual({s.name for s in definition.signals}, set(signals))

    def test_known_payloads(self):
        for name, frame_id, signals, expected_hex in TEST_CASES:
            with self.subTest(message=name):
                definition = self.db.get_message_by_name(name)
                expected_data = bytes.fromhex(expected_hex)
                encoded = definition.encode(signals, scaling=True, strict=True)
                self.assertEqual(encoded, expected_data)
                decoded = self.db.decode_message(
                    frame_id, expected_data, decode_choices=False, scaling=True
                )
                self.assert_signals_equal(decoded, signals)

    def test_virtual_can_transport(self):
        channel = f"can_guard_test_{uuid4().hex}"
        with (
            can.Bus(interface="virtual", channel=channel) as tx_bus,
            can.Bus(interface="virtual", channel=channel) as rx_bus,
        ):
            for name, frame_id, signals, expected_hex in TEST_CASES:
                with self.subTest(message=name):
                    definition = self.db.get_message_by_name(name)
                    payload = definition.encode(signals, scaling=True, strict=True)
                    tx_bus.send(
                        can.Message(arbitration_id=frame_id, data=payload,
                                    is_extended_id=False, check=True),
                        timeout=1.0,
                    )
                    received = rx_bus.recv(timeout=1.0)
                    self.assertIsNotNone(received, f"Timeout für {name}")
                    self.assertEqual(received.arbitration_id, frame_id)
                    self.assertFalse(received.is_extended_id)
                    self.assertEqual(received.dlc, definition.length)
                    self.assertEqual(bytes(received.data), bytes.fromhex(expected_hex))
                    decoded = self.db.decode_message(
                        received.arbitration_id, received.data,
                        decode_choices=False, scaling=True,
                    )
                    self.assert_signals_equal(decoded, signals)

    def test_ldr_16_bit_little_endian(self):
        definition = self.db.get_message_by_name("SENSOR_NODE_1")
        for ldr, expected_hex in ((0, "00 00 00"), (255, "00 FF 00"), (256, "00 00 01"), (1023, "00 FF 03")):
            with self.subTest(ldr=ldr):
                encoded = definition.encode({"Switch_Status": 0, "LDR_Brightness": ldr}, strict=True)
                self.assertEqual(encoded, bytes.fromhex(expected_hex))

    def test_dht_boundaries(self):
        for spec in DHT_MESSAGES:
            temp, hum, status = (s.name for s in spec.signals)
            definition = self.db.get_message_by_name(spec.name)
            for temperature, humidity, state in ((-20.0, 0.0, 1), (0.0, 50.0, 0), (60.0, 100.0, 0)):
                with self.subTest(message=spec.name, temp=temperature, hum=humidity, status=state):
                    expected = {temp: temperature, hum: humidity, status: state}
                    encoded = definition.encode(expected, strict=True)
                    decoded = definition.decode(encoded, decode_choices=False)
                    self.assert_signals_equal(decoded, expected)

    def test_negative_temperature_bytes(self):
        definition = self.db.get_message_by_name("SENSOR_NODE_3_S1")
        # -5,3 °C -> -53 = 0xFFCB (int16, Little Endian) -> CB FF
        encoded = definition.encode(
            {"DHT11_1_Temp": -5.3, "DHT11_1_Hum": 0.0, "DHT11_1_Status": 0}, strict=True)
        self.assertEqual(encoded[:2], bytes.fromhex("CB FF"))

    def test_invalid_values_are_rejected(self):
        for name, _, signals, _ in TEST_CASES:
            definition = self.db.get_message_by_name(name)
            for signal in definition.signals:
                if signal.maximum is None:
                    continue
                with self.subTest(message=name, signal=signal.name):
                    invalid = dict(signals)
                    invalid[signal.name] = signal.maximum + max(abs(signal.scale), 1)
                    with self.assertRaises(cantools.database.errors.EncodeError):
                        definition.encode(invalid, strict=True)

    def test_truncated_payloads_are_rejected(self):
        for name, frame_id, _, expected_hex in TEST_CASES:
            with self.subTest(message=name):
                incomplete = bytes.fromhex(expected_hex)[:-1]
                with self.assertRaises(cantools.database.errors.DecodeError):
                    self.db.decode_message(frame_id, incomplete, decode_choices=False,
                                           allow_truncated=False)


if __name__ == "__main__":
    unittest.main(verbosity=2)
