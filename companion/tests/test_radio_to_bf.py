"""Tests for tools/radio_to_bf.py — the Radiomaster → UDP 9004 RC injector.

We test the pure functions (axis_to_pwm, channels_from_axes, sweep value,
mapping JSON load). Pygame interaction is exercised via `--sweep` mode
which doesn't require a HID; integration tests against real hardware are
manual (see docs).
"""
from __future__ import annotations

import json
import math
import struct
import sys
import tempfile
import unittest
from pathlib import Path

# tools/ is a sibling package, not under racer_companion. Add it to path.
_TOOLS = Path(__file__).resolve().parents[1] / "tools"
sys.path.insert(0, str(_TOOLS))

import radio_to_bf as rb  # noqa: E402


class TestAxisToPwm(unittest.TestCase):
    def test_center(self):
        self.assertEqual(rb.axis_to_pwm(0.0), 1500)

    def test_full_positive(self):
        self.assertEqual(rb.axis_to_pwm(1.0), 2000)

    def test_full_negative(self):
        self.assertEqual(rb.axis_to_pwm(-1.0), 1000)

    def test_half_positive(self):
        self.assertEqual(rb.axis_to_pwm(0.5), 1750)

    def test_invert(self):
        # +1.0 with invert=True should map to PWM_MIN.
        self.assertEqual(rb.axis_to_pwm(1.0, invert=True), 1000)
        self.assertEqual(rb.axis_to_pwm(-1.0, invert=True), 2000)

    def test_deadband_centers(self):
        # |axis| below deadband returns 1500 exactly — no jitter.
        self.assertEqual(rb.axis_to_pwm(0.03, deadband=0.04), 1500)
        self.assertEqual(rb.axis_to_pwm(-0.03, deadband=0.04), 1500)
        # Just above deadband passes through.
        self.assertGreater(rb.axis_to_pwm(0.05, deadband=0.04), 1500)

    def test_clamp_above_one(self):
        # Pygame can occasionally report >|1| on a poorly-calibrated stick.
        self.assertEqual(rb.axis_to_pwm(1.5), 2000)
        self.assertEqual(rb.axis_to_pwm(-1.5), 1000)

    def test_nan_returns_center(self):
        self.assertEqual(rb.axis_to_pwm(float("nan")), 1500)

    def test_inf_returns_center(self):
        self.assertEqual(rb.axis_to_pwm(float("inf")), 1500)
        self.assertEqual(rb.axis_to_pwm(float("-inf")), 1500)


class TestChannelsFromAxes(unittest.TestCase):
    def test_default_mapping_returns_16(self):
        mapping = [
            rb.ChannelMap(name="roll", axis=0),
            rb.ChannelMap(name="pitch", axis=1),
            rb.ChannelMap(name="throttle", axis=2),
            rb.ChannelMap(name="yaw", axis=3),
        ]
        out = rb.channels_from_axes([0.0, 0.0, 0.0, 0.0], mapping)
        # MUST return 16 channels — BF's rc_packet has channels[16] and a
        # strict size check rejects any other length.
        self.assertEqual(len(out), 16)
        self.assertEqual(out, [1500] * 16)

    def test_extended_mapping(self):
        mapping = [
            rb.ChannelMap(name="roll", axis=0),
            rb.ChannelMap(name="pitch", axis=1, invert=True),
            rb.ChannelMap(name="throttle", axis=2, invert=True),
            rb.ChannelMap(name="yaw", axis=3),
            rb.ChannelMap(name="aux1", axis=4),
        ]
        out = rb.channels_from_axes([1.0, 1.0, 1.0, -1.0, -1.0], mapping)
        self.assertEqual(out[:5], [2000, 1000, 1000, 1000, 1000])
        # Channels 6-16 stay at center (the BF default for unmapped channels).
        self.assertEqual(out[5:], [1500] * 11)

    def test_missing_axis_uses_center(self):
        mapping = [rb.ChannelMap(name="aux4", axis=7)]
        out = rb.channels_from_axes([0.5, 0.5, 0.5, 0.5], mapping)
        self.assertEqual(out, [1500] * 16)

    def test_first_sixteen_used(self):
        # Mapping with 20 entries: only first 16 honored (rc_packet has 16
        # channel slots).
        mapping = [rb.ChannelMap(name=f"ch{i}", axis=0) for i in range(20)]
        out = rb.channels_from_axes([1.0], mapping)
        self.assertEqual(len(out), 16)

    def test_aux_override(self):
        mapping = [rb.ChannelMap(name="roll", axis=0)]
        # Override channel index 6 (= AUX3, aka 7th channel) to 1900.
        out = rb.channels_from_axes(
            [0.0], mapping, aux_overrides={6: 1900},
        )
        self.assertEqual(out[0], 1500)
        self.assertEqual(out[6], 1900)
        self.assertEqual(out[1:6], [1500] * 5)
        self.assertEqual(out[7:], [1500] * 9)

    def test_aux_override_clamps(self):
        mapping = [rb.ChannelMap(name="roll", axis=0)]
        out = rb.channels_from_axes(
            [0.0], mapping, aux_overrides={2: 5000, 3: -100},
        )
        self.assertEqual(out[2], 2000)
        self.assertEqual(out[3], 1000)


class TestUdpPacketLayout(unittest.TestCase):
    def test_packet_size_is_40(self):
        # BF SITL rc_packet: double timestamp + uint16 channels[16] = 40 B.
        # BF strictly checks n == sizeof(rc_packet) — a wrong size is silently dropped.
        self.assertEqual(rb.UDP_RC_PACKET.size, 40)

    def test_packet_round_trip(self):
        timestamp = 12.345
        channels = tuple(1000 + i * 10 for i in range(16))
        packed = rb.UDP_RC_PACKET.pack(timestamp, *channels)
        self.assertEqual(len(packed), 40)
        unpacked = struct.unpack("<d16H", packed)
        self.assertAlmostEqual(unpacked[0], timestamp, places=6)
        self.assertEqual(unpacked[1:], channels)

    def test_packet_endianness(self):
        # Little-endian: byte 0 of a uint16=0x0001 is 0x01.
        packed = rb.UDP_RC_PACKET.pack(0.0, 1, *([1500] * 15))
        # Header (timestamp=0): 8 bytes of zeros.
        self.assertEqual(packed[:8], b"\x00" * 8)
        # First channel = 1: bytes 8-9 should be 0x01 0x00 (LE).
        self.assertEqual(packed[8:10], b"\x01\x00")


class TestLoadMapping(unittest.TestCase):
    def test_default_radiomaster_pocket_loads(self):
        path = (Path(__file__).resolve().parents[1] /
                "config" / "radiomaster_pocket.json")
        mapping = rb.load_mapping(path)
        # 8 channels expected.
        self.assertEqual(len(mapping), 8)
        # First 4: AETR.
        self.assertEqual(mapping[0].name, "roll")
        self.assertEqual(mapping[1].name, "pitch")
        self.assertEqual(mapping[2].name, "throttle")
        self.assertEqual(mapping[3].name, "yaw")
        # Pitch inverted (stick-up = -y in pygame on most platforms).
        self.assertTrue(mapping[1].invert)
        self.assertTrue(mapping[2].invert)

    def test_load_minimal(self):
        with tempfile.TemporaryDirectory() as td:
            p = Path(td) / "m.json"
            p.write_text(json.dumps({
                "channels": [
                    {"name": "x", "axis": 0},
                ],
            }))
            mapping = rb.load_mapping(p)
            self.assertEqual(len(mapping), 1)
            self.assertEqual(mapping[0].name, "x")
            self.assertEqual(mapping[0].axis, 0)
            self.assertFalse(mapping[0].invert)
            self.assertEqual(mapping[0].deadband, 0.0)


class TestSweepValue(unittest.TestCase):
    def test_in_band(self):
        # Sweep amplitude is ±0.5 — never saturates.
        for t in [0.0, 1.0, 2.0, 3.5, 7.7]:
            for ch in range(8):
                v = rb.sweep_axis_value(t, ch)
                self.assertGreaterEqual(v, -0.5)
                self.assertLessEqual(v, 0.5)

    def test_unique_phase_per_channel(self):
        # At t=0, channels should NOT all read the same value (otherwise a
        # sweep would visualize as identical curves).
        values_t0 = [rb.sweep_axis_value(0.0, ch) for ch in range(8)]
        self.assertGreater(len(set(values_t0)), 1)


class TestParseAuxOverride(unittest.TestCase):
    def test_basic(self):
        ch, pwm = rb._parse_aux_override("7=1900")
        self.assertEqual(ch, 6)   # 1-based 7 → 0-based 6
        self.assertEqual(pwm, 1900)

    def test_rejects_bad_format(self):
        import argparse
        with self.assertRaises(argparse.ArgumentTypeError):
            rb._parse_aux_override("not_a_spec")

    def test_rejects_out_of_range_channel(self):
        import argparse
        with self.assertRaises(argparse.ArgumentTypeError):
            rb._parse_aux_override("0=1500")
        with self.assertRaises(argparse.ArgumentTypeError):
            rb._parse_aux_override("17=1500")

    def test_rejects_out_of_range_pwm(self):
        import argparse
        with self.assertRaises(argparse.ArgumentTypeError):
            rb._parse_aux_override("7=900")
        with self.assertRaises(argparse.ArgumentTypeError):
            rb._parse_aux_override("7=2100")


class TestFakeJoystickIntegration(unittest.TestCase):
    """Integration test using a fake pygame.joystick that returns canned axis
    values. Exercises the joystick → channels_from_axes → pack pipeline
    without a real HID device."""

    def test_fake_joystick_to_packet(self):
        class FakeJoystick:
            def __init__(self, axes):
                self._axes = axes
            def get_axis(self, i):
                return self._axes[i]
            def get_numaxes(self):
                return len(self._axes)

        # Mode-2 quad pilot, sticks: roll right + throttle high.
        # AETR with pitch and throttle inverted (SDL Y-axis convention).
        # axis 0 (roll)     = +0.6  → +0.6 * 500 + 1500 = 1800
        # axis 1 (pitch)    =  0.0  → centered, deadband=0.04 → 1500
        # axis 2 (throttle) = -1.0  → invert → +1.0 → 2000
        # axis 3 (yaw)      = +0.05 → deadband=0.04 doesn't gate → 1525
        # axes 4-7 (aux)    =  0.0  → 1500 each
        js = FakeJoystick([0.6, 0.0, -1.0, 0.05, 0.0, 0.0, 0.0, 0.0])
        path = (Path(__file__).resolve().parents[1] /
                "config" / "radiomaster_pocket.json")
        mapping = rb.load_mapping(path)
        axis_vals = [js.get_axis(i) for i in range(js.get_numaxes())]
        channels = rb.channels_from_axes(axis_vals, mapping)

        self.assertEqual(channels[0], 1800)   # roll
        self.assertEqual(channels[1], 1500)   # pitch (centered with deadband)
        self.assertEqual(channels[2], 2000)   # throttle (high — invert OK)
        self.assertEqual(channels[3], 1525)   # yaw (just past deadband)
        self.assertEqual(channels[4:], [1500] * 12)

        # Pack into the wire format. Must be 40 bytes; must round-trip.
        packed = rb.UDP_RC_PACKET.pack(123.456, *channels)
        self.assertEqual(len(packed), 40)


if __name__ == "__main__":
    unittest.main()
