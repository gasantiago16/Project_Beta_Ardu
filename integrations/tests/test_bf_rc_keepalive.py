"""Tests for `bf_rc_keepalive` — pin the BF SITL UDP RC wire format
and the neutral-channel layout.

BF SITL strict-checks `n == sizeof(rc_packet)` (40 B). A wire-format
regression is silently dropped. Match `radio_to_bf`'s pin pattern.
"""
from __future__ import annotations

import struct
import sys
import unittest
from pathlib import Path

_TOOLS = Path(__file__).resolve().parents[1] / "tools"
sys.path.insert(0, str(_TOOLS))

import bf_rc_keepalive as kp  # noqa: E402


class TestWireFormat(unittest.TestCase):
    def test_rc_packet_is_40_bytes(self):
        """BF SITL drops anything else silently. See target.h:rc_packet."""
        self.assertEqual(kp.RC_PACKET.size, 40)

    def test_rc_packet_format_matches_target_h(self):
        # double timestamp + 16 uint16 channels.
        # `<d16H` matches BF SITL's host-native LE double + LE u16[16].
        self.assertEqual(kp.RC_PACKET.format, "<d16H")

    def test_packet_pack_round_trip(self):
        ts = 1234.5678
        chs = list(range(1000, 1000 + kp.NUM_RC_CHANNELS))  # 1000..1015
        buf = kp.RC_PACKET.pack(ts, *chs)
        self.assertEqual(len(buf), 40)
        out = kp.RC_PACKET.unpack(buf)
        self.assertAlmostEqual(out[0], ts, places=6)
        self.assertEqual(list(out[1:]), chs)


class TestNeutralChannels(unittest.TestCase):
    # `defaults.txt:59` binds ARM box to `aux 0 0 0 1700 2100`, so AUX1
    # in [1700, 2100] = "arm". The keepalive's HIGH value MUST land in
    # this band; LOW must not.
    ARM_BAND_LOW = 1700
    ARM_BAND_HIGH = 2100

    def test_aux1_high_in_arm_band(self):
        rc = kp.build_neutral_channels(aux1_high=True)
        self.assertEqual(len(rc), kp.NUM_RC_CHANNELS)
        self.assertEqual(rc[0], kp.PWM_MID)   # roll
        self.assertEqual(rc[1], kp.PWM_MID)   # pitch
        self.assertEqual(rc[2], kp.PWM_MIN)   # throttle LOW (failsafe-safe)
        self.assertEqual(rc[3], kp.PWM_MID)   # yaw
        # AUX1 must be in BF's arm-box band (1700..2100), or BF won't
        # see this as "arm requested".
        self.assertGreaterEqual(rc[4], self.ARM_BAND_LOW)
        self.assertLessEqual(rc[4], self.ARM_BAND_HIGH)
        for ch in (5, 6, 7):
            self.assertEqual(rc[ch], kp.PWM_MIN, f"AUX{ch-3} should be LOW")

    def test_aux1_low_outside_arm_band(self):
        rc = kp.build_neutral_channels(aux1_high=False)
        # MUST be below the arm band so this tool can't engage BF's
        # ARM box on its own (mission_demo handles arming via MSP
        # override; we want the keepalive's fallback value to be safe).
        self.assertLess(rc[4], self.ARM_BAND_LOW,
                        "AUX1 LOW must be below arm-box threshold (1700)")

    def test_throttle_is_idle_safe(self):
        # If mission_demo's MSP overrides drop, BF falls back to this
        # stream. Throttle MUST be below min_check (1050) so motors
        # don't spin uncommanded.
        rc = kp.build_neutral_channels(aux1_high=True)
        self.assertLess(rc[2], 1050,
                        "Throttle must be below BF's min_check (1050)")


if __name__ == "__main__":
    unittest.main()
