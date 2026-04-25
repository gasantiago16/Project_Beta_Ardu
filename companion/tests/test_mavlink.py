import struct
import unittest

from racer_companion import mavlink


# Reference vectors generated against pymavlink 2.4.x. If you change the
# encoder, regenerate by running pymavlink against the same input fields.
# Field-order or CRC_EXTRA mistakes will surface here.

class TestCrc16(unittest.TestCase):
    def test_pymavlink_reference(self):
        # CRC-16/MCRF4XX of empty data + CRC_EXTRA=50 (HEARTBEAT extra)
        # cross-validated against pymavlink 2.4.49 x25crc.
        self.assertEqual(mavlink.crc16_x25(b"", 50), 0x1D16)

    def test_deterministic(self):
        # Same input must yield same output.
        self.assertEqual(
            mavlink.crc16_x25(b"\x09\x00\x00\x00\x01", 50),
            mavlink.crc16_x25(b"\x09\x00\x00\x00\x01", 50),
        )

    def test_changes_with_extra(self):
        # Different CRC_EXTRA → different CRC for same data.
        self.assertNotEqual(
            mavlink.crc16_x25(b"hello", 50),
            mavlink.crc16_x25(b"hello", 83),
        )


class TestEncodeFrame(unittest.TestCase):
    def test_heartbeat_frame_structure(self):
        frame = mavlink.encode_heartbeat(seq=0)
        # MAVLink v2 header is 10 bytes: STX | LEN | INCOMPAT | COMPAT | SEQ
        # | SYSID | COMPID | MSGID(3)
        # Plus 9-byte HEARTBEAT payload + 2-byte CRC = 21 bytes total
        self.assertEqual(len(frame), 21)
        self.assertEqual(frame[0], 0xFD)             # STX v2
        self.assertEqual(frame[1], 9)                # LEN = 9 (heartbeat payload)
        self.assertEqual(frame[2], 0)                # INCOMPAT_FLAGS
        self.assertEqual(frame[3], 0)                # COMPAT_FLAGS
        self.assertEqual(frame[4], 0)                # SEQ
        self.assertEqual(frame[5], mavlink.SYSID_COMPANION)
        self.assertEqual(frame[6], mavlink.COMPID_ONBOARD_COMPUTER)
        # MSGID = 0 (heartbeat) little-endian 3 bytes
        self.assertEqual(frame[7:10], b"\x00\x00\x00")

    def test_seq_increments(self):
        frame0 = mavlink.encode_heartbeat(seq=0)
        frame5 = mavlink.encode_heartbeat(seq=5)
        self.assertEqual(frame0[4], 0)
        self.assertEqual(frame5[4], 5)

    def test_unknown_msgid_raises(self):
        with self.assertRaises(ValueError):
            mavlink.encode_frame(99999, b"", 0)

    def test_oversized_payload_raises(self):
        with self.assertRaises(ValueError):
            mavlink.encode_frame(mavlink.MSGID_HEARTBEAT, b"\x00" * 254, 0)

    def test_crc_changes_with_payload(self):
        f1 = mavlink.encode_heartbeat(seq=0)
        f2 = mavlink.encode_companion_state(
            seq=0, distance_m=10.0, alt_m=5.0, heading_deg=0.0,
            state=1, safety_reasons_bitmap=0,
        )
        # CRC bytes (last 2) must differ
        self.assertNotEqual(f1[-2:], f2[-2:])

    def test_heartbeat_byte_for_byte_pymavlink(self):
        # Ground truth captured from pymavlink 2.4.49:
        # mavutil.mavlink.MAVLink(None, srcSystem=1, srcComponent=191).send(
        #     common.MAVLink_heartbeat_message(
        #         type=18, autopilot=8, base_mode=0, custom_mode=0,
        #         system_status=4, mavlink_version=3))
        # with mav.seq=0
        expected = bytes.fromhex("fd0900000001bf000000000000001208000403aec6")
        actual = mavlink.encode_heartbeat(seq=0)
        self.assertEqual(actual.hex(), expected.hex())


class TestStatustext(unittest.TestCase):
    def test_pads_to_50_bytes(self):
        frame = mavlink.encode_statustext(seq=0, severity=mavlink.SEV_INFO, text="hi")
        # Payload = 1 byte severity + 50 bytes text = 51 bytes
        self.assertEqual(frame[1], 51)
        # Severity byte first, then 'hi' followed by 48 nulls
        payload = frame[10:10 + 51]
        self.assertEqual(payload[0], mavlink.SEV_INFO)
        self.assertEqual(payload[1:3], b"hi")
        self.assertEqual(payload[3:], b"\x00" * 48)

    def test_truncates_long_text(self):
        text = "x" * 100
        frame = mavlink.encode_statustext(seq=0, severity=0, text=text)
        payload = frame[10:10 + 51]
        self.assertEqual(payload[1:51], b"x" * 50)


class TestCompanionState(unittest.TestCase):
    def test_payload_layout(self):
        frame = mavlink.encode_companion_state(
            seq=0, distance_m=12.5, alt_m=3.25, heading_deg=180.0,
            state=2, safety_reasons_bitmap=0b00000110,
        )
        self.assertEqual(frame[1], 14)  # 3 floats + 2 uint8
        # MSGID is 12500 = 0x30D4 → little-endian: D4 30 00
        self.assertEqual(frame[7:10], b"\xd4\x30\x00")
        # Decode payload
        payload = frame[10:10 + 14]
        d, a, h, st, sr = struct.unpack("<fffBB", payload)
        self.assertAlmostEqual(d, 12.5)
        self.assertAlmostEqual(a, 3.25)
        self.assertAlmostEqual(h, 180.0)
        self.assertEqual(st, 2)
        self.assertEqual(sr, 0b00000110)


class TestSafetyBitmap(unittest.TestCase):
    def test_simple_match(self):
        bm = mavlink.safety_bitmap_from_reasons(["no_gps_fix"])
        self.assertEqual(bm, mavlink.SAFETY_BIT["no_gps_fix"])

    def test_with_value_suffix(self):
        bm = mavlink.safety_bitmap_from_reasons(["low_sats_4"])
        self.assertEqual(bm, mavlink.SAFETY_BIT["low_sats"])

    def test_multiple(self):
        bm = mavlink.safety_bitmap_from_reasons([
            "low_sats_4", "low_vbat_13.5", "geofence_120m",
        ])
        expected = (mavlink.SAFETY_BIT["low_sats"]
                    | mavlink.SAFETY_BIT["low_vbat"]
                    | mavlink.SAFETY_BIT["geofence"])
        self.assertEqual(bm, expected)

    def test_unknown_reason_ignored(self):
        bm = mavlink.safety_bitmap_from_reasons(["nuclear_meltdown"])
        self.assertEqual(bm, 0)


class TestPublisherFactory(unittest.TestCase):
    def test_null_when_empty(self):
        p = mavlink.make_publisher(None)
        self.assertIsInstance(p, mavlink.NullPublisher)
        p.send(b"")
        p.close()

    def test_udp_parsing(self):
        p = mavlink.make_publisher("udp://127.0.0.1:14550")
        self.assertIsInstance(p, mavlink.UdpPublisher)
        self.assertEqual(p.host, "127.0.0.1")
        self.assertEqual(p.port, 14550)
        p.close()

    def test_unknown_scheme_raises(self):
        with self.assertRaises(ValueError):
            mavlink.make_publisher("ftp://nope")


class TestMavlinkSender(unittest.TestCase):
    def test_tick_emits_heartbeat_at_rate(self):
        captured: list[bytes] = []

        class Capture:
            def send(self, frame): captured.append(frame)
            def close(self): pass

        sender = mavlink.MavlinkSender(
            Capture(), mavlink.SenderConfig(heartbeat_hz=1.0, state_hz=5.0),
        )

        # First tick at t=0 should emit heartbeat + state
        sender.tick(now=0.0, state_name="IDLE", distance_m=0, alt_m=0,
                    heading_deg=0, state_code=0, safety_bitmap=0)
        self.assertGreaterEqual(len(captured), 1)

        # Tick at t=0.1 (within heartbeat interval, within state interval)
        before = len(captured)
        sender.tick(now=0.1, state_name="IDLE", distance_m=0, alt_m=0,
                    heading_deg=0, state_code=0, safety_bitmap=0)
        # Both heartbeat and state on cooldown → no new frame
        self.assertEqual(len(captured), before)

        # Tick at t=0.3 (state interval = 0.2s elapsed) → state fires
        sender.tick(now=0.3, state_name="IDLE", distance_m=0, alt_m=0,
                    heading_deg=0, state_code=0, safety_bitmap=0)
        self.assertGreater(len(captured), before)

    def test_statustext_only_on_change(self):
        captured: list[bytes] = []

        class Capture:
            def send(self, frame): captured.append(frame)
            def close(self): pass

        sender = mavlink.MavlinkSender(Capture())
        for _ in range(3):
            sender.tick(now=0, state_name="X", distance_m=0, alt_m=0,
                        heading_deg=0, state_code=0, safety_bitmap=0,
                        status_text="hello")
        # Only first call emits STATUSTEXT (text unchanged)
        statustext_frames = [f for f in captured if f[7] == mavlink.MSGID_STATUSTEXT]
        self.assertEqual(len(statustext_frames), 1)


if __name__ == "__main__":
    unittest.main()
