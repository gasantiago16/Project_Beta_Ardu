import struct
import unittest

from racer_companion import msp


class TestEncode(unittest.TestCase):
    def test_request_no_payload(self):
        out = msp.encode_request(msp.MSP_API_VERSION)
        self.assertEqual(out, b"$M<\x00\x01\x01")

    def test_request_checksum(self):
        out = msp.encode_request(0x42, b"\x01\x02\x03")
        expected = 3 ^ 0x42 ^ 1 ^ 2 ^ 3
        self.assertEqual(out[-1], expected)

    def test_set_raw_rc_8_channels(self):
        out = msp.encode_set_raw_rc([1500] * 8)
        self.assertEqual(out[:3], b"$M<")
        self.assertEqual(out[3], 16)
        self.assertEqual(out[4], 200)
        self.assertEqual(len(out), 3 + 1 + 1 + 16 + 1)

    def test_set_raw_rc_wrong_count(self):
        with self.assertRaises(ValueError):
            msp.encode_set_raw_rc([1500] * 7)

    def test_set_raw_rc_out_of_range(self):
        with self.assertRaises(ValueError):
            msp.encode_set_raw_rc([1500] * 7 + [3000])


class TestDecode(unittest.TestCase):
    def test_raw_gps(self):
        payload = struct.pack(
            "<BBiiHHH", 1, 12, int(37.7749 * 1e7), int(-122.4194 * 1e7), 100, 250, 1800
        )
        gps = msp.decode_raw_gps(payload)
        self.assertTrue(gps.fix)
        self.assertEqual(gps.num_sat, 12)
        self.assertAlmostEqual(gps.lat_deg, 37.7749, places=4)
        self.assertAlmostEqual(gps.lon_deg, -122.4194, places=4)
        self.assertEqual(gps.alt_m, 100)
        self.assertAlmostEqual(gps.course_deg, 180.0)

    def test_attitude(self):
        payload = struct.pack("<hhh", 100, -50, 270)
        att = msp.decode_attitude(payload)
        self.assertAlmostEqual(att.roll_deg, 10.0)
        self.assertAlmostEqual(att.pitch_deg, -5.0)
        self.assertEqual(att.yaw_deg, 270)

    def test_altitude(self):
        payload = struct.pack("<ih", 500, -25)
        alt = msp.decode_altitude(payload)
        self.assertEqual(alt.alt_cm, 500)
        self.assertEqual(alt.vario_cms, -25)

    def test_analog(self):
        payload = struct.pack("<BHHh", 155, 1234, 80, 250)
        a = msp.decode_analog(payload)
        self.assertAlmostEqual(a.vbat_v, 15.5, places=1)
        self.assertEqual(a.mah, 1234)
        self.assertEqual(a.rssi, 80)
        self.assertAlmostEqual(a.amperage_a, 2.5, places=2)

    def test_rc(self):
        payload = struct.pack("<8H", 1500, 1501, 1502, 1503, 1504, 1505, 1506, 1507)
        rc = msp.decode_rc(payload)
        self.assertEqual(rc, [1500, 1501, 1502, 1503, 1504, 1505, 1506, 1507])


class TestParser(unittest.TestCase):
    def _build_response(self, cmd: int, payload: bytes) -> bytes:
        size = len(payload)
        chk = size ^ cmd
        for b in payload:
            chk ^= b
        return b"$M>" + bytes([size, cmd]) + payload + bytes([chk & 0xFF])

    def _client_with_buf(self, data: bytes) -> msp.MspClient:
        c = msp.MspClient.__new__(msp.MspClient)
        c.buf = bytearray(data)
        c.ser = None
        return c

    def test_parses_a_response(self):
        payload = struct.pack("<hhh", 100, -50, 270)
        frame = self._build_response(msp.MSP_ATTITUDE, payload)
        c = self._client_with_buf(frame)
        frames = c._drain_frames()
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0][0], msp.MSP_ATTITUDE)
        self.assertEqual(frames[0][1], payload)

    def test_skips_bad_checksum(self):
        payload = b"\x01\x02"
        bad = b"$M>" + bytes([2, 100]) + payload + bytes([0x00])
        c = self._client_with_buf(bad)
        self.assertEqual(c._drain_frames(), [])

    def test_skips_junk_prefix(self):
        good_payload = struct.pack("<ih", 500, -25)
        good = self._build_response(msp.MSP_ALTITUDE, good_payload)
        c = self._client_with_buf(b"\x00\xff\xab" + good)
        frames = c._drain_frames()
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0][0], msp.MSP_ALTITUDE)

    def test_partial_frame_waits(self):
        good_payload = struct.pack("<hhh", 100, -50, 270)
        good = self._build_response(msp.MSP_ATTITUDE, good_payload)
        c = self._client_with_buf(good[:5])  # truncated
        self.assertEqual(c._drain_frames(), [])
        c.buf.extend(good[5:])
        frames = c._drain_frames()
        self.assertEqual(len(frames), 1)

    def test_two_back_to_back(self):
        p1 = struct.pack("<hhh", 100, -50, 270)
        p2 = struct.pack("<ih", 500, -25)
        f1 = self._build_response(msp.MSP_ATTITUDE, p1)
        f2 = self._build_response(msp.MSP_ALTITUDE, p2)
        c = self._client_with_buf(f1 + f2)
        frames = c._drain_frames()
        self.assertEqual(len(frames), 2)
        self.assertEqual(frames[0][0], msp.MSP_ATTITUDE)
        self.assertEqual(frames[1][0], msp.MSP_ALTITUDE)


if __name__ == "__main__":
    unittest.main()
