import unittest
from dataclasses import replace

from racer_companion import msp, safety


class TestClampRc(unittest.TestCase):
    def test_clamp_below(self):
        self.assertEqual(safety.clamp_rc([500] * 8), [1000] * 8)

    def test_clamp_above(self):
        out = safety.clamp_rc([2500] * 8)
        expected = [2000] * 8
        expected[msp.CH_THROTTLE] = 1700
        self.assertEqual(out, expected)


class TestEvaluate(unittest.TestCase):
    def setUp(self):
        self.cfg = safety.SafetyConfig()
        self.now = 1000.0
        self.gps = msp.GpsReading(
            fix=True, num_sat=10, lat_deg=37.0, lon_deg=-122.0,
            alt_m=10, speed_cms=0, course_deg=0.0, received_at=self.now - 0.1,
        )
        self.att = msp.AttitudeReading(roll_deg=0, pitch_deg=0, yaw_deg=0, received_at=self.now - 0.1)
        self.alt = msp.AltitudeReading(alt_cm=500, vario_cms=0, received_at=self.now - 0.1)
        self.analog = msp.AnalogReading(vbat_v=15.5, mah=0, rssi=80, amperage_a=10, received_at=self.now - 0.1)

    def test_ok(self):
        s = safety.evaluate(self.cfg, 37.0, -122.0, self.gps, self.alt, self.att, self.analog, self.now)
        self.assertTrue(s.ok, msg=s.reasons)

    def test_low_sats(self):
        gps = replace(self.gps, num_sat=4)
        s = safety.evaluate(self.cfg, 37.0, -122.0, gps, self.alt, self.att, self.analog, self.now)
        self.assertFalse(s.ok)
        self.assertTrue(any("low_sats" in r for r in s.reasons))

    def test_no_fix(self):
        gps = replace(self.gps, fix=False)
        s = safety.evaluate(self.cfg, 37.0, -122.0, gps, self.alt, self.att, self.analog, self.now)
        self.assertFalse(s.ok)
        self.assertIn("no_gps_fix", s.reasons)

    def test_stale_gps(self):
        gps = replace(self.gps, received_at=self.now - 5.0)
        s = safety.evaluate(self.cfg, 37.0, -122.0, gps, self.alt, self.att, self.analog, self.now)
        self.assertFalse(s.ok)
        self.assertIn("stale_gps", s.reasons)

    def test_stale_altitude(self):
        alt = replace(self.alt, received_at=self.now - 5.0)
        s = safety.evaluate(self.cfg, 37.0, -122.0, self.gps, alt, self.att, self.analog, self.now)
        self.assertFalse(s.ok)
        self.assertIn("stale_altitude", s.reasons)

    def test_low_vbat(self):
        analog = replace(self.analog, vbat_v=10.0)
        s = safety.evaluate(self.cfg, 37.0, -122.0, self.gps, self.alt, self.att, analog, self.now)
        self.assertFalse(s.ok)
        self.assertTrue(any("low_vbat" in r for r in s.reasons))

    def test_geofence(self):
        gps = replace(self.gps, lat_deg=37.0 + 0.002)  # ~222m north
        s = safety.evaluate(self.cfg, 37.0, -122.0, gps, self.alt, self.att, self.analog, self.now)
        self.assertFalse(s.ok)
        self.assertTrue(any("geofence" in r for r in s.reasons))

    def test_no_home_no_geofence(self):
        gps = replace(self.gps, lat_deg=37.0 + 0.002)
        s = safety.evaluate(self.cfg, None, None, gps, self.alt, self.att, self.analog, self.now)
        self.assertTrue(s.ok, msg=s.reasons)


if __name__ == "__main__":
    unittest.main()
