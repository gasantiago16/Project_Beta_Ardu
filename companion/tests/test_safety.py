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

    def test_throttle_cap_in_band(self):
        # A request within the legal band but above the hard cap must clip.
        rc = [1500] * 8
        rc[msp.CH_THROTTLE] = 1900
        out = safety.clamp_rc(rc)
        self.assertEqual(out[msp.CH_THROTTLE], 1700)

    def test_throttle_below_cap_unchanged(self):
        rc = [1500] * 8
        rc[msp.CH_THROTTLE] = 1650
        out = safety.clamp_rc(rc)
        self.assertEqual(out[msp.CH_THROTTLE], 1650)


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
        self.rc_at = self.now - 0.1

    def evaluate(self, **over):
        kw = dict(
            cfg=self.cfg, home_lat=37.0, home_lon=-122.0,
            gps=self.gps, altitude=self.alt, attitude=self.att, analog=self.analog,
            now=self.now, last_rc_received_at=self.rc_at,
        )
        kw.update(over)
        return safety.evaluate(**kw)

    def test_ok(self):
        s = self.evaluate()
        self.assertTrue(s.ok, msg=s.reasons)

    def test_low_sats(self):
        s = self.evaluate(gps=replace(self.gps, num_sat=4))
        self.assertFalse(s.ok)
        self.assertTrue(any("low_sats" in r for r in s.reasons))

    def test_no_fix(self):
        s = self.evaluate(gps=replace(self.gps, fix=False))
        self.assertFalse(s.ok)
        self.assertIn("no_gps_fix", s.reasons)

    def test_stale_gps(self):
        s = self.evaluate(gps=replace(self.gps, received_at=self.now - 5.0))
        self.assertFalse(s.ok)
        self.assertIn("stale_gps", s.reasons)

    def test_stale_altitude(self):
        s = self.evaluate(altitude=replace(self.alt, received_at=self.now - 5.0))
        self.assertFalse(s.ok)
        self.assertIn("stale_altitude", s.reasons)

    def test_low_vbat(self):
        s = self.evaluate(analog=replace(self.analog, vbat_v=10.0))
        self.assertFalse(s.ok)
        self.assertTrue(any("low_vbat" in r for r in s.reasons))

    def test_no_analog_telemetry(self):
        # BUG-1 fix: analog absent must trip safety, not silently pass.
        s = self.evaluate(analog=None)
        self.assertFalse(s.ok)
        self.assertIn("no_analog_telemetry", s.reasons)

    def test_stale_analog(self):
        s = self.evaluate(analog=replace(self.analog, received_at=self.now - 5.0))
        self.assertFalse(s.ok)
        self.assertIn("stale_analog", s.reasons)

    def test_no_rc_telemetry(self):
        s = self.evaluate(last_rc_received_at=None)
        self.assertFalse(s.ok)
        self.assertIn("no_rc_telemetry", s.reasons)

    def test_stale_rc(self):
        s = self.evaluate(last_rc_received_at=self.now - 5.0)
        self.assertFalse(s.ok)
        self.assertIn("stale_rc", s.reasons)

    def test_geofence(self):
        s = self.evaluate(gps=replace(self.gps, lat_deg=37.0 + 0.002))  # ~222m north
        self.assertFalse(s.ok)
        self.assertTrue(any("geofence" in r for r in s.reasons))

    def test_no_home_no_geofence(self):
        s = self.evaluate(
            home_lat=None, home_lon=None,
            gps=replace(self.gps, lat_deg=37.0 + 0.002),
        )
        self.assertTrue(s.ok, msg=s.reasons)

    def test_transit_runaway_in_transit(self):
        # In TRANSIT and distance > cap → runaway reason.
        s = self.evaluate(in_transit=True, distance_to_target_m=500.0)
        self.assertFalse(s.ok)
        self.assertTrue(any("transit_runaway" in r for r in s.reasons))

    def test_transit_runaway_not_in_transit(self):
        # Same distance but not in TRANSIT → no runaway reason.
        s = self.evaluate(in_transit=False, distance_to_target_m=500.0)
        self.assertTrue(s.ok, msg=s.reasons)

    def test_transit_runaway_distance_unknown(self):
        # In TRANSIT with no distance info — can't evaluate, must not trip.
        s = self.evaluate(in_transit=True, distance_to_target_m=None)
        self.assertTrue(s.ok, msg=s.reasons)

    def test_heading_diverged(self):
        s = self.evaluate(heading_diverged=True)
        self.assertFalse(s.ok)
        self.assertIn("heading_diverged", s.reasons)

    def test_acro_active(self):
        s = self.evaluate(acro_active=True)
        self.assertFalse(s.ok)
        self.assertIn("acro_active", s.reasons)


class TestHeadingDivergenceTracker(unittest.TestCase):
    def setUp(self):
        self.tr = safety.HeadingDivergenceTracker(threshold_deg=60.0, window_s=3.0)

    def test_does_not_trip_when_not_pitching_forward(self):
        # Even with huge errors, no forward pitch → no divergence.
        for t in range(0, 10):
            tripped = self.tr.update(now=float(t), heading_err_deg=180.0,
                                     pitching_forward=False)
            self.assertFalse(tripped)

    def test_does_not_trip_within_window(self):
        # 1 second of forward flight at 90° error — too short to trip.
        self.assertFalse(self.tr.update(now=0.0, heading_err_deg=90.0, pitching_forward=True))
        self.assertFalse(self.tr.update(now=0.5, heading_err_deg=90.0, pitching_forward=True))
        self.assertFalse(self.tr.update(now=1.0, heading_err_deg=90.0, pitching_forward=True))

    def test_trips_after_full_window_of_high_error(self):
        # Sustain 90° error for > window_s while pitching forward → trips.
        for i in range(40):  # 40 ticks over 4 seconds
            t = i * 0.1
            tripped = self.tr.update(now=t, heading_err_deg=90.0, pitching_forward=True)
        self.assertTrue(tripped)

    def test_does_not_trip_below_threshold(self):
        # Sustained but low error — no trip.
        for i in range(40):
            t = i * 0.1
            tripped = self.tr.update(now=t, heading_err_deg=30.0, pitching_forward=True)
            self.assertFalse(tripped)

    def test_window_resets_on_pitch_stop(self):
        # Build up high-error window, then stop pitching forward → reset.
        for i in range(40):
            self.tr.update(now=i * 0.1, heading_err_deg=90.0, pitching_forward=True)
        self.assertTrue(any(True for _ in self.tr.samples))
        # Stop pitching forward (e.g., entered HOLD)
        self.tr.update(now=4.0, heading_err_deg=0.0, pitching_forward=False)
        self.assertEqual(len(self.tr.samples), 0)


class TestFlightModeReader(unittest.TestCase):
    def test_returns_none_before_box_names(self):
        r = safety.FlightModeReader()
        self.assertIsNone(r.is_acro_active())

    def test_angle_active_means_not_acro(self):
        r = safety.FlightModeReader()
        # Mimic a typical BF box layout. ANGLE at index 1.
        r.update_box_names(["ARM", "ANGLE", "HORIZON", "BEEPER"])
        self.assertEqual(r.angle_idx, 1)
        # flightModeFlags with bit 1 set → ANGLE active
        r.update_status(msp.StatusReading(flight_mode_flags=0b0010, received_at=0.0))
        self.assertEqual(r.is_acro_active(), False)

    def test_horizon_active_means_not_acro(self):
        r = safety.FlightModeReader()
        r.update_box_names(["ARM", "ANGLE", "HORIZON", "BEEPER"])
        r.update_status(msp.StatusReading(flight_mode_flags=0b0100, received_at=0.0))
        self.assertEqual(r.is_acro_active(), False)

    def test_neither_angle_nor_horizon_is_acro(self):
        r = safety.FlightModeReader()
        r.update_box_names(["ARM", "ANGLE", "HORIZON", "BEEPER"])
        # Only ARM (bit 0) set
        r.update_status(msp.StatusReading(flight_mode_flags=0b0001, received_at=0.0))
        self.assertEqual(r.is_acro_active(), True)

    def test_missing_angle_or_horizon_in_layout(self):
        # Pathological: a layout with no ANGLE/HORIZON named boxes.
        # is_acro_active() should still produce a sensible result (True, since
        # neither is active).
        r = safety.FlightModeReader()
        r.update_box_names(["ARM", "BEEPER"])
        r.update_status(msp.StatusReading(flight_mode_flags=0xFFFF, received_at=0.0))
        self.assertEqual(r.is_acro_active(), True)


if __name__ == "__main__":
    unittest.main()
