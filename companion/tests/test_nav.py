import unittest

from racer_companion import nav


class TestHaversine(unittest.TestCase):
    def test_zero(self):
        self.assertAlmostEqual(nav.haversine_m(37.0, -122.0, 37.0, -122.0), 0.0)

    def test_one_degree_lat(self):
        d = nav.haversine_m(37.0, -122.0, 38.0, -122.0)
        self.assertAlmostEqual(d, 111195.0, delta=200)


class TestBearing(unittest.TestCase):
    def test_north(self):
        self.assertAlmostEqual(nav.bearing_deg(37.0, -122.0, 38.0, -122.0), 0.0, delta=0.1)

    def test_east(self):
        self.assertAlmostEqual(nav.bearing_deg(37.0, -122.0, 37.0, -121.0), 90.0, delta=0.5)

    def test_south(self):
        self.assertAlmostEqual(nav.bearing_deg(37.0, -122.0, 36.0, -122.0), 180.0, delta=0.1)


class TestHeadingError(unittest.TestCase):
    def test_simple(self):
        self.assertAlmostEqual(nav.heading_error_deg(90, 80), 10.0)

    def test_wrap_positive(self):
        self.assertAlmostEqual(nav.heading_error_deg(10, 350), 20.0)

    def test_wrap_negative(self):
        self.assertAlmostEqual(nav.heading_error_deg(350, 10), -20.0)

    def test_180(self):
        self.assertAlmostEqual(abs(nav.heading_error_deg(0, 180)), 180.0)


class TestControllers(unittest.TestCase):
    def test_yaw_centered_when_aligned(self):
        self.assertEqual(nav.yaw_command_us(90, 90, nav.NavTuning()), 1500)

    def test_yaw_clamped(self):
        t = nav.NavTuning(yaw_kp=100.0, yaw_max_us=200.0)
        self.assertEqual(nav.yaw_command_us(90, 0, t), 1700)

    def test_pitch_only_when_aligned(self):
        t = nav.NavTuning()
        self.assertEqual(nav.pitch_command_us(50, 60, t), 1500)
        self.assertGreater(nav.pitch_command_us(50, 5, t), 1500)

    def test_throttle_at_target(self):
        self.assertEqual(nav.throttle_command_us(5.0, 500, 0, nav.NavTuning(throttle_hover=1300)), 1300)

    def test_throttle_clamped(self):
        t = nav.NavTuning(throttle_hover=1300, throttle_max_offset=200, throttle_kp_per_m=1000)
        self.assertEqual(nav.throttle_command_us(50.0, 0, 0, t), 1500)


class TestComputeRc(unittest.TestCase):
    def test_arrival_centers(self):
        wp = nav.Waypoint(37.0, -122.0, 5.0)
        rc, dbg = nav.compute_rc(wp, 37.0, -122.0, 500, 0.0, 0, nav.NavTuning())
        self.assertTrue(dbg["arrived"])
        self.assertEqual(rc[nav.CH_ROLL], 1500)
        self.assertEqual(rc[nav.CH_PITCH], 1500)
        self.assertEqual(rc[nav.CH_YAW], 1500)

    def test_returns_8_channels(self):
        wp = nav.Waypoint(37.0, -122.0, 5.0)
        rc, _ = nav.compute_rc(wp, 37.001, -122.001, 500, 0.0, 0, nav.NavTuning())
        self.assertEqual(len(rc), 8)


if __name__ == "__main__":
    unittest.main()
