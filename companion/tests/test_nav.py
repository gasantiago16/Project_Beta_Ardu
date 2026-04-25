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

    def test_climb_phase_zeros_horizontal_sticks(self):
        # BUG-4 fix: during climb, never command roll/pitch/yaw away from
        # center even if the bearing/distance to waypoint suggest it.
        wp = nav.Waypoint(37.001, -121.999, 5.0)  # ~140m away
        rc, dbg = nav.compute_rc(
            wp, 37.0, -122.0, 0, 90.0, 0, nav.NavTuning(),
            climb_phase=True,
        )
        self.assertEqual(rc[nav.CH_ROLL], 1500)
        self.assertEqual(rc[nav.CH_PITCH], 1500)
        self.assertEqual(rc[nav.CH_YAW], 1500)
        self.assertTrue(dbg["climb_phase"])
        # Throttle should still climb to target altitude (alt=0 cm, target=5m)
        self.assertGreater(rc[nav.CH_THROTTLE], 1300)

    def test_climb_phase_off_resumes_horizontal(self):
        # Without climb_phase, far waypoint produces non-centered pitch/yaw.
        wp = nav.Waypoint(37.001, -121.999, 5.0)
        rc, dbg = nav.compute_rc(
            wp, 37.0, -122.0, 500, 0.0, 0, nav.NavTuning(),
            climb_phase=False,
        )
        self.assertFalse(dbg["climb_phase"])
        # Heading is north (0°); waypoint is NE (~45°). Yaw should command right.
        self.assertGreater(rc[nav.CH_YAW], 1500)


class TestPositionErrorBody(unittest.TestCase):
    def test_no_error_centered(self):
        f, r = nav.position_error_body_m(37.0, -122.0, 37.0, -122.0, 0.0)
        self.assertAlmostEqual(f, 0.0)
        self.assertAlmostEqual(r, 0.0)

    def test_north_offset_facing_north_is_forward(self):
        # Waypoint 1° north of drone, drone facing north → all error forward.
        f, r = nav.position_error_body_m(37.001, -122.0, 37.0, -122.0, 0.0)
        self.assertGreater(f, 0)
        self.assertAlmostEqual(r, 0.0, delta=1.0)

    def test_north_offset_facing_east_is_right(self):
        # Waypoint 1° north, drone facing east (heading=90°) → north is to the LEFT
        # of the drone in body frame, so right should be NEGATIVE.
        f, r = nav.position_error_body_m(37.001, -122.0, 37.0, -122.0, 90.0)
        self.assertLess(r, 0)
        self.assertAlmostEqual(f, 0.0, delta=1.0)

    def test_east_offset_facing_north_is_right(self):
        f, r = nav.position_error_body_m(37.0, -121.999, 37.0, -122.0, 0.0)
        self.assertGreater(r, 0)
        self.assertAlmostEqual(f, 0.0, delta=1.0)


class TestHoldCommand(unittest.TestCase):
    def test_centered_when_at_waypoint(self):
        wp = nav.Waypoint(37.0, -122.0, 5.0)
        roll, pitch = nav.hold_command_us(wp, 37.0, -122.0, 0.0, nav.NavTuning())
        self.assertEqual(roll, 1500)
        self.assertEqual(pitch, 1500)

    def test_drifted_north_facing_north_pitches_forward(self):
        # Waypoint is north of where drone has drifted; drone facing north.
        wp = nav.Waypoint(37.00005, -122.0, 5.0)  # ~5.5m north
        roll, pitch = nav.hold_command_us(wp, 37.0, -122.0, 0.0, nav.NavTuning())
        self.assertEqual(roll, 1500)
        self.assertGreater(pitch, 1500)

    def test_clamped_to_max_offset(self):
        # 100m offset is realistic for a HOLD-edge case (linearized ENU is
        # still accurate at this scale). Saturates the controller at max.
        wp = nav.Waypoint(37.0009, -122.0, 5.0)  # ~100m north
        t = nav.NavTuning()
        roll, pitch = nav.hold_command_us(wp, 37.0, -122.0, 0.0, t)
        self.assertLessEqual(pitch, 1500 + t.hold_max_offset_us)
        self.assertGreaterEqual(pitch, 1500 - t.hold_max_offset_us)
        # And drone-south-of-waypoint must produce pitch > 1500 (forward).
        self.assertGreater(pitch, 1500)
        self.assertEqual(roll, 1500)  # exactly aligned in lon

    def test_nan_inputs_return_centered(self):
        # If GPS reports NaN, return centered sticks rather than raising
        # ValueError out of int(round(NaN)).
        wp = nav.Waypoint(37.0, -122.0, 5.0)
        t = nav.NavTuning()
        roll, pitch = nav.hold_command_us(wp, float("nan"), -122.0, 0.0, t)
        self.assertEqual((roll, pitch), (1500, 1500))

    def test_heading_180_correct_sign(self):
        # Drone facing south (heading=180), waypoint north of drone.
        # In body frame, the waypoint is BEHIND the drone, so forward_m < 0,
        # so pitch should command BACKWARD (< 1500) to drift south... wait.
        # We want the drone to move TOWARD the waypoint, which is NORTH.
        # Drone facing south, drone wants to go north = backward in body frame.
        # Pitch backward (< 1500) on the stick = nose up = drift backward (south).
        # That is WRONG — we want to drift north.
        # The hold controller commands the drone to drift in the direction of
        # body_forward (positive pitch = positive forward = drift forward).
        # If forward_m is negative (waypoint behind us), pitch < 1500 = drift
        # backward, which means drift toward the waypoint. Drone facing south
        # drifting backward = drone moving north = toward waypoint. ✓
        wp = nav.Waypoint(37.00005, -122.0, 5.0)  # ~5.5m north
        t = nav.NavTuning()
        roll, pitch = nav.hold_command_us(wp, 37.0, -122.0, 180.0, t)
        # Drone facing south, waypoint is BEHIND → pitch < 1500 (drift backward = north).
        self.assertLess(pitch, 1500)

    def test_heading_270_correct_sign(self):
        # Drone facing west, waypoint east of drone (positive east_m).
        # Body right = -north*sin(270) + east*cos(270) = -east_m.
        # east_m positive → right_m negative → roll < 1500 = bank left.
        # Drone facing west, banking left = drone drifts south.
        # WAIT — roll-LEFT in ANGLE = drone drifts LEFT (in body frame).
        # Body-left when facing west is SOUTH. But waypoint is east.
        # Hmm, recheck: if the controller wants to push the drone toward the
        # waypoint, and the waypoint is east of the drone, the drone needs
        # to drift east. Drone facing west → east is BEHIND. So we should
        # be commanding pitch (forward/back), not roll.
        # forward_m = north*cos(270) + east*sin(270) = 0 + east*(-1) = -east_m.
        # east_m positive → forward_m negative → pitch < 1500 = backward stick
        # = drone drifts backward = drone drifts EAST (the direction behind a
        # west-facing drone). ✓
        wp = nav.Waypoint(37.0, -121.99997, 5.0)  # ~2.66m east of drone
        t = nav.NavTuning()
        roll, pitch = nav.hold_command_us(wp, 37.0, -122.0, 270.0, t)
        self.assertLess(pitch, 1500)
        # roll: right_m = -north*sin(270) + east*cos(270) = 0 + 0 = 0 (north_m is 0).
        self.assertEqual(roll, 1500)


class TestComputeRcInHold(unittest.TestCase):
    def test_in_hold_engages_position_controller(self):
        # Drone drifted ~3m east of waypoint, facing north. HOLD controller
        # should command roll-left to return.
        wp = nav.Waypoint(37.0, -122.0, 5.0)
        rc, dbg = nav.compute_rc(
            wp, 37.0, -121.99997, 500, 0.0, 0, nav.NavTuning(),
            in_hold=True,
        )
        self.assertTrue(dbg["in_hold"])
        # Drone drifted east → waypoint is to the LEFT in body frame → roll < 1500.
        self.assertLess(rc[nav.CH_ROLL], 1500)
        # Yaw must stay centered in HOLD (no yawing while position-holding).
        self.assertEqual(rc[nav.CH_YAW], 1500)

    def test_climb_phase_overrides_in_hold(self):
        # Both flags True → climb_phase wins (more dangerous to mis-state).
        wp = nav.Waypoint(37.0, -122.0, 5.0)
        rc, dbg = nav.compute_rc(
            wp, 37.0, -121.99997, 0, 0.0, 0, nav.NavTuning(),
            climb_phase=True, in_hold=True,
        )
        self.assertEqual(rc[nav.CH_ROLL], 1500)
        self.assertEqual(rc[nav.CH_PITCH], 1500)
        self.assertTrue(dbg["climb_phase"])


if __name__ == "__main__":
    unittest.main()
