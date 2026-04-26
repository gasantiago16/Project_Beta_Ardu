"""Unit tests for the mission demo script.

Pure-function tests on waypoint math, circle parameter generator, and
state-machine transitions. The full mission can't be unit-tested without
a real BF SITL — for that, run `python -m integrations.tools.mission_demo`
against the live stack.
"""
from __future__ import annotations

import math
import sys
import unittest
from dataclasses import dataclass
from pathlib import Path

# integrations/tools is a sibling of integrations/tests.
_TOOLS = Path(__file__).resolve().parents[1] / "tools"
sys.path.insert(0, str(_TOOLS))

import mission_demo as md  # noqa: E402


# ── Fakes ──────────────────────────────────────────────────────────────────


@dataclass
class FakeGps:
    fix: bool = True
    num_sat: int = 12
    lat_deg: float = 41.39
    lon_deg: float = -73.95
    alt_m: int = 100
    speed_cms: int = 0
    course_deg: float = 0.0
    received_at: float = 0.0


@dataclass
class FakeAttitude:
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0
    received_at: float = 0.0


@dataclass
class FakeAltitude:
    alt_cm: int = 0
    vario_cms: int = 0
    received_at: float = 0.0


@dataclass
class FakeSnap:
    gps: object = None
    attitude: object = None
    altitude: object = None
    analog: object = None
    rc: object = None
    rc_received_at: float | None = None


# ── Waypoint math ─────────────────────────────────────────────────────────


class TestOffsetWaypoint(unittest.TestCase):
    def test_origin(self):
        w = md.offset_waypoint(41.0, -74.0, 0.0, 0.0, 50.0)
        self.assertAlmostEqual(w.lat_deg, 41.0, places=10)
        self.assertAlmostEqual(w.lon_deg, -74.0, places=10)
        self.assertAlmostEqual(w.alt_m, 50.0, places=10)

    def test_north_offset(self):
        # 110.54 m north → +0.001 deg lat.
        w = md.offset_waypoint(41.0, -74.0, 110.54, 0.0, 0.0)
        self.assertAlmostEqual(w.lat_deg, 41.001, places=6)
        self.assertAlmostEqual(w.lon_deg, -74.0, places=10)

    def test_east_offset_at_mid_lat(self):
        # 1 deg lon at 41° lat ≈ 84 km. So 84 m east ≈ 0.001 deg lon.
        target_m = 111_320.0 * math.cos(math.radians(41.0)) * 0.001
        w = md.offset_waypoint(41.0, -74.0, 0.0, target_m, 0.0)
        self.assertAlmostEqual(w.lon_deg, -74.0 + 0.001, places=4)

    def test_label_passthrough(self):
        w = md.offset_waypoint(0.0, 0.0, 0.0, 0.0, 0.0, label="TEST")
        self.assertEqual(w.label, "TEST")


class TestBuildCorners(unittest.TestCase):
    def setUp(self):
        self.home_lat = 41.0
        self.home_lon = -74.0
        self.half = 80.0
        self.alt = 75.0
        self.corners = md.build_corners(
            self.home_lat, self.home_lon, self.half, self.alt,
        )

    def test_five_keys(self):
        self.assertEqual(set(self.corners.keys()),
                         {"SW", "SE", "NE", "NW", "CENTER"})

    def test_all_at_cruise_alt(self):
        for w in self.corners.values():
            self.assertEqual(w.alt_m, self.alt)

    def test_center_is_home(self):
        c = self.corners["CENTER"]
        self.assertAlmostEqual(c.lat_deg, self.home_lat, places=10)
        self.assertAlmostEqual(c.lon_deg, self.home_lon, places=10)

    def test_corner_geometry(self):
        # NE is north + east of home; SW is south + west.
        ne = self.corners["NE"]
        sw = self.corners["SW"]
        self.assertGreater(ne.lat_deg, self.home_lat)
        self.assertGreater(ne.lon_deg, self.home_lon)
        self.assertLess(sw.lat_deg, self.home_lat)
        self.assertLess(sw.lon_deg, self.home_lon)

    def test_diagonals_symmetric(self):
        # SW and NE are the same distance from CENTER; same for SE / NW.
        from racer_companion import nav
        c = self.corners["CENTER"]
        d_ne = nav.haversine_m(c.lat_deg, c.lon_deg,
                               self.corners["NE"].lat_deg,
                               self.corners["NE"].lon_deg)
        d_sw = nav.haversine_m(c.lat_deg, c.lon_deg,
                               self.corners["SW"].lat_deg,
                               self.corners["SW"].lon_deg)
        self.assertAlmostEqual(d_ne, d_sw, delta=1.0)

    def test_diagonal_length_matches_half_size(self):
        # Diagonal half-extent is half · sqrt(2).
        from racer_companion import nav
        c = self.corners["CENTER"]
        d = nav.haversine_m(c.lat_deg, c.lon_deg,
                            self.corners["NE"].lat_deg,
                            self.corners["NE"].lon_deg)
        expected = self.half * math.sqrt(2)
        self.assertAlmostEqual(d, expected, delta=1.0)


# ── Mission state machine ─────────────────────────────────────────────────


class TestMissionStateTransitions(unittest.TestCase):
    def setUp(self):
        self.cfg = md.MissionConfig(
            map_half_size_m=80.0,
            cruise_alt_m=75.0,
            arrival_radius_m=5.0,
            circle_period_s=1.0,         # short circles for tests
            handover_duration_s=1.0,
            los_duration_s=1.0,
            climb_timeout_s=2.0,
            min_satellites=8,
        )
        self.m = md.Mission(self.cfg)

    def _snap(self, lat=None, lon=None, alt_m=0.0, sats=12, fix=True):
        return FakeSnap(
            gps=FakeGps(fix=fix, num_sat=sats,
                        lat_deg=lat if lat is not None else 41.0,
                        lon_deg=lon if lon is not None else -74.0),
            altitude=FakeAltitude(alt_cm=int(alt_m * 100)),
            attitude=FakeAttitude(yaw_deg=0.0),
        )

    def test_init_holds_until_gps_fix(self):
        # No fix → stay in INIT.
        self.m.update(self._snap(fix=False), now=0.0)
        self.assertEqual(self.m.phase, md.Phase.INIT)
        # Low sats → stay in INIT.
        self.m.update(self._snap(sats=4), now=0.1)
        self.assertEqual(self.m.phase, md.Phase.INIT)
        # Good fix → CLIMB.
        self.m.update(self._snap(), now=0.2)
        self.assertEqual(self.m.phase, md.Phase.CLIMB)

    def test_climb_locks_home_and_corners(self):
        self.m.update(self._snap(lat=41.0, lon=-74.0), now=0.0)
        self.assertIsNotNone(self.m.home)
        self.assertEqual(self.m.home.lat_deg, 41.0)
        self.assertIn("NE", self.m.corners)

    def test_climb_to_x_leg_1(self):
        self.m.update(self._snap(), now=0.0)  # → CLIMB
        # Below cruise alt: stay in CLIMB.
        self.m.update(self._snap(alt_m=10.0), now=0.1)
        self.assertEqual(self.m.phase, md.Phase.CLIMB)
        # At/above 95% cruise: advance to X_LEG_1.
        self.m.update(self._snap(alt_m=72.0), now=0.5)
        self.assertEqual(self.m.phase, md.Phase.X_LEG_1)

    def test_climb_timeout_advances(self):
        self.m.update(self._snap(), now=0.0)
        # Far below cruise but past timeout.
        self.m.update(self._snap(alt_m=5.0), now=10.0)
        self.assertEqual(self.m.phase, md.Phase.X_LEG_1)

    def test_x_pattern_full_chain(self):
        # CLIMB → X_LEG_1 → X_LEG_2 → X_LEG_3 → TO_CENTER → CIRCLE_1 →
        # CIRCLE_2 → HANDOVER → LOS_TEST → LANDING_APPROACH → DESCENT → DONE
        self.m.update(self._snap(), now=0.0)
        self.m.update(self._snap(alt_m=75.0), now=0.5)  # → X_LEG_1
        self.assertEqual(self.m.phase, md.Phase.X_LEG_1)

        # Spoof "arrived" by jumping the GPS to the NE corner.
        ne = self.m.corners["NE"]
        self.m.update(self._snap(lat=ne.lat_deg, lon=ne.lon_deg, alt_m=75.0), now=1.0)
        self.assertEqual(self.m.phase, md.Phase.X_LEG_2)

        se = self.m.corners["SE"]
        self.m.update(self._snap(lat=se.lat_deg, lon=se.lon_deg, alt_m=75.0), now=2.0)
        self.assertEqual(self.m.phase, md.Phase.X_LEG_3)

        nw = self.m.corners["NW"]
        self.m.update(self._snap(lat=nw.lat_deg, lon=nw.lon_deg, alt_m=75.0), now=3.0)
        self.assertEqual(self.m.phase, md.Phase.TO_CENTER)

        c = self.m.corners["CENTER"]
        self.m.update(self._snap(lat=c.lat_deg, lon=c.lon_deg, alt_m=75.0), now=4.0)
        self.assertEqual(self.m.phase, md.Phase.CIRCLE_1)

    def test_handover_then_los(self):
        # Force into HANDOVER directly to test the timer.
        self.m.update(self._snap(), now=0.0)
        self.m.phase = md.Phase.HANDOVER
        self.m.phase_started_at = 100.0
        self.m.update(self._snap(), now=100.5)  # 0.5s in → still handover
        self.assertEqual(self.m.phase, md.Phase.HANDOVER)
        self.m.update(self._snap(), now=101.5)  # 1.5s in → LOS
        self.assertEqual(self.m.phase, md.Phase.LOS_TEST)

    def test_descent_terminates_at_low_alt(self):
        # Drone reaches ground → DONE.
        self.m.phase = md.Phase.LANDING_DESCENT
        self.m.phase_started_at = 0.0
        self.m.update(self._snap(alt_m=0.5), now=10.0)
        self.assertEqual(self.m.phase, md.Phase.DONE)

    def test_descent_timeout_force_exits(self):
        # Altitude oscillates above arrival threshold forever → timeout
        # fires and DONE is reached anyway. Without this, noisy altitude
        # could trap the mission in DESCENT indefinitely.
        # Override the timeout to something cheap for the test.
        self.m.cfg.descent_timeout_s = 10.0
        self.m.phase = md.Phase.LANDING_DESCENT
        self.m.phase_started_at = 0.0
        # Stay just above the threshold past the timeout.
        for t in range(0, 16, 1):
            self.m.update(self._snap(alt_m=2.0), now=float(t))
        self.assertEqual(self.m.phase, md.Phase.DONE)

    def test_should_not_send_msp_during_handover_or_los(self):
        for phase in (md.Phase.HANDOVER, md.Phase.LOS_TEST,
                      md.Phase.INIT, md.Phase.DONE):
            self.m.phase = phase
            self.assertFalse(self.m.should_send_msp(),
                             f"should NOT send MSP during {phase}")

    def test_should_send_msp_during_active_phases(self):
        for phase in (md.Phase.CLIMB, md.Phase.X_LEG_1, md.Phase.CIRCLE_1,
                      md.Phase.LANDING_APPROACH, md.Phase.LANDING_DESCENT):
            self.m.phase = phase
            self.assertTrue(self.m.should_send_msp(),
                            f"SHOULD send MSP during {phase}")


# ── Circle target ─────────────────────────────────────────────────────────


class TestCircleTarget(unittest.TestCase):
    def setUp(self):
        cfg = md.MissionConfig(circle_radius_m=50.0, circle_period_s=10.0)
        self.m = md.Mission(cfg)
        # Manually seed home + corners.
        self.m.update(FakeSnap(
            gps=FakeGps(lat_deg=41.0, lon_deg=-74.0),
            altitude=FakeAltitude(alt_cm=0),
            attitude=FakeAttitude(),
        ), now=0.0)
        self.m.circle_phase_offset_s = 100.0

    def test_t_equals_offset_returns_north_point(self):
        # theta = 0 → cos=1, sin=0 → north_off=R, east_off=0.
        # So target is R meters north of CENTER.
        target = self.m._circle_target(now=100.0)
        from racer_companion import nav
        center = self.m.corners["CENTER"]
        d = nav.haversine_m(center.lat_deg, center.lon_deg,
                            target.lat_deg, target.lon_deg)
        self.assertAlmostEqual(d, 50.0, delta=1.0)
        # Bearing from center to target ≈ 0° (north).
        bearing = nav.bearing_deg(center.lat_deg, center.lon_deg,
                                  target.lat_deg, target.lon_deg)
        self.assertAlmostEqual(bearing, 0.0, delta=2.0)

    def test_quarter_period_returns_east_point(self):
        # theta = π/2 → cos=0, sin=1 → north_off=0, east_off=R.
        target = self.m._circle_target(now=102.5)  # 25% of 10s = 2.5s
        from racer_companion import nav
        center = self.m.corners["CENTER"]
        bearing = nav.bearing_deg(center.lat_deg, center.lon_deg,
                                  target.lat_deg, target.lon_deg)
        self.assertAlmostEqual(bearing, 90.0, delta=2.0)

    def test_radius_constant_at_all_angles(self):
        from racer_companion import nav
        center = self.m.corners["CENTER"]
        for t_off in (0.0, 1.5, 3.7, 7.2, 9.99):
            target = self.m._circle_target(now=100.0 + t_off)
            d = nav.haversine_m(center.lat_deg, center.lon_deg,
                                target.lat_deg, target.lon_deg)
            self.assertAlmostEqual(d, 50.0, delta=1.0)


# ── RC computation ────────────────────────────────────────────────────────


class TestComputeRc(unittest.TestCase):
    def setUp(self):
        cfg = md.MissionConfig()  # default hover = 1300
        self.m = md.Mission(cfg)
        # Lock home + corners.
        self.m.update(FakeSnap(
            gps=FakeGps(lat_deg=41.0, lon_deg=-74.0),
            altitude=FakeAltitude(alt_cm=0),
            attitude=FakeAttitude(),
        ), now=0.0)

    def test_init_returns_centered_no_gps(self):
        rc = self.m.compute_rc(FakeSnap(), now=0.0)
        self.assertEqual(rc[0:3], [1500, 1500, 1500])  # roll, pitch, yaw centered

    def test_climb_throttles_above_hover(self):
        self.m.phase = md.Phase.CLIMB
        snap = FakeSnap(
            gps=FakeGps(lat_deg=41.0, lon_deg=-74.0),
            altitude=FakeAltitude(alt_cm=1000),  # 10m
            attitude=FakeAttitude(),
        )
        rc = self.m.compute_rc(snap, now=1.0)
        self.assertGreater(rc[3], self.m.cfg.throttle_hover_us)

    def test_descent_throttles_below_hover(self):
        # The whole point of LANDING_DESCENT is to bias throttle below
        # hover so the drone sinks. If anyone "fixes" this to follow
        # altitude-hold, the demo never lands.
        self.m.phase = md.Phase.LANDING_DESCENT
        snap = FakeSnap(
            gps=FakeGps(lat_deg=41.0, lon_deg=-74.0),
            altitude=FakeAltitude(alt_cm=5000),  # 50m
            attitude=FakeAttitude(),
        )
        rc = self.m.compute_rc(snap, now=1.0)
        self.assertLess(rc[3], self.m.cfg.throttle_hover_us)

    def test_default_hover_matches_bf_default(self):
        # 1300 µs to match `sitl/defaults.txt:gps_rescue_throttle_hover=1300`.
        # If anyone bumps this back to 1500, climb-throttle saturates the
        # 1700-µs hard cap and Iris blows past 75m altitude.
        self.assertEqual(md.MissionConfig().throttle_hover_us, 1300)

    def test_waypoint_pitches_forward_when_aligned(self):
        # Drone at home, target = NE corner. Drone heading aligned to bearing.
        ne = self.m.corners["NE"]
        from racer_companion import nav
        bearing = nav.bearing_deg(41.0, -74.0, ne.lat_deg, ne.lon_deg)
        snap = FakeSnap(
            gps=FakeGps(lat_deg=41.0, lon_deg=-74.0),
            altitude=FakeAltitude(alt_cm=7500),  # 75m, at cruise
            attitude=FakeAttitude(yaw_deg=bearing),
        )
        self.m.phase = md.Phase.X_LEG_1
        rc = self.m.compute_rc(snap, now=1.0)
        # Pitch (CH_PITCH = 1) should be > 1500 (forward).
        self.assertGreater(rc[1], 1500)
        # Yaw (CH_YAW = 2) close to centered (already aligned).
        self.assertAlmostEqual(rc[2], 1500, delta=20)


if __name__ == "__main__":
    unittest.main()
