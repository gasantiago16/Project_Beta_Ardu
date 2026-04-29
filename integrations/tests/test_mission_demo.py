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


# ── Tilt-throttle feedforward ─────────────────────────────────────────────


class TestTiltThrottleFeedforward(unittest.TestCase):
    """When mission_demo commands forward pitch, throttle must
    simultaneously boost to counter the cos(tilt) lift loss. Without
    this, mission16 showed altitude sagging to 2-3 m and the drone
    hitting chemical-plant obstacles. Pure-P altitude controller is
    too slow to react after the sag has already happened."""

    def setUp(self):
        cfg = md.MissionConfig()
        # Override hover for predictable arithmetic in assertions.
        cfg.throttle_hover_us = 1500
        cfg.throttle_kp_per_m = 0  # disable altitude P so we can
                                    # measure the feedforward in isolation
        self.m = md.Mission(cfg)
        # Lock home + corners.
        self.m.update(FakeSnap(
            gps=FakeGps(lat_deg=41.0, lon_deg=-74.0),
            altitude=FakeAltitude(alt_cm=0),
            attitude=FakeAttitude(),
        ), now=0.0)

    def test_hover_with_no_pitch_no_throttle_boost(self):
        # Drone at target, no forward command (yaw not aligned →
        # forward=0, no tilt boost). Throttle = hover exactly.
        self.m.phase = md.Phase.X_LEG_1
        snap = FakeSnap(
            gps=FakeGps(lat_deg=41.0, lon_deg=-74.0),
            altitude=FakeAltitude(alt_cm=int(
                self.m.cfg.cruise_alt_m * 100)),  # at target
            attitude=FakeAttitude(yaw_deg=180.0),  # 180° off → forward=0
        )
        rc = self.m.compute_rc(snap, now=1.0)
        self.assertEqual(rc[md.SLOT_THROTTLE], self.m.cfg.throttle_hover_us)

    def test_forward_pitch_adds_throttle_proportional_boost(self):
        # When commanding forward pitch (yaw aligned, distance > 0),
        # throttle must be ABOVE hover by tilt_throttle_factor × forward.
        from racer_companion import nav
        ne = self.m.corners["NE"]
        bearing = nav.bearing_deg(41.0, -74.0, ne.lat_deg, ne.lon_deg)
        snap = FakeSnap(
            gps=FakeGps(lat_deg=41.0, lon_deg=-74.0),
            altitude=FakeAltitude(alt_cm=int(
                self.m.cfg.cruise_alt_m * 100)),  # at target alt
            attitude=FakeAttitude(yaw_deg=bearing),  # aligned
        )
        self.m.phase = md.Phase.X_LEG_1
        rc = self.m.compute_rc(snap, now=1.0)
        # forward = pitch_kp_per_m * distance, clamped to pitch_max_us
        forward = rc[md.SLOT_PITCH] - md.PWM_MID
        expected_boost = int(forward * self.m.cfg.tilt_throttle_factor)
        self.assertEqual(rc[md.SLOT_THROTTLE],
                         self.m.cfg.throttle_hover_us + expected_boost)
        self.assertGreater(rc[md.SLOT_THROTTLE], self.m.cfg.throttle_hover_us,
                           "tilt comp must lift throttle above hover")

    def test_disabled_when_factor_zero(self):
        # Setting tilt_throttle_factor = 0 should make throttle
        # ignore commanded pitch (only altitude P drives it).
        self.m.cfg.tilt_throttle_factor = 0.0
        from racer_companion import nav
        ne = self.m.corners["NE"]
        bearing = nav.bearing_deg(41.0, -74.0, ne.lat_deg, ne.lon_deg)
        snap = FakeSnap(
            gps=FakeGps(lat_deg=41.0, lon_deg=-74.0),
            altitude=FakeAltitude(alt_cm=int(
                self.m.cfg.cruise_alt_m * 100)),
            attitude=FakeAttitude(yaw_deg=bearing),
        )
        self.m.phase = md.Phase.X_LEG_1
        rc = self.m.compute_rc(snap, now=1.0)
        self.assertEqual(rc[md.SLOT_THROTTLE], self.m.cfg.throttle_hover_us)


# ── ARM_SWITCH-latch recovery ─────────────────────────────────────────────


class TestArmSwitchRecovery(unittest.TestCase):
    """The shim/Docker MSP path occasionally drops a SET_RAW_RC frame,
    BF flags BAD_RX_RECOVERY (bit 3), and while AUX1 is still high
    BF latches ARM_SWITCH (bit 25). Without intervention the drone
    falls — only AUX1 going LOW clears the latch. compute_rc spots
    the latch mid-flight and drops AUX1 LOW + idle throttle until
    both bits clear."""

    ARM_SWITCH = 1 << 25
    BAD_RX_RECOVERY = 1 << 3   # matches BF's armingDisableFlags_e bit order

    def setUp(self):
        cfg = md.MissionConfig()
        self.m = md.Mission(cfg)
        # Lock home + corners so we can reach a flying phase.
        self.m.update(FakeSnap(
            gps=FakeGps(lat_deg=41.0, lon_deg=-74.0),
            altitude=FakeAltitude(alt_cm=0),
            attitude=FakeAttitude(),
        ), now=0.0)
        self.snap = FakeSnap(
            gps=FakeGps(lat_deg=41.0, lon_deg=-74.0),
            altitude=FakeAltitude(alt_cm=2000),  # 20m, mid-flight
            attitude=FakeAttitude(),
        )

    def test_low_stage_drops_aux1_when_arm_switch_set(self):
        self.m.phase = md.Phase.X_LEG_1
        self.m.last_arming_flags = self.ARM_SWITCH
        rc = self.m.compute_rc(self.snap, now=1.0)
        self.assertEqual(rc[md.SLOT_AUX1], md.PWM_MIN)
        self.assertEqual(rc[md.SLOT_THROTTLE], self.m.cfg.throttle_idle_us)

    def test_bad_rx_recovery_alone_drops_aux1_low(self):
        # BAD_RX_RECOVERY (bit 3) alone foreshadows the latch — drop
        # AUX1 preemptively so we never give BF a LOW→HIGH transition
        # while bad-rx is set.
        self.m.phase = md.Phase.X_LEG_1
        self.m.last_arming_flags = self.BAD_RX_RECOVERY
        rc = self.m.compute_rc(self.snap, now=1.0)
        self.assertEqual(rc[md.SLOT_AUX1], md.PWM_MIN)

    def test_hold_idle_subphase_keeps_throttle_low(self):
        # First RECOVERY_HOLD_IDLE_S of HOLD must keep throttle idle
        # so the LOW→HIGH AUX1 transition lands with THROTTLE bit
        # clear (else BF re-latches ARM_SWITCH on the edge).
        self.m.phase = md.Phase.LANDING_APPROACH
        self.m.last_arming_flags = self.ARM_SWITCH
        self.m.compute_rc(self.snap, now=1.0)  # LOW; schedules hold
        self.m.last_arming_flags = 0
        # 0.1 s into HOLD — well inside the idle sub-phase.
        rc = self.m.compute_rc(self.snap, now=1.1)
        self.assertEqual(rc[md.SLOT_AUX1], md.PWM_MAX,
                         "AUX1 must rise so the LOW→HIGH edge fires")
        self.assertEqual(rc[md.SLOT_THROTTLE], self.m.cfg.throttle_idle_us,
                         "throttle must stay idle in idle sub-phase")

    def test_hold_hover_subphase_keeps_drone_at_altitude(self):
        # After the idle sub-phase, throttle goes to hover so the
        # drone doesn't free-fall through the rest of HOLD. AUX1 has
        # already settled HIGH, so raising throttle here doesn't
        # create a new LOW→HIGH AUX1 edge.
        self.m.phase = md.Phase.LANDING_APPROACH
        self.m.last_arming_flags = self.ARM_SWITCH
        self.m.compute_rc(self.snap, now=1.0)  # LOW; schedules hold
        self.m.last_arming_flags = 0
        # Past idle sub-phase, still inside hold window.
        idle_end = 1.0 + self.m.RECOVERY_HOLD_IDLE_S + 0.1
        rc = self.m.compute_rc(self.snap, now=idle_end)
        self.assertEqual(rc[md.SLOT_AUX1], md.PWM_MAX)
        self.assertEqual(rc[md.SLOT_THROTTLE], self.m.cfg.throttle_hover_us,
                         "throttle must be at hover in hover sub-phase")

    def test_normal_flow_resumes_after_hold_window_expires(self):
        # Once RECOVERY_HOLD_S elapses with clean flags, per-phase
        # compute resumes (LANDING_APPROACH descent throttle returns).
        self.m.phase = md.Phase.LANDING_APPROACH
        self.m.last_arming_flags = self.ARM_SWITCH
        self.m.compute_rc(self.snap, now=1.0)
        self.m.last_arming_flags = 0
        rc = self.m.compute_rc(self.snap, now=1.0 + self.m.RECOVERY_HOLD_S + 0.1)
        # Normal flow active again — throttle is whatever the descent
        # controller wants (above idle, below max).
        self.assertGreater(rc[md.SLOT_THROTTLE], self.m.cfg.throttle_idle_us)

    def test_armable_keeps_aux1_high_during_flight(self):
        # Sanity: when flags are clean from the start, AUX1 stays HIGH
        # so BF stays armed.
        self.m.phase = md.Phase.X_LEG_1
        self.m.last_arming_flags = 0  # ARMABLE
        rc = self.m.compute_rc(self.snap, now=1.0)
        self.assertEqual(rc[md.SLOT_AUX1], md.PWM_MAX)

    def test_recovery_skipped_during_init(self):
        # During INIT the AUX1=LOW is the normal flow, not recovery —
        # don't waste log lines yelling about a "latch" that's the
        # baseline state. Verify the recovery branch leaves the
        # diagnostic counter at zero.
        self.m.phase = md.Phase.INIT
        self.m.last_arming_flags = self.ARM_SWITCH
        self.m.compute_rc(self.snap, now=1.0)
        self.assertEqual(self.m._recovery_ticks, 0)

    def test_unset_flags_no_recovery(self):
        # Before MSP_STATUS_EX has been received, last_arming_flags = -1.
        # Recovery must not fire on the unsigned interpretation of -1.
        self.m.phase = md.Phase.X_LEG_1
        self.m.last_arming_flags = -1
        rc = self.m.compute_rc(self.snap, now=1.0)
        self.assertEqual(rc[md.SLOT_AUX1], md.PWM_MAX)
        self.assertEqual(self.m._recovery_ticks, 0)


# ── Flip detection ────────────────────────────────────────────────────────


class TestFlipDetection(unittest.TestCase):
    """Iris in Pegasus can flip on hard-pitched commands when altitude
    drops below the tilt-induced lift loss threshold. With BF's
    runaway_takeoff_prevention=OFF, BF doesn't auto-detect; mission_
    demo must spot |roll|>90° or |pitch|>90° sustained for
    FLIP_REQUIRED_TICKS and force DONE so the orchestrator can
    respawn the drone."""

    def setUp(self):
        cfg = md.MissionConfig()
        self.m = md.Mission(cfg)
        # Lock home/corners + leave INIT so update() runs the flip
        # check (it's gated to non-INIT phases).
        self.m.update(FakeSnap(
            gps=FakeGps(lat_deg=41.0, lon_deg=-74.0),
            altitude=FakeAltitude(alt_cm=0),
            attitude=FakeAttitude(),
        ), now=0.0)
        self.m.phase = md.Phase.X_LEG_1

    def _flipped_snap(self, roll_deg=180.0, pitch_deg=0.0):
        return FakeSnap(
            gps=FakeGps(lat_deg=41.0, lon_deg=-74.0),
            altitude=FakeAltitude(alt_cm=500),
            attitude=FakeAttitude(roll_deg=roll_deg, pitch_deg=pitch_deg),
        )

    def test_brief_flipped_attitude_does_not_abort(self):
        # A single tick of attitude > 90° must NOT abort — could be
        # a transient bad MSP frame or a recovery transition.
        self.m.update(self._flipped_snap(roll_deg=180.0), now=1.0)
        self.assertNotEqual(self.m.phase, md.Phase.DONE)
        self.assertEqual(self.m._flip_tick_count, 1)

    def test_sustained_flip_aborts_to_done(self):
        # FLIP_REQUIRED_TICKS+1 consecutive flipped ticks → DONE.
        for i in range(self.m.FLIP_REQUIRED_TICKS + 1):
            self.m.update(self._flipped_snap(roll_deg=180.0), now=1.0 + i * 0.02)
        self.assertEqual(self.m.phase, md.Phase.DONE)

    def test_flip_counter_resets_on_normal_attitude(self):
        # 24 flipped ticks (just under threshold), then 1 normal —
        # counter resets to 0; mission stays alive.
        for i in range(self.m.FLIP_REQUIRED_TICKS - 1):
            self.m.update(self._flipped_snap(roll_deg=180.0), now=1.0 + i * 0.02)
        self.assertEqual(self.m._flip_tick_count,
                         self.m.FLIP_REQUIRED_TICKS - 1)
        # Normal attitude tick.
        self.m.update(FakeSnap(
            gps=FakeGps(lat_deg=41.0, lon_deg=-74.0),
            altitude=FakeAltitude(alt_cm=500),
            attitude=FakeAttitude(roll_deg=0.0, pitch_deg=0.0),
        ), now=1.5)
        self.assertEqual(self.m._flip_tick_count, 0)
        self.assertNotEqual(self.m.phase, md.Phase.DONE)

    def test_pitch_flip_also_detected(self):
        # 90°+ pitch (drone pointing straight down or up) is also
        # a flip. Should trip the same path.
        for i in range(self.m.FLIP_REQUIRED_TICKS + 1):
            self.m.update(self._flipped_snap(pitch_deg=120.0), now=1.0 + i * 0.02)
        self.assertEqual(self.m.phase, md.Phase.DONE)

    def test_init_phase_skips_flip_check(self):
        # During INIT, attitude readings can be wonky before BF's
        # first MSP_ATTITUDE response stabilizes. Flip check is
        # gated to non-INIT phases.
        m2 = md.Mission(md.MissionConfig())
        # Don't set home; phase stays INIT.
        for i in range(m2.FLIP_REQUIRED_TICKS + 5):
            m2.update(FakeSnap(
                gps=FakeGps(fix=False),  # keep INIT
                attitude=FakeAttitude(roll_deg=180.0),
            ), now=1.0 + i * 0.02)
        self.assertEqual(m2.phase, md.Phase.INIT)
        self.assertEqual(m2._flip_tick_count, 0)


if __name__ == "__main__":
    unittest.main()
