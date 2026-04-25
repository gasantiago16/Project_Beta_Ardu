"""Property-based safety tests using hypothesis.

The example-based tests cover specific scenarios; these cover the input
*space*. The invariants below have to hold for EVERY valid input, not just
the ones a human imagined. If hypothesis can find a counter-example, that's
the bug — and it'll save the failing input in `.hypothesis/examples/` so the
regression sticks.

The tests are intentionally tiny. The point isn't comprehensive property
coverage — it's that the load-bearing invariants (clamp output bounded,
empty reasons iff ok, etc.) are checked under random inputs.
"""
from __future__ import annotations

import unittest

try:
    from hypothesis import given, settings, strategies as st
except ImportError:  # pragma: no cover
    raise unittest.SkipTest("hypothesis not installed")

from racer_companion import msp, safety


# Reuse this throughout — fast tests, but allow plenty of examples.
FAST = settings(max_examples=200, deadline=None)

# The companion intentionally tolerates 50us off-band before raising. Use
# the same band as encode_set_raw_rc's check.
RC_INPUT = st.integers(min_value=msp.CHANNEL_MIN - 50, max_value=msp.CHANNEL_MAX + 50)
RC_VECTOR = st.lists(RC_INPUT, min_size=8, max_size=8)


class TestClampInvariants(unittest.TestCase):
    @FAST
    @given(rc=RC_VECTOR)
    def test_output_within_hard_band(self, rc):
        out = safety.clamp_rc(rc)
        for v in out:
            self.assertGreaterEqual(v, safety.RC_HARD_MIN)
            self.assertLessEqual(v, safety.RC_HARD_MAX)

    @FAST
    @given(rc=RC_VECTOR)
    def test_throttle_never_exceeds_hard_max(self, rc):
        out = safety.clamp_rc(rc)
        self.assertLessEqual(out[msp.CH_THROTTLE], safety.THROTTLE_HARD_MAX)

    @FAST
    @given(rc=RC_VECTOR)
    def test_clamp_is_idempotent(self, rc):
        # clamp(clamp(x)) == clamp(x). Catches any path that re-runs clamp
        # with side-effects or mid-stream re-raises.
        once = safety.clamp_rc(rc)
        twice = safety.clamp_rc(once)
        self.assertEqual(once, twice)

    @FAST
    @given(rc=RC_VECTOR)
    def test_clamp_preserves_length(self, rc):
        self.assertEqual(len(safety.clamp_rc(rc)), 8)


# ──────────────────────────────────────────────────────────────────────────
# Strategies for safety.evaluate() inputs.
#
# The earlier version sampled `received_at` and `now` from the same wide
# uniform [0, 1e6], so 99.9999% of cases produced stale telemetry — every
# fresh-data branch was effectively unreached. Below, we anchor `now` and
# build telemetry strategies whose `received_at` is near `now` (so the
# fresh branch IS exercised) plus a parallel `STALE_*` strategy for the
# stale branch. The geofence-test strategies also include the poles and
# antimeridian, the actually-adversarial inputs for haversine.
# ──────────────────────────────────────────────────────────────────────────

NOW = 1000.0  # fixed reference; staleness comes from received_at offsets

FRESH_RECEIVED_AT = st.floats(
    min_value=NOW - 0.99, max_value=NOW, allow_nan=False, allow_infinity=False,
)


def _gps_strategy(received_at_strategy):
    return st.builds(
        msp.GpsReading,
        fix=st.booleans(),
        num_sat=st.integers(min_value=0, max_value=30),
        # Include the poles and antimeridian — the adversarial cases for
        # haversine and bearing math.
        lat_deg=st.floats(min_value=-90, max_value=90, allow_nan=False, allow_infinity=False),
        lon_deg=st.floats(min_value=-180, max_value=180, allow_nan=False, allow_infinity=False),
        alt_m=st.integers(min_value=0, max_value=2000),
        speed_cms=st.integers(min_value=0, max_value=10000),
        course_deg=st.floats(min_value=0, max_value=360, allow_nan=False, allow_infinity=False),
        received_at=received_at_strategy,
    )


def _attitude_strategy(received_at_strategy):
    return st.builds(
        msp.AttitudeReading,
        roll_deg=st.floats(min_value=-180, max_value=180, allow_nan=False, allow_infinity=False),
        pitch_deg=st.floats(min_value=-90, max_value=90, allow_nan=False, allow_infinity=False),
        yaw_deg=st.floats(min_value=0, max_value=360, allow_nan=False, allow_infinity=False),
        received_at=received_at_strategy,
    )


def _altitude_strategy(received_at_strategy):
    return st.builds(
        msp.AltitudeReading,
        alt_cm=st.integers(min_value=-1000, max_value=200000),
        vario_cms=st.integers(min_value=-2000, max_value=2000),
        received_at=received_at_strategy,
    )


def _analog_strategy(received_at_strategy):
    return st.builds(
        msp.AnalogReading,
        # Include sub-min_vbat values so the low_vbat branch fires.
        vbat_v=st.floats(min_value=0, max_value=30, allow_nan=False, allow_infinity=False),
        mah=st.integers(min_value=0, max_value=10000),
        rssi=st.integers(min_value=0, max_value=255),
        amperage_a=st.floats(min_value=0, max_value=200, allow_nan=False, allow_infinity=False),
        received_at=received_at_strategy,
    )


# Wide-anything strategies (mostly stale).
GPS = _gps_strategy(st.floats(min_value=0, max_value=NOW, allow_nan=False, allow_infinity=False))
ATTITUDE = _attitude_strategy(st.floats(min_value=0, max_value=NOW, allow_nan=False, allow_infinity=False))
ALTITUDE = _altitude_strategy(st.floats(min_value=0, max_value=NOW, allow_nan=False, allow_infinity=False))
ANALOG = _analog_strategy(st.floats(min_value=0, max_value=NOW, allow_nan=False, allow_infinity=False))

# Fresh telemetry that's actually live (received_at within staleness window).
GPS_FRESH = _gps_strategy(FRESH_RECEIVED_AT)
ATTITUDE_FRESH = _attitude_strategy(FRESH_RECEIVED_AT)
ALTITUDE_FRESH = _altitude_strategy(FRESH_RECEIVED_AT)
ANALOG_FRESH = _analog_strategy(FRESH_RECEIVED_AT)


class TestEvaluateInvariants(unittest.TestCase):
    """Properties that must hold for every input, valid or pathological."""

    @FAST
    @given(
        gps=st.none() | GPS_FRESH,
        att=st.none() | ATTITUDE_FRESH,
        alt=st.none() | ALTITUDE_FRESH,
        an=st.none() | ANALOG_FRESH,
        rc_at=st.none() | FRESH_RECEIVED_AT,
        in_transit=st.booleans(),
        heading_diverged=st.booleans(),
        acro_active=st.booleans(),
        distance=st.floats(min_value=0, max_value=10_000, allow_nan=False, allow_infinity=False),
        home=st.tuples(
            st.floats(min_value=-90, max_value=90, allow_nan=False, allow_infinity=False),
            st.floats(min_value=-180, max_value=180, allow_nan=False, allow_infinity=False),
        ) | st.just((None, None)),
    )
    def test_ok_iff_no_reasons(self, gps, att, alt, an, rc_at, in_transit,
                                heading_diverged, acro_active, distance, home):
        # Use mostly-fresh telemetry so the ok=True path is reachable for some
        # input combinations — the biconditional is checked in BOTH directions.
        s = safety.evaluate(
            safety.SafetyConfig(),
            home_lat=home[0], home_lon=home[1],
            gps=gps, altitude=alt, attitude=att, analog=an,
            now=NOW, last_rc_received_at=rc_at,
            distance_to_target_m=distance, in_transit=in_transit,
            heading_diverged=heading_diverged, acro_active=acro_active,
        )
        self.assertEqual(s.ok, len(s.reasons) == 0)

    @FAST
    @given(
        gps=st.none() | GPS,
        att=st.none() | ATTITUDE,
        alt=st.none() | ALTITUDE,
        an=st.none() | ANALOG,
        rc_at=st.none() | st.floats(min_value=0, max_value=NOW, allow_nan=False, allow_infinity=False),
        in_transit=st.booleans(),
        heading_diverged=st.booleans(),
        acro_active=st.booleans(),
        distance=st.one_of(
            st.none(),
            st.floats(min_value=0, max_value=1e9, allow_nan=False, allow_infinity=False),
        ),
        home=st.tuples(
            st.floats(min_value=-90, max_value=90, allow_nan=False, allow_infinity=False),
            st.floats(min_value=-180, max_value=180, allow_nan=False, allow_infinity=False),
        ) | st.just((None, None)),
    )
    def test_no_exceptions_on_any_input(self, gps, att, alt, an, rc_at,
                                         in_transit, heading_diverged,
                                         acro_active, distance, home):
        # Totality: `evaluate` must never raise, on any combination of inputs
        # — including geofence-active (home set), runaway-active (in_transit
        # with distance), heading_diverged, acro_active. Every code path
        # the function can take is reachable here.
        try:
            safety.evaluate(
                safety.SafetyConfig(),
                home_lat=home[0], home_lon=home[1],
                gps=gps, altitude=alt, attitude=att, analog=an,
                now=NOW, last_rc_received_at=rc_at,
                distance_to_target_m=distance, in_transit=in_transit,
                heading_diverged=heading_diverged, acro_active=acro_active,
            )
        except Exception as e:  # pragma: no cover
            self.fail(f"evaluate raised on valid input: {type(e).__name__}: {e}")

    @FAST
    @given(rc_at=st.none() | st.floats(min_value=0, max_value=1e6, allow_nan=False, allow_infinity=False))
    def test_no_telemetry_always_unsafe(self, rc_at):
        # All telemetry None must always produce ok=False, regardless of other inputs.
        s = safety.evaluate(
            safety.SafetyConfig(),
            home_lat=None, home_lon=None,
            gps=None, altitude=None, attitude=None, analog=None,
            now=NOW, last_rc_received_at=rc_at,
        )
        self.assertFalse(s.ok)

    @FAST
    @given(num_sat=st.integers(min_value=0, max_value=7))
    def test_low_sats_implies_low_sats_reason(self, num_sat):
        # Directional: num_sat below threshold MUST produce a low_sats_* reason.
        cfg = safety.SafetyConfig()
        gps = msp.GpsReading(
            fix=True, num_sat=num_sat, lat_deg=37.0, lon_deg=-122.0,
            alt_m=10, speed_cms=0, course_deg=0.0, received_at=NOW - 0.1,
        )
        att = msp.AttitudeReading(roll_deg=0, pitch_deg=0, yaw_deg=0, received_at=NOW - 0.1)
        alt = msp.AltitudeReading(alt_cm=500, vario_cms=0, received_at=NOW - 0.1)
        an = msp.AnalogReading(vbat_v=15.5, mah=0, rssi=80, amperage_a=10, received_at=NOW - 0.1)
        s = safety.evaluate(cfg, None, None, gps, alt, att, an, NOW,
                            last_rc_received_at=NOW - 0.1)
        self.assertTrue(any("low_sats" in r for r in s.reasons))


class TestHeadingTrackerInvariants(unittest.TestCase):
    @FAST
    @given(
        err=st.floats(min_value=-180, max_value=180, allow_nan=False, allow_infinity=False),
    )
    def test_never_trips_when_not_pitching_forward(self, err):
        # No matter how big the heading error, never trip when pitching_forward=False.
        # First call clears samples; second confirms still false.
        tr = safety.HeadingDivergenceTracker()
        self.assertFalse(tr.update(now=0.0, heading_err_deg=err, pitching_forward=False))
        self.assertFalse(tr.update(now=10.0, heading_err_deg=err, pitching_forward=False))

    @FAST
    @given(
        # Sample errors strictly below the threshold (with safety margin).
        err=st.floats(min_value=-59, max_value=59, allow_nan=False, allow_infinity=False),
    )
    def test_never_trips_strictly_below_threshold(self, err):
        # An err drawn from [−threshold, threshold) cannot push the mean above
        # threshold (mean is bounded by max(|err|)). Never trip.
        tr = safety.HeadingDivergenceTracker(threshold_deg=60.0, window_s=3.0)
        tripped = False
        for i in range(100):  # 10s of forward flight
            tripped = tr.update(now=i * 0.1, heading_err_deg=err, pitching_forward=True) or tripped
        self.assertFalse(tripped)


if __name__ == "__main__":
    unittest.main()
