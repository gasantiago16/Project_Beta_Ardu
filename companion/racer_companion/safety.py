"""Safety: bound RC outputs, geofence, telemetry-staleness watchdog.

`evaluate()` returns SafetyStatus.ok=False if any check fails. The state
machine consumes that and forces RELEASED, which makes the main loop stop
sending MSP. BF then reverts to RX values within ~500ms (MSP-override timeout).

Defense-in-depth principle: every telemetry stream that can silently fail
(GPS, altitude, attitude, analog/vbat, RC) MUST have a "no_*_telemetry"
reason for the absent case, not just a "stale_*" reason for the present-but-old
case. A loose VBAT pad or unsubscribed MSP_RC must not pass the gate.

Sliding-window trackers (HeadingDivergenceTracker, FlightModeReader) are
caller-owned: the main loop holds them and feeds the booleans they produce
into evaluate(). Keeping evaluate() pure makes it trivially testable.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

from .msp import (
    CH_THROTTLE,
    AltitudeReading,
    AnalogReading,
    AttitudeReading,
    GpsReading,
    StatusReading,
)

RC_HARD_MIN = 1000
RC_HARD_MAX = 2000
THROTTLE_HARD_MAX = 1700

TELEMETRY_MAX_AGE_S = 1.0
RC_MAX_AGE_S = 1.0


@dataclass
class SafetyConfig:
    geofence_radius_m: float = 100.0
    min_satellites: int = 8
    min_vbat: float = 14.4
    max_distance_from_target_during_transit_m: float = 200.0


@dataclass
class SafetyStatus:
    ok: bool
    reasons: list[str] = field(default_factory=list)


def evaluate(
    cfg: SafetyConfig,
    home_lat: float | None,
    home_lon: float | None,
    gps: GpsReading | None,
    altitude: AltitudeReading | None,
    attitude: AttitudeReading | None,
    analog: AnalogReading | None,
    now: float,
    *,
    last_rc_received_at: float | None = None,
    distance_to_target_m: float | None = None,
    in_transit: bool = False,
    heading_diverged: bool = False,
    acro_active: bool = False,
) -> SafetyStatus:
    reasons: list[str] = []

    if gps is None:
        reasons.append("no_gps_telemetry")
    else:
        if not gps.fix:
            reasons.append("no_gps_fix")
        if gps.num_sat < cfg.min_satellites:
            reasons.append(f"low_sats_{gps.num_sat}")
        if now - gps.received_at > TELEMETRY_MAX_AGE_S:
            reasons.append("stale_gps")

    if altitude is None or now - altitude.received_at > TELEMETRY_MAX_AGE_S:
        reasons.append("stale_altitude")
    if attitude is None or now - attitude.received_at > TELEMETRY_MAX_AGE_S:
        reasons.append("stale_attitude")

    # Analog/vbat: absent-stream and stale-stream both block, mirroring GPS.
    # A loose VBAT pad or unsubscribed MSP_ANALOG must not silently pass.
    if analog is None:
        reasons.append("no_analog_telemetry")
    elif now - analog.received_at > TELEMETRY_MAX_AGE_S:
        reasons.append("stale_analog")
    elif analog.vbat_v < cfg.min_vbat:
        reasons.append(f"low_vbat_{analog.vbat_v:.1f}")

    # RC freshness: caller passes received_at of last MSP_RC. Without RC we
    # cannot trust aux_active gating; safety-critical to detect.
    if last_rc_received_at is None:
        reasons.append("no_rc_telemetry")
    elif now - last_rc_received_at > RC_MAX_AGE_S:
        reasons.append("stale_rc")

    if gps is not None and home_lat is not None and home_lon is not None and gps.fix:
        from .nav import haversine_m  # local import to avoid cycle on module load

        d = haversine_m(home_lat, home_lon, gps.lat_deg, gps.lon_deg)
        if d > cfg.geofence_radius_m:
            reasons.append(f"geofence_{d:.0f}m")

    # Transit-runaway: if we're in TRANSIT and somehow heading away from the
    # waypoint past the configured cap, abort. Catches a bearing-controller
    # blowup that the geofence (home-relative) wouldn't see if home is
    # between the drone and the waypoint.
    if in_transit and distance_to_target_m is not None:
        if distance_to_target_m > cfg.max_distance_from_target_during_transit_m:
            reasons.append(f"transit_runaway_{distance_to_target_m:.0f}m")

    if heading_diverged:
        reasons.append("heading_diverged")

    if acro_active:
        reasons.append("acro_active")

    return SafetyStatus(ok=(len(reasons) == 0), reasons=reasons)


def clamp_rc(channels: list[int]) -> list[int]:
    out = []
    for i, c in enumerate(channels):
        c = max(RC_HARD_MIN, min(RC_HARD_MAX, c))
        if i == CH_THROTTLE:
            c = min(THROTTLE_HARD_MAX, c)
        out.append(c)
    return out


# ── Sliding-window safety trackers ──────────────────────────────────────────


@dataclass
class HeadingDivergenceTracker:
    """Detects sustained bearing-tracking failure when actively flying forward.

    Race quads run mag_hardware=NONE (mag interference is severe), so yaw is
    gyro-integrated only. Yaw drift ~1°/min minimum; over a 5-min flight the
    bearing controller's frame can drift far enough that compute_rc commands
    a forward stick that flies AWAY from the waypoint. Geofence catches this
    eventually but only after lateral drift exceeds the radius — by then the
    drone has flown ~100m off-bearing.

    Tracks |heading_err_deg| while actively pitching forward (state==TRANSIT).
    If the time-windowed mean exceeds threshold for the full window, returns
    True and the caller appends a `heading_diverged` safety reason.

    Window resets whenever forward flight stops, so a brief yaw-error spike
    during HOLD or while turning at a waypoint does not trip it.

    Implementation note: uses `forward_started_at` rather than samples[0] for
    the "have we observed long enough?" gate. samples[0] timestamps suffer
    floating-point drift (sum of 0.1s dt's accumulates rounding error and
    flips the strict `<` check); a single anchor timestamp is robust.
    """
    threshold_deg: float = 60.0
    window_s: float = 3.0
    samples: deque = field(default_factory=deque)
    forward_started_at: float | None = None

    def update(self, now: float, heading_err_deg: float, pitching_forward: bool) -> bool:
        if not pitching_forward:
            self.samples.clear()
            self.forward_started_at = None
            return False
        if self.forward_started_at is None:
            self.forward_started_at = now
        self.samples.append((now, abs(heading_err_deg)))
        # Expire old samples outside the window.
        while self.samples and now - self.samples[0][0] > self.window_s:
            self.samples.popleft()
        # Need at least window_s of forward flight before tripping — avoids
        # false trip on a single bad attitude packet at start of TRANSIT.
        if now - self.forward_started_at < self.window_s:
            return False
        if not self.samples:
            return False
        mean = sum(e for _, e in self.samples) / len(self.samples)
        return mean > self.threshold_deg


@dataclass
class FlightModeReader:
    """Decodes BF flight-mode flags via MSP_STATUS_EX + MSP_BOXNAMES.

    The flightModeFlags field is a bitmap whose bits are indices into the
    box-name list returned by MSP_BOXNAMES. Without box names the bitmap is
    just opaque bits, so this reader is two-stage:
      1) update_box_names() sets ANGLE/HORIZON indices once at startup
      2) update_status() ingests every MSP_STATUS_EX response

    is_acro_active() returns:
      None  — box names not yet known (caller should treat as fail-open
              during the first few seconds; safelock.lua is primary defense)
      True  — armed and neither ANGLE nor HORIZON active (assumed ACRO)
      False — ANGLE or HORIZON active
    """
    box_names: list[str] | None = None
    angle_idx: int | None = None
    horizon_idx: int | None = None
    flight_mode_flags: int = 0

    def update_box_names(self, names: list[str]) -> None:
        self.box_names = list(names)
        self.angle_idx = next(
            (i for i, n in enumerate(self.box_names) if n.upper() == "ANGLE"), None,
        )
        self.horizon_idx = next(
            (i for i, n in enumerate(self.box_names) if n.upper() == "HORIZON"), None,
        )

    def update_status(self, status: StatusReading) -> None:
        self.flight_mode_flags = status.flight_mode_flags

    def is_acro_active(self) -> bool | None:
        if self.box_names is None:
            return None
        angle_on = (
            self.angle_idx is not None
            and (self.flight_mode_flags & (1 << self.angle_idx)) != 0
        )
        horizon_on = (
            self.horizon_idx is not None
            and (self.flight_mode_flags & (1 << self.horizon_idx)) != 0
        )
        return not (angle_on or horizon_on)
