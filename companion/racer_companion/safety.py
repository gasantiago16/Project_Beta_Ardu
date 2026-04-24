"""Safety: bound RC outputs, geofence, telemetry-staleness watchdog.

`evaluate()` returns SafetyStatus.ok=False if any check fails. The state
machine consumes that and forces RELEASED, which makes the main loop stop
sending MSP. BF then reverts to RX values within ~500ms (MSP-override timeout).
"""
from __future__ import annotations

from dataclasses import dataclass, field

from .msp import (
    CH_THROTTLE,
    AltitudeReading,
    AnalogReading,
    AttitudeReading,
    GpsReading,
)

RC_HARD_MIN = 1000
RC_HARD_MAX = 2000
THROTTLE_HARD_MAX = 1700

TELEMETRY_MAX_AGE_S = 1.0


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
    if analog is not None and analog.vbat_v < cfg.min_vbat:
        reasons.append(f"low_vbat_{analog.vbat_v:.1f}")

    if gps is not None and home_lat is not None and home_lon is not None and gps.fix:
        from .nav import haversine_m  # local import to avoid cycle on module load

        d = haversine_m(home_lat, home_lon, gps.lat_deg, gps.lon_deg)
        if d > cfg.geofence_radius_m:
            reasons.append(f"geofence_{d:.0f}m")

    return SafetyStatus(ok=(len(reasons) == 0), reasons=reasons)


def clamp_rc(channels: list[int]) -> list[int]:
    out = []
    for i, c in enumerate(channels):
        c = max(RC_HARD_MIN, min(RC_HARD_MAX, c))
        if i == CH_THROTTLE:
            c = min(THROTTLE_HARD_MAX, c)
        out.append(c)
    return out
