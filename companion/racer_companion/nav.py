"""Navigation math + simple P controllers.

Bearing convention: 0° = North, 90° = East. Compass heading from MSP_ATTITUDE
follows the same convention (0° = North) on a calibrated BF setup.

Stick mapping assumes BF is in ANGLE mode while companion-active. In ANGLE,
roll/pitch sticks command angle (1500=level, 2000=max tilt). In ACRO this
controller produces unsafe behavior — operator MUST switch BF to ANGLE before
flipping companion-active AUX. See docs/safety_runbook.md.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from .msp import CH_PITCH, CH_ROLL, CH_THROTTLE, CH_YAW, CHANNEL_MID

EARTH_RADIUS_M = 6_371_000.0


@dataclass
class Waypoint:
    lat_deg: float
    lon_deg: float
    alt_m: float


@dataclass
class NavTuning:
    yaw_kp: float = 4.0
    yaw_max_us: float = 200.0
    pitch_kp_per_m: float = 4.0
    pitch_max_us: float = 150.0
    yaw_align_threshold_deg: float = 25.0
    throttle_hover: int = 1300
    throttle_kp_per_m: float = 6.0
    throttle_kd_per_cms: float = 0.4
    throttle_max_offset: int = 200
    arrival_radius_m: float = 3.0
    # HOLD-state position controller (body-frame, no yaw). Small gains so
    # we lazily counter wind drift inside the hold disc instead of either
    # (a) doing nothing (old behavior — drone would drift indefinitely until
    # state.py kicked it back to TRANSIT) or (b) commanding aggressive
    # corrections that overshoot. Pair with `arrival_radius_m`.
    hold_kp_per_m: float = 8.0
    hold_max_offset_us: float = 60.0


# Local ENU approximation: degrees → meters for small offsets near a given
# latitude. 111195 m/deg latitude is the geographic mean (off by <0.3% in
# the polar/equatorial extremes; far better than haversine accuracy at
# sub-100m scales). For longitude, scale by cos(lat).
_M_PER_DEG_LAT = 111195.0


def position_error_body_m(
    waypoint_lat: float, waypoint_lon: float,
    current_lat: float, current_lon: float,
    current_heading_deg: float,
) -> tuple[float, float]:
    """Project the world-frame waypoint→drone offset into the drone's body
    frame. Returns (forward_m, right_m). Positive forward = waypoint is
    ahead of the drone's nose; positive right = waypoint is to the right.

    Used by the HOLD position controller (no yaw, just translation). For
    distances <100m the flat-earth ENU approximation here is precise enough
    that haversine isn't worth the cosine.
    """
    north_m = (waypoint_lat - current_lat) * _M_PER_DEG_LAT
    east_m = (waypoint_lon - current_lon) * _M_PER_DEG_LAT * math.cos(math.radians(current_lat))
    h = math.radians(current_heading_deg)
    forward = north_m * math.cos(h) + east_m * math.sin(h)
    right = -north_m * math.sin(h) + east_m * math.cos(h)
    return forward, right


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
    return EARTH_RADIUS_M * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dlam = math.radians(lon2 - lon1)
    y = math.sin(dlam) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlam)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def heading_error_deg(target_bearing: float, current_heading: float) -> float:
    return (target_bearing - current_heading + 540.0) % 360.0 - 180.0


def yaw_command_us(target_bearing: float, current_heading: float, t: NavTuning) -> int:
    err = heading_error_deg(target_bearing, current_heading)
    delta = max(-t.yaw_max_us, min(t.yaw_max_us, t.yaw_kp * err))
    return int(round(CHANNEL_MID + delta))


def pitch_command_us(distance_m: float, heading_err_deg: float, t: NavTuning) -> int:
    if abs(heading_err_deg) > t.yaw_align_threshold_deg:
        return CHANNEL_MID
    forward = max(0.0, min(t.pitch_max_us, t.pitch_kp_per_m * distance_m))
    return int(round(CHANNEL_MID + forward))


def throttle_command_us(target_alt_m: float, current_alt_cm: int, vario_cms: int, t: NavTuning) -> int:
    err_m = target_alt_m - (current_alt_cm / 100.0)
    raw = t.throttle_hover + (t.throttle_kp_per_m * err_m) - (t.throttle_kd_per_cms * vario_cms)
    lo = t.throttle_hover - t.throttle_max_offset
    hi = t.throttle_hover + t.throttle_max_offset
    return int(round(max(lo, min(hi, raw))))


def hold_command_us(
    waypoint: "Waypoint", current_lat: float, current_lon: float,
    current_heading_deg: float, t: NavTuning,
) -> tuple[int, int]:
    """Body-frame P controller for HOLD state. Returns (roll_us, pitch_us).
    Yaw stays centered — we don't rotate the drone to hold position; we just
    translate. In ANGLE mode, roll = bank-right → drift right; pitch
    forward-stick = nose down → drift forward. Both stick directions match
    the drift direction we want to command.

    Output is bounded to ±hold_max_offset_us. Gains are intentionally smaller
    than the TRANSIT pitch controller — we want to lazily oppose wind drift,
    not slingshot back to the waypoint.

    NaN/Inf inputs (corrupt GPS, attitude packet glitch) collapse to centered
    sticks rather than raising — `int(round(NaN))` would propagate a
    ValueError up through the loop. main.py gates on `gps.fix` so this is
    defense-in-depth, not the primary protection.
    """
    forward_m, right_m = position_error_body_m(
        waypoint.lat_deg, waypoint.lon_deg, current_lat, current_lon, current_heading_deg,
    )
    if not (math.isfinite(forward_m) and math.isfinite(right_m)):
        return CHANNEL_MID, CHANNEL_MID
    max_off = t.hold_max_offset_us
    roll_delta = max(-max_off, min(max_off, t.hold_kp_per_m * right_m))
    pitch_delta = max(-max_off, min(max_off, t.hold_kp_per_m * forward_m))
    return (
        int(round(CHANNEL_MID + roll_delta)),
        int(round(CHANNEL_MID + pitch_delta)),
    )


def compute_rc(
    waypoint: Waypoint,
    current_lat: float,
    current_lon: float,
    current_alt_cm: int,
    current_heading_deg: float,
    vario_cms: int,
    t: NavTuning,
    *,
    climb_phase: bool = False,
    in_hold: bool = False,
) -> tuple[list[int], dict]:
    """Compute RC channels for the current tick.

    `climb_phase=True` zeros horizontal stick output (roll/pitch/yaw centered)
    while still computing throttle to climb-target altitude. Defense-in-depth
    against low-altitude lateral travel during the CLIMB state, which would
    happen on a downhill takeoff site if the bearing controller engaged at
    ground level.

    `in_hold=True` runs the body-frame HOLD position controller (small roll
    + pitch corrections, yaw centered) instead of the bearing-track / arrival
    branches. Gives wind-drift compensation while in HOLD; the state machine
    still kicks back to TRANSIT past `arrival_radius_m × 1.5`.

    Mutually exclusive flags: climb_phase wins if both are set (climb is the
    earlier phase chronologically and a more dangerous one to mis-state).
    """
    distance = haversine_m(current_lat, current_lon, waypoint.lat_deg, waypoint.lon_deg)
    target_bearing = bearing_deg(current_lat, current_lon, waypoint.lat_deg, waypoint.lon_deg)
    h_err = heading_error_deg(target_bearing, current_heading_deg)

    if climb_phase:
        roll = CHANNEL_MID
        pitch = CHANNEL_MID
        yaw = CHANNEL_MID
    elif in_hold:
        roll, pitch = hold_command_us(
            waypoint, current_lat, current_lon, current_heading_deg, t,
        )
        yaw = CHANNEL_MID
    elif distance < t.arrival_radius_m:
        roll = CHANNEL_MID
        pitch = CHANNEL_MID
        yaw = CHANNEL_MID
    else:
        roll = CHANNEL_MID
        pitch = pitch_command_us(distance, h_err, t)
        yaw = yaw_command_us(target_bearing, current_heading_deg, t)

    throttle = throttle_command_us(waypoint.alt_m, current_alt_cm, vario_cms, t)

    rc = [CHANNEL_MID] * 8
    rc[CH_ROLL] = roll
    rc[CH_PITCH] = pitch
    rc[CH_YAW] = yaw
    rc[CH_THROTTLE] = throttle
    return rc, {
        "distance_m": distance,
        "target_bearing_deg": target_bearing,
        "heading_err_deg": h_err,
        "arrived": distance < t.arrival_radius_m,
        "climb_phase": climb_phase,
        "in_hold": in_hold,
    }
