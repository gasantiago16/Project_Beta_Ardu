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
) -> tuple[list[int], dict]:
    """Compute RC channels for the current tick.

    `climb_phase=True` zeros horizontal stick output (roll/pitch/yaw centered)
    while still computing throttle to climb-target altitude. Defense-in-depth
    against low-altitude lateral travel during the CLIMB state, which would
    happen on a downhill takeoff site if the bearing controller engaged at
    ground level.
    """
    distance = haversine_m(current_lat, current_lon, waypoint.lat_deg, waypoint.lon_deg)
    target_bearing = bearing_deg(current_lat, current_lon, waypoint.lat_deg, waypoint.lon_deg)
    h_err = heading_error_deg(target_bearing, current_heading_deg)

    if climb_phase:
        roll = CHANNEL_MID
        pitch = CHANNEL_MID
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
    }
