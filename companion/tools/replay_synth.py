"""Synthetic GPS replay — runs the controller offline against a fake trajectory.

Useful for tuning P gains and visualizing controller behavior before any flight.
Generates a position N meters from the configured waypoint, integrates the
controller's outputs as if the quad responded immediately, and prints what
would have been commanded each tick.

USAGE:
    python -m tools.replay_synth --config /etc/racer-companion/config.json
"""
from __future__ import annotations

import argparse
import math

from racer_companion import config as cfg_mod
from racer_companion import nav


def displace(lat: float, lon: float, bearing_deg: float, distance_m: float) -> tuple[float, float]:
    R = nav.EARTH_RADIUS_M
    brg = math.radians(bearing_deg)
    d = distance_m / R
    phi1 = math.radians(lat)
    lam1 = math.radians(lon)
    phi2 = math.asin(math.sin(phi1) * math.cos(d) + math.cos(phi1) * math.sin(d) * math.cos(brg))
    lam2 = lam1 + math.atan2(
        math.sin(brg) * math.sin(d) * math.cos(phi1),
        math.cos(d) - math.sin(phi1) * math.sin(phi2),
    )
    return math.degrees(phi2), math.degrees(lam2)


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--config", required=True)
    p.add_argument("--start-bearing", type=float, default=180.0,
                   help="Compass bearing FROM waypoint to fake start (degrees)")
    p.add_argument("--start-distance-m", type=float, default=50.0)
    p.add_argument("--ticks", type=int, default=300)
    p.add_argument("--ground-speed-mps", type=float, default=3.0)
    p.add_argument("--current-heading", type=float, default=0.0)
    p.add_argument("--dt", type=float, default=0.1)
    args = p.parse_args()

    cfg = cfg_mod.load(args.config)
    wp = cfg.waypoint

    cur_lat, cur_lon = displace(wp.lat_deg, wp.lon_deg, args.start_bearing, args.start_distance_m)
    cur_alt_cm = int(wp.alt_m * 100)
    cur_heading = args.current_heading

    print(f"start ({cur_lat:.7f}, {cur_lon:.7f}) heading {cur_heading}°")
    print(f"target ({wp.lat_deg:.7f}, {wp.lon_deg:.7f}) alt {wp.alt_m}m")

    for i in range(args.ticks):
        rc, dbg = nav.compute_rc(wp, cur_lat, cur_lon, cur_alt_cm, cur_heading, 0, cfg.nav)
        t = i * args.dt
        print(
            f"t={t:5.1f}s d={dbg['distance_m']:6.1f}m "
            f"brg={dbg['target_bearing_deg']:6.1f}° h_err={dbg['heading_err_deg']:+6.1f}° "
            f"hdg={cur_heading:6.1f}° RC[r,p,y,t]={rc[:4]}"
        )
        if dbg["arrived"]:
            print("ARRIVED")
            return 0

        # Crude integration of controller outputs.
        forward_us = rc[nav.CH_PITCH] - 1500
        forward_frac = max(0.0, min(1.0, forward_us / cfg.nav.pitch_max_us))
        speed = forward_frac * args.ground_speed_mps

        yaw_us = rc[nav.CH_YAW] - 1500
        yaw_rate_dps = yaw_us / cfg.nav.yaw_kp  # rough inverse of controller gain
        cur_heading = (cur_heading + yaw_rate_dps * args.dt) % 360.0

        cur_lat, cur_lon = displace(cur_lat, cur_lon, cur_heading, speed * args.dt)

    print("REACHED TICK LIMIT WITHOUT ARRIVAL — check tuning")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
