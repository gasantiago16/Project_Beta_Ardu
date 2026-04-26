"""Probe BF SITL via TCP MSP and print what GPS it's reporting.

Use BEFORE picking the waypoint lat/lon for `companion/config/sim_waypoint.json`.
BF's reported lat/lon depends on (a) its build's GPS origin defaults and
(b) the NED meters our Pegasus bridge has been sending in fdm.position_xyz.
Until we run, we don't know what coordinate frame BF is in. This tool
reads MSP_RAW_GPS for a few seconds and prints what BF says.

Run with the full Phase-2 stack up:
    docker compose -f sitl/docker-compose.yml up -d        # T1
    python ~/Project_Beta_Ardu/integrations/orchestrators/final_world_betaflight.py   # T2 (or any FDM source)
    python -m integrations.tools.discover_bf_gps           # T3 — this script

Or against just BF SITL with a synthetic FDM source:
    docker compose -f sitl/docker-compose.yml up -d
    python -m integrations.tools.fake_pegasus_loop --duration 30 &
    python -m integrations.tools.discover_bf_gps

If the GPS reads (0, 0, fix=False, sats=0) the whole time, BF isn't
getting a coherent position from our FDM bridge — diagnose with the bridge
stats first.
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time
from pathlib import Path

# Make racer_companion importable without requiring `pip install -e companion`.
_COMPANION = Path(__file__).resolve().parents[2] / "companion"
sys.path.insert(0, str(_COMPANION))

from racer_companion import msp  # noqa: E402


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--bf-host", default="127.0.0.1")
    p.add_argument("--bf-msp-port", type=int, default=5761)
    p.add_argument("--duration", type=float, default=15.0)
    p.add_argument("--rate-hz", type=float, default=2.0,
                   help="How often to poll MSP_RAW_GPS.")
    args = p.parse_args()

    url = f"tcp://{args.bf_host}:{args.bf_msp_port}"
    print(f"[discover_bf_gps] Connecting to {url}...", flush=True)
    try:
        client = msp.MspClient(url)
    except OSError as e:
        print(
            f"[discover_bf_gps] FATAL: cannot connect to BF SITL at "
            f"{args.bf_host}:{args.bf_msp_port} ({type(e).__name__}: {e}).\n"
            f"  Is `docker compose -f sitl/docker-compose.yml up` running?\n"
            f"  Override --bf-host / --bf-msp-port if SITL is elsewhere.",
            file=sys.stderr,
        )
        return 2
    print("[discover_bf_gps] Connected. Polling MSP_RAW_GPS...", flush=True)
    print(f"  {'time':>6}  {'fix':>4}  {'sats':>4}  {'lat':>12}  {'lon':>12}  {'alt_m':>6}",
          flush=True)

    period = 1.0 / max(0.5, args.rate_hz)
    t0 = time.monotonic()
    last_request = 0.0
    last_seen_gps = None

    try:
        while time.monotonic() - t0 < args.duration:
            now = time.monotonic()
            if now - last_request >= period:
                try:
                    client.send(msp.MSP_RAW_GPS)
                    last_request = now
                except OSError as e:
                    print(f"[discover_bf_gps] send failed: {e}", file=sys.stderr)
                    return 3
            try:
                for cmd, payload in client.poll():
                    if cmd == msp.MSP_RAW_GPS:
                        gps = msp.decode_raw_gps(payload)
                        last_seen_gps = gps
                        print(
                            f"  {now - t0:>6.1f}  {str(gps.fix):>4}  "
                            f"{gps.num_sat:>4}  {gps.lat_deg:>12.7f}  "
                            f"{gps.lon_deg:>12.7f}  {gps.alt_m:>6}",
                            flush=True,
                        )
            except OSError as e:
                print(f"[discover_bf_gps] poll failed: {e}", file=sys.stderr)
                return 3
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n[discover_bf_gps] Interrupted", flush=True)
    finally:
        client.close()

    print("\n[discover_bf_gps] Summary:", flush=True)
    if last_seen_gps is None:
        print("  NO GPS PACKETS RECEIVED. BF isn't replying to MSP_RAW_GPS — "
              "check the MSP TCP path is up (containers running, port 5761 "
              "reachable).", flush=True)
        return 1
    if not last_seen_gps.fix or last_seen_gps.num_sat < 6:
        print(f"  GPS unstable: fix={last_seen_gps.fix} sats={last_seen_gps.num_sat}.",
              flush=True)
        print("  Likely cause: Phase-2 bridge isn't sending FDM, or BF's "
              "internal GPS module hasn't accepted the position yet.", flush=True)
        return 1
    print(f"  Last good GPS: lat={last_seen_gps.lat_deg:.7f} "
          f"lon={last_seen_gps.lon_deg:.7f} alt={last_seen_gps.alt_m}m "
          f"sats={last_seen_gps.num_sat}", flush=True)
    print()
    print("  Pick a waypoint near these coords. Quick recipes:", flush=True)
    # Convert a fixed metric offset to degrees, scaling lon by cos(lat) so
    # the printed "north" and "east" distances are physically equal at any
    # latitude. WGS-84 mean: 1 deg lat ≈ 110_540 m; 1 deg lon ≈ 111_320·cos(lat) m.
    target_m = 30.0
    nudge_lat = target_m / 110_540.0
    cos_lat = max(0.05, math.cos(math.radians(last_seen_gps.lat_deg)))
    nudge_lon = target_m / (111_320.0 * cos_lat)
    print(f"  ~{target_m:.0f} m north:   "
          f"lat={last_seen_gps.lat_deg + nudge_lat:.7f} "
          f"lon={last_seen_gps.lon_deg:.7f}", flush=True)
    print(f"  ~{target_m:.0f} m east:    "
          f"lat={last_seen_gps.lat_deg:.7f} "
          f"lon={last_seen_gps.lon_deg + nudge_lon:.7f}", flush=True)
    print(f"  Update `waypoint.lat_deg` / `waypoint.lon_deg` in "
          f"companion/config/sim_waypoint.json accordingly.", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
