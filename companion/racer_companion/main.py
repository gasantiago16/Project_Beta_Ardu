"""Main loop: telemetry in, controller, MSP_SET_RAW_RC out (only when active).

Safety invariant: this loop sends MSP_SET_RAW_RC only when state machine
returns is_active() True. In any other state it stays silent; BF reverts
to RX values via its MSP-override timeout (~500ms on BF 4.5+).
"""
from __future__ import annotations

import argparse
import json
import logging
import sys
import time

from . import config as cfg_mod
from . import mavlink as mavlink_mod
from . import msp
from . import nav
from . import safety as safety_mod
from . import state as state_mod

log = logging.getLogger("racer_companion")


def main() -> int:
    p = argparse.ArgumentParser(description="Project_Beta_Ardu companion runner")
    p.add_argument("--config", required=True)
    p.add_argument("--dry-run", action="store_true",
                   help="Do not open serial, do not send MSP. Logs decisions only.")
    p.add_argument("--log-level", default="INFO")
    args = p.parse_args()

    logging.basicConfig(
        level=args.log_level.upper(),
        format="%(asctime)s %(levelname)s %(name)s %(message)s",
    )

    cfg = cfg_mod.load(args.config)
    log.info("waypoint=%.7f,%.7f alt=%.1fm",
             cfg.waypoint.lat_deg, cfg.waypoint.lon_deg, cfg.waypoint.alt_m)

    client: msp.MspClient | None = None
    if not args.dry_run:
        client = msp.MspClient(cfg.companion.serial_port, cfg.companion.baud)
        log.info("MSP client open on %s @ %d", cfg.companion.serial_port, cfg.companion.baud)
    else:
        log.warning("DRY RUN — no serial port opened")

    mav_sender: mavlink_mod.MavlinkSender | None = None
    if cfg.companion.mavlink_publisher and not args.dry_run:
        try:
            pub = mavlink_mod.make_publisher(cfg.companion.mavlink_publisher)
            mav_sender = mavlink_mod.MavlinkSender(
                pub,
                mavlink_mod.SenderConfig(
                    heartbeat_hz=cfg.companion.mavlink_heartbeat_hz,
                    state_hz=cfg.companion.mavlink_state_hz,
                ),
            )
            log.info("MAVLink publisher: %s", cfg.companion.mavlink_publisher)
        except Exception as e:
            log.warning("MAVLink publisher init failed (%s) — continuing without", e)
            mav_sender = None

    ctx = state_mod.StateContext()
    last_gps: msp.GpsReading | None = None
    last_attitude: msp.AttitudeReading | None = None
    last_altitude: msp.AltitudeReading | None = None
    last_analog: msp.AnalogReading | None = None
    last_rc: list[int] | None = None
    home_lat: float | None = None
    home_lon: float | None = None

    loop_dt = 1.0 / cfg.companion.loop_hz
    telem_dt = 1.0 / cfg.companion.telemetry_hz
    last_telem = 0.0

    log_fh = open(cfg.companion.log_path, "a") if cfg.companion.log_path else None

    try:
        while True:
            t0 = time.monotonic()

            if client is not None:
                for cmd, payload in client.poll():
                    if cmd == msp.MSP_RAW_GPS:
                        last_gps = msp.decode_raw_gps(payload)
                        if (home_lat is None and last_gps.fix
                                and last_gps.num_sat >= cfg.safety.min_satellites):
                            home_lat = last_gps.lat_deg
                            home_lon = last_gps.lon_deg
                            log.info("Home set: %.7f, %.7f", home_lat, home_lon)
                    elif cmd == msp.MSP_ATTITUDE:
                        last_attitude = msp.decode_attitude(payload)
                    elif cmd == msp.MSP_ALTITUDE:
                        last_altitude = msp.decode_altitude(payload)
                    elif cmd == msp.MSP_ANALOG:
                        last_analog = msp.decode_analog(payload)
                    elif cmd == msp.MSP_RC:
                        last_rc = msp.decode_rc(payload)

            if t0 - last_telem > telem_dt:
                if client is not None:
                    client.send(msp.MSP_RAW_GPS)
                    client.send(msp.MSP_ATTITUDE)
                    client.send(msp.MSP_ALTITUDE)
                    client.send(msp.MSP_ANALOG)
                    client.send(msp.MSP_RC)
                last_telem = t0

            aux_active = (
                last_rc is not None
                and len(last_rc) > cfg.companion.aux_channel_index
                and last_rc[cfg.companion.aux_channel_index] >= cfg.companion.aux_active_us_min
            )

            status = safety_mod.evaluate(
                cfg.safety, home_lat, home_lon,
                last_gps, last_altitude, last_attitude, last_analog, t0,
            )

            current_alt_m = last_altitude.alt_cm / 100.0 if last_altitude else 0.0
            current_heading = last_attitude.yaw_deg if last_attitude else 0.0
            vario = last_altitude.vario_cms if last_altitude else 0
            if last_gps is not None and last_gps.fix:
                rc, dbg = nav.compute_rc(
                    cfg.waypoint, last_gps.lat_deg, last_gps.lon_deg,
                    last_altitude.alt_cm if last_altitude else 0,
                    current_heading, vario, cfg.nav,
                )
                distance = dbg["distance_m"]
            else:
                rc = [1500] * 8
                rc[msp.CH_THROTTLE] = cfg.nav.throttle_hover
                distance = 1e9

            new_state = state_mod.step(
                ctx, now=t0,
                aux_companion_active=aux_active,
                safety_ok=status.ok,
                current_alt_m=current_alt_m,
                distance_to_target_m=distance,
                climb_target_alt_m=cfg.companion.climb_target_alt_m,
                arrival_radius_m=cfg.nav.arrival_radius_m,
                arrival_dwell_s=cfg.companion.arrival_dwell_s,
            )

            rc_sent: list[int] | None = None
            if state_mod.is_active(new_state):
                rc_sent = safety_mod.clamp_rc(rc)
                if client is not None:
                    client.send_raw_rc(rc_sent)

            if log_fh is not None:
                log_fh.write(json.dumps({
                    "t": t0,
                    "state": new_state.name,
                    "aux_active": aux_active,
                    "safety_ok": status.ok,
                    "safety_reasons": status.reasons,
                    "rc_sent": rc_sent,
                    "alt_m": current_alt_m,
                    "heading_deg": current_heading,
                    "distance_m": distance,
                    "home": [home_lat, home_lon],
                }) + "\n")
                log_fh.flush()

            if mav_sender is not None:
                mav_sender.tick(
                    now=t0,
                    state_name=new_state.name,
                    distance_m=min(distance, 1e6),  # cap the 1e9 sentinel
                    alt_m=current_alt_m,
                    heading_deg=current_heading,
                    state_code=new_state.value,
                    safety_bitmap=mavlink_mod.safety_bitmap_from_reasons(status.reasons),
                    status_text=(", ".join(status.reasons[:3]) if status.reasons else None),
                )

            elapsed = time.monotonic() - t0
            if elapsed < loop_dt:
                time.sleep(loop_dt - elapsed)
    except KeyboardInterrupt:
        log.info("Shutting down (KeyboardInterrupt)")
    finally:
        if client is not None:
            client.close()
        if log_fh is not None:
            log_fh.close()
        if mav_sender is not None:
            mav_sender.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
