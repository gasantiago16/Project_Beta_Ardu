"""Main loop: telemetry in, controller, MSP_SET_RAW_RC out (only when active).

Safety invariant: this loop sends MSP_SET_RAW_RC only when state machine
returns is_active() True. In any other state it stays silent; BF reverts
to RX values via its MSP-override timeout (~500ms on BF 4.5+).

Tick ordering matters and is non-obvious:
  1. Poll MSP — fold incoming packets into last_* readings + receive timestamps.
  2. Periodically request telemetry (incl. MSP_STATUS_EX, MSP_BOXNAMES once).
  3. Compute distance/bearing/heading_err from the last GPS+attitude readings.
  4. Update sliding-window trackers (heading divergence, flight-mode reader).
  5. Evaluate safety using the prior tick's state context for runaway/in_transit.
  6. Step state machine with the new safety_ok.
  7. Compute RC, gating climb_phase by the NEW state.
  8. Send MSP_SET_RAW_RC only if new state is_active().

Trackers feed booleans into evaluate(); they are caller-owned so evaluate()
stays pure and trivially testable.
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

# How often to request MSP_BOXNAMES until we've received it. After first
# success the reader is fully primed; further requests are a waste of bytes.
BOXNAMES_RETRY_S = 5.0


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
    last_rc_received_at: float | None = None
    home_lat: float | None = None
    home_lon: float | None = None

    heading_tracker = safety_mod.HeadingDivergenceTracker()
    mode_reader = safety_mod.FlightModeReader()
    last_boxnames_request = -1e9

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
                        last_rc_received_at = t0
                    elif cmd == msp.MSP_STATUS_EX:
                        try:
                            mode_reader.update_status(msp.decode_status_ex(payload))
                        except ValueError:
                            pass
                    elif cmd == msp.MSP_BOXNAMES:
                        names = msp.decode_box_names(payload)
                        if names:
                            mode_reader.update_box_names(names)
                            log.info("BF box names received (%d entries; angle_idx=%s, horizon_idx=%s)",
                                     len(names), mode_reader.angle_idx, mode_reader.horizon_idx)

            if t0 - last_telem > telem_dt:
                if client is not None:
                    client.send(msp.MSP_RAW_GPS)
                    client.send(msp.MSP_ATTITUDE)
                    client.send(msp.MSP_ALTITUDE)
                    client.send(msp.MSP_ANALOG)
                    client.send(msp.MSP_RC)
                    client.send(msp.MSP_STATUS_EX)
                last_telem = t0

            # Re-request box names every BOXNAMES_RETRY_S until primed (handles
            # FC reboot mid-flight and slow startups).
            if (mode_reader.box_names is None
                    and t0 - last_boxnames_request > BOXNAMES_RETRY_S
                    and client is not None):
                client.send(msp.MSP_BOXNAMES)
                last_boxnames_request = t0

            aux_active = (
                last_rc is not None
                and len(last_rc) > cfg.companion.aux_channel_index
                and last_rc[cfg.companion.aux_channel_index] >= cfg.companion.aux_active_us_min
            )

            # ── Compute geometry without commanding RC yet ─────────────────
            current_alt_m = last_altitude.alt_cm / 100.0 if last_altitude else 0.0
            current_heading = last_attitude.yaw_deg if last_attitude else 0.0
            vario = last_altitude.vario_cms if last_altitude else 0
            if last_gps is not None and last_gps.fix:
                distance = nav.haversine_m(
                    last_gps.lat_deg, last_gps.lon_deg,
                    cfg.waypoint.lat_deg, cfg.waypoint.lon_deg,
                )
                target_bearing = nav.bearing_deg(
                    last_gps.lat_deg, last_gps.lon_deg,
                    cfg.waypoint.lat_deg, cfg.waypoint.lon_deg,
                )
                heading_err = nav.heading_error_deg(target_bearing, current_heading)
            else:
                distance = 1e9
                target_bearing = 0.0
                heading_err = 0.0

            # ── Update sliding-window trackers (uses prior tick's state) ──
            prev_state = ctx.state
            pitching_forward = (prev_state == state_mod.State.TRANSIT
                                and abs(heading_err) < cfg.nav.yaw_align_threshold_deg)
            heading_diverged = heading_tracker.update(t0, heading_err, pitching_forward)
            acro = mode_reader.is_acro_active()
            # Fail-open on None (box names not yet primed) — safelock.lua is
            # primary defense and the runbook gates field operation on it.
            acro_active = (acro is True)

            # ── Safety eval ────────────────────────────────────────────────
            status = safety_mod.evaluate(
                cfg.safety, home_lat, home_lon,
                last_gps, last_altitude, last_attitude, last_analog, t0,
                last_rc_received_at=last_rc_received_at,
                distance_to_target_m=distance,
                in_transit=(prev_state == state_mod.State.TRANSIT),
                heading_diverged=heading_diverged,
                acro_active=acro_active,
            )

            # ── State step ────────────────────────────────────────────────
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

            # ── RC compute (uses new state for climb-phase gating) ────────
            climb_phase = (
                new_state == state_mod.State.CLIMB
                and current_alt_m < 0.8 * cfg.companion.climb_target_alt_m
            )
            if last_gps is not None and last_gps.fix:
                rc, _dbg = nav.compute_rc(
                    cfg.waypoint, last_gps.lat_deg, last_gps.lon_deg,
                    last_altitude.alt_cm if last_altitude else 0,
                    current_heading, vario, cfg.nav,
                    climb_phase=climb_phase,
                )
            else:
                rc = [1500] * 8
                rc[msp.CH_THROTTLE] = cfg.nav.throttle_hover

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
                    "heading_err_deg": heading_err,
                    "distance_m": distance,
                    "climb_phase": climb_phase,
                    "acro_active": acro_active,
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
