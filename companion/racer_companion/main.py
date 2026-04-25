"""Main loop: telemetry in, controller, MSP_SET_RAW_RC out (only when active).

Safety invariant: this loop sends MSP_SET_RAW_RC only when state machine
returns is_active() True. In any other state it stays silent; BF reverts
to RX values via its MSP-override timeout (~500ms on BF 4.5+).

The FlightController abstraction (BetaflightAdapter today) owns all wire-
level state — telemetry decoding, polling cadence, flight-mode tracking.
This loop deals only in TelemetrySnapshot + a few derived booleans.

Tick ordering matters and is non-obvious:
  1. fc.tick() — consume bytes, refresh snapshot, request next telem batch.
  2. Compute distance/bearing/heading_err from the snapshot's GPS+attitude.
  3. Update sliding-window trackers (heading divergence) using prior state.
  4. Evaluate safety using prior state for runaway/in_transit checks.
  5. Step state machine with the new safety_ok.
  6. Compute RC, gating climb_phase / in_hold by the NEW state.
  7. fc.send_overrides() only if new state is_active().
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
from .fc.betaflight import BetaflightAdapter

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

    fc: BetaflightAdapter | None = None
    if not args.dry_run:
        fc = BetaflightAdapter.open(
            cfg.companion.serial_port, cfg.companion.baud,
            telemetry_period_s=1.0 / cfg.companion.telemetry_hz,
        )
        log.info("FC open on %s @ %d", cfg.companion.serial_port, cfg.companion.baud)
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
    home_lat: float | None = None
    home_lon: float | None = None
    heading_tracker = safety_mod.HeadingDivergenceTracker()

    loop_dt = 1.0 / cfg.companion.loop_hz
    log_fh = open(cfg.companion.log_path, "a") if cfg.companion.log_path else None

    try:
        while True:
            t0 = time.monotonic()

            # ── FC tick (poll, decode, periodic telem requests) ──────────
            snap = fc.tick(t0) if fc is not None else _empty_snapshot()

            # Set home on first GPS fix that meets sat-count requirement.
            if (home_lat is None and snap.gps is not None and snap.gps.fix
                    and snap.gps.num_sat >= cfg.safety.min_satellites):
                home_lat = snap.gps.lat_deg
                home_lon = snap.gps.lon_deg
                log.info("Home set: %.7f, %.7f", home_lat, home_lon)

            aux_active = (
                snap.rc is not None
                and len(snap.rc) > cfg.companion.aux_channel_index
                and snap.rc[cfg.companion.aux_channel_index] >= cfg.companion.aux_active_us_min
            )

            # ── Compute geometry without commanding RC yet ─────────────────
            current_alt_m = snap.altitude.alt_cm / 100.0 if snap.altitude else 0.0
            current_heading = snap.attitude.yaw_deg if snap.attitude else 0.0
            vario = snap.altitude.vario_cms if snap.altitude else 0
            if snap.gps is not None and snap.gps.fix:
                distance = nav.haversine_m(
                    snap.gps.lat_deg, snap.gps.lon_deg,
                    cfg.waypoint.lat_deg, cfg.waypoint.lon_deg,
                )
                target_bearing = nav.bearing_deg(
                    snap.gps.lat_deg, snap.gps.lon_deg,
                    cfg.waypoint.lat_deg, cfg.waypoint.lon_deg,
                )
                heading_err = nav.heading_error_deg(target_bearing, current_heading)
            else:
                distance = 1e9
                target_bearing = 0.0
                heading_err = 0.0

            # ── Sliding-window trackers (uses prior tick's state) ─────────
            prev_state = ctx.state
            pitching_forward = (prev_state == state_mod.State.TRANSIT
                                and abs(heading_err) < cfg.nav.yaw_align_threshold_deg)
            heading_diverged = heading_tracker.update(t0, heading_err, pitching_forward)
            acro_check = fc.is_acro_active() if fc is not None else None
            # Fail-open on None (BOXNAMES not yet primed) — safelock.lua is the
            # primary defense and the runbook gates field operation on it.
            acro_active = (acro_check is True)

            # ── Safety eval ────────────────────────────────────────────────
            status = safety_mod.evaluate(
                cfg.safety, home_lat, home_lon,
                snap.gps, snap.altitude, snap.attitude, snap.analog, t0,
                last_rc_received_at=snap.rc_received_at,
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

            # ── RC compute (uses new state for climb-phase / hold gating) ─
            climb_phase = (
                new_state == state_mod.State.CLIMB
                and current_alt_m < 0.8 * cfg.companion.climb_target_alt_m
            )
            in_hold = (new_state == state_mod.State.HOLD)
            if snap.gps is not None and snap.gps.fix:
                rc, _dbg = nav.compute_rc(
                    cfg.waypoint, snap.gps.lat_deg, snap.gps.lon_deg,
                    snap.altitude.alt_cm if snap.altitude else 0,
                    current_heading, vario, cfg.nav,
                    climb_phase=climb_phase,
                    in_hold=in_hold,
                )
            else:
                rc = [1500] * 8
                rc[msp.CH_THROTTLE] = cfg.nav.throttle_hover

            rc_sent: list[int] | None = None
            if state_mod.is_active(new_state):
                rc_sent = safety_mod.clamp_rc(rc)
                if fc is not None:
                    fc.send_overrides(rc_sent)

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
                    "in_hold": in_hold,
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
        if fc is not None:
            fc.close()
        if log_fh is not None:
            log_fh.close()
        if mav_sender is not None:
            mav_sender.close()

    return 0


def _empty_snapshot():
    """Empty snapshot for dry-run mode (no FC, no telemetry)."""
    from .fc.protocol import TelemetrySnapshot
    return TelemetrySnapshot()


if __name__ == "__main__":
    sys.exit(main())
