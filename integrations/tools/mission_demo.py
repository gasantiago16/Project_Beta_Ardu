"""Autonomous mission demo: Iris flies an X-pattern + circles + handover +
LOS test + autoland in Final_World.

Mission sequence (start at SW corner of the map):

  INIT          Wait for GPS fix + min satellite count.
  CLIMB         Take off + climb to cruise altitude (75 m default) at SW.
  X_LEG_1       Diagonal SW → NE (first leg of the X).
  X_LEG_2       Edge NE → SE (traverse to start the other diagonal).
  X_LEG_3       Diagonal SE → NW (second leg of the X — completes the X).
  TO_CENTER     Fly to the map's center waypoint.
  CIRCLE_1      First 50 m radius circle around center.
  CIRCLE_2      Second 50 m radius circle.
  HANDOVER      Mission stops sending MSP overrides for 10 s; pilot's RC
                (via UDP 9004 / radio_to_bf) flows through. BF arbitrates:
                MSP off ⇒ pilot has channels 1-4 too.
  LOS_TEST      Mission STILL doesn't send MSP, AND the operator should
                kill `radio_to_bf` if running. BF sees no RC updates →
                failsafe (GPS Rescue per defaults.txt) triggers after
                `failsafe_delay = 4 s`. We watch what happens for 7 s.
  LANDING_APPROACH  Mission resumes MSP overrides, flies to NE corner
                    (opposite of SW start) at cruise altitude.
  LANDING_DESCENT   Slow throttle reduction at NE corner until on the
                    ground (alt < 1 m), then disarm.
  DONE          Mission complete; mission script exits.

Architecture: this script is independent of the Pegasus orchestrator and
the racer_companion. It connects to BF SITL via TCP 5761 (MSP) using the
existing BetaflightAdapter. Pegasus must be running in T2 to provide
physics. radio_to_bf can run in T4 for the handover phase.

Usage (4-terminal):
  T1: docker compose -f sitl/docker-compose.yml up
  T2: python integrations/orchestrators/final_world_betaflight.py
  T4: python -m integrations.tools.radio_to_bf  (optional, for HANDOVER)
  T3: python -m integrations.tools.mission_demo

This is a DEMO script, not a production controller. It uses simple P
controllers on position + altitude derived from the racer_companion's
nav.py. Tuning is conservative: it'll fly slowly and stably, not race-quad
agility. Iris airframe is assumed (rotor_max_omega = 1023 rad/s).
"""
from __future__ import annotations

import argparse
import logging
import math
import sys
import time
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path

# Reuse companion's MSP wire + nav math (no companion code changes).
_COMPANION = Path(__file__).resolve().parents[2] / "companion"
sys.path.insert(0, str(_COMPANION))

from racer_companion import msp, nav  # noqa: E402
from racer_companion.fc.betaflight import BetaflightAdapter  # noqa: E402

log = logging.getLogger("mission_demo")


# ── Constants ──────────────────────────────────────────────────────────────

# WGS-84 mean per-degree distance, used to translate map-frame meters into
# lat/lon waypoints. Same convention as discover_bf_gps's recipe.
_M_PER_DEG_LAT = 110_540.0
_M_PER_DEG_LON_EQUATOR = 111_320.0

# RC channel ranges (PWM µs).
PWM_MIN = 1000
PWM_MID = 1500
PWM_MAX = 2000


# ── Phase enum ─────────────────────────────────────────────────────────────


class Phase(Enum):
    INIT = "INIT"
    CLIMB = "CLIMB"
    X_LEG_1 = "X_LEG_1"               # SW → NE
    X_LEG_2 = "X_LEG_2"               # NE → SE
    X_LEG_3 = "X_LEG_3"               # SE → NW
    TO_CENTER = "TO_CENTER"
    CIRCLE_1 = "CIRCLE_1"
    CIRCLE_2 = "CIRCLE_2"
    HANDOVER = "HANDOVER"
    LOS_TEST = "LOS_TEST"
    LANDING_APPROACH = "LANDING_APPROACH"  # to NE (opposite of start)
    LANDING_DESCENT = "LANDING_DESCENT"
    DONE = "DONE"


# ── Mission config ─────────────────────────────────────────────────────────


@dataclass
class MissionConfig:
    bf_host: str = "127.0.0.1"
    bf_port: int = 5761
    loop_hz: float = 50.0

    # Map geometry — half-side of the square map in meters.
    # The four corners are at (±half, ±half) from home in (north, east) m.
    # Tune to your actual map bounds.
    map_half_size_m: float = 80.0
    cruise_alt_m: float = 75.0
    circle_radius_m: float = 50.0
    circle_period_s: float = 30.0          # one full circle in 30 s
    arrival_radius_m: float = 5.0          # waypoint reached when within
    descent_arrival_alt_m: float = 1.0     # consider "landed" below this

    # Phase timings.
    handover_duration_s: float = 10.0
    los_duration_s: float = 7.0
    climb_timeout_s: float = 30.0
    leg_timeout_s: float = 60.0
    descent_timeout_s: float = 90.0    # hard exit if alt never reaches floor

    # Min sats required before starting the mission.
    min_satellites: int = 8
    # Min altitude before we accept that CLIMB has succeeded. Without this
    # gate, a stuck-on-the-ground drone with hover_throttle wrong would
    # silently advance through the X-pattern timing-out leg by leg, then
    # "complete" without ever lifting off.
    min_climb_alt_m: float = 5.0

    # Tuning — these are conservative; not race-quad agility.
    yaw_kp: float = 4.0
    yaw_max_us: float = 200.0
    pitch_kp_per_m: float = 2.0
    pitch_max_us: float = 200.0
    yaw_align_threshold_deg: float = 25.0

    # Iris hover throttle = 1300 µs to match `sitl/defaults.txt`'s
    # `gps_rescue_throttle_hover = 1300`. Setting this to 1500 like a
    # generic hover would make the climb-throttle saturate at the
    # 1700 hard cap, blowing past 75 m on the way up.
    throttle_hover_us: int = 1300
    throttle_kp_per_m: float = 6.0
    throttle_max_offset: int = 300


# ── Map waypoints ──────────────────────────────────────────────────────────


@dataclass
class Waypoint:
    """Lat/lon/alt waypoint."""
    lat_deg: float
    lon_deg: float
    alt_m: float
    label: str = ""


def offset_waypoint(
    home_lat: float, home_lon: float,
    north_m: float, east_m: float, alt_m: float,
    label: str = "",
) -> Waypoint:
    """Return a Waypoint at (north_m, east_m) offset from home."""
    cos_lat = max(0.05, math.cos(math.radians(home_lat)))
    lat = home_lat + (north_m / _M_PER_DEG_LAT)
    lon = home_lon + (east_m / (_M_PER_DEG_LON_EQUATOR * cos_lat))
    return Waypoint(lat_deg=lat, lon_deg=lon, alt_m=alt_m, label=label)


def build_corners(
    home_lat: float, home_lon: float,
    half: float, cruise_alt_m: float,
) -> dict[str, Waypoint]:
    """Compute SW/SE/NE/NW corner + CENTER waypoints from home."""
    return {
        "SW": offset_waypoint(home_lat, home_lon, -half, -half, cruise_alt_m, "SW"),
        "SE": offset_waypoint(home_lat, home_lon, -half, +half, cruise_alt_m, "SE"),
        "NE": offset_waypoint(home_lat, home_lon, +half, +half, cruise_alt_m, "NE"),
        "NW": offset_waypoint(home_lat, home_lon, +half, -half, cruise_alt_m, "NW"),
        "CENTER": offset_waypoint(home_lat, home_lon, 0.0, 0.0, cruise_alt_m, "CENTER"),
    }


# Mission waypoint sequence per phase.
PHASE_WAYPOINT_KEY = {
    Phase.X_LEG_1: "NE",
    Phase.X_LEG_2: "SE",
    Phase.X_LEG_3: "NW",
    Phase.TO_CENTER: "CENTER",
    Phase.LANDING_APPROACH: "NE",   # opposite of SW start
}


# ── Mission state machine ──────────────────────────────────────────────────


class Mission:
    """State machine + RC stick generator for the demo mission.

    Per-tick contract:
      - `update(snap, now)` advances the mission state machine using the
        BetaflightAdapter's TelemetrySnapshot.
      - `should_send_msp()` returns True when the mission is actively
        controlling — False during HANDOVER and LOS_TEST.
      - `compute_rc()` returns 8-channel RC values to send via
        MSP_SET_RAW_RC. Only valid when should_send_msp() is True.
    """

    def __init__(self, cfg: MissionConfig):
        self.cfg = cfg
        self.phase = Phase.INIT
        self.phase_started_at = 0.0
        self.home: Waypoint | None = None
        self.corners: dict[str, Waypoint] = {}
        self.circle_phase_offset_s = 0.0  # wraps every circle_period_s

    # ── Public interface ────────────────────────────────────────────────

    def update(self, snap, now: float) -> None:
        """Advance the state machine. snap is a TelemetrySnapshot."""
        if self.phase == Phase.INIT:
            self._update_init(snap, now)
        elif self.phase == Phase.CLIMB:
            self._update_climb(snap, now)
        elif self.phase in (Phase.X_LEG_1, Phase.X_LEG_2, Phase.X_LEG_3,
                            Phase.TO_CENTER, Phase.LANDING_APPROACH):
            self._update_waypoint_leg(snap, now)
        elif self.phase in (Phase.CIRCLE_1, Phase.CIRCLE_2):
            self._update_circle(snap, now)
        elif self.phase == Phase.HANDOVER:
            self._update_handover(snap, now)
        elif self.phase == Phase.LOS_TEST:
            self._update_los_test(snap, now)
        elif self.phase == Phase.LANDING_DESCENT:
            self._update_descent(snap, now)
        elif self.phase == Phase.DONE:
            pass

    def should_send_msp(self) -> bool:
        """During HANDOVER and LOS_TEST we deliberately stop sending MSP
        so the pilot (or BF's failsafe) takes over."""
        return self.phase not in (
            Phase.INIT, Phase.HANDOVER, Phase.LOS_TEST, Phase.DONE,
        )

    def compute_rc(self, snap, now: float) -> list[int]:
        """RC values for MSP_SET_RAW_RC. 8 channels; channels 5-8 stay at
        midpoint (the companion's mask=15 only covers channels 1-4 anyway)."""
        rc = [PWM_MID] * 8
        if not snap.gps or not snap.gps.fix:
            # No GPS — hover throttle, level sticks. Better than nothing.
            rc[msp.CH_THROTTLE] = self.cfg.throttle_hover_us
            return rc

        if self.phase == Phase.CLIMB:
            self._compute_climb_rc(rc, snap)
        elif self.phase in (Phase.X_LEG_1, Phase.X_LEG_2, Phase.X_LEG_3,
                            Phase.TO_CENTER, Phase.LANDING_APPROACH):
            self._compute_waypoint_rc(rc, snap, self._current_target())
        elif self.phase in (Phase.CIRCLE_1, Phase.CIRCLE_2):
            self._compute_waypoint_rc(rc, snap, self._circle_target(now))
        elif self.phase == Phase.LANDING_DESCENT:
            self._compute_descent_rc(rc, snap)
        return rc

    # ── Phase transitions ───────────────────────────────────────────────

    def _enter(self, phase: Phase, now: float) -> None:
        log.info("[mission] %s → %s", self.phase.value, phase.value)
        self.phase = phase
        self.phase_started_at = now

    def _phase_elapsed(self, now: float) -> float:
        return now - self.phase_started_at

    # ── Per-phase update logic ──────────────────────────────────────────

    def _update_init(self, snap, now: float) -> None:
        if snap.gps and snap.gps.fix and snap.gps.num_sat >= self.cfg.min_satellites:
            # Lock in home + corner waypoints from the first good GPS.
            self.home = Waypoint(
                lat_deg=snap.gps.lat_deg,
                lon_deg=snap.gps.lon_deg,
                alt_m=snap.altitude.alt_cm / 100.0 if snap.altitude else 0.0,
                label="HOME",
            )
            self.corners = build_corners(
                self.home.lat_deg, self.home.lon_deg,
                self.cfg.map_half_size_m, self.cfg.cruise_alt_m,
            )
            log.info("[mission] HOME locked: lat=%.7f lon=%.7f sats=%d",
                     self.home.lat_deg, self.home.lon_deg, snap.gps.num_sat)
            for k, w in self.corners.items():
                log.info("  %s: lat=%.7f lon=%.7f alt=%.1f", k,
                         w.lat_deg, w.lon_deg, w.alt_m)
            self._enter(Phase.CLIMB, now)

    def _update_climb(self, snap, now: float) -> None:
        if not snap.altitude:
            return
        alt_m = snap.altitude.alt_cm / 100.0
        if alt_m >= self.cfg.cruise_alt_m * 0.95:
            self._enter(Phase.X_LEG_1, now)
        elif self._phase_elapsed(now) > self.cfg.climb_timeout_s:
            # Refuse to advance if the drone never lifted off — would
            # silently complete the mission on the ground. Operator must
            # tune throttle_hover_us if this fires.
            if alt_m < self.cfg.min_climb_alt_m:
                log.error(
                    "[mission] CLIMB timeout AND altitude %.2fm < %.1fm — "
                    "the drone never lifted off. Likely throttle_hover_us "
                    "wrong. Aborting.", alt_m, self.cfg.min_climb_alt_m,
                )
                self._enter(Phase.DONE, now)
                return
            log.warning(
                "[mission] CLIMB timeout but airborne at %.1fm — proceeding",
                alt_m,
            )
            self._enter(Phase.X_LEG_1, now)

    def _update_waypoint_leg(self, snap, now: float) -> None:
        target = self._current_target()
        if target is None:
            return
        dist = self._distance_to(snap, target)
        if dist is None:
            return
        if dist < self.cfg.arrival_radius_m:
            log.info("[mission] arrived %s (dist=%.1fm)", target.label, dist)
            self._advance_after_waypoint(now)
        elif self._phase_elapsed(now) > self.cfg.leg_timeout_s:
            log.warning("[mission] %s timeout (dist=%.1fm) — advancing",
                        self.phase.value, dist)
            self._advance_after_waypoint(now)

    def _advance_after_waypoint(self, now: float) -> None:
        nxt = {
            Phase.X_LEG_1: Phase.X_LEG_2,
            Phase.X_LEG_2: Phase.X_LEG_3,
            Phase.X_LEG_3: Phase.TO_CENTER,
            Phase.TO_CENTER: Phase.CIRCLE_1,
            Phase.LANDING_APPROACH: Phase.LANDING_DESCENT,
        }[self.phase]
        # Reset circle phase offset on entering CIRCLE_1.
        if nxt == Phase.CIRCLE_1:
            self.circle_phase_offset_s = now
        self._enter(nxt, now)

    def _update_circle(self, snap, now: float) -> None:
        elapsed_in_phase = self._phase_elapsed(now)
        # One full lap per circle_period_s.
        if elapsed_in_phase >= self.cfg.circle_period_s:
            if self.phase == Phase.CIRCLE_1:
                self._enter(Phase.CIRCLE_2, now)
            else:
                self._enter(Phase.HANDOVER, now)

    def _update_handover(self, snap, now: float) -> None:
        if self._phase_elapsed(now) >= self.cfg.handover_duration_s:
            log.warning("[mission] simulating LOSS OF SIGNAL — kill radio_to_bf "
                        "now if it's running. BF should failsafe in ~4 s.")
            self._enter(Phase.LOS_TEST, now)

    def _update_los_test(self, snap, now: float) -> None:
        if self._phase_elapsed(now) >= self.cfg.los_duration_s:
            log.info("[mission] resuming control after LOS test")
            self._enter(Phase.LANDING_APPROACH, now)

    def _update_descent(self, snap, now: float) -> None:
        if snap.altitude:
            alt_m = snap.altitude.alt_cm / 100.0
            if alt_m <= self.cfg.descent_arrival_alt_m:
                log.info("[mission] LANDED at NE corner (alt=%.2fm)", alt_m)
                self._enter(Phase.DONE, now)
                return
        # Hard timeout — without it the drone could oscillate around the
        # arrival-alt threshold forever (noisy altitude estimate + idle
        # throttle).
        if self._phase_elapsed(now) > self.cfg.descent_timeout_s:
            alt_str = f"{snap.altitude.alt_cm / 100.0:.2f}" if snap.altitude else "?"
            log.warning(
                "[mission] DESCENT timeout after %.0fs (alt=%sm). "
                "Forcing DONE; check throttle tuning + position hold.",
                self.cfg.descent_timeout_s, alt_str,
            )
            self._enter(Phase.DONE, now)

    # ── Target computation ─────────────────────────────────────────────

    def _current_target(self) -> Waypoint | None:
        key = PHASE_WAYPOINT_KEY.get(self.phase)
        if key is None or not self.corners:
            return None
        return self.corners[key]

    def _circle_target(self, now: float) -> Waypoint | None:
        """Moving target on a horizontal circle around CENTER."""
        if not self.corners or self.home is None:
            return None
        center = self.corners["CENTER"]
        elapsed = now - self.circle_phase_offset_s
        theta = 2.0 * math.pi * elapsed / self.cfg.circle_period_s
        # offset in (north, east)
        north_off = self.cfg.circle_radius_m * math.cos(theta)
        east_off = self.cfg.circle_radius_m * math.sin(theta)
        return offset_waypoint(
            center.lat_deg, center.lon_deg,
            north_off, east_off, self.cfg.cruise_alt_m,
            label=f"CIRCLE@{math.degrees(theta) % 360:.0f}°",
        )

    def _distance_to(self, snap, target: Waypoint) -> float | None:
        if not snap.gps or not snap.gps.fix:
            return None
        return nav.haversine_m(
            snap.gps.lat_deg, snap.gps.lon_deg,
            target.lat_deg, target.lon_deg,
        )

    # ── RC computation ─────────────────────────────────────────────────

    def _compute_climb_rc(self, rc: list[int], snap) -> None:
        """Throttle up; sticks centered."""
        if not snap.altitude:
            rc[msp.CH_THROTTLE] = self.cfg.throttle_hover_us + 80
            return
        alt_m = snap.altitude.alt_cm / 100.0
        err_m = self.cfg.cruise_alt_m - alt_m
        # Above hover by enough to climb.
        offset = max(40, min(self.cfg.throttle_max_offset,
                             int(self.cfg.throttle_kp_per_m * err_m)))
        rc[msp.CH_THROTTLE] = self.cfg.throttle_hover_us + offset

    def _compute_waypoint_rc(self, rc: list[int], snap, target: Waypoint) -> None:
        """Bearing-track + altitude hold to the target."""
        if not snap.gps or not snap.gps.fix:
            rc[msp.CH_THROTTLE] = self.cfg.throttle_hover_us
            return
        distance = nav.haversine_m(
            snap.gps.lat_deg, snap.gps.lon_deg,
            target.lat_deg, target.lon_deg,
        )
        target_bearing = nav.bearing_deg(
            snap.gps.lat_deg, snap.gps.lon_deg,
            target.lat_deg, target.lon_deg,
        )
        current_heading = snap.attitude.yaw_deg if snap.attitude else 0.0
        h_err = nav.heading_error_deg(target_bearing, current_heading)
        # Yaw control.
        yaw_delta = max(-self.cfg.yaw_max_us,
                        min(self.cfg.yaw_max_us, self.cfg.yaw_kp * h_err))
        rc[msp.CH_YAW] = int(round(PWM_MID + yaw_delta))
        # Pitch only when aligned.
        if abs(h_err) < self.cfg.yaw_align_threshold_deg:
            forward = max(0.0, min(self.cfg.pitch_max_us,
                                   self.cfg.pitch_kp_per_m * distance))
            rc[msp.CH_PITCH] = int(round(PWM_MID + forward))
        else:
            rc[msp.CH_PITCH] = PWM_MID
        # Altitude hold via throttle.
        if snap.altitude:
            err_m = target.alt_m - (snap.altitude.alt_cm / 100.0)
            t_offset = max(-self.cfg.throttle_max_offset,
                           min(self.cfg.throttle_max_offset,
                               int(self.cfg.throttle_kp_per_m * err_m)))
            rc[msp.CH_THROTTLE] = self.cfg.throttle_hover_us + t_offset
        else:
            rc[msp.CH_THROTTLE] = self.cfg.throttle_hover_us

    def _compute_descent_rc(self, rc: list[int], snap) -> None:
        """Slow throttle reduction at NE corner — let gravity bring it down."""
        # Hold position over NE corner via the waypoint controller.
        if "NE" in self.corners:
            self._compute_waypoint_rc(rc, snap, self.corners["NE"])
        # Then bias throttle below hover so it sinks.
        rc[msp.CH_THROTTLE] = max(
            PWM_MIN, self.cfg.throttle_hover_us - 100,
        )


# ── Main loop ──────────────────────────────────────────────────────────────


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--bf-host", default="127.0.0.1")
    p.add_argument("--bf-port", type=int, default=5761)
    p.add_argument("--map-half-size", type=float, default=80.0,
                   help="Half-side of the square map in meters (default 80 = 160x160 m).")
    p.add_argument("--cruise-alt", type=float, default=75.0)
    p.add_argument("--circle-radius", type=float, default=50.0)
    p.add_argument("--rate-hz", type=float, default=50.0)
    p.add_argument("--log-level", default="INFO")
    args = p.parse_args()

    logging.basicConfig(
        level=args.log_level.upper(),
        format="%(asctime)s %(levelname)s %(name)s %(message)s",
    )

    cfg = MissionConfig(
        bf_host=args.bf_host,
        bf_port=args.bf_port,
        map_half_size_m=args.map_half_size,
        cruise_alt_m=args.cruise_alt,
        circle_radius_m=args.circle_radius,
        loop_hz=args.rate_hz,
    )
    mission = Mission(cfg)

    log.info("[mission_demo] connecting to BF SITL at tcp://%s:%d",
             cfg.bf_host, cfg.bf_port)
    fc = BetaflightAdapter.open(
        f"tcp://{cfg.bf_host}:{cfg.bf_port}", baud=115200,
        telemetry_period_s=1.0 / 10.0,
    )

    loop_dt = 1.0 / cfg.loop_hz
    try:
        while mission.phase != Phase.DONE:
            t0 = time.monotonic()
            snap = fc.tick(t0)
            mission.update(snap, t0)
            if mission.should_send_msp():
                rc = mission.compute_rc(snap, t0)
                # Clamp throttle hard to keep crashes survivable in case of
                # a controller blowup. Same hard cap as the racer_companion.
                rc[msp.CH_THROTTLE] = min(rc[msp.CH_THROTTLE], 1700)
                fc.send_overrides(rc)
            elapsed = time.monotonic() - t0
            if elapsed < loop_dt:
                time.sleep(loop_dt - elapsed)
    except KeyboardInterrupt:
        log.info("[mission_demo] interrupted")
    finally:
        fc.close()

    log.info("[mission_demo] done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
