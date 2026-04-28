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
import csv
import logging
import math
import os
import socket
import struct
import sys
import tempfile
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

# MSP_SET_RAW_RC wire-order slot indices for BF's default rcmap "AETR".
# After task #42 fixed the swap in companion/racer_companion/msp.py,
# these are identical to `msp.CH_*`. Kept as local names for clarity
# in this demo (it's nice to see "SLOT_THROTTLE" right next to the
# wire format).
SLOT_ROLL = 0
SLOT_PITCH = 1
SLOT_THROTTLE = 2
SLOT_YAW = 3
SLOT_AUX1 = 4
SLOT_AUX2 = 5
SLOT_AUX3 = 6
SLOT_AUX4 = 7

# BF SITL UDP RC port wire format. We send the SAME 8 channels we send
# via MSP_SET_RAW_RC, padded to 16 channels with PWM_MID, packed as 1
# host-native double timestamp + 16 LE uint16 PWM µs values = 40 bytes.
# BF strict-checks `n == sizeof(rc_packet)`; wrong size = silent drop.
# Format pinned in `test_bf_rc_keepalive.py` and matches `radio_to_bf.py`
# (the same struct must agree with both or BF drops one of them).
# Sending here in addition to MSP gives BF's RX state machine a "real
# RX" stream — eliminates the chronic RX_FAILSAFE bit-2 set that drove
# the original 25-30 s ARM_SWITCH-latch cycle (mission6 = 42 bit-2
# hits, mission7 with this dual-write = 0). Same numbers in both
# paths means no precedence fight. A separate ~10 s bit-1 FAILSAFE
# pulse path remains; see TODO.md item 2. UDP send is gated by the
# same `should_send_msp()` check as MSP; HANDOVER/LOS_TEST stop both
# so radio_to_bf (HANDOVER) or silence (LOS_TEST) owns UDP 9004.
_UDP_RC_PACKET = struct.Struct("<d16H")
assert _UDP_RC_PACKET.size == 40, f"rc_packet must be 40B, got {_UDP_RC_PACKET.size}"
_NUM_UDP_RC_CHANNELS = 16

# Respawn signal file. mission_demo writes it (touch) when it aborts
# due to flip detection; the Pegasus orchestrator polls for it and
# calls world.reset() when present so the drone respawns at its
# original spawn point. Same tempdir convention as `bf_sim_state.txt`
# so both processes agree without CLI plumbing.
DEFAULT_RESPAWN_SIGNAL_FILE = os.path.join(
    tempfile.gettempdir(), "bf_respawn.signal",
)

# Target state file. mission_demo writes its current waypoint here
# every tick (n_m, e_m, alt_m, label) — relative to the home point
# captured during INIT. The orchestrator reads this and renders a
# colored sphere at the matching spawn-relative position so the
# operator can SEE whether the drone is following the controller's
# intent. Same tempdir convention as the other state files.
DEFAULT_TARGET_STATE_FILE = os.path.join(
    tempfile.gettempdir(), "bf_target_state.txt",
)


# ── Phase enum ─────────────────────────────────────────────────────────────


class Phase(Enum):
    INIT = "INIT"
    ARM = "ARM"
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
    map_half_size_m: float = 30.0           # tightened from 80 for sim_loop
    # mission17 (cruise_alt=25) made things worse — drone hit
    # obstacles in a different part of the chemical plant scene and
    # X_LEG_3 ended 73 m short. Reverted to 15. The right fix for
    # the obstacle problem is shrinking the map via `--map-half-size`
    # (smaller X-pattern = stays in open ground), not raising alt.
    cruise_alt_m: float = 15.0
    circle_radius_m: float = 15.0
    circle_period_s: float = 30.0          # one full circle in 30 s
    arrival_radius_m: float = 5.0          # waypoint reached when within
    descent_arrival_alt_m: float = 1.0     # consider "landed" below this

    # Phase timings.
    init_settle_s: float = 5.0              # AUX1=LOW dwell before ARM phase;
                                             # BF needs to observe arm-box OFF
                                             # for several cycles before an
                                             # OFF→ON transition will arm.
                                             # Empirically <1s = BF won't arm.
    arm_duration_s: float = 2.0             # idle throttle + AUX1 high to arm
    handover_duration_s: float = 10.0
    los_duration_s: float = 7.0
    climb_timeout_s: float = 60.0           # was 30; Iris sim climbs ~0.5 m/s
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
    # Apr 28 night: pitch_kp_per_m 2.0 → 4.0 per mission14 trace.
    # 2.0 produced only +74 µs forward stick at distance=37 m; drone
    # stalled 31-34 m from corner because forward thrust was too
    # weak. 4.0 doubles that (pitch_us = 1500 + min(pitch_max,
    # kp*distance), saturates at distance ≥ pitch_max/kp).
    # Matches `racer_companion/nav.py::NavTuning` default — the
    # 2.0 here was a mission_demo-specific outlier.
    pitch_kp_per_m: float = 4.0
    # mission16 tested pitch_max=300 to extend pitch-saturation
    # range from 50 m to 75 m on the X_LEG_3 long diagonal. Helped
    # X_LEG_1/3 modestly (e.g., X_LEG_3 41.8→33.7 m short) but the
    # trace decile data revealed the actual bottleneck: drone tilts
    # forward → cos(tilt) lift loss → altitude drops to 2-3 m
    # during pitched flight → drone hits chemical-plant obstacles
    # at low alt and stalls (X_LEG_3 distance frozen at 33.7 m for
    # ~30 s). Reverted to 200 because raising it doesn't address
    # the real coupling problem (TODO #11). Real fix needs altitude/
    # pitch coupling — ideas in TODO #11.
    pitch_max_us: float = 200.0
    yaw_align_threshold_deg: float = 25.0

    # Iris hover throttle = 1641 PWM, confirmed by
    # `iris_hover_calibrate` (motor_norm 0.6411, vz < 1 mm/s after 4 s
    # at this value). Was 1640 (1 µs below true hover) and earlier 1300
    # (chasing a phantom `gps_rescue_throttle_hover` BF setting that
    # 4.5.1 renamed/removed — Iris sank silently during CLIMB).
    throttle_hover_us: int = 1641
    # P-only altitude controller — no Kd. Apr 28 evening: kp=6 gave
    # ±25 m oscillation around 15 m target in mission12 (peaks 36-40,
    # dips -16). Lowered to 3 with max_offset=200 to reduce overshoot.
    # The historical kp=3/max=100 was too gentle for Iris ground
    # physics, but the current max_offset=200 doubles the saturation
    # range so kp=3 still produces +90 µs throttle at err=30 m (full
    # climb authority). If CLIMB phase fails to lift off, try
    # decoupling CLIMB kp (aggressive) from X_LEG kp (gentle) per
    # TODO.md item 9. Real fix is still a Kd term keyed to vario.
    throttle_kp_per_m: float = 3.0
    throttle_max_offset: int = 200
    # Idle throttle during ARM phase. Must be < BF's min_check (1050)
    # so the THROTTLE arming-disable flag clears before AUX1 high
    # triggers the ARM box.
    throttle_idle_us: int = 950


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
        # Mid-flight latch recovery (BF SITL artifact). Set by the main
        # loop from fc._last_arming_flags; -1 means we haven't received
        # any STATUS_EX yet.
        self.last_arming_flags = -1
        # Number of consecutive recovery ticks performed in this episode
        # — diagnostic only; logged so we can spot frequent recoveries.
        self._recovery_ticks = 0
        self._recovery_logged_at = 0.0
        # Post-recovery idle-throttle hold. After we drop AUX1 LOW to
        # clear the ARM_SWITCH latch, BF re-evaluates arming the moment
        # AUX1 returns HIGH. If throttle is above min_check (1050) at
        # that transition, the THROTTLE bit re-latches ARM_SWITCH and
        # we loop forever. Hold idle throttle (with AUX1 HIGH) for this
        # long after exiting the LOW stage so BF sees a clean ARMABLE
        # at the LOW→HIGH edge.
        # Apr 28 evening: bumped from 0.6 to 1.5 s. mission10 showed the
        # relatch fires 1.5-3 s after a clean HOLD ends, suggesting BF
        # needs longer to fully settle internal failsafe state.
        # mission11: confirmed 1.5 s drops recovery cadence from 53 →
        # 5 over 240 s. But idle throttle for 1.5 s = drone free-falls
        # ~13 m per recovery, so legs still don't reach corners.
        self._recovery_hold_until = 0.0
        self.RECOVERY_HOLD_S = 1.5
        # Within the HOLD stage, the FIRST `RECOVERY_HOLD_IDLE_S` lets
        # the AUX1 LOW→HIGH transition (at LOW→HOLD boundary) land with
        # throttle idle so the THROTTLE bit (set when throttle >
        # min_check) doesn't immediately re-latch ARM_SWITCH. After
        # that brief idle window, raise throttle to hover so the drone
        # doesn't free-fall through the rest of HOLD. AUX1 stays HIGH
        # the whole time — no new transition, no new latch
        # opportunity.
        self.RECOVERY_HOLD_IDLE_S = 0.3
        # Per-tick waypoint diagnostic CSV (opt-in via --waypoint-
        # trace-csv). Used Apr 28 night to chase TODO #10 — why legs
        # time out 28-49 m short of corners. Writer + file handle are
        # set by main() if the CLI flag is provided; None otherwise.
        self._waypoint_csv = None
        self._waypoint_csv_fh = None
        # Flip detection (Apr 29). Iris in Pegasus can flip on
        # hard-pitched commands when altitude drops below the tilt-
        # induced lift loss threshold. With runaway_takeoff_
        # prevention=OFF, BF doesn't auto-detect; mission_demo keeps
        # sending throttle commands obliviously for the rest of the
        # CLIMB timeout. Detect via |roll| > 90° or |pitch| > 90°
        # sustained for `_flip_required_ticks` consecutive ticks
        # (avoids false positives from transient gimbal-lock readings
        # at boot or recovery transitions). On detection, log and
        # force DONE.
        self._flip_tick_count = 0
        self.FLIP_ANGLE_DEG = 90.0
        self.FLIP_REQUIRED_TICKS = 25  # 0.5s at 50Hz
        # When a flip-induced DONE fires, write a signal file the
        # orchestrator polls for. Set by main() from the CLI flag;
        # default is `bf_respawn.signal` in tempdir. None disables
        # the signal write (the orchestrator simply won't see it).
        self._respawn_signal_path: str | None = None
        # Target state file for the orchestrator's in-sim "target
        # bubble" marker (Apr 29 UX work). Set by main() from CLI
        # flag; default DEFAULT_TARGET_STATE_FILE. None disables.
        self._target_state_path: str | None = None

    # ── Public interface ────────────────────────────────────────────────

    def _active_target(self, now: float) -> Waypoint | None:
        """Return the Waypoint the controller is currently chasing, or
        None if the phase has no target. Used both by the per-phase RC
        compute and by the target-state-file publisher (Apr 29 in-sim
        UX work)."""
        if self.phase in (Phase.X_LEG_1, Phase.X_LEG_2, Phase.X_LEG_3,
                          Phase.TO_CENTER, Phase.LANDING_APPROACH,
                          Phase.LANDING_DESCENT):
            return self._current_target()
        if self.phase in (Phase.CIRCLE_1, Phase.CIRCLE_2):
            return self._circle_target(now)
        return None

    def _publish_target_state(self, now: float) -> None:
        """Write the current target to the state file the orchestrator
        polls for the in-sim 'target bubble' marker. Format mirrors
        bf_sim_state.txt (n e alt yaw label) but yaw is unused so we
        write 0. Coordinates are home-relative meters in N/E/up.
        Best-effort — file errors are swallowed."""
        if not self._target_state_path or self.home is None:
            return
        target = self._active_target(now)
        if target is None:
            return
        cos_lat = max(0.05, math.cos(math.radians(self.home.lat_deg)))
        n_m = (target.lat_deg - self.home.lat_deg) * _M_PER_DEG_LAT
        e_m = (target.lon_deg - self.home.lon_deg) * _M_PER_DEG_LON_EQUATOR * cos_lat
        # Target alt is relative-to-home (cruise_alt_m) — same convention
        # as the mission state machine's _rel_alt() compares against.
        try:
            with open(self._target_state_path, "w") as f:
                f.write(f"{n_m:.3f} {e_m:.3f} {target.alt_m:.3f} {target.label}\n")
        except OSError:
            pass

    def update(self, snap, now: float) -> None:
        """Advance the state machine. snap is a TelemetrySnapshot."""
        # Flip detection — only after we've left INIT (during INIT we
        # might see weird attitude values before BF's first MSP
        # ATTITUDE response, and we don't want to abort the mission
        # before it's even started).
        if self.phase not in (Phase.INIT, Phase.ARM, Phase.DONE) and snap.attitude:
            roll = abs(snap.attitude.roll_deg)
            pitch = abs(snap.attitude.pitch_deg)
            if roll > self.FLIP_ANGLE_DEG or pitch > self.FLIP_ANGLE_DEG:
                self._flip_tick_count += 1
                if self._flip_tick_count >= self.FLIP_REQUIRED_TICKS:
                    log.error(
                        "[mission] FLIP detected: roll=%.1f° pitch=%.1f° "
                        "for %d ticks — forcing DONE so the orchestrator "
                        "can respawn (Pegasus + Iris cannot self-right)",
                        snap.attitude.roll_deg, snap.attitude.pitch_deg,
                        self._flip_tick_count,
                    )
                    if self._respawn_signal_path:
                        try:
                            with open(self._respawn_signal_path, "w") as _f:
                                _f.write(f"flip {now:.3f}\n")
                            log.info("[mission] wrote respawn signal: %s",
                                     self._respawn_signal_path)
                        except OSError as e:
                            log.warning("[mission] respawn signal "
                                        "write failed: %s", e)
                    self._enter(Phase.DONE, now)
                    return
            else:
                # Reset counter on any tick where attitude is OK so
                # transient readings don't accumulate.
                self._flip_tick_count = 0

        # Publish current target for the orchestrator's in-sim
        # marker. Cheap (one file write per tick at 50 Hz, file size
        # ~50 B). Pre-INIT phases skip via _active_target returning
        # None when home isn't locked yet.
        self._publish_target_state(now)

        if self.phase == Phase.INIT:
            self._update_init(snap, now)
        elif self.phase == Phase.ARM:
            self._update_arm(snap, now)
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
        so the pilot (or BF's failsafe) takes over.
        We DO send during INIT — BF's RX_FAILSAFE bit latches if it ever
        sees no RC for ~1 s, and INIT can take that long while waiting
        for the first GPS frame. compute_rc() returns AUX1 low + idle
        throttle in INIT, so the arm box won't engage; we're just
        keeping the RC stream alive."""
        return self.phase not in (
            Phase.HANDOVER, Phase.LOS_TEST, Phase.DONE,
        )

    def compute_rc(self, snap, now: float) -> list[int]:
        """RC values for MSP_SET_RAW_RC. We set AUX1 high to arm (no
        pilot radio in SITL). Channels 6-8 (AUX2-4) at PWM_MIN to match
        flight_profile.py's known-good pattern — having them at PWM_MID
        was empirically blocking BF from arming through the shim, even
        though no `aux` bindings exist for AUX2-4 in defaults.txt."""
        rc = [PWM_MID] * 8
        rc[SLOT_AUX2] = PWM_MIN
        rc[SLOT_AUX3] = PWM_MIN
        rc[SLOT_AUX4] = PWM_MIN
        # Default: idle throttle. Only flying phases push it up. Pre-flight
        # ticks (no GPS yet, ARM phase) MUST send idle so BF's THROTTLE
        # arming-disable bit clears — otherwise high throttle + AUX1 high
        # together latch ARM_SWITCH and BF refuses to arm.
        rc[SLOT_THROTTLE] = self.cfg.throttle_idle_us
        # AUX1: low during INIT (don't engage ARM box yet), high once we
        # transition to ARM and onward. Without this gate, BF sees AUX1
        # high BEFORE we've started the arm sequence and may latch
        # ARM_SWITCH on the inevitable first-tick high-throttle blip.
        rc[SLOT_AUX1] = PWM_MIN if self.phase == Phase.INIT else PWM_MAX

        # ── ARM_SWITCH-latch recovery (BF SITL artifact) ─────────────
        # Past INIT/ARM the only way ARM_SWITCH (bit 25) sets is the
        # BF SITL latch path: any disable bit appeared while AUX1 was
        # HIGH. The classical trigger was BAD_RX_RECOVERY (bit 3) from
        # MSP frame corruption — closed off by the shim send-lock. The
        # remaining trigger is the THROTTLE bit (bit 7) re-asserting at
        # the moment AUX1 transitions LOW→HIGH out of recovery: when
        # AUX1 just rose, BF checks all disable bits, throttle is still
        # above min_check (1050) for any flying phase, so ARM_SWITCH
        # re-latches.
        #
        # Two-stage recovery solves it:
        #  1. LOW stage: AUX1=LOW + idle throttle while ARM_SWITCH or
        #     BAD_RX_RECOVERY is set. Clears the latch.
        #  2. HOLD stage: AUX1=HIGH + idle throttle for RECOVERY_HOLD_S
        #     so BF sees the LOW→HIGH transition with throttle below
        #     min_check, evaluates ARMABLE cleanly, and doesn't re-latch.
        # After both stages, normal phase logic resumes.
        # Real BF on STM32 with a real RX doesn't trip this; the latch
        # is specific to the shim/Docker MSP path on Windows.
        ARM_SWITCH_BIT = 1 << 25
        BAD_RX_RECOVERY_BIT = 1 << 3
        LATCH_MASK = ARM_SWITCH_BIT | BAD_RX_RECOVERY_BIT
        flags = self.last_arming_flags
        in_flying_phase = self.phase not in (Phase.INIT, Phase.ARM, Phase.DONE)
        if flags > 0 and (flags & LATCH_MASK) and in_flying_phase:
            # LOW stage. Drive AUX1 LOW + idle throttle. Schedule the
            # subsequent HOLD stage so we don't immediately bounce back
            # to high-throttle phase logic on the next tick.
            rc[SLOT_AUX1] = PWM_MIN
            rc[SLOT_THROTTLE] = self.cfg.throttle_idle_us
            self._recovery_hold_until = now + self.RECOVERY_HOLD_S
            self._recovery_ticks += 1
            if now - self._recovery_logged_at >= 0.5:
                self._recovery_logged_at = now
                log.warning(
                    "[recover] LOW stage — flags=0x%x phase=%s "
                    "(tick %d)",
                    flags, self.phase.value, self._recovery_ticks,
                )
            return rc
        if in_flying_phase and now < self._recovery_hold_until:
            # HOLD stage. AUX1 is already HIGH (set above). Two
            # sub-phases:
            #   1. IDLE (first RECOVERY_HOLD_IDLE_S): keep throttle
            #      idle so the LOW→HIGH AUX1 transition that just
            #      happened (at LOW→HOLD boundary) lands with all
            #      disable bits clear. THROTTLE bit was clear in LOW
            #      stage; staying idle preserves that.
            #   2. HOVER (remaining time): raise throttle to hover so
            #      the drone doesn't free-fall through the rest of
            #      HOLD. AUX1 stays HIGH continuously, so no new
            #      transition; THROTTLE bit may set as throttle goes
            #      high but no LOW→HIGH AUX1 edge means no fresh
            #      ARM_SWITCH latching opportunity.
            hold_remaining = self._recovery_hold_until - now
            in_idle_subphase = (
                hold_remaining > self.RECOVERY_HOLD_S - self.RECOVERY_HOLD_IDLE_S
            )
            if in_idle_subphase:
                rc[SLOT_THROTTLE] = self.cfg.throttle_idle_us
                subphase = "idle"
            else:
                rc[SLOT_THROTTLE] = self.cfg.throttle_hover_us
                subphase = "hover"
            self._recovery_ticks += 1
            if now - self._recovery_logged_at >= 0.5:
                self._recovery_logged_at = now
                log.warning(
                    "[recover] HOLD stage (%s) — flags=0x%x phase=%s "
                    "(%.2fs remaining)",
                    subphase, flags, self.phase.value, hold_remaining,
                )
            return rc
        if not snap.gps or not snap.gps.fix:
            return rc

        if self.phase == Phase.ARM:
            # Idle throttle + AUX1 already high (set above). Hold for
            # arm_duration_s — BF clears THROTTLE arming-disable bit
            # (throttle below min_check) and ARM box engages.
            rc[SLOT_THROTTLE] = self.cfg.throttle_idle_us
        elif self.phase == Phase.CLIMB:
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
        # Wait for GPS fix AND altitude AND ARMABLE flags. The
        # arming-flags check is the real-flight best-practice "wait for
        # OSD to say ARMABLE before flipping the switch" — applies
        # equally to sim. Without it we'd see ARM_SWITCH (bit 25)
        # latch when AUX1 transitions LOW→HIGH while ANY other
        # disable bit is still set (BOOT_GRACE_TIME, CALIBRATING,
        # NO_ACC_CAL, etc.). Once latched, AUX1 has to drop below
        # 1700 to clear it — which mission_demo never does.
        ready = (snap.gps and snap.gps.fix
                 and snap.gps.num_sat >= self.cfg.min_satellites
                 and snap.altitude is not None)
        if not ready:
            return
        if self.home is None:
            self.home = Waypoint(
                lat_deg=snap.gps.lat_deg,
                lon_deg=snap.gps.lon_deg,
                alt_m=snap.altitude.alt_cm / 100.0,
                label="HOME",
            )
            self.corners = build_corners(
                self.home.lat_deg, self.home.lon_deg,
                self.cfg.map_half_size_m, self.cfg.cruise_alt_m,
            )
            log.info("[mission] HOME locked: lat=%.7f lon=%.7f alt=%.2fm sats=%d",
                     self.home.lat_deg, self.home.lon_deg, self.home.alt_m,
                     snap.gps.num_sat)
            for k, w in self.corners.items():
                log.info("  %s: lat=%.7f lon=%.7f alt=%.1f (rel)", k,
                         w.lat_deg, w.lon_deg, w.alt_m)
        # Wait for ARMABLE (armingDisableFlags == 0). The main loop
        # populates `self.last_arming_flags` from MSP_STATUS_EX every
        # tick. -1 means we haven't received a STATUS_EX yet.
        flags = getattr(self, "last_arming_flags", -1)
        if flags != 0:
            # Periodic log so the user can see what's blocking arming.
            elapsed = self._phase_elapsed(now)
            if int(elapsed) % 5 == 0 and elapsed - getattr(
                self, "_last_armable_log", -10.0,
            ) > 4.0:
                self._last_armable_log = elapsed
                log.info("[mission] waiting for ARMABLE (arm_flags=0x%x, "
                         "elapsed=%.1fs)", max(0, flags), elapsed)
            return
        if self._phase_elapsed(now) >= self.cfg.init_settle_s:
            self._enter(Phase.ARM, now)

    def _update_arm(self, snap, now: float) -> None:
        if self._phase_elapsed(now) >= self.cfg.arm_duration_s:
            self._enter(Phase.CLIMB, now)

    def _rel_alt(self, snap) -> float | None:
        """Altitude above HOME (where INIT captured baseline). BF's MSP_ALTITUDE
        is offset from BF's barometric baseline at boot, which doesn't match
        our absolute home, so all alt comparisons must be relative."""
        if not snap.altitude or self.home is None:
            return None
        return (snap.altitude.alt_cm / 100.0) - self.home.alt_m

    def _update_climb(self, snap, now: float) -> None:
        rel = self._rel_alt(snap)
        if rel is None:
            return
        if rel >= self.cfg.cruise_alt_m * 0.95:
            self._enter(Phase.X_LEG_1, now)
        elif self._phase_elapsed(now) > self.cfg.climb_timeout_s:
            # Refuse to advance if the drone never lifted off — would
            # silently complete the mission on the ground. Operator must
            # tune throttle_hover_us if this fires.
            if rel < self.cfg.min_climb_alt_m:
                log.error(
                    "[mission] CLIMB timeout AND rel_alt %.2fm < %.1fm — "
                    "the drone never lifted off. Likely throttle_hover_us "
                    "wrong. Aborting.", rel, self.cfg.min_climb_alt_m,
                )
                self._enter(Phase.DONE, now)
                return
            log.warning(
                "[mission] CLIMB timeout but airborne at rel_alt=%.1fm — proceeding",
                rel,
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
        rel = self._rel_alt(snap)
        if rel is not None:
            if rel <= self.cfg.descent_arrival_alt_m:
                log.info("[mission] LANDED at NE corner (rel_alt=%.2fm)", rel)
                self._enter(Phase.DONE, now)
                return
        # Hard timeout — without it the drone could oscillate around the
        # arrival-alt threshold forever (noisy altitude estimate + idle
        # throttle).
        if self._phase_elapsed(now) > self.cfg.descent_timeout_s:
            rel_str = f"{rel:.2f}" if rel is not None else "?"
            log.warning(
                "[mission] DESCENT timeout after %.0fs (rel_alt=%sm). "
                "Forcing DONE; check throttle tuning + position hold.",
                self.cfg.descent_timeout_s, rel_str,
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
        """Throttle up; sticks centered. Altitude judged relative to HOME."""
        rel = self._rel_alt(snap)
        if rel is None:
            rc[SLOT_THROTTLE] = self.cfg.throttle_hover_us + 80
            return
        err_m = self.cfg.cruise_alt_m - rel
        # Above hover by enough to climb.
        offset = max(40, min(self.cfg.throttle_max_offset,
                             int(self.cfg.throttle_kp_per_m * err_m)))
        rc[SLOT_THROTTLE] = self.cfg.throttle_hover_us + offset

    def _compute_waypoint_rc(self, rc: list[int], snap, target: Waypoint) -> None:
        """Bearing-track + altitude hold to the target."""
        if not snap.gps or not snap.gps.fix:
            rc[SLOT_THROTTLE] = self.cfg.throttle_hover_us
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
        rc[SLOT_YAW] = int(round(PWM_MID + yaw_delta))
        # Pitch only when aligned.
        if abs(h_err) < self.cfg.yaw_align_threshold_deg:
            forward = max(0.0, min(self.cfg.pitch_max_us,
                                   self.cfg.pitch_kp_per_m * distance))
            rc[SLOT_PITCH] = int(round(PWM_MID + forward))
        else:
            rc[SLOT_PITCH] = PWM_MID
        # Altitude hold via throttle. target.alt_m is the cruise altitude
        # (a relative value). Compare against rel_alt, not absolute alt.
        rel = self._rel_alt(snap)
        if rel is not None:
            err_m = target.alt_m - rel
            t_offset = max(-self.cfg.throttle_max_offset,
                           min(self.cfg.throttle_max_offset,
                               int(self.cfg.throttle_kp_per_m * err_m)))
            rc[SLOT_THROTTLE] = self.cfg.throttle_hover_us + t_offset
        else:
            rc[SLOT_THROTTLE] = self.cfg.throttle_hover_us
        # Per-tick diagnostic trace (TODO #10). Captures the
        # information needed to decide whether the leg-timeout
        # shortfall is a yaw-gate problem (yaw_aligned=False most
        # ticks → drop yaw_align_threshold_deg) or a pitch-saturation
        # problem (pitch_us pinned at PWM_MID + pitch_max_us → bump
        # pitch_kp_per_m / pitch_max_us). One writer per Mission;
        # None in production (no CLI flag) → zero overhead.
        if self._waypoint_csv is not None:
            yaw_aligned = abs(h_err) < self.cfg.yaw_align_threshold_deg
            self._waypoint_csv.writerow([
                f"{time.monotonic():.3f}",
                self.phase.value,
                f"{rel:.2f}" if rel is not None else "",
                f"{distance:.2f}",
                f"{h_err:.2f}",
                int(yaw_aligned),
                rc[SLOT_PITCH],
                rc[SLOT_THROTTLE],
                target.label,
            ])

    def _compute_descent_rc(self, rc: list[int], snap) -> None:
        """Slow throttle reduction at NE corner — let gravity bring it down."""
        # Hold position over NE corner via the waypoint controller.
        if "NE" in self.corners:
            self._compute_waypoint_rc(rc, snap, self.corners["NE"])
        # Then bias throttle below hover so it sinks.
        rc[SLOT_THROTTLE] = max(
            PWM_MIN, self.cfg.throttle_hover_us - 100,
        )


# ── Main loop ──────────────────────────────────────────────────────────────


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--bf-host", default="127.0.0.1")
    p.add_argument("--bf-port", type=int, default=5761)
    p.add_argument("--map-half-size", type=float, default=30.0,
                   help="Half-side of the square map in meters (default 30 = 60x60 m).")
    p.add_argument("--cruise-alt", type=float, default=15.0)
    p.add_argument("--circle-radius", type=float, default=15.0)
    p.add_argument("--rate-hz", type=float, default=50.0)
    p.add_argument("--log-level", default="INFO")
    # UDP 9004 RC dual-write — companion to MSP_SET_RAW_RC. ON by
    # default because it kills the chronic RX_FAILSAFE / ARM_SWITCH
    # latch that yesterday's mission6 still suffered from. Same RC
    # values go to MSP TCP and UDP, no precedence fight. See
    # TODO.md item 1(a) and MEMORY.md v0.7.1.
    p.add_argument("--udp-rc-host", default="127.0.0.1",
                   help="BF SITL UDP RC host (default: 127.0.0.1)")
    p.add_argument("--udp-rc-port", type=int, default=9004,
                   help="BF SITL UDP RC port (default: 9004 per "
                        "betaflight/4.5.1 src/main/target/SITL/sitl.c).")
    p.add_argument("--no-udp-rc", action="store_true",
                   help="Disable UDP 9004 RC dual-write. Default ON. "
                        "Use only when running radio_to_bf as the UDP "
                        "RC source (HANDOVER tests, real pilot input).")
    p.add_argument("--waypoint-trace-csv", default=None,
                   help="Path to a CSV file. When set, mission_demo "
                        "writes one row per tick from _compute_"
                        "waypoint_rc with (mono_t, phase, rel_alt, "
                        "distance_m, h_err_deg, yaw_aligned, "
                        "pitch_us, throttle_us, target). Used to "
                        "diagnose why X_LEG legs time out short of "
                        "corners (TODO #10). Off by default.")
    p.add_argument("--respawn-signal-file", default=DEFAULT_RESPAWN_SIGNAL_FILE,
                   help=f"Path to the respawn signal file mission_"
                        f"demo touches when it aborts due to flip "
                        f"detection. The Pegasus orchestrator polls "
                        f"for this file and calls `world.reset()` to "
                        f"respawn the drone, so the operator can run "
                        f"mission_demo again without restarting the "
                        f"sim. Default: {DEFAULT_RESPAWN_SIGNAL_FILE}. "
                        f"Empty string disables the signal write.")
    p.add_argument("--target-state-file", default=DEFAULT_TARGET_STATE_FILE,
                   help=f"Path mission_demo writes its current target "
                        f"to every tick (n e alt label, home-relative "
                        f"meters). The Pegasus orchestrator reads this "
                        f"to position a 'target bubble' sphere in the "
                        f"sim so the operator can see commanded vs "
                        f"actual position. Default: {DEFAULT_TARGET_STATE_FILE}. "
                        f"Empty string disables.")
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
    mission._respawn_signal_path = args.respawn_signal_file or None
    mission._target_state_path = args.target_state_file or None
    # Pre-clear any stale signal from a prior run. Without this,
    # if a previous mission_demo crashed mid-write or if the
    # orchestrator missed a poll cycle, the next mission_demo's
    # first flip-check could trigger an unintended respawn from
    # a ghost signal file.
    if mission._respawn_signal_path:
        try:
            os.remove(mission._respawn_signal_path)
        except OSError:
            pass

    log.info("[mission_demo] connecting to BF SITL at tcp://%s:%d",
             cfg.bf_host, cfg.bf_port)
    fc = BetaflightAdapter.open(
        f"tcp://{cfg.bf_host}:{cfg.bf_port}", baud=115200,
        telemetry_period_s=1.0 / 10.0,
    )

    # Wrap MspClient.poll to also extract MSP_STATUS_EX armingDisableFlags
    # so we can see in real-time why BF is/isn't arming.
    import struct as _struct
    _orig_handle_frame = fc._handle_frame
    fc._last_arming_flags = -1
    fc._last_flight_modes = -1
    fc._last_arming_log = 0.0
    def _handle_with_arming(cmd, payload, frame_now):
        _orig_handle_frame(cmd, payload, frame_now)
        if cmd == msp.MSP_STATUS_EX and len(payload) >= 21:
            # BF 4.5.1 MSP_STATUS_EX layout (verified empirically by
            # walking byte-by-byte in tests/probe_status.py):
            #   [0-1] cycleTime u16
            #   [2-3] i2cErrCount u16
            #   [4-5] sensors u16  (ACC|BARO|MAG|GPS|RNG|GYRO bitmap)
            #   [6-9] flightModeFlags u32
            #   [10] pidProfileIndex u8
            #   [11-12] avgSystemLoadPct u16
            #   [13] rateProfileIndex u8
            #   [14-15] (some short u16 — possibly task/MSP version count)
            #   [16] armingDisableFlagsCount u8 (= 26)
            #   [17-20] armingDisableFlags u32  ← THE bits we care about
            try:
                fm = _struct.unpack_from("<I", payload, 6)[0]
                arm_flags = _struct.unpack_from("<I", payload, 17)[0]
                fc._last_arming_flags = arm_flags
                fc._last_flight_modes = fm
                fc._last_status_payload = payload.hex()
            except _struct.error:
                pass
    fc._handle_frame = _handle_with_arming

    def _decode_arming(flags: int) -> str:
        names = [
            "NO_GYRO", "FAILSAFE", "RX_FAILSAFE", "BAD_RX_RECOVERY",
            "BOXFAILSAFE", "RUNAWAY_TAKEOFF", "CRASH_DETECTED",
            "THROTTLE", "ANGLE", "BOOT_GRACE_TIME", "NOPREARM",
            "LOAD", "CALIBRATING", "CLI", "CMS_MENU", "BST",
            "MSP", "PARALYZE", "GPS", "RESC", "RPMFILTER",
            "REBOOT_REQUIRED", "DSHOT_BITBANG", "ACC_CALIBRATION",
            "MOTOR_PROTOCOL", "ARM_SWITCH",
        ]
        if flags == 0:
            return "ARMABLE"
        return ",".join(n for i, n in enumerate(names) if flags & (1 << i))

    # UDP RC dual-write socket. We send the SAME 8 channels via MSP
    # AND UDP 9004 every tick; BF's RX state machine treats the UDP
    # path as "real RX" and stops chronically tripping RX_FAILSAFE.
    # connect() lets us use send() and surfaces ICMP unreachable as
    # ConnectionRefusedError on Windows (instead of latching the
    # socket on first failure). Best-effort — swallow connect-time
    # OSError so mission_demo doesn't fail startup if BF SITL isn't
    # quite ready yet. Same pattern as bf_rc_keepalive / radio_to_bf.
    udp_rc_sock: socket.socket | None = None
    udp_rc_addr = (args.udp_rc_host, args.udp_rc_port)
    udp_rc_send_errs = 0
    if not args.no_udp_rc:
        udp_rc_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            udp_rc_sock.connect(udp_rc_addr)
        except OSError:
            pass
        log.info("[mission_demo] UDP RC dual-write → %s:%d (40-B rc_packet)",
                 udp_rc_addr[0], udp_rc_addr[1])
    else:
        log.info("[mission_demo] --no-udp-rc set; MSP-only RC. "
                 "Expect periodic RX_FAILSAFE / ARM_SWITCH latches.")

    loop_dt = 1.0 / cfg.loop_hz
    last_rc_log = 0.0
    try:
        # Optional per-tick waypoint trace (TODO #10 diagnostic). Open
        # inside the try block so a failed open() doesn't leak the
        # already-opened FC + UDP socket. Header written once;
        # per-tick rows are emitted from _compute_waypoint_rc.
        if args.waypoint_trace_csv:
            mission._waypoint_csv_fh = open(
                args.waypoint_trace_csv, "w", newline="", encoding="utf-8",
            )
            mission._waypoint_csv = csv.writer(mission._waypoint_csv_fh)
            mission._waypoint_csv.writerow([
                "mono_t", "phase", "rel_alt_m", "distance_m", "h_err_deg",
                "yaw_aligned", "pitch_us", "throttle_us", "target",
            ])
            log.info("[mission_demo] waypoint trace → %s",
                     args.waypoint_trace_csv)
        while mission.phase != Phase.DONE:
            t0 = time.monotonic()
            # SEND RC FIRST, then poll telemetry. Reverse order (telem
            # first) caused BF to never arm when the RC arrived behind a
            # backlog of 6 telemetry requests. Sending RC at the head of
            # each tick keeps it the freshest packet in BF's queue.
            snap = mission.last_snap if hasattr(mission, "last_snap") else None
            if snap is not None and mission.should_send_msp():
                rc = mission.compute_rc(snap, t0)
                rc[SLOT_THROTTLE] = min(rc[SLOT_THROTTLE], 1841)
                fc.send_overrides(rc)
                # UDP RC dual-write: pad rc[8] to 16 channels with
                # PWM_MID, pack as 40-B rc_packet, send to BF's UDP RX
                # port. Same values as MSP, no precedence fight. A
                # send error doesn't break the loop — BF can lose a
                # frame here and there; what matters is the steady
                # 50-Hz stream so the RX state machine stays happy.
                if udp_rc_sock is not None:
                    udp_channels = list(rc) + [PWM_MID] * (
                        _NUM_UDP_RC_CHANNELS - len(rc)
                    )
                    try:
                        udp_rc_sock.send(
                            _UDP_RC_PACKET.pack(t0, *udp_channels)
                        )
                    except OSError as e:
                        udp_rc_send_errs += 1
                        if udp_rc_send_errs <= 3 or udp_rc_send_errs % 200 == 0:
                            log.warning(
                                "[mission_demo] UDP RC send failed "
                                "(%d so far): %s",
                                udp_rc_send_errs, e,
                            )
                if t0 - last_rc_log > 1.0:
                    last_rc_log = t0
                    rel = (snap.altitude.alt_cm / 100.0 - mission.home.alt_m) if (
                        snap.altitude and mission.home) else None
                    arm_flag_str = _decode_arming(fc._last_arming_flags) if fc._last_arming_flags >= 0 else "?"
                    payload_hex = getattr(fc, "_last_status_payload", "?")
                    log.info(
                        "[rc] %s thr=%d aux1=%d rel_alt=%s arm=0x%x[%s] fm=0x%x",
                        mission.phase.value, rc[SLOT_THROTTLE], rc[SLOT_AUX1],
                        f"{rel:.2f}m" if rel is not None else "?",
                        max(0, fc._last_arming_flags), arm_flag_str,
                        max(0, fc._last_flight_modes),
                    )
            new_snap = fc.tick(t0)
            mission.last_snap = new_snap
            mission.last_arming_flags = fc._last_arming_flags
            mission.update(new_snap, t0)
            elapsed = time.monotonic() - t0
            if elapsed < loop_dt:
                time.sleep(loop_dt - elapsed)
    except KeyboardInterrupt:
        log.info("[mission_demo] interrupted")
    finally:
        fc.close()
        if udp_rc_sock is not None:
            try:
                udp_rc_sock.close()
            except OSError:
                pass
            log.info("[mission_demo] UDP RC: %d packets failed to send",
                     udp_rc_send_errs)
        if mission._waypoint_csv_fh is not None:
            try:
                mission._waypoint_csv_fh.close()
            except OSError:
                pass
            log.info("[mission_demo] waypoint trace closed: %s",
                     args.waypoint_trace_csv)

    log.info("[mission_demo] done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
