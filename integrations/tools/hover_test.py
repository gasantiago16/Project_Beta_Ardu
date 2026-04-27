"""Standalone arm + throttle test via MSP override.

Bypasses the full mission state machine (which needs GPS the BF SITL
container doesn't provide) so you can see Iris actually lift off in
Isaac Sim. Run this AFTER the orchestrator + Pegasus are up and the
bridge is exchanging packets.

Sequence:
  t=0      send neutral RC on channels 1-4 (centered sticks, throttle low)
           via MSP_SET_RAW_RC at 50 Hz so BF's failsafe stays happy.
  t=2      send "arm" RC: roll/pitch/yaw centered, throttle 1000 (min),
           AUX1 high → BF ARM box closes.
  t=3      ramp throttle from 1000 to --hover-throttle over 1 s.
  t=3+    hold hover throttle. Iris lifts off and hangs at whatever
           altitude the sim's drag balance settles to.
  Ctrl-C → throttle to 1000, disarm via AUX1 low, exit.

This script does NOT navigate. It's just to prove BF responds to MSP
override + Iris physics responds to BF motors.
"""
from __future__ import annotations

import argparse
import logging
import sys
import time

from racer_companion.fc.betaflight import BetaflightAdapter

log = logging.getLogger("hover_test")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--bf-host", default="127.0.0.1")
    p.add_argument("--bf-port", type=int, default=5761)
    p.add_argument("--rate-hz", type=float, default=50.0)
    p.add_argument("--hover-throttle", type=int, default=1500,
                   help="PWM us. Iris hovers around 1500 with default thrust "
                        "curve; bump to 1600 if it sags. Race-quad: ~1300.")
    p.add_argument("--ramp-s", type=float, default=1.0)
    p.add_argument("--duration-s", type=float, default=20.0,
                   help="How long to hold hover after ramp.")
    args = p.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

    log.info("connecting to BF SITL at tcp://%s:%d", args.bf_host, args.bf_port)
    fc = BetaflightAdapter.open(
        f"tcp://{args.bf_host}:{args.bf_port}", baud=115200,
        telemetry_period_s=1.0,
    )

    dt = 1.0 / args.rate_hz

    def send(roll: int, pitch: int, throttle: int, yaw: int,
             aux1: int = 1000, aux2: int = 1000,
             aux3: int = 1000, aux4: int = 1000) -> None:
        # MSP_SET_RAW_RC frame slots, given BF's default rcmap "AETR" +
        # rxRuntimeState.rcReadRawFn pulling raw[i] then rcmap-translating
        # into rcData[ROLL=0, PITCH=1, YAW=2, THROTTLE=3]:
        #   slot 0 -> rcData[ROLL]      (Aileron)
        #   slot 1 -> rcData[PITCH]     (Elevator)
        #   slot 2 -> rcData[THROTTLE]  (Throttle)   ← throttle here
        #   slot 3 -> rcData[YAW]       (Rudder)     ← yaw here
        #   slot 4+ -> rcData[AUX1..]
        # i.e. MSP slot order is [R, P, T, Y, AUX1, ...]. Verified live
        # via integrations/tools/bf_diag against BF 4.5.1.
        # NOTE: the companion's racer_companion/msp.py defines
        # CH_THROTTLE=3 + CH_YAW=2 — that is INCONSISTENT with this
        # MSP slot layout. See HANDOFF.md "Real-flight risks" for the
        # bug write-up; this tool sends the slot order BF expects.
        fc.send_overrides([roll, pitch, throttle, yaw, aux1, aux2, aux3, aux4])

    try:
        # Phase 1: neutral RC, 2 s. Lets BF's RC failsafe deassert.
        log.info("phase 1: idle RC (2 s)")
        t0 = time.monotonic()
        while time.monotonic() - t0 < 2.0:
            send(1500, 1500, 1000, 1500, aux1=1000)
            time.sleep(dt)

        # Phase 2: arm. AUX1 high while throttle is min.
        log.info("phase 2: ARMING (AUX1 -> 2000)")
        t0 = time.monotonic()
        while time.monotonic() - t0 < 1.0:
            send(1500, 1500, 1000, 1500, aux1=2000)
            time.sleep(dt)

        # Phase 3: ramp throttle from 1000 -> hover.
        log.info("phase 3: throttle ramp 1000 -> %d over %.1f s",
                 args.hover_throttle, args.ramp_s)
        ramp_start = time.monotonic()
        while True:
            elapsed = time.monotonic() - ramp_start
            if elapsed >= args.ramp_s:
                break
            t = elapsed / args.ramp_s
            thr = int(1000 + t * (args.hover_throttle - 1000))
            send(1500, 1500, thr, 1500, aux1=2000)
            time.sleep(dt)

        # Phase 4: hold hover.
        log.info("phase 4: holding hover throttle=%d for %.1f s",
                 args.hover_throttle, args.duration_s)
        hold_start = time.monotonic()
        while time.monotonic() - hold_start < args.duration_s:
            send(1500, 1500, args.hover_throttle, 1500, aux1=2000)
            time.sleep(dt)

    except KeyboardInterrupt:
        log.info("interrupted; landing + disarming")
    finally:
        # Land cushion: throttle to 1000, then disarm.
        log.info("landing: throttle 1000 + AUX1 low (1 s)")
        t0 = time.monotonic()
        while time.monotonic() - t0 < 1.0:
            send(1500, 1500, 1000, 1500, aux1=1000)
            time.sleep(dt)
        try:
            fc.close()
        except Exception:
            pass

    log.info("done")
    return 0


if __name__ == "__main__":
    sys.exit(main())
