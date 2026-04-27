"""Takeoff -> hover -> land profile, open-loop time-based.

Single MSP TCP connection (sticks-only — no companion needed). Drives
BF SITL through:
  1. idle      throttle low, AUX1 disarm. Wait for ANGLE flag to clear.
  2. arm       AUX1 high, throttle still low.
  3. climb     throttle = climb_us. Iris lifts off.
  4. hover     throttle = hover_us (slightly above true equilibrium so
               Iris doesn't sink). Hold for hover_s.
  5. descend   throttle = descend_us (below hover). Iris descends.
  6. touchdown throttle = 950, AUX1 still high. Soft contact.
  7. disarm    AUX1 low, throttle 950.

Needs ANGLE mode bound (`aux 1 1 0 900 2100` in defaults.txt) for
auto-leveling — otherwise Iris flips on contact.

Run:
  docker compose -f sitl/docker-compose.yml up -d
  # ensure relay is on the right port (e.g. 28500)
  python -m integrations.tools.takeoff_hover_land --motor-port 28500

Watch Iris in Isaac Sim — she should rise smoothly to hover, hold
steady, then descend back to the asphalt.
"""
from __future__ import annotations

import argparse
import socket
import struct
import sys
import time

# MSP framing — duplicated minimally to keep this tool standalone.
MSP_HEADER_REQ = b"$M<"
MSP_SET_RAW_RC = 200


def msp_set_raw_rc(channels: list[int]) -> bytes:
    payload = struct.pack(f"<{len(channels)}H", *channels)
    pkt = bytes([len(payload), MSP_SET_RAW_RC]) + payload
    chk = 0
    for b in pkt:
        chk ^= b
    return MSP_HEADER_REQ + pkt + bytes([chk])


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--bf-host", default="127.0.0.1")
    p.add_argument("--bf-port", type=int, default=5761)
    p.add_argument("--rc-rate-hz", type=float, default=50.0)
    p.add_argument("--motor-port", type=int, default=28500,
                   help="Documented for log output only — actual port is "
                        "set by the in-container relay.")

    p.add_argument("--idle-s", type=float, default=12.0,
                   help="Wait for BF's ANGLE arming flag to clear before arming.")
    p.add_argument("--arm-s", type=float, default=1.0,
                   help="AUX1 high, throttle still low (sets up arming gate).")
    p.add_argument("--climb-s", type=float, default=4.0,
                   help="Climb to altitude phase. Bump if Iris hasn't visibly "
                        "left the asphalt by hover phase.")
    p.add_argument("--hover-s", type=float, default=10.0,
                   help="Hover-and-watch phase.")
    p.add_argument("--descend-s", type=float, default=4.0,
                   help="Controlled descent phase. Bump if Iris is still in "
                        "the air at touchdown.")
    p.add_argument("--touchdown-s", type=float, default=2.0,
                   help="Throttle-low cushion before disarm. Lets Iris settle.")

    p.add_argument("--throttle-min", type=int, default=950,
                   help="Idle/disarm throttle PWM. Must be < BF mincheck "
                        "(default 1050) for THROTTLE arming flag to clear.")
    p.add_argument("--climb-throttle", type=int, default=1700,
                   help="PWM during climb. Pick above hover-equilibrium so "
                        "Iris lifts off cleanly. Iris ~1700, race-quad ~1400.")
    p.add_argument("--hover-throttle", type=int, default=1650,
                   help="PWM during hover. Iris true equilibrium ~1640; 1650 "
                        "leaves a hair of climb-rate margin so she doesn't "
                        "sink before descent phase.")
    p.add_argument("--descend-throttle", type=int, default=1500,
                   help="PWM during descent. Below hover -> Iris drifts down. "
                        "1500 is roughly half of climb-thrust margin.")
    args = p.parse_args()

    s = socket.create_connection((args.bf_host, args.bf_port), timeout=2)
    print(f"[fly] connected {args.bf_host}:{args.bf_port} (motor relay -> {args.motor_port})",
          flush=True)

    dt = 1.0 / args.rc_rate_hz
    t_start = time.monotonic()

    # MSP_SET_RAW_RC slot order (BF default rcmap "AETR"):
    #   [Roll, Pitch, THROTTLE, Yaw, AUX1, AUX2, AUX3, AUX4]
    def send(throttle: int, aux1: int) -> None:
        s.sendall(msp_set_raw_rc([1500, 1500, throttle, 1500, aux1, 1000, 1000, 1000]))

    def hold_for(label: str, duration: float, throttle: int, aux1: int) -> None:
        elapsed_at_start = time.monotonic() - t_start
        print(f"[fly] t={elapsed_at_start:5.1f}s {label} for {duration:.1f}s "
              f"(throttle={throttle} aux1={aux1})", flush=True)
        end = time.monotonic() + duration
        while time.monotonic() < end:
            send(throttle, aux1)
            time.sleep(dt)

    def ramp(label: str, duration: float, t0: int, t1: int, aux1: int) -> None:
        elapsed_at_start = time.monotonic() - t_start
        print(f"[fly] t={elapsed_at_start:5.1f}s {label} ramp throttle "
              f"{t0} -> {t1} over {duration:.1f}s (aux1={aux1})", flush=True)
        ramp_start = time.monotonic()
        while True:
            r = (time.monotonic() - ramp_start) / duration
            if r >= 1.0:
                break
            thr = int(t0 + r * (t1 - t0))
            send(thr, aux1)
            time.sleep(dt)

    try:
        # Phase 1: idle + Phase 2: arm gate
        hold_for("idle (waiting on ANGLE arming flag)",
                 args.idle_s, args.throttle_min, 1000)
        hold_for("arm-aux high", args.arm_s, args.throttle_min, 2000)

        # Phase 3: climb (smooth ramp to climb-throttle)
        ramp("climb", 1.5, args.throttle_min, args.climb_throttle, 2000)
        hold_for("climb-hold", args.climb_s - 1.5, args.climb_throttle, 2000)

        # Phase 4: hover
        ramp("settle-to-hover", 1.0, args.climb_throttle, args.hover_throttle, 2000)
        hold_for("hover", args.hover_s, args.hover_throttle, 2000)

        # Phase 5: descend (smooth ramp to descend-throttle)
        ramp("descend", 1.5, args.hover_throttle, args.descend_throttle, 2000)
        hold_for("descend-hold", args.descend_s - 1.5, args.descend_throttle, 2000)

        # Phase 6: touchdown cushion (throttle low while still armed)
        ramp("touchdown", 0.5, args.descend_throttle, args.throttle_min, 2000)
        hold_for("touchdown-hold", args.touchdown_s - 0.5, args.throttle_min, 2000)

        # Phase 7: disarm
        hold_for("disarm", 1.0, args.throttle_min, 1000)

    except KeyboardInterrupt:
        print("\n[fly] interrupted; emergency disarm", flush=True)
        for _ in range(20):
            try:
                send(args.throttle_min, 1000)
            except OSError:
                break
            time.sleep(dt)
    finally:
        s.close()

    print(f"[fly] done in {time.monotonic() - t_start:.1f}s", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
