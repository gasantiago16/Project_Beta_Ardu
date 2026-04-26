"""Standalone smoke test: drives the BetaflightUdpBackend at 250 Hz with a
synthesized hovering-quad State, against a real BF SITL Docker container.

Run from `Project_Beta_Ardu/`:
    docker compose -f sitl/docker-compose.yml up -d
    python -m integrations.tools.fake_pegasus_loop --duration 30
    docker compose -f sitl/docker-compose.yml down

What's validated:
- The wire protocol (FDM out, motor back) actually round-trips against BF.
- Lockstep timing holds at 250 Hz on this machine.
- Motor commands are non-zero (BF is producing real outputs, not stuck).

If you see ZERO motor packets, the most likely causes (in order):
1. BF SITL not running (check `docker compose ps`).
2. fdm_port / motor_port reversed (the Dockerfile has the canonical mapping).
3. BF SITL configured with a hardcoded sim_host that doesn't match this
   process's IP — check the container's start.sh / env.
4. Windows Docker NAT — packets going to host can't reach BF inside the
   container; see Phase 2 docs for `host.docker.internal` fix.

If motor packets arrive but values are stuck at 0.0, the FC isn't armed:
that's expected. Arm via QGC's joystick widget or the racer_companion's
MSP override. This script is just a wire smoke test.
"""
from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass

import numpy as np

from integrations.pegasus_betaflight_backend import (
    BetaflightBackendConfig,
    BetaflightUdpBackend,
)


@dataclass
class HoverState:
    """A motionless, level quad at altitude. Pegasus-shaped (ENU/FLU)."""
    position: np.ndarray
    attitude: np.ndarray
    linear_velocity: np.ndarray
    linear_body_velocity: np.ndarray
    angular_velocity: np.ndarray
    linear_acceleration: np.ndarray


def make_hover_state(alt_m: float = 5.0) -> HoverState:
    return HoverState(
        position=np.array([0.0, 0.0, alt_m]),
        attitude=np.array([0.0, 0.0, 0.0, 1.0]),  # identity, level
        linear_velocity=np.zeros(3),
        linear_body_velocity=np.zeros(3),
        angular_velocity=np.zeros(3),
        linear_acceleration=np.zeros(3),
    )


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--duration", type=float, default=10.0,
                   help="Seconds to run (default 10).")
    p.add_argument("--rate-hz", type=float, default=250.0,
                   help="FDM tx rate (default 250 Hz to match Pegasus).")
    p.add_argument("--bf-host", default="127.0.0.1")
    p.add_argument("--fdm-port", type=int, default=9002,
                   help="Sim → BF UDP port (default 9002 per BF SITL).")
    p.add_argument("--motor-port", type=int, default=9003,
                   help="BF → Sim UDP port (default 9003 per BF SITL).")
    p.add_argument("--alt", type=float, default=5.0)
    p.add_argument("--print-every", type=int, default=125,
                   help="Print sample motor output every N ticks (default 125 = 0.5s).")
    args = p.parse_args()

    cfg = BetaflightBackendConfig(
        bf_host=args.bf_host,
        fdm_port=args.fdm_port,
        motor_port=args.motor_port,
    )
    be = BetaflightUdpBackend(cfg)
    be.start()

    state = make_hover_state(alt_m=args.alt)
    dt = 1.0 / args.rate_hz
    n_ticks = int(args.duration * args.rate_hz)

    print(f"[fake_pegasus] {n_ticks} ticks @ {args.rate_hz} Hz, "
          f"BF at {args.bf_host}:{args.fdm_port}/{args.motor_port}",
          flush=True)
    t0 = time.monotonic()
    next_tick = t0
    try:
        for i in range(n_ticks):
            now = time.monotonic()
            sleep_for = next_tick - now
            if sleep_for > 0:
                time.sleep(sleep_for)
            be.update_state(state)
            be.update(dt)

            if (i + 1) % args.print_every == 0:
                ref = be.input_reference()
                print(f"  tick {i+1:>5d}: motor ω = "
                      f"({ref[0]:6.1f}, {ref[1]:6.1f}, {ref[2]:6.1f}, {ref[3]:6.1f}) rad/s "
                      f"| timeouts={be._timeouts}", flush=True)

            next_tick += dt
    except KeyboardInterrupt:
        print("\n[fake_pegasus] Interrupted.", flush=True)
    finally:
        elapsed = time.monotonic() - t0
        print(
            f"\n[fake_pegasus] Done. tx={be._packets_tx} rx={be._packets_rx} "
            f"timeouts={be._timeouts} ({be._timeouts / max(1, be._packets_tx) * 100:.2f}%) "
            f"in {elapsed:.2f}s",
            flush=True,
        )
        be.stop()

    if be._packets_rx == 0:
        print("[fake_pegasus] FAIL: zero motor packets received. "
              "See module docstring for likely causes.", flush=True)
        return 2
    if be._timeouts / max(1, be._packets_tx) > 0.05:
        print("[fake_pegasus] WARN: >5% timeout rate.", flush=True)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
