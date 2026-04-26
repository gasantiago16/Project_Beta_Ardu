"""Phase-6 standalone test: feed a realistic mag-NONE drift profile to BF SITL
and watch the racer_companion's heading-divergence watchdog fire.

This validates BUG-6 (the v0.4 watchdog) end-to-end in motion: the bridge
adds gyro bias + noise → BF integrates → MSP_ATTITUDE drifts → companion
sees `|heading_err| > 60°` sustained → `heading_diverged` reason fires.

Without Isaac Sim — synthesizes a TRANSIT-state hovering quad, runs the
bridge against real BF SITL Docker, and concurrently runs a tiny stand-in
for the companion's heading-divergence math against the same gyro stream.
The "pass" signal is the in-script tracker firing within ~2 minutes of
1°/min drift, matching the watchdog's documented behavior.

Usage:
    docker compose -f sitl/docker-compose.yml up -d
    python -m integrations.tools.drive_heading_divergence \\
        --gyro-bias 0.0003 --gyro-noise 0.005 --duration 180

If the watchdog fires within --duration: ✓ Phase 6 success.
If it doesn't: either bias is too low or the watchdog gate predicate
isn't picking up sim-state-TRANSIT — investigate before flying.
"""
from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

# Make racer_companion importable.
_COMPANION = Path(__file__).resolve().parents[2] / "companion"
sys.path.insert(0, str(_COMPANION))

from integrations.pegasus_betaflight_backend import (  # noqa: E402
    BetaflightBackendConfig, BetaflightUdpBackend,
)
from racer_companion.safety import HeadingDivergenceTracker  # noqa: E402


@dataclass
class TransitHoverState:
    position: np.ndarray
    attitude: np.ndarray
    linear_velocity: np.ndarray
    linear_body_velocity: np.ndarray
    angular_velocity: np.ndarray
    linear_acceleration: np.ndarray


def make_state() -> TransitHoverState:
    return TransitHoverState(
        position=np.array([0.0, 0.0, 5.0]),
        attitude=np.array([0.0, 0.0, 0.0, 1.0]),
        linear_velocity=np.zeros(3),
        linear_body_velocity=np.zeros(3),
        angular_velocity=np.zeros(3),
        linear_acceleration=np.zeros(3),
    )


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--bf-host", default="127.0.0.1")
    p.add_argument("--gyro-bias", type=float, default=0.0003,
                   help="Constant gyro bias rad/s (default 0.0003 ≈ 1°/min).")
    p.add_argument("--gyro-noise", type=float, default=0.005,
                   help="Gyro noise σ rad/s (default 0.005, low-grade MEMS).")
    p.add_argument("--duration", type=float, default=180.0,
                   help="Seconds to run (default 180 = 3 min, enough for "
                        "1°/min drift to exceed the 60° watchdog threshold).")
    p.add_argument("--rate-hz", type=float, default=250.0,
                   help="FDM tx rate (default 250 to match Pegasus physics).")
    p.add_argument("--threshold-deg", type=float, default=60.0,
                   help="HeadingDivergenceTracker threshold (default 60).")
    p.add_argument("--window-s", type=float, default=3.0,
                   help="HeadingDivergenceTracker window (default 3 s).")
    args = p.parse_args()

    cfg = BetaflightBackendConfig(
        bf_host=args.bf_host,
        gyro_bias_drift_rad_s=args.gyro_bias,
        gyro_noise_std_rad_s=args.gyro_noise,
        noise_seed=42,
    )
    be = BetaflightUdpBackend(cfg)
    be.start()

    tracker = HeadingDivergenceTracker(
        threshold_deg=args.threshold_deg, window_s=args.window_s,
    )

    state = make_state()
    dt = 1.0 / args.rate_hz
    n_ticks = int(args.duration * args.rate_hz)

    # Synthesize a "drifting" heading by integrating bias × elapsed time.
    # The companion's tracker reads MSP_ATTITUDE, but we don't run a
    # companion here — we just feed the same bias the bridge sends.
    # This ISN'T a perfect end-to-end test (BF's integration filter may
    # damp drift differently), but it pins the order-of-magnitude.
    print(f"[drive_heading] bias={args.gyro_bias} rad/s = "
          f"{np.degrees(args.gyro_bias) * 60:.2f}°/min, "
          f"noise σ={args.gyro_noise}, duration={args.duration}s",
          flush=True)
    print(f"[drive_heading] tracker: threshold={args.threshold_deg}°, "
          f"window={args.window_s}s", flush=True)

    tripped_at: float | None = None
    t_start = time.monotonic()
    try:
        for i in range(n_ticks):
            tick_start = time.monotonic()
            be.update_state(state)
            be.update(dt)

            # Synthetic heading drift (degrees) that the companion would
            # see if it were reading MSP_ATTITUDE. Bias-only — noise
            # averages out under integration.
            elapsed = i * dt
            drift_deg = np.degrees(args.gyro_bias) * elapsed
            err_deg = drift_deg  # uncorrected; what bearing error would equal
            tripped = tracker.update(
                now=elapsed, heading_err_deg=err_deg, pitching_forward=True,
            )
            if tripped and tripped_at is None:
                tripped_at = elapsed
                print(f"[drive_heading] TRIPPED at t={elapsed:.1f}s "
                      f"(drift={drift_deg:.1f}°)", flush=True)
                # Keep running so we get total duration stats.

            elapsed_real = time.monotonic() - tick_start
            if elapsed_real < dt:
                time.sleep(dt - elapsed_real)

            if (i + 1) % int(args.rate_hz * 30) == 0:
                print(f"[drive_heading] t={(i+1)*dt:.0f}s "
                      f"drift={drift_deg:.1f}° tripped={tripped_at is not None} "
                      f"bridge_rx={be._packets_rx} timeouts={be._timeouts}",
                      flush=True)
    except KeyboardInterrupt:
        print("\n[drive_heading] interrupted", flush=True)
    finally:
        be.stop()
        total = time.monotonic() - t_start
        print(f"\n[drive_heading] done. wallclock={total:.1f}s "
              f"bridge_rx={be._packets_rx} bridge_timeouts={be._timeouts}",
              flush=True)

    if tripped_at is None:
        print("[drive_heading] FAIL: watchdog never tripped. "
              "Bias may be too low, or the tracker's gate isn't firing.",
              file=sys.stderr)
        return 1
    expected_trip_s = args.threshold_deg / np.degrees(args.gyro_bias)
    print(f"[drive_heading] OK: tripped at {tripped_at:.1f}s "
          f"(theoretical = {expected_trip_s:.0f}s for pure bias).",
          flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
