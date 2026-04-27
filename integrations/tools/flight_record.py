"""Headless flight recorder + plotter.

Tails sim_loop's state file at ~10 Hz for `--duration-s` seconds and
saves both:
  - a CSV of (t, n_m, e_m, alt_m, yaw_deg)
  - a PNG with a top-down trajectory + altitude trace

Use this in test scripts or CI where a live matplotlib window isn't
practical. For interactive flying, use `flight_viewer` instead.

Run alongside the sim stack:

  T1>  docker compose up -d
  T2>  python -m integrations.tools.sim_loop --motor-port 35000
  T3>  python -m integrations.tools.bf_gps_shim
  T4>  python -m integrations.tools.mission_demo --bf-port 5762  &
  T4>  python -m integrations.tools.flight_record --duration-s 90
"""
from __future__ import annotations

import argparse
import csv
import os
import sys
import tempfile
import time

import matplotlib
matplotlib.use("Agg")  # headless — no display backend needed
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


DEFAULT_STATE_FILE = os.path.join(tempfile.gettempdir(), "bf_sim_state.txt")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--state-file", default=DEFAULT_STATE_FILE)
    p.add_argument("--duration-s", type=float, default=90.0,
                   help="How long to record after the state file first appears.")
    p.add_argument("--map-half-size", type=float, default=30.0)
    p.add_argument("--csv", default="flight.csv")
    p.add_argument("--png", default="flight.png")
    p.add_argument("--rate-hz", type=float, default=10.0,
                   help="Sample rate. sim_loop writes at 10 Hz so anything "
                        "higher just deduplicates.")
    args = p.parse_args()

    print(f"[record] watching {args.state_file}", flush=True)
    # Wait for state file to appear (up to 30 s).
    deadline = time.monotonic() + 30.0
    while not os.path.exists(args.state_file):
        if time.monotonic() > deadline:
            print(f"[record] FATAL: state file never appeared; is sim_loop "
                  f"running with --state-file matching this default?",
                  file=sys.stderr)
            return 2
        time.sleep(0.5)

    period = 1.0 / args.rate_hz
    samples: list[tuple[float, float, float, float, float]] = []
    last_mtime = 0.0
    t0 = time.monotonic()
    print(f"[record] recording for {args.duration_s:.0f} s "
          f"@ {args.rate_hz:.0f} Hz...", flush=True)
    while time.monotonic() - t0 < args.duration_s:
        try:
            mtime = os.path.getmtime(args.state_file)
            if mtime != last_mtime:
                last_mtime = mtime
                with open(args.state_file, "r") as f:
                    parts = f.read().strip().split()
                if len(parts) >= 4:
                    t = time.monotonic() - t0
                    n, e, alt, yaw = (float(parts[0]), float(parts[1]),
                                      float(parts[2]), float(parts[3]))
                    samples.append((t, n, e, alt, yaw))
        except (OSError, ValueError):
            pass
        time.sleep(period)

    print(f"[record] captured {len(samples)} samples", flush=True)

    # CSV
    with open(args.csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "n_m", "e_m", "alt_m", "yaw_deg"])
        w.writerows(samples)
    print(f"[record] wrote {args.csv}", flush=True)

    if not samples:
        print("[record] no samples — skipping plot", file=sys.stderr)
        return 1

    # PNG plot
    fig, (ax_xy, ax_alt) = plt.subplots(
        1, 2, figsize=(14, 7),
        gridspec_kw={"width_ratios": [3, 2]},
    )
    h = args.map_half_size
    ax_xy.set_xlim(-h * 1.4, h * 1.4)
    ax_xy.set_ylim(-h * 1.4, h * 1.4)
    ax_xy.set_aspect("equal")
    ax_xy.set_xlabel("East (m)")
    ax_xy.set_ylabel("North (m)")
    ax_xy.set_title("Top-down trajectory")
    ax_xy.grid(True, alpha=0.3)
    ax_xy.add_patch(Rectangle(
        (-h, -h), 2 * h, 2 * h,
        fill=False, edgecolor="gray", linestyle="--", alpha=0.5,
    ))
    waypoints = [
        ("SW", -h, -h), ("SE", -h, +h),
        ("NE", +h, +h), ("NW", +h, -h),
        ("CENTER", 0.0, 0.0),
    ]
    for label, n, e in waypoints:
        ax_xy.plot(e, n, "x", color="C1", markersize=12, markeredgewidth=2)
        ax_xy.annotate(label, (e, n), textcoords="offset points",
                       xytext=(8, 8), fontsize=9, color="C1")
    x_pattern = [(-h, -h), (+h, +h), (+h, -h), (-h, +h), (0, 0)]
    ax_xy.plot(
        [e for n, e in x_pattern], [n for n, e in x_pattern],
        ":", color="C1", alpha=0.4, label="X-pattern target",
    )

    ts = [s[0] for s in samples]
    ns = [s[1] for s in samples]
    es = [s[2] for s in samples]
    alts = [s[3] for s in samples]
    ax_xy.plot(es, ns, "-", color="C0", alpha=0.7, label="Trajectory")
    ax_xy.plot(es[0], ns[0], "o", color="green", markersize=10, label="Start")
    ax_xy.plot(es[-1], ns[-1], "s", color="red", markersize=10, label="End")
    ax_xy.legend(loc="upper left", fontsize=8)

    ax_alt.set_xlabel("Time (s)")
    ax_alt.set_ylabel("Altitude (m AGL)")
    ax_alt.set_title("Altitude trace")
    ax_alt.grid(True, alpha=0.3)
    # alt is published as the integrator's value (origin_alt + sim z).
    # Plot relative-to-start so the shape's the meaningful signal.
    alt0 = alts[0]
    ax_alt.plot(ts, [a - alt0 for a in alts], "-", color="C2")

    fig.suptitle(
        f"Project_Beta_Ardu HITL — flight_record "
        f"({len(samples)} samples over {ts[-1]:.0f} s)"
    )
    plt.tight_layout()
    plt.savefig(args.png, dpi=120)
    print(f"[record] wrote {args.png}", flush=True)

    # Stats
    n_min, n_max = min(ns), max(ns)
    e_min, e_max = min(es), max(es)
    alt_max = max(a - alt0 for a in alts)
    print(f"[record] stats:")
    print(f"  N range:  {n_min:+.1f} .. {n_max:+.1f} m  "
          f"({n_max - n_min:.1f} m span)")
    print(f"  E range:  {e_min:+.1f} .. {e_max:+.1f} m  "
          f"({e_max - e_min:.1f} m span)")
    print(f"  Peak alt: {alt_max:+.1f} m AGL")
    return 0


if __name__ == "__main__":
    sys.exit(main())
