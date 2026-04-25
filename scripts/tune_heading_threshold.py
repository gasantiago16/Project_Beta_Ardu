"""Tune the heading-divergence watchdog thresholds against a recorded log.

Replays the `heading_err_deg` time series from a companion log (the JSONL
written by main.py to `companion.log_path`) through HeadingDivergenceTracker
across a grid of thresholds × windows, and reports per-cell metrics:
  - trip_edges     : count of False→True transitions (number of distinct trips)
  - tripped_seconds: total time the tracker was reporting True

A 40°/2s cell with 12 short trips totaling 1.2s is a flapper — reject.
A 70°/4s cell with 1 trip totaling 25s is a keeper. Edge-count alone is
misleading for tuning; both metrics are needed.

Field requirements in the input log (per line):
    t              — monotonic seconds
    heading_err_deg — current bearing error vs commanded
    state          — state name (we only count samples where state==TRANSIT)

Synthetic logs (no real flight data yet): see `--synthetic` for a quick
smoke test that generates a fake "drift after 30 seconds of clean flight"
trace.

Output is ASCII-only — Windows consoles in cp1252 / cp437 can't render the
degree sign; using `deg` makes the output portable.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

# Allow `from racer_companion.safety import ...` regardless of cwd.
COMPANION = Path(__file__).resolve().parents[1] / "companion"
sys.path.insert(0, str(COMPANION))

from racer_companion.safety import HeadingDivergenceTracker  # noqa: E402


DEFAULT_THRESHOLDS = (40.0, 50.0, 60.0, 70.0, 80.0)
DEFAULT_WINDOWS = (1.0, 1.5, 2.0, 3.0, 4.0, 5.0)


def load_log(path: Path) -> list[tuple[float, float, str]]:
    """Parse the companion JSONL log. Returns (t, err_deg, state) tuples.

    Tolerant of malformed lines: a torn last line from a process killed
    mid-write is common; skipping is preferable to aborting the whole tune
    run. Logs warnings to stderr so users can spot truncated logs.
    """
    samples: list[tuple[float, float, str]] = []
    bad = 0
    with path.open(encoding="utf-8") as fh:
        for line_no, raw in enumerate(fh, 1):
            raw = raw.strip()
            if not raw:
                continue
            try:
                d = json.loads(raw)
                t = float(d["t"])
                err = float(d.get("heading_err_deg", 0.0))
                state = str(d.get("state", "IDLE"))
                samples.append((t, err, state))
            except (json.JSONDecodeError, KeyError, TypeError, ValueError):
                bad += 1
    if bad:
        print(f"# warning: skipped {bad} malformed line(s) in {path}", file=sys.stderr)
    # Warn on non-monotonic timestamps — HeadingDivergenceTracker expects them.
    last_t = -math.inf
    out_of_order = 0
    for t, _, _ in samples:
        if t < last_t:
            out_of_order += 1
        last_t = t
    if out_of_order:
        print(f"# warning: {out_of_order} sample(s) have non-monotonic timestamps "
              f"in {path}; trip counts may be incorrect", file=sys.stderr)
    return samples


def synthetic_drift_trace(
    duration_s: float = 60.0, dt: float = 0.02,
    drift_start_s: float = 30.0, drift_rate_deg_s: float = 5.0,
    noise_amp: float = 5.0,
) -> list[tuple[float, float, str]]:
    """Generate a synthetic TRANSIT-state log:
    - clean flight (small noise around 0° error) until drift_start_s
    - then yaw drift accumulates at drift_rate_deg_s

    Note: the drift accumulator wraps to [-180, 180] so very long drifts
    can fold back near 0. For the default parameters (30s clean + 30s drift
    at 5°/s = 150° max), no wrap occurs.
    """
    out: list[tuple[float, float, str]] = []
    t = 0.0
    while t < duration_s:
        # Cheap deterministic "noise" using sin so the test is reproducible.
        noise = noise_amp * math.sin(t * 7.3)
        if t < drift_start_s:
            err = noise
        else:
            err = noise + drift_rate_deg_s * (t - drift_start_s)
        # Wrap to [-180, 180]
        err = ((err + 180) % 360) - 180
        out.append((t, err, "TRANSIT"))
        t += dt
    return out


def measure_cell(
    samples: list[tuple[float, float, str]],
    threshold_deg: float, window_s: float,
) -> tuple[int, float]:
    """Replay through a tracker. Returns (trip_edges, tripped_seconds).

    `trip_edges` = count of False→True transitions (distinct trip events).
    `tripped_seconds` = sum of dt over samples where the tracker is True.

    Engagement gate matches main.py's `transit_active` — `state == "TRANSIT"`
    only, no error-magnitude filter. If main.py is ever changed to gate
    differently, this MUST be updated to match or the table will report
    trip counts the live tracker can't reproduce.
    """
    tracker = HeadingDivergenceTracker(threshold_deg=threshold_deg, window_s=window_s)
    trips = 0
    tripped_secs = 0.0
    last = False
    last_t: float | None = None
    for t, err, state in samples:
        transit_active = (state == "TRANSIT")
        tripped = tracker.update(t, err, transit_active)
        if tripped and not last:
            trips += 1
        if tripped and last_t is not None:
            tripped_secs += t - last_t
        last = tripped
        last_t = t
    return trips, tripped_secs


def count_trips(
    samples: list[tuple[float, float, str]],
    threshold_deg: float, window_s: float,
) -> int:
    """Convenience: trip-edge count only (kept for backward compatibility
    with existing tests)."""
    return measure_cell(samples, threshold_deg, window_s)[0]


def sweep(
    samples: list[tuple[float, float, str]],
    thresholds: tuple[float, ...] = DEFAULT_THRESHOLDS,
    windows: tuple[float, ...] = DEFAULT_WINDOWS,
) -> dict[tuple[float, float], tuple[int, float]]:
    """Cartesian product sweep. Returns {(threshold, window): (edges, secs)}."""
    return {
        (th, w): measure_cell(samples, th, w)
        for w in windows for th in thresholds
    }


def render_table(
    results: dict[tuple[float, float], tuple[int, float]],
    thresholds: tuple[float, ...], windows: tuple[float, ...],
    field: str = "edges",
) -> str:
    """Render one of the two metrics as a fixed-width ASCII table.

    `field` ∈ {'edges', 'secs'} selects which column of the (edges, secs)
    tuple to render. Title row mentions the unit.
    """
    cell_w = 7
    header_label = f"thr(deg)\\win(s)"
    label_w = max(len(header_label), 14)
    parts = [f"{header_label:<{label_w}} |"]
    for w in windows:
        parts.append(f"{w:>{cell_w-1}.1f}s")
    parts.append("")  # trailing separator
    header = " ".join(parts).rstrip()
    sep = "-" * len(header)
    lines = [header, sep]
    for th in thresholds:
        row = [f"{th:>{label_w-1}.1f} |"]
        for w in windows:
            edges, secs = results[(th, w)]
            value = edges if field == "edges" else secs
            if field == "edges":
                row.append(f"{value:>{cell_w}d}")
            else:
                row.append(f"{value:>{cell_w}.1f}")
        lines.append(" ".join(row).rstrip())
    return "\n".join(lines)


def _positive_finite(value: float, name: str) -> float:
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError(
            f"{name} must be a positive finite number; got {value}"
        )
    return value


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--log", type=Path, help="JSONL log path (from companion.log_path)")
    src.add_argument("--synthetic", action="store_true",
                     help="Replay a synthetic drift trace instead of a real log.")
    p.add_argument("--thresholds", type=float, nargs="+", default=list(DEFAULT_THRESHOLDS),
                   help="Threshold values to sweep (degrees, positive).")
    p.add_argument("--windows", type=float, nargs="+", default=list(DEFAULT_WINDOWS),
                   help="Window values to sweep (seconds, positive).")
    args = p.parse_args()

    try:
        thresholds = tuple(sorted(_positive_finite(v, "threshold") for v in args.thresholds))
        windows = tuple(sorted(_positive_finite(v, "window") for v in args.windows))
    except argparse.ArgumentTypeError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 2
    if not thresholds or not windows:
        print("ERROR: need at least one threshold and one window", file=sys.stderr)
        return 2

    if args.synthetic:
        samples = synthetic_drift_trace()
        print(f"# synthetic trace: {len(samples)} samples, drift starts at 30s")
    else:
        if not args.log.exists():
            print(f"ERROR: log not found: {args.log}", file=sys.stderr)
            return 2
        samples = load_log(args.log)
        n_transit = sum(1 for _, _, s in samples if s == "TRANSIT")
        print(f"# log {args.log}: {len(samples)} samples ({n_transit} in TRANSIT)")

    results = sweep(samples, thresholds, windows)
    print()
    print("# trip_edges (count of False->True transitions)")
    print(render_table(results, thresholds, windows, field="edges"))
    print()
    print("# tripped_seconds (total time tracker was True)")
    print(render_table(results, thresholds, windows, field="secs"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
