"""Replay a recorded MSP byte log through the companion's parser.

Use after a flight or SITL run (recorded via `recorder.RecordingAdapter`)
to inspect the decoded telemetry stream offline.

Usage:
    python -m racer_companion.tools.replay --log path/to/session.jsonl
    python -m racer_companion.tools.replay --log s.jsonl --realtime  # natural pace

By default we drain everything as fast as possible. `--realtime` paces to
the original recording timing — useful when stepping through a flight to
match it against video.
"""
from __future__ import annotations

import argparse
import sys
import time

from racer_companion import msp
from racer_companion.recorder import ReplayAdapter


def _format_frame(cmd: int, payload: bytes) -> str:
    name = {
        msp.MSP_RAW_GPS: "MSP_RAW_GPS",
        msp.MSP_ATTITUDE: "MSP_ATTITUDE",
        msp.MSP_ALTITUDE: "MSP_ALTITUDE",
        msp.MSP_ANALOG: "MSP_ANALOG",
        msp.MSP_RC: "MSP_RC",
        msp.MSP_STATUS_EX: "MSP_STATUS_EX",
        msp.MSP_BOXNAMES: "MSP_BOXNAMES",
        msp.MSP_SET_RAW_RC: "MSP_SET_RAW_RC",
    }.get(cmd, f"MSP_{cmd}")
    return f"{name}({cmd}) {len(payload)}B {payload[:32].hex()}"


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--log", required=True, help="JSONL recording file path")
    p.add_argument("--realtime", action="store_true",
                   help="Pace to the original recording timing.")
    p.add_argument("--max-frames", type=int, default=0,
                   help="Stop after N frames (0 = no limit).")
    args = p.parse_args()

    if args.realtime:
        time_source = time.monotonic
    else:
        # `inf` makes ReplayAdapter believe all events are due → drains immediately.
        time_source = lambda: float("inf")

    adapter = ReplayAdapter(args.log, time_source=time_source)
    client = msp.MspClient.from_adapter(adapter)

    n = 0
    while not adapter.exhausted:
        for cmd, payload in client.poll():
            print(_format_frame(cmd, payload))
            n += 1
            if args.max_frames and n >= args.max_frames:
                return 0
        if args.realtime:
            time.sleep(0.01)

    print(f"--- replay complete: {n} frames ---")
    return 0


if __name__ == "__main__":
    sys.exit(main())
