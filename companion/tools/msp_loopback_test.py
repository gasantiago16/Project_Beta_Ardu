"""Bench test (props OFF) — verifies UART comms with a real Betaflight FC.

Sequence:
1. Open serial.
2. Request MSP_API_VERSION + MSP_FC_VARIANT, print response.
3. Send center-stick MSP_SET_RAW_RC at 50Hz for N seconds while requesting MSP_RC.
4. Print MSP_RC echoes — if values match what was sent (and the AUX channel
   that activates msp_override is HIGH), the override is working.

USAGE:
    python -m tools.msp_loopback_test --port /dev/ttyUSB0
"""
from __future__ import annotations

import argparse
import time

from racer_companion import msp


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--port", required=True)
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--duration", type=float, default=5.0)
    args = p.parse_args()

    client = msp.MspClient(args.port, args.baud)
    print(f"Opened {args.port} @ {args.baud}")

    client.send(msp.MSP_API_VERSION)
    client.send(msp.MSP_FC_VARIANT)
    time.sleep(0.3)
    for cmd, payload in client.poll():
        if cmd == msp.MSP_API_VERSION:
            if len(payload) >= 3:
                print(f"  MSP_API_VERSION: protocol={payload[0]} api={payload[1]}.{payload[2]}")
        elif cmd == msp.MSP_FC_VARIANT:
            print(f"  MSP_FC_VARIANT: {payload[:4]!r}")
        else:
            print(f"  unexpected cmd={cmd} payload={payload.hex()}")

    print(f"Sending centered MSP_SET_RAW_RC for {args.duration}s (props OFF only)...")
    t_end = time.monotonic() + args.duration
    last = 0.0
    while time.monotonic() < t_end:
        client.send_raw_rc([1500, 1500, 1500, 1000, 1500, 1500, 1500, 1500])
        client.send(msp.MSP_RC)
        time.sleep(0.02)
        for cmd, payload in client.poll():
            if cmd == msp.MSP_RC:
                rc = msp.decode_rc(payload)
                now = time.monotonic()
                if now - last > 0.5:
                    print(f"  MSP_RC echo: {rc[:8]}")
                    last = now

    client.close()
    print("Done. Verify MSP_RC echo matches sent values when AUX-companion is HIGH.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
