"""UDP 9004 RC heartbeat for BF SITL — keeps the RX state machine happy.

Why this tool exists
--------------------

BF SITL has two ways to receive RC values:
  1. **MSP overrides** (`MSP_SET_RAW_RC` over TCP 5761) — what mission_demo
     and racer_companion use. With `msp_override_channels_mask=255`, MSP
     wins on every channel.
  2. **UDP RC** (port 9004, 40-byte `rc_packet`) — BF's "real RX" path
     in SITL. Equivalent to a SerialRX wired to a UART on real hardware.

With ONLY MSP overrides flowing, BF's RX state machine still sees zero
"real" RX frames and stays in RX_FAILSAFE (arming-disable bit 2) for the
entire flight. After `failsafe_delay` (20 s at max) of bit-2 set, BF
transitions to STAGE2 failsafe → bit 1 FAILSAFE fires → with AUX1 high,
ARM_SWITCH latches mid-flight and the drone disarms.

This tool plugs the gap: it sends 40-byte UDP RC packets at 50 Hz with
neutral sticks, AUX1 high, AUX2-4 low. BF treats this stream as a real
RX, the failsafe timer resets every frame, RX_FAILSAFE never trips, and
ARM_SWITCH never latches mid-flight. mission_demo's MSP overrides on
channels 1-4 still take precedence for actual control via the override
mask, so this is purely a "keep BF happy" stream.

Differs from `radio_to_bf.py` which is the pilot-injection tool
(joystick → UDP, Phase 4 of the HITL plan). This is the no-radio,
no-joystick, headless companion-only equivalent.

Wire format (verified against `betaflight/4.5.1/src/main/target/SITL/target.h`):
    typedef struct {
        double   timestamp;             // 8 B, host-native double
        uint16_t channels[16];          // 32 B, raw PWM µs, 1000..2000
    } rc_packet;                        // 40 B total
BF strict-checks the packet size — wrong size = silent drop.

Usage:
    python -m integrations.tools.bf_rc_keepalive
    python -m integrations.tools.bf_rc_keepalive --bf-host 127.0.0.1 --bf-port 9004
    python -m integrations.tools.bf_rc_keepalive --aux1-low      # AUX1 LOW (disarm side)

**Apr 28 status: PARKED.** This tool conflicts with mission_demo's MSP
override on AUX1 in ways we haven't fully understood. The launch
sequence below is for solo-bench testing — don't add T3 to the
production launch flow until TODO.md item 1 is resolved. The most
likely path forward (item 1a) is to make mission_demo write the same
RC values to UDP 9004 directly, eliminating the precedence fight.

Run alongside the rest of the stack:
    T1: docker compose -f sitl/docker-compose.yml up -d
    T2: python -m integrations.tools.bf_gps_shim
    T3: python -m integrations.tools.bf_rc_keepalive   # ← solo bench only
    T4: python integrations/orchestrators/final_world_betaflight.py --motor-port 38500
    T5: python -m integrations.tools.mission_demo --bf-port 5762
"""
from __future__ import annotations

import argparse
import logging
import socket
import struct
import sys
import time


# 1 host-native double timestamp + 16 uint16 channels = 40 bytes.
# Matches `radio_to_bf.py`'s UDP_RC_PACKET; both must agree because
# any mismatch silently drops at BF.
RC_PACKET = struct.Struct("<d16H")
assert RC_PACKET.size == 40, f"rc_packet must be 40B, got {RC_PACKET.size}"

NUM_RC_CHANNELS = 16
PWM_MIN = 1000
PWM_MAX = 2000
PWM_MID = 1500


log = logging.getLogger("bf_rc_keepalive")


def build_neutral_channels(*, aux1_high: bool) -> list[int]:
    """Roll/Pitch/Yaw centered at 1500 µs, throttle low at 1000 µs,
    AUX1 HIGH (2000) by default — arm side. AUX2-4 + extra channels
    at 1500.

    Why AUX1 HIGH by default (Apr 28 finding): with the keepalive
    flowing AND `msp_override_channels_mask=255`, the keepalive's AUX1
    value reaches BF's arm-box logic alongside mission_demo's MSP
    override. Empirically the keepalive value can win — possibly
    because BF reads UDP 9004 RC into the same channel buffer the
    arm-box reads from, and the MSP override is layered separately.
    To avoid AUX1 fights, the keepalive sends the SAME value
    mission_demo sends (HIGH=2000 in the arm band). If they agree, BF
    sees AUX1=2000 from any source and the arm box fires.

    NOT safe from boot — empirical Apr 28: HIGH from boot can still
    latch ARM_SWITCH on the very first AUX1 LOW→HIGH transition (i.e.
    the moment the keepalive starts and BF's RX state goes from "no
    signal" to "valid RC with AUX1 HIGH"). BAD_RX_RECOVERY (bit 3) is
    briefly set during that transition, and ARM_SWITCH latches because
    the rule is "any disable bit set at LOW→HIGH = latch." The
    documented launch sequence in this tool's header is for solo bench
    use only — do not wire this into the production launch flow until
    `TODO.md` item 1 is resolved.

    Throttle-low is the load-bearing safety: if mission_demo crashes,
    BF falls back to this stream. Throttle below `min_check` (1050)
    keeps motors from spinning even with the arm box engaged."""
    rc = [PWM_MID] * NUM_RC_CHANNELS
    rc[0] = PWM_MID                # roll
    rc[1] = PWM_MID                # pitch
    rc[2] = PWM_MIN                # throttle (LOW; safe failsafe value)
    rc[3] = PWM_MID                # yaw
    rc[4] = PWM_MAX if aux1_high else PWM_MIN   # AUX1 — arm side / disarm side
    rc[5] = PWM_MIN                # AUX2 — race-mode etc., keep low
    rc[6] = PWM_MIN                # AUX3
    rc[7] = PWM_MIN                # AUX4
    return rc


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--bf-host", default="127.0.0.1",
                   help="BF SITL host (default: 127.0.0.1)")
    p.add_argument("--bf-port", type=int, default=9004,
                   help="BF SITL UDP RC port (default: 9004)")
    p.add_argument("--rate-hz", type=float, default=50.0,
                   help="Send rate. BF expects RC every ~10-20 ms; "
                        "50 Hz keeps the rx-loss timer well-fed.")
    p.add_argument("--aux1-low", action="store_true",
                   help="Send AUX1 LOW (1000 µs, disarm side) instead "
                        "of HIGH. Default is HIGH because mission_demo "
                        "needs AUX1 HIGH to engage BF's arm box and the "
                        "keepalive's AUX1 value can fight the MSP "
                        "override (Apr 28 finding); they must agree. "
                        "Use --aux1-low only when running this tool as "
                        "a sanity stream WITHOUT mission_demo connected.")
    p.add_argument("--log-period-s", type=float, default=10.0,
                   help="How often to print a heartbeat line (default 10s).")
    p.add_argument("--log-level", default="INFO")
    args = p.parse_args()

    logging.basicConfig(
        level=args.log_level.upper(),
        format="%(asctime)s %(levelname)s %(name)s %(message)s",
    )

    log.warning(
        "do NOT run this tool alongside `radio_to_bf` — both write to "
        "UDP %d and the last 40-byte packet wins; the operator can't "
        "tell which one BF is acting on. Use one or the other.",
        args.bf_port,
    )

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    addr = (args.bf_host, args.bf_port)
    # connect() lets us use send() instead of sendto() and surfaces ICMP
    # port-unreachable as ConnectionRefusedError on Windows. Without it,
    # the first sendto to a non-listening port can latch WSAECONNRESET
    # on the socket and every subsequent send fails forever even after
    # BF comes up. Best-effort — swallow OSError so we don't fail
    # startup if BF isn't quite ready yet. Same pattern as radio_to_bf.
    try:
        sock.connect(addr)
    except OSError:
        pass
    period = 1.0 / max(1.0, args.rate_hz)
    channels = build_neutral_channels(aux1_high=not args.aux1_low)

    log.info("sending UDP RC heartbeat → %s:%d at %.0f Hz "
             "(AUX1=%s, throttle=%d µs)",
             addr[0], addr[1], args.rate_hz,
             "LOW" if args.aux1_low else "HIGH",
             channels[2])

    sent = 0
    send_errs = 0
    last_log = time.monotonic()
    try:
        while True:
            t0 = time.monotonic()
            packet = RC_PACKET.pack(t0, *channels)
            try:
                sock.send(packet)
                sent += 1
            except OSError as e:
                # Don't crash on a transient send error. Windows can
                # raise ConnectionResetError after an ICMP unreachable;
                # subsequent sends will recover once BF comes up.
                send_errs += 1
                if send_errs <= 3 or send_errs % 200 == 0:
                    log.warning("send failed (%d so far): %s", send_errs, e)
            now = time.monotonic()
            if now - last_log >= args.log_period_s:
                last_log = now
                log.info("heartbeat: %d packets sent (%d send errors)",
                         sent, send_errs)
            sleep_for = period - (time.monotonic() - t0)
            if sleep_for > 0:
                time.sleep(sleep_for)
    except KeyboardInterrupt:
        log.info("interrupted; %d packets sent total (%d errors)",
                 sent, send_errs)
    finally:
        sock.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
