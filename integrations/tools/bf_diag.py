"""Live BF SITL diagnostic — polls MSP_STATUS_EX + MSP_RC (and optionally
pumps an ARM sequence on the same socket).

BF SITL's MSP TCP server only accepts ONE concurrent connection — a
second client kicks the first. So we can't run hover_test + bf_diag in
parallel; instead, this tool can do BOTH on a single socket: pump the
arming RC override stream (centered roll/pitch/yaw + throttle ramp +
AUX1 high) AND poll MSP_STATUS_EX / MSP_RC interleaved.

Two questions this answers:
  1. What channel values does BF actually have right now? If they
     don't match what we're sending, the MSP override path is broken.
  2. Which arming-disable flags are set, decoded by name, live?

Run modes:
  --mode probe        # passive: just poll, send no RC. Baseline state.
  --mode arm          # active: pump the same arm sequence as hover_test
                      #   AND poll status — single socket.

Run prerequisites:
  T1 (always): docker compose -f sitl/docker-compose.yml up -d
  T2 (only if you want IMU updates → ANGLE flag clearing):
        python -m integrations.tools.fake_pegasus_loop --duration 60
        # or run the orchestrator
"""
from __future__ import annotations

import argparse
import socket
import struct
import sys
import time

# ── MSP v1 wire format ──────────────────────────────────────────────────────
# Request:  $M< <size> <cmd> <payload> <checksum>
# Response: $M> <size> <cmd> <payload> <checksum>
#   checksum = XOR of size + cmd + payload bytes
MSP_HEADER_REQ = b"$M<"
MSP_HEADER_RESP = b"$M>"

MSP_RC = 105
MSP_STATUS_EX = 150
MSP_SET_RAW_RC = 200


def build_request(cmd: int, payload: bytes = b"") -> bytes:
    size = len(payload)
    pkt = bytes([size, cmd]) + payload
    chk = 0
    for b in pkt:
        chk ^= b
    return MSP_HEADER_REQ + pkt + bytes([chk])


def build_set_raw_rc(channels: list[int]) -> bytes:
    """MSP_SET_RAW_RC — payload is N uint16 channel values."""
    payload = struct.pack(f"<{len(channels)}H", *channels)
    return build_request(MSP_SET_RAW_RC, payload)


# ── Arming-disable flag bit names, BF 4.5.1 order ───────────────────────────
ARMING_FLAG_NAMES = [
    "NO_GYRO",          # 0
    "FAILSAFE",         # 1
    "RX_FAILSAFE",      # 2  ← the "RXLOSS" we keep seeing
    "NOT_DISARMED",     # 3
    "BOXFAILSAFE",      # 4
    "RUNAWAY_TAKEOFF",  # 5
    "CRASH_DETECTED",   # 6
    "THROTTLE",         # 7
    "ANGLE",            # 8
    "BOOT_GRACE_TIME",  # 9
    "NOPREARM",         # 10
    "LOAD",             # 11
    "CALIBRATING",      # 12
    "CLI",              # 13
    "CMS_MENU",         # 14
    "BST",              # 15
    "MSP",              # 16
    "PARALYZE",         # 17
    "GPS",              # 18
    "RESC",             # 19
    "DSHOT_TELEM",      # 20
    "REBOOT_REQUIRED",  # 21
    "DSHOT_BITBANG",    # 22
    "ACC_CALIBRATION",  # 23
    "MOTOR_PROTOCOL",   # 24
    "ARM_SWITCH",       # 25  ← derived; fires if any other is active when arming
]


def decode_arming_flags(flags: int) -> str:
    if flags == 0:
        return "(none — armable)"
    return " ".join(
        name for i, name in enumerate(ARMING_FLAG_NAMES) if flags & (1 << i)
    )


# ── Stream parser: extract MSP responses from a rolling byte buffer ─────────
class MspStream:
    def __init__(self) -> None:
        self.buf = b""

    def feed(self, chunk: bytes) -> list[tuple[int, bytes]]:
        """Append bytes; return all complete (cmd, payload) frames found."""
        out: list[tuple[int, bytes]] = []
        self.buf += chunk
        while True:
            idx = self.buf.find(MSP_HEADER_RESP)
            if idx < 0:
                # No header — drop everything up to the last '$' (might be
                # mid-header) so the buf doesn't grow unbounded.
                last_dollar = self.buf.rfind(b"$")
                if last_dollar > 0:
                    self.buf = self.buf[last_dollar:]
                break
            if len(self.buf) < idx + 5:
                break
            size = self.buf[idx + 3]
            cmd = self.buf[idx + 4]
            frame_end = idx + 5 + size + 1
            if len(self.buf) < frame_end:
                break
            payload = self.buf[idx + 5 : idx + 5 + size]
            out.append((cmd, payload))
            self.buf = self.buf[frame_end:]
        return out


def parse_status_ex(payload: bytes) -> dict:
    if len(payload) < 22:
        return {"error": f"short payload: {len(payload)}B"}
    o = 0
    cycle_time = struct.unpack_from("<H", payload, o)[0]; o += 2
    _i2c_err = struct.unpack_from("<H", payload, o)[0]; o += 2
    _sensors = struct.unpack_from("<H", payload, o)[0]; o += 2
    _flight_mode_flags = struct.unpack_from("<I", payload, o)[0]; o += 4
    _pid_profile_idx = payload[o]; o += 1
    _sys_load = struct.unpack_from("<H", payload, o)[0]; o += 2
    _pid_profile_count = payload[o]; o += 1
    _rate_profile_idx = payload[o]; o += 1
    fmf_extra_bytes = payload[o]; o += 1
    o += fmf_extra_bytes
    _arming_flags_count = payload[o]; o += 1
    arming_disable_flags = struct.unpack_from("<I", payload, o)[0]; o += 4
    return {
        "cycle_time_us": cycle_time,
        "arming_disable_flags": arming_disable_flags,
        "arming_disable_names": decode_arming_flags(arming_disable_flags),
    }


def parse_rc(payload: bytes) -> list[int]:
    n = len(payload) // 2
    return list(struct.unpack_from(f"<{n}H", payload, 0))


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--bf-host", default="127.0.0.1")
    p.add_argument("--bf-port", type=int, default=5761)
    p.add_argument("--duration-s", type=float, default=20.0)
    p.add_argument("--interval-s", type=float, default=0.5,
                   help="Status/RC poll period.")
    p.add_argument("--mode", choices=["probe", "arm"], default="probe",
                   help="probe = passive (no RC sent). arm = pump arm sequence.")
    p.add_argument("--rc-rate-hz", type=float, default=50.0,
                   help="RC pump rate when --mode=arm.")
    p.add_argument("--hover-throttle", type=int, default=1700)
    p.add_argument("--throttle-min", type=int, default=950,
                   help="Throttle PWM during idle/disarm phases. Must be < BF "
                        "min_check (default 1050) for the THROTTLE arming "
                        "flag to clear. 950 leaves margin.")
    p.add_argument("--idle-s", type=float, default=10.0,
                   help="Idle (low throttle, AUX1 disarm) duration before "
                        "raising AUX1 to arm. ANGLE arming flag takes ~7 s "
                        "to clear in BF SITL after FDM IMU starts flowing.")
    args = p.parse_args()

    s = socket.create_connection((args.bf_host, args.bf_port), timeout=2)
    s.setblocking(False)
    print(f"[bf_diag] connected {args.bf_host}:{args.bf_port} mode={args.mode}",
          flush=True)

    stream = MspStream()
    t0 = time.monotonic()
    last_poll = 0.0
    last_rc = 0.0
    rc_period = 1.0 / args.rc_rate_hz

    last_status: dict | None = None
    last_rc_values: list[int] | None = None
    last_print = 0.0

    # MSP_SET_RAW_RC frame slot layout (default BF rcmap "AETR"):
    #   slot 0 -> rcData[ROLL]      (Aileron)
    #   slot 1 -> rcData[PITCH]     (Elevator)
    #   slot 2 -> rcData[THROTTLE]  (Throttle)
    #   slot 3 -> rcData[YAW]       (Rudder)
    #   slot 4+ -> rcData[AUX1..]
    # i.e. [R, P, T, Y, AUX1, ...]. Verified empirically against BF
    # 4.5.1 (see HANDOFF.md / project_beta_ardu_arming_blocker memo).
    def rc_for(t: float) -> list[int] | None:
        """RC override schedule for --mode=arm. Returns None when nothing
        to send (probe mode)."""
        if args.mode != "arm":
            return None
        # Order is [Roll, Pitch, THROTTLE, Yaw, AUX1..]. Slot 2 is
        # throttle per BF AETR rcmap (see slot-layout comment above).
        idle = args.idle_s
        if t < idle:
            # Idle: centered, throttle low, AUX1 disarm. Wait for
            # ANGLE / RX_FAILSAFE flags to clear before arming.
            return [1500, 1500, args.throttle_min, 1500, 1000, 1000, 1000, 1000]
        elif t < idle + 1.0:
            # ARM: AUX1 high, throttle still low.
            return [1500, 1500, args.throttle_min, 1500, 2000, 1000, 1000, 1000]
        elif t < idle + 3.0:
            # Throttle ramp throttle_min -> hover over 2 s.
            ramp = (t - (idle + 1.0)) / 2.0
            thr = int(args.throttle_min + ramp * (args.hover_throttle - args.throttle_min))
            return [1500, 1500, thr, 1500, 2000, 1000, 1000, 1000]
        elif t < args.duration_s - 1.0:
            # Hold hover.
            return [1500, 1500, args.hover_throttle, 1500, 2000, 1000, 1000, 1000]
        else:
            # Disarm.
            return [1500, 1500, args.throttle_min, 1500, 1000, 1000, 1000, 1000]

    while True:
        now = time.monotonic()
        elapsed = now - t0
        if elapsed >= args.duration_s:
            break

        # Send RC override at rc_rate_hz (arm mode only).
        if now - last_rc >= rc_period:
            ch = rc_for(elapsed)
            if ch is not None:
                try:
                    s.sendall(build_set_raw_rc(ch))
                except OSError as e:
                    print(f"[bf_diag] sendall RC failed: {e}", flush=True)
                    break
            last_rc = now

        # Poll status + RC at interval_s.
        if now - last_poll >= args.interval_s:
            try:
                s.sendall(build_request(MSP_STATUS_EX))
                s.sendall(build_request(MSP_RC))
            except OSError as e:
                print(f"[bf_diag] sendall poll failed: {e}", flush=True)
                break
            last_poll = now

        # Drain any responses.
        try:
            chunk = s.recv(4096)
            if chunk:
                for cmd, payload in stream.feed(chunk):
                    if cmd == MSP_STATUS_EX:
                        last_status = parse_status_ex(payload)
                    elif cmd == MSP_RC:
                        last_rc_values = parse_rc(payload)
        except BlockingIOError:
            pass
        except OSError as e:
            print(f"[bf_diag] recv failed: {e}", flush=True)
            break

        # Periodic print.
        if now - last_print >= args.interval_s:
            ch = rc_for(elapsed)
            phase = "idle"
            if ch is not None:
                if elapsed < 2.0: phase = "idle"
                elif elapsed < 3.0: phase = "arm-aux"
                elif elapsed < 5.0: phase = "ramp"
                elif elapsed < args.duration_s - 1.0: phase = "hold"
                else: phase = "disarm"
            sent = ch if ch else "(probe)"
            flags = last_status.get("arming_disable_flags", -1) if last_status else -1
            names = last_status.get("arming_disable_names", "(no resp yet)") if last_status else "(no resp yet)"
            rc_str = (
                f"{last_rc_values[0]} {last_rc_values[1]} {last_rc_values[2]} {last_rc_values[3]} | "
                f"{last_rc_values[4]} {last_rc_values[5]} {last_rc_values[6]} {last_rc_values[7]}"
                if last_rc_values and len(last_rc_values) >= 8
                else "(no resp yet)"
            )
            print(
                f"[bf_diag] t={elapsed:5.1f}s phase={phase:7s} "
                f"sent={sent} | flags=0x{max(flags,0):08x} [{names}]\n"
                f"[bf_diag]               rc[1..8]={rc_str}",
                flush=True,
            )
            last_print = now

        time.sleep(0.005)

    s.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
