"""Closed-loop altitude flight profile: climb-to-target -> hover -> yaw -> descend.

Reads BF's MSP_ALTITUDE for live altitude feedback and runs a P
controller on throttle to track a target altitude. Drives yaw via the
RC yaw channel during the spin phases. Single MSP TCP connection.

Profile (default):
  1. idle 12 s        AUX1 low, ANGLE arming flag clears
  2. arm  1 s         AUX1 high, throttle still low
  3. climb            target_alt_m. P controller on throttle until reached.
  4. hover  hover_s   hold altitude.
  5. yaw-left  3 s    yaw stick low + altitude hold.
  6. yaw-right 3 s    yaw stick high + altitude hold.
  7. descend 10 s     linear altitude ramp from target to 0.
  8. touchdown        throttle min once below 0.5 m.
  9. disarm           AUX1 low.

BF SITL's MSP_ALTITUDE comes from the virtual barometer driven by
the FDM packet's `pressure` field. Bridge derives pressure from
state.position[2] + origin_alt_m (default 100). For Pegasus's
spawn-on-asphalt at z≈1.3, BF's altimeter reads ~101 m. We treat
that as the ground baseline and target relative climb above it.

Run:
  docker compose -f sitl/docker-compose.yml up -d
  # ensure relay is on the right port (28500 default for our setup)
  python -m integrations.tools.flight_profile --motor-port 28500
"""
from __future__ import annotations

import argparse
import socket
import struct
import sys
import time

# MSP framing
MSP_HEADER_REQ = b"$M<"
MSP_HEADER_RESP = b"$M>"
MSP_ALTITUDE = 109
MSP_SET_RAW_RC = 200


def msp_msg(cmd: int, payload: bytes = b"") -> bytes:
    pkt = bytes([len(payload), cmd]) + payload
    chk = 0
    for b in pkt:
        chk ^= b
    return MSP_HEADER_REQ + pkt + bytes([chk])


def msp_set_raw_rc(channels: list[int]) -> bytes:
    return msp_msg(MSP_SET_RAW_RC, struct.pack(f"<{len(channels)}H", *channels))


class MspStream:
    """Rolling-buffer MSP response parser."""
    def __init__(self) -> None:
        self.buf = b""

    def feed(self, chunk: bytes) -> list[tuple[int, bytes]]:
        out: list[tuple[int, bytes]] = []
        self.buf += chunk
        while True:
            idx = self.buf.find(MSP_HEADER_RESP)
            if idx < 0:
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
            out.append((cmd, self.buf[idx + 5 : idx + 5 + size]))
            self.buf = self.buf[frame_end:]
        return out


def parse_altitude(payload: bytes) -> tuple[float, float]:
    """MSP_ALTITUDE: int32 alt (cm), int16 vario (cm/s).
    Returns (alt_m, vario_m_s)."""
    if len(payload) < 6:
        return 0.0, 0.0
    alt_cm, vario_cm_s = struct.unpack_from("<iH", payload, 0)
    # vario is signed
    vario_cm_s = struct.unpack_from("<h", payload, 4)[0]
    return alt_cm / 100.0, vario_cm_s / 100.0


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--bf-host", default="127.0.0.1")
    p.add_argument("--bf-port", type=int, default=5761)
    p.add_argument("--motor-port", type=int, default=28500,
                   help="For log output only.")
    p.add_argument("--rc-rate-hz", type=float, default=50.0)

    p.add_argument("--idle-s", type=float, default=12.0)
    p.add_argument("--arm-s", type=float, default=1.0)
    p.add_argument("--target-alt-m", type=float, default=10.0,
                   help="Target altitude above ground baseline (m).")
    p.add_argument("--climb-timeout-s", type=float, default=30.0,
                   help="Hard timeout if altitude target isn't reached. PD "
                        "controller is conservative; 30s is safe for 10 m.")
    p.add_argument("--hover-s", type=float, default=10.0)
    p.add_argument("--yaw-left-s", type=float, default=3.0)
    p.add_argument("--yaw-right-s", type=float, default=3.0)
    p.add_argument("--descend-s", type=float, default=10.0)

    p.add_argument("--throttle-min", type=int, default=950,
                   help="Idle/disarm throttle. Must be < BF mincheck (1050).")
    p.add_argument("--hover-throttle", type=int, default=1641,
                   help="PD bias point during climb/hover/yaw. 1641 = exact "
                        "Iris equilibrium per `iris_hover_calibrate` (motor_norm "
                        "0.6411). Was 1650 — close but biased the controller "
                        "9 µs above true hover.")
    p.add_argument("--descend-throttle", type=int, default=1500,
                   help="PD bias point during descent. Below hover → Iris "
                        "naturally settles; PD on top tracks the ramp.")
    p.add_argument("--throttle-kp", type=float, default=14.0,
                   help="Throttle P gain in PWM us per m of altitude error. "
                        "Steady-state climb rate ≈ (Kp/Kd)·err.")
    p.add_argument("--throttle-kd", type=float, default=80.0,
                   help="Throttle D gain in PWM us per m/s of vario. Brakes "
                        "the climb so we don't overshoot the target. With "
                        "Kp=14, Kd=80 ⇒ ~0.18·err m/s climb steady state.")
    p.add_argument("--throttle-min-active", type=int, default=1300,
                   help="Floor for the controller's throttle output during "
                        "active flight. Below this Iris falls.")
    p.add_argument("--throttle-max-active", type=int, default=1850,
                   help="Ceiling for controller output. Above this Iris "
                        "climbs uncontrollably.")
    p.add_argument("--touchdown-alt-m", type=float, default=0.5,
                   help="When alt drops below this during descent, switch "
                        "to throttle-min for cushioned touchdown.")
    p.add_argument("--yaw-left-pwm", type=int, default=1300)
    p.add_argument("--yaw-right-pwm", type=int, default=1700)
    args = p.parse_args()

    s = socket.create_connection((args.bf_host, args.bf_port), timeout=2)
    s.setblocking(False)
    print(f"[fly] connected {args.bf_host}:{args.bf_port} "
          f"(motor relay -> {args.motor_port})", flush=True)

    stream = MspStream()
    dt = 1.0 / args.rc_rate_hz
    alt_m = 0.0
    vario = 0.0
    alt_baseline_m: float | None = None  # set after first alt sample post-arm

    def send_rc(throttle: int, yaw: int, aux1: int) -> None:
        """MSP slot order [R, P, T, Y, AUX1, ...] given AETR rcmap."""
        try:
            s.sendall(msp_set_raw_rc([1500, 1500, throttle, yaw,
                                       aux1, 1000, 1000, 1000]))
        except OSError:
            pass

    def poll_alt() -> None:
        try:
            s.sendall(msp_msg(MSP_ALTITUDE))
        except OSError:
            pass

    def drain_responses() -> None:
        nonlocal alt_m, vario
        try:
            chunk = s.recv(4096)
        except (BlockingIOError, OSError):
            return
        if not chunk:
            return
        for cmd, payload in stream.feed(chunk):
            if cmd == MSP_ALTITUDE:
                alt_m, vario = parse_altitude(payload)

    def throttle_for_alt(target_m: float, bias: int | None = None) -> int:
        """PD controller throttle clamped to active range.
        Kp pushes toward target; Kd brakes by subtracting vario so a
        fast climb naturally throttles back before overshoot.
        Pass bias=descend_throttle during descent so Iris actually
        settles instead of holding altitude."""
        b = args.hover_throttle if bias is None else bias
        err = target_m - (alt_m - (alt_baseline_m or 0.0))
        out = b + int(args.throttle_kp * err - args.throttle_kd * vario)
        return max(args.throttle_min_active,
                   min(args.throttle_max_active, out))

    t_start = time.monotonic()
    last_poll = 0.0
    last_print = 0.0

    def log(label: str, throttle: int, yaw: int) -> None:
        nonlocal last_print
        now = time.monotonic()
        if now - last_print < 0.5:
            return
        rel = (alt_m - alt_baseline_m) if alt_baseline_m is not None else alt_m
        print(f"[fly] t={now - t_start:5.1f}s {label:>14s} "
              f"throttle={throttle} yaw={yaw} alt={alt_m:6.2f}m "
              f"rel_alt={rel:+6.2f}m vario={vario:+5.2f}m/s",
              flush=True)
        last_print = now

    def loop_until(deadline: float, label: str,
                   throttle_fn, yaw_pwm: int, aux1: int) -> None:
        """Pump RC at rc-rate-hz, poll alt every 200 ms, until deadline."""
        nonlocal last_poll
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now - last_poll >= 0.2:
                poll_alt()
                last_poll = now
            drain_responses()
            thr = throttle_fn() if callable(throttle_fn) else throttle_fn
            send_rc(thr, yaw_pwm, aux1)
            log(label, thr, yaw_pwm)
            time.sleep(dt)

    try:
        # Phase 1: idle (ANGLE flag clears)
        loop_until(t_start + args.idle_s, "idle",
                   args.throttle_min, 1500, 1000)

        # Capture baseline altitude — BF's barometer reads ~101 m on the
        # asphalt at default origin; our targets are RELATIVE to this.
        alt_baseline_m = alt_m
        print(f"[fly] altitude baseline = {alt_baseline_m:.2f} m "
              f"(target absolute = {alt_baseline_m + args.target_alt_m:.2f} m)",
              flush=True)

        # Phase 2: arm
        loop_until(time.monotonic() + args.arm_s, "arm",
                   args.throttle_min, 1500, 2000)

        # Phase 3: climb (closed-loop)
        climb_deadline = time.monotonic() + args.climb_timeout_s
        target_reached = False
        while time.monotonic() < climb_deadline and not target_reached:
            now = time.monotonic()
            if now - last_poll >= 0.2:
                poll_alt()
                last_poll = now
            drain_responses()
            thr = throttle_for_alt(args.target_alt_m)
            send_rc(thr, 1500, 2000)
            log("climb", thr, 1500)
            rel = alt_m - alt_baseline_m
            if abs(rel - args.target_alt_m) < 0.7 and abs(vario) < 0.4:
                target_reached = True
            time.sleep(dt)
        if target_reached:
            print(f"[fly] climb: reached target {args.target_alt_m:.1f} m", flush=True)
        else:
            print(f"[fly] climb: TIMEOUT at rel_alt={alt_m - alt_baseline_m:.2f} m, "
                  f"continuing anyway", flush=True)

        # Phase 4: hover
        loop_until(time.monotonic() + args.hover_s, "hover",
                   lambda: throttle_for_alt(args.target_alt_m), 1500, 2000)

        # Phase 5: yaw-left
        loop_until(time.monotonic() + args.yaw_left_s, "yaw-left",
                   lambda: throttle_for_alt(args.target_alt_m),
                   args.yaw_left_pwm, 2000)

        # Phase 6: yaw-right
        loop_until(time.monotonic() + args.yaw_right_s, "yaw-right",
                   lambda: throttle_for_alt(args.target_alt_m),
                   args.yaw_right_pwm, 2000)

        # Phase 7: descent (linear ramp on TARGET altitude over descend_s,
        # then hold target=0 until we actually reach ground). Lower bias
        # during descent so Iris settles instead of fighting gravity.
        descent_start = time.monotonic()
        descent_end = descent_start + args.descend_s
        descent_extension_cap = descent_end + 8.0  # never wait forever
        while time.monotonic() < descent_extension_cap:
            now = time.monotonic()
            if now - last_poll >= 0.2:
                poll_alt()
                last_poll = now
            drain_responses()
            if now < descent_end:
                r = (now - descent_start) / args.descend_s
                tgt = args.target_alt_m * (1.0 - r)
                label = f"descend(t={tgt:.1f}m)"
            else:
                tgt = 0.0
                label = "descend-hold"
            thr = throttle_for_alt(tgt, bias=args.descend_throttle)
            send_rc(thr, 1500, 2000)
            log(label, thr, 1500)
            rel = alt_m - alt_baseline_m
            if rel < args.touchdown_alt_m:
                print(f"[fly] descent: rel_alt {rel:.2f}m < touchdown threshold, "
                      f"early-exit to touchdown", flush=True)
                break
            time.sleep(dt)

        # Phase 8: touchdown cushion
        loop_until(time.monotonic() + 1.5, "touchdown",
                   args.throttle_min, 1500, 2000)

        # Phase 9: disarm
        loop_until(time.monotonic() + 1.0, "disarm",
                   args.throttle_min, 1500, 1000)

    except KeyboardInterrupt:
        print("\n[fly] interrupted; emergency disarm", flush=True)
        for _ in range(20):
            send_rc(args.throttle_min, 1500, 1000)
            time.sleep(dt)
    finally:
        s.close()

    print(f"[fly] done in {time.monotonic() - t_start:.1f}s", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
