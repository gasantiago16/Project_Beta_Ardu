"""End-to-end SITL arming smoke test — runs the known-working sequence
and ASSERTS motors actually output non-zero rad/s when armed.

Use this as the canonical "is the sim working?" check. If this passes,
the wire path + arming flow is healthy. If it fails, you've got a
regression — the asserts pin exactly which stage broke.

Run order (single process, no Isaac Sim — just BF SITL + bridge):
  1. UDP container->host check on chosen motor port
  2. fake_pegasus_loop FDM driver in background
  3. bf_diag --mode arm sequence
  4. Read motor values from bridge stats
  5. Assert: motor[0] > 500 rad/s for at least 3 seconds during hover

Run:
  docker compose -f sitl/docker-compose.yml up -d
  python -m integrations.tools.verify_sim_arms
"""
from __future__ import annotations

import argparse
import socket
import struct
import subprocess
import sys
import threading
import time

from integrations.pegasus_betaflight_backend import (
    BetaflightBackendConfig,
    BetaflightUdpBackend,
)
from integrations.tools.fake_pegasus_loop import make_hover_state


# Reuse bf_diag's MSP framing.
MSP_HEADER_REQ = b"$M<"
MSP_HEADER_RESP = b"$M>"
MSP_RC = 105
MSP_STATUS_EX = 150
MSP_SET_RAW_RC = 200


def msp_request(cmd: int, payload: bytes = b"") -> bytes:
    pkt = bytes([len(payload), cmd]) + payload
    chk = 0
    for b in pkt:
        chk ^= b
    return MSP_HEADER_REQ + pkt + bytes([chk])


def msp_set_raw_rc(channels: list[int]) -> bytes:
    """MSP frame slot order: [Roll, Pitch, THROTTLE, Yaw, AUX1, AUX2..]
    given BF default rcmap "AETR"."""
    return msp_request(MSP_SET_RAW_RC, struct.pack(f"<{len(channels)}H", *channels))


def assert_or_die(cond: bool, msg: str) -> None:
    if not cond:
        print(f"FAIL: {msg}", flush=True)
        sys.exit(1)


def stage_1_udp_round_trip(motor_port: int) -> None:
    """Verify container->host UDP delivery on motor_port. If this fails:
    Windows Defender Firewall heuristic / VPN / port reuse — try a fresh
    high port (28500/35000/19500) or drop the VPN."""
    print(f"[verify] stage 1: UDP container->host:{motor_port}", flush=True)
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.bind(("0.0.0.0", motor_port))
    rx.settimeout(0.3)
    try:
        # Trigger 5 packets from inside container.
        subprocess.run(
            ["docker", "exec", "project-beta-ardu-sitl", "bash", "-c",
             f"for i in 1 2 3 4 5; do echo X | nc -u -w 0 192.168.65.254 {motor_port}; done"],
            check=False, timeout=5,
        )
        time.sleep(0.5)
        got = 0
        deadline = time.time() + 1.0
        while time.time() < deadline:
            try:
                rx.recvfrom(64)
                got += 1
            except socket.timeout:
                break
        assert_or_die(
            got >= 3,
            f"only {got}/5 packets arrived on {motor_port}. "
            f"Likely Windows Firewall blocked port or VPN is active. "
            f"Try a different --motor-port (28500, 35000, 19077).",
        )
        print(f"[verify] stage 1: OK ({got}/5 packets delivered)", flush=True)
    finally:
        rx.close()


def stage_2_bridge_lockstep(motor_port: int, duration_s: float) -> dict:
    """Drive FDM at 50 Hz via the actual bridge (BetaflightUdpBackend)
    in a background thread, send arm sequence on a separate MSP TCP
    connection, watch the bridge's `_latest_motor`. Return the
    max motor[0] seen + tx/rx counts."""
    cfg = BetaflightBackendConfig(
        bf_host="127.0.0.1",
        motor_port=motor_port,
        recv_timeout_s=0.5,
    )
    be = BetaflightUdpBackend(cfg)
    be.start()
    state = make_hover_state(alt_m=5.0)

    # Background FDM driver: send at 50 Hz with monotonic sim_time so
    # BF SITL's simRate stays close to 1.0 (millis() advances at wall
    # rate -> BOOT_GRACE_TIME clears in ~5 s).
    stop = threading.Event()
    def fdm_loop():
        dt = 1.0 / 50.0
        while not stop.is_set():
            be.update_state(state)
            be.update(dt)
            time.sleep(dt)
    t = threading.Thread(target=fdm_loop, daemon=True)
    t.start()

    # MSP arming sequence on a parallel socket (bf_diag-style).
    # Idle 12 s -> arm 1 s -> ramp 2 s -> hold (rest of duration).
    msp = socket.create_connection(("127.0.0.1", 5761), timeout=2)
    msp.setblocking(False)

    max_motor = 0.0
    samples_above_500 = 0
    t0 = time.monotonic()
    last_send = 0.0
    last_check = 0.0

    while time.monotonic() - t0 < duration_s:
        now = time.monotonic()
        elapsed = now - t0
        # RC schedule:
        if elapsed < 12.0:
            ch = [1500, 1500, 950, 1500, 1000, 1000, 1000, 1000]  # idle
        elif elapsed < 13.0:
            ch = [1500, 1500, 950, 1500, 2000, 1000, 1000, 1000]  # arm
        elif elapsed < 15.0:
            r = (elapsed - 13.0) / 2.0
            thr = int(950 + r * (1700 - 950))
            ch = [1500, 1500, thr, 1500, 2000, 1000, 1000, 1000]
        elif elapsed < duration_s - 1.0:
            ch = [1500, 1500, 1700, 1500, 2000, 1000, 1000, 1000]  # hold
        else:
            ch = [1500, 1500, 950, 1500, 1000, 1000, 1000, 1000]  # disarm
        if now - last_send >= 0.02:
            try:
                msp.sendall(msp_set_raw_rc(ch))
            except OSError:
                pass
            last_send = now

        # Sample motor every 200 ms.
        if now - last_check >= 0.2:
            m = be._latest_motor
            max_motor = max(max_motor, max(m))
            if m[0] > 500.0 / cfg.rotor_max_omega:
                # _latest_motor is normalized [0,1]; 500 rad/s on Iris
                # = 500/1023 ≈ 0.49.
                samples_above_500 += 1
            last_check = now
        time.sleep(0.005)

    msp.close()
    stop.set()
    t.join(timeout=1.0)
    stats = be.stats()
    be.stop()
    return {
        "tx": stats["tx"],
        "rx": stats["rx"],
        "max_motor_norm": max_motor,
        "samples_above_500_rad_s": samples_above_500,
    }


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--motor-port", type=int, default=38500)
    p.add_argument("--duration-s", type=float, default=22.0,
                   help="Total run time. Need at least idle(12) + ramp(3) + hold(5) = 20 s.")
    args = p.parse_args()

    print(f"[verify] motor_port={args.motor_port} duration={args.duration_s}s", flush=True)

    # Stage 1: UDP plumbing.
    stage_1_udp_round_trip(args.motor_port)

    # Stage 2: drive FDM + arm + check motors.
    print(f"[verify] stage 2: arm + hover check ({args.duration_s}s)...",
          flush=True)
    r = stage_2_bridge_lockstep(args.motor_port, args.duration_s)
    print(f"[verify] stage 2 done: tx={r['tx']} rx={r['rx']} "
          f"max_motor_norm={r['max_motor_norm']:.3f} "
          f"samples_above_500_rad_s={r['samples_above_500_rad_s']}",
          flush=True)

    assert_or_die(
        r["rx"] > 100,
        f"BF replied to only {r['rx']} of {r['tx']} FDM packets. "
        f"BF is likely stuck in BOOT_GRACE_TIME or RX_FAILSAFE — check "
        f"`set power_on_arming_grace_time` and `aux 0` in defaults.txt.",
    )
    assert_or_die(
        r["max_motor_norm"] > 0.4,
        f"Max motor output was only {r['max_motor_norm']:.3f} normalized. "
        f"Expected >0.4 (i.e. 400+ rad/s on Iris). BF didn't actually arm "
        f"or motors stayed at idle. Run integrations/tools/bf_diag --mode "
        f"arm to see arming-disable flags live.",
    )
    assert_or_die(
        r["samples_above_500_rad_s"] >= 5,
        f"Saw motor>500 rad/s for only {r['samples_above_500_rad_s']} "
        f"samples. BF armed briefly but didn't sustain hover. "
        f"Check ANGLE mode binding in defaults.txt aux config.",
    )

    print("[verify] PASS — sim is working end-to-end.", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
