"""IMU frame probe: send a known fake-quad state via the bridge,
read back BF's MSP_ATTITUDE response, compare.

Pegasus state.attitude is xyzw. The bridge translates it to BF-FDM
quat (NED-FRD wxyz) via _state_attitude_ned_frd + _quat_pegasus_to_bf.
This tool injects specific Pegasus attitudes and reads BF's reported
roll/pitch/yaw — if they don't match the input, the frame conversion
is wrong, and the sign of each axis tells us which.

Test cases:
  - identity (level Iris):                  BF should report roll=0 pitch=0 yaw=0
  - +30 deg roll about body-X (right wing down): BF should report roll=+30 pitch=0
  - +30 deg pitch about body-Y (nose down):      BF should report roll=0 pitch=-30
                                                  (BF convention: pitch+ = nose UP)
  - +30 deg yaw about body-Z (turn right):       BF should report yaw=+30 (heading
                                                  changes clockwise when viewed top-down)

Usage:
  docker compose -f sitl/docker-compose.yml up -d
  # ensure relay is on the right port
  python -m integrations.tools.probe_attitude --motor-port 28500
"""
from __future__ import annotations

import argparse
import socket
import struct
import sys
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation

from integrations.pegasus_betaflight_backend import (
    BetaflightBackendConfig,
    BetaflightUdpBackend,
)
from integrations.tools.fake_pegasus_loop import HoverState, make_hover_state


MSP_HEADER_REQ = b"$M<"
MSP_HEADER_RESP = b"$M>"
MSP_ATTITUDE = 108


def msp_request(cmd: int, payload: bytes = b"") -> bytes:
    pkt = bytes([len(payload), cmd]) + payload
    chk = 0
    for b in pkt:
        chk ^= b
    return MSP_HEADER_REQ + pkt + bytes([chk])


def parse_attitude(payload: bytes) -> tuple[float, float, float]:
    """MSP_ATTITUDE: int16 roll (1/10 deg), int16 pitch (1/10 deg),
    int16 yaw (deg). Returns degrees."""
    roll, pitch, yaw = struct.unpack_from("<3h", payload, 0)
    return roll / 10.0, pitch / 10.0, float(yaw)


def make_state_with_attitude(roll_deg: float, pitch_deg: float, yaw_deg: float) -> HoverState:
    """Build a HoverState rotated to the given roll/pitch/yaw (intrinsic
    Tait-Bryan ZYX, ENU convention: yaw about world Z=Up, then pitch
    about new Y=Left, then roll about new X=Forward).

    Pegasus state.attitude is xyzw, body-to-world ENU/FLU.
    """
    # ZYX intrinsic = "rotate yaw, then pitch, then roll" in body frame.
    r = Rotation.from_euler("ZYX",
                            [yaw_deg, pitch_deg, roll_deg],
                            degrees=True)
    qx, qy, qz, qw = r.as_quat()
    s = make_hover_state(alt_m=5.0)
    s.attitude = np.array([qx, qy, qz, qw])
    return s


def query_attitude(motor_port: int) -> tuple[float, float, float] | None:
    """Open a fresh MSP TCP connection, query MSP_ATTITUDE, return
    (roll_deg, pitch_deg, yaw_deg). None on failure."""
    s = socket.create_connection(("127.0.0.1", 5761), timeout=2)
    s.settimeout(1.5)
    try:
        s.sendall(msp_request(MSP_ATTITUDE))
        time.sleep(0.3)
        buf = b""
        try:
            while True:
                c = s.recv(4096)
                if not c:
                    break
                buf += c
        except socket.timeout:
            pass
        i = buf.find(MSP_HEADER_RESP)
        if i < 0 or len(buf) < i + 5:
            return None
        sz = buf[i + 3]
        cmd = buf[i + 4]
        if cmd != MSP_ATTITUDE or sz < 6:
            return None
        return parse_attitude(buf[i + 5 : i + 5 + sz])
    finally:
        s.close()


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--motor-port", type=int, default=28500)
    p.add_argument("--settle-s", type=float, default=4.0,
                   help="How long to drive each test case before reading attitude.")
    args = p.parse_args()

    cases = [
        ("identity (level)", 0, 0, 0),
        ("+30 deg roll (right wing down)", 30, 0, 0),
        ("-30 deg roll (left wing down)", -30, 0, 0),
        ("+30 deg pitch (nose up)", 0, 30, 0),
        ("-30 deg pitch (nose down)", 0, -30, 0),
        ("+45 deg yaw (CW from above, ENU)", 0, 0, 45),
    ]

    cfg = BetaflightBackendConfig(
        bf_host="127.0.0.1",
        motor_port=args.motor_port,
        recv_timeout_s=0.5,
    )
    be = BetaflightUdpBackend(cfg)
    be.start()

    # Background FDM driver — updates state via shared variable.
    current_state = [make_hover_state(alt_m=5.0)]
    stop = threading.Event()
    def fdm_loop():
        dt = 1.0 / 50.0
        while not stop.is_set():
            be.update_state(current_state[0])
            be.update(dt)
            time.sleep(dt)
    t = threading.Thread(target=fdm_loop, daemon=True)
    t.start()

    print(f"[probe] driving FDM at 50 Hz; settle {args.settle_s}s per case",
          flush=True)
    print(f"{'case':<35} {'sent rpy':<25} {'BF reports rpy':<25}", flush=True)
    print("-" * 85, flush=True)

    try:
        for label, r, pi, y in cases:
            current_state[0] = make_state_with_attitude(r, pi, y)
            time.sleep(args.settle_s)
            att = query_attitude(args.motor_port)
            sent = f"({r:+.1f}, {pi:+.1f}, {y:+.1f})"
            if att is None:
                got = "(no resp)"
            else:
                got = f"({att[0]:+.1f}, {att[1]:+.1f}, {att[2]:+.1f})"
            print(f"{label:<35} {sent:<25} {got:<25}", flush=True)
    finally:
        stop.set()
        t.join(timeout=1.0)
        be.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
