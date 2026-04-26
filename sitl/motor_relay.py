#!/usr/bin/env python3
"""In-container UDP motor relay: 127.0.0.1:9002 (BF SITL output) -> host:PORT.

Replaces socat -u UDP4-RECVFROM:9002,fork. The fork-per-packet model adds
~50-200 ms of latency because each child does its own socket setup +
sendto + exit, which serializes on the parent's accept loop. A single-
process Python loop with one bound socket and one outbound socket
handles 250 Hz lockstep with sub-millisecond per-packet latency.

Why we need a relay at all: Windows Defender Firewall on the dev host
empirically blocks inbound UDP on 9002/9012 without admin (other ports
like 38500 stay open). BF SITL's PORT_PWM is hardcoded to 9002, so we
can't redirect at the BF layer. The relay accepts on container loopback
and forwards to host on a port the firewall doesn't block.

Usage:  python3 motor_relay.py <host_ipv4> <host_port>
        e.g. python3 motor_relay.py 192.168.65.254 38500
"""
import socket
import sys


def main() -> int:
    if len(sys.argv) != 3:
        print(f"usage: {sys.argv[0]} <host_ipv4> <host_port>", file=sys.stderr)
        return 2
    host_ip = sys.argv[1]
    host_port = int(sys.argv[2])

    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.bind(("127.0.0.1", 9002))
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(
        f"[relay] 127.0.0.1:9002 -> {host_ip}:{host_port} ready",
        flush=True,
    )

    forwarded = 0
    while True:
        # Each BF servo_packet is exactly 16 bytes (4 floats). Sized at 64
        # to leave headroom; recvfrom blocks until a packet arrives.
        buf, _ = rx.recvfrom(64)
        tx.sendto(buf, (host_ip, host_port))
        forwarded += 1
        # Periodic heartbeat so the container logs prove the relay is
        # alive without spamming a line per packet.
        if forwarded % 1000 == 0:
            print(f"[relay] forwarded {forwarded} packets", flush=True)


if __name__ == "__main__":
    sys.exit(main())
