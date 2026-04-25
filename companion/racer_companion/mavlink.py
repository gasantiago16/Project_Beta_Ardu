"""Hand-rolled MAVLink v2 encoder for the 3 message types we actually emit.

This is intentionally minimal — no XML codegen, no dialect fork, no pymavlink
dependency. If you need to emit additional message types, add them here and
get the field order + CRC_EXTRA right. The MAVLink spec docs are at
https://mavlink.io/en/guide/serialization.html.

Wire format (v2, unsigned):
  STX (0xFD) | LEN | INCOMPAT_FLAGS | COMPAT_FLAGS | SEQ | SYSID | COMPID
  | MSGID(3 LE) | PAYLOAD | CRC(2 LE)

CRC is X.25 (CRC-16/MCRF4XX, poly 0x1021 reflected, init 0xFFFF) over
everything from LEN through the trailing CRC_EXTRA byte (per-msgid).

Field order in payload is sorted by C-type size descending — this is how the
spec lays out v2 wire format and is non-obvious. Get it wrong and receivers
report bad CRC.

Backends:
  UDPPublisher — for QGroundControl on a phone or laptop on the same WiFi
  SerialPublisher — for an eventual radio-side integration (UART to ELRS
                    backchannel or similar)
"""
from __future__ import annotations

import socket
import struct
import time
from dataclasses import dataclass
from typing import Optional, Protocol


MAVLINK_STX_V2 = 0xFD

# System / component IDs we emit as.
SYSID_COMPANION = 1
COMPID_ONBOARD_COMPUTER = 191  # MAV_COMP_ID_ONBOARD_COMPUTER

# Message IDs.
MSGID_HEARTBEAT = 0
MSGID_STATUSTEXT = 253
MSGID_COMPANION_STATE = 12500  # custom; user-defined range to avoid collision

# CRC_EXTRA values. For HEARTBEAT/STATUSTEXT these come from the MAVLink
# common.xml hashing algorithm; for COMPANION_STATE we pick a stable value.
# DO NOT change these without re-running pymavlink against a sample frame.
CRC_EXTRA = {
    MSGID_HEARTBEAT: 50,
    MSGID_STATUSTEXT: 83,
    MSGID_COMPANION_STATE: 42,
}

# MAV_TYPE / MAV_AUTOPILOT / MAV_STATE — minimum enums we use.
MAV_TYPE_ONBOARD_CONTROLLER = 18
MAV_AUTOPILOT_INVALID = 8           # we are not an autopilot
MAV_STATE_ACTIVE = 4

# STATUSTEXT severities (RFC 5424-style).
SEV_EMERGENCY = 0
SEV_ALERT = 1
SEV_CRITICAL = 2
SEV_ERROR = 3
SEV_WARNING = 4
SEV_NOTICE = 5
SEV_INFO = 6
SEV_DEBUG = 7


def crc16_x25(data: bytes, crc_extra: int) -> int:
    """X.25 CRC over the given bytes plus the message's CRC_EXTRA byte."""
    crc = 0xFFFF
    for byte in data + bytes([crc_extra]):
        tmp = (byte ^ (crc & 0xFF)) & 0xFF
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def encode_frame(msgid: int, payload: bytes, seq: int,
                 sysid: int = SYSID_COMPANION,
                 compid: int = COMPID_ONBOARD_COMPUTER) -> bytes:
    if msgid not in CRC_EXTRA:
        raise ValueError(f"no CRC_EXTRA registered for msgid={msgid}")
    if len(payload) > 253:
        raise ValueError(f"payload too long: {len(payload)}")
    incompat = 0
    compat = 0
    header = bytes([
        MAVLINK_STX_V2,
        len(payload),
        incompat,
        compat,
        seq & 0xFF,
        sysid & 0xFF,
        compid & 0xFF,
        msgid & 0xFF,
        (msgid >> 8) & 0xFF,
        (msgid >> 16) & 0xFF,
    ])
    crc_input = header[1:] + payload  # CRC excludes the STX byte
    crc = crc16_x25(crc_input, CRC_EXTRA[msgid])
    return header + payload + struct.pack("<H", crc)


def encode_heartbeat(seq: int) -> bytes:
    # Field order: uint32 custom_mode, uint8 type, uint8 autopilot,
    #              uint8 base_mode, uint8 system_status, uint8 mavlink_version
    payload = struct.pack(
        "<IBBBBB",
        0,                              # custom_mode
        MAV_TYPE_ONBOARD_CONTROLLER,    # type
        MAV_AUTOPILOT_INVALID,          # autopilot
        0,                              # base_mode
        MAV_STATE_ACTIVE,               # system_status
        3,                              # mavlink_version
    )
    return encode_frame(MSGID_HEARTBEAT, payload, seq)


def encode_statustext(seq: int, severity: int, text: str) -> bytes:
    text_bytes = text.encode("ascii", errors="replace")[:50]
    text_bytes = text_bytes.ljust(50, b"\x00")
    # Field order: uint8 severity, char[50] text
    payload = bytes([severity & 0xFF]) + text_bytes
    return encode_frame(MSGID_STATUSTEXT, payload, seq)


def encode_companion_state(seq: int, *, distance_m: float, alt_m: float,
                           heading_deg: float, state: int,
                           safety_reasons_bitmap: int) -> bytes:
    # Field order (size descending): float×3, uint8×2.
    payload = struct.pack(
        "<fffBB",
        float(distance_m),
        float(alt_m),
        float(heading_deg),
        state & 0xFF,
        safety_reasons_bitmap & 0xFF,
    )
    return encode_frame(MSGID_COMPANION_STATE, payload, seq)


# Safety-reason bitmap — keep stable so the radio side can decode.
SAFETY_BIT = {
    "no_gps_telemetry": 1 << 0,
    "no_gps_fix":       1 << 1,
    "low_sats":         1 << 2,
    "stale_gps":        1 << 3,
    "stale_altitude":   1 << 4,
    "stale_attitude":   1 << 5,
    "low_vbat":         1 << 6,
    "geofence":         1 << 7,
}


def safety_bitmap_from_reasons(reasons: list[str]) -> int:
    """Convert SafetyStatus.reasons to an 8-bit bitmap. Reasons may have
    a `_<value>` suffix (e.g., `low_sats_4`) — we strip after the prefix."""
    bm = 0
    for reason in reasons:
        for prefix, bit in SAFETY_BIT.items():
            if reason == prefix or reason.startswith(prefix + "_"):
                bm |= bit
                break
    return bm


# ── Backends ─────────────────────────────────────────────────────────────


class Publisher(Protocol):
    def send(self, frame: bytes) -> None: ...
    def close(self) -> None: ...


class UdpPublisher:
    """Sends MAVLink frames to a UDP host:port. QGroundControl listens on
    14550 by default."""

    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)

    def send(self, frame: bytes) -> None:
        try:
            self.sock.sendto(frame, (self.host, self.port))
        except OSError:
            pass  # don't let MAVLink failures kill the main loop

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass


class SerialPublisher:
    """Sends MAVLink frames over a UART. Use for an eventual radio-side
    integration (companion → ELRS RX backchannel UART, or similar)."""

    def __init__(self, port: str, baud: int = 57600):
        import serial  # lazy
        self.ser = serial.Serial(port, baud, timeout=0)

    def send(self, frame: bytes) -> None:
        try:
            self.ser.write(frame)
        except Exception:
            pass

    def close(self) -> None:
        try:
            self.ser.close()
        except Exception:
            pass


class NullPublisher:
    """No-op publisher — used when MAVLink is disabled in config."""

    def send(self, frame: bytes) -> None:
        pass

    def close(self) -> None:
        pass


def make_publisher(spec: Optional[str]) -> Publisher:
    """Construct a publisher from a config string.

    Spec formats:
      None or ""      → NullPublisher (disabled)
      "udp://h:p"     → UdpPublisher
      "serial:///dev/ttyAMA1[@115200]" → SerialPublisher
    """
    if not spec:
        return NullPublisher()
    if spec.startswith("udp://"):
        host, port = spec[len("udp://"):].rsplit(":", 1)
        return UdpPublisher(host, int(port))
    if spec.startswith("serial://"):
        rest = spec[len("serial://"):]
        if "@" in rest:
            port_str, baud_str = rest.rsplit("@", 1)
            return SerialPublisher(port_str, int(baud_str))
        return SerialPublisher(rest)
    raise ValueError(f"unknown MAVLink publisher spec: {spec!r}")


# ── Sender state ─────────────────────────────────────────────────────────


@dataclass
class SenderConfig:
    heartbeat_hz: float = 1.0
    state_hz: float = 5.0


class MavlinkSender:
    """Owns sequence counters and rate-limiting for the 3 message types."""

    def __init__(self, publisher: Publisher, cfg: Optional[SenderConfig] = None):
        self.publisher = publisher
        self.cfg = cfg or SenderConfig()
        self._seq = 0
        # Init far in the past so the first tick always fires both messages.
        self._last_heartbeat = -1e9
        self._last_state = -1e9
        self._last_status_text: Optional[str] = None

    def _next_seq(self) -> int:
        s = self._seq
        self._seq = (self._seq + 1) & 0xFF
        return s

    def tick(self, *, now: float, state_name: str,
             distance_m: float, alt_m: float, heading_deg: float,
             state_code: int, safety_bitmap: int,
             status_text: Optional[str] = None) -> None:
        if now - self._last_heartbeat >= 1.0 / max(self.cfg.heartbeat_hz, 0.001):
            self.publisher.send(encode_heartbeat(self._next_seq()))
            self._last_heartbeat = now

        if now - self._last_state >= 1.0 / max(self.cfg.state_hz, 0.001):
            self.publisher.send(encode_companion_state(
                self._next_seq(),
                distance_m=distance_m, alt_m=alt_m, heading_deg=heading_deg,
                state=state_code, safety_reasons_bitmap=safety_bitmap,
            ))
            self._last_state = now

        # STATUSTEXT only on change — avoids flooding the link.
        if status_text and status_text != self._last_status_text:
            self.publisher.send(encode_statustext(
                self._next_seq(), SEV_INFO, status_text,
            ))
            self._last_status_text = status_text

    def close(self) -> None:
        self.publisher.close()
