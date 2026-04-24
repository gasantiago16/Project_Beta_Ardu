"""MSP v1 wire protocol for Betaflight.

Header `$M<` requests-to-FC, `$M>` responses-from-FC, `$M!` errors-from-FC.
After header: size (1B), command (1B), payload (size B), checksum (1B).
Checksum = XOR of size + command + every payload byte.

Only commands actually used by this companion are implemented. Do not extend
without re-reading `betaflight/src/main/msp/msp.c` and `msp_protocol.h` —
silent encoding mistakes have flown drones into trees.
"""
from __future__ import annotations

import struct
import time
from dataclasses import dataclass
from typing import Optional

MSP_API_VERSION = 1
MSP_FC_VARIANT = 2
MSP_RC = 105
MSP_RAW_GPS = 106
MSP_ATTITUDE = 108
MSP_ALTITUDE = 109
MSP_ANALOG = 110
MSP_SET_RAW_RC = 200

CH_ROLL = 0
CH_PITCH = 1
CH_YAW = 2
CH_THROTTLE = 3
CH_AUX1 = 4
CH_AUX2 = 5
CH_AUX3 = 6
CH_AUX4 = 7

CHANNEL_MIN = 1000
CHANNEL_MID = 1500
CHANNEL_MAX = 2000


@dataclass
class GpsReading:
    fix: bool
    num_sat: int
    lat_deg: float
    lon_deg: float
    alt_m: int
    speed_cms: int
    course_deg: float
    received_at: float


@dataclass
class AttitudeReading:
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    received_at: float


@dataclass
class AltitudeReading:
    alt_cm: int
    vario_cms: int
    received_at: float


@dataclass
class AnalogReading:
    vbat_v: float
    mah: int
    rssi: int
    amperage_a: float
    received_at: float


def encode_request(cmd: int, payload: bytes = b"") -> bytes:
    if len(payload) > 255:
        raise ValueError("MSP v1 payload limited to 255 bytes")
    size = len(payload)
    chk = size ^ cmd
    for b in payload:
        chk ^= b
    return b"$M<" + bytes([size, cmd]) + payload + bytes([chk & 0xFF])


def encode_set_raw_rc(channels: list[int]) -> bytes:
    if len(channels) != 8:
        raise ValueError("MSP_SET_RAW_RC requires exactly 8 channels")
    for i, c in enumerate(channels):
        if not (CHANNEL_MIN - 50 <= c <= CHANNEL_MAX + 50):
            raise ValueError(f"Channel {i} out of range: {c}")
    payload = b"".join(struct.pack("<H", c) for c in channels)
    return encode_request(MSP_SET_RAW_RC, payload)


def decode_raw_gps(payload: bytes) -> GpsReading:
    if len(payload) < 16:
        raise ValueError("MSP_RAW_GPS payload too short")
    fix, num_sat, lat, lon, alt_m, speed_cms, course_x10 = struct.unpack(
        "<BBiiHHH", payload[:16]
    )
    return GpsReading(
        fix=bool(fix),
        num_sat=num_sat,
        lat_deg=lat / 1e7,
        lon_deg=lon / 1e7,
        alt_m=alt_m,
        speed_cms=speed_cms,
        course_deg=course_x10 / 10.0,
        received_at=time.monotonic(),
    )


def decode_attitude(payload: bytes) -> AttitudeReading:
    if len(payload) < 6:
        raise ValueError("MSP_ATTITUDE payload too short")
    angx_x10, angy_x10, heading = struct.unpack("<hhh", payload[:6])
    return AttitudeReading(
        roll_deg=angx_x10 / 10.0,
        pitch_deg=angy_x10 / 10.0,
        yaw_deg=float(heading),
        received_at=time.monotonic(),
    )


def decode_altitude(payload: bytes) -> AltitudeReading:
    if len(payload) < 6:
        raise ValueError("MSP_ALTITUDE payload too short")
    alt_cm, vario_cms = struct.unpack("<ih", payload[:6])
    return AltitudeReading(alt_cm=alt_cm, vario_cms=vario_cms, received_at=time.monotonic())


def decode_analog(payload: bytes) -> AnalogReading:
    if len(payload) < 7:
        raise ValueError("MSP_ANALOG payload too short")
    vbat_b, mah, rssi, amp_x100 = struct.unpack("<BHHh", payload[:7])
    return AnalogReading(
        vbat_v=vbat_b * 0.1,
        mah=mah,
        rssi=rssi,
        amperage_a=amp_x100 * 0.01,
        received_at=time.monotonic(),
    )


def decode_rc(payload: bytes) -> list[int]:
    if len(payload) % 2:
        raise ValueError("MSP_RC payload must be even length")
    n = len(payload) // 2
    return list(struct.unpack(f"<{n}H", payload))


class MspClient:
    def __init__(self, port: str, baud: int = 115200, timeout: float = 0.0):
        import serial  # lazy: lets the module import without pyserial for offline tests
        self.ser = serial.Serial(port, baud, timeout=timeout)
        self.buf = bytearray()

    def close(self) -> None:
        if self.ser is not None:
            self.ser.close()

    def send(self, cmd: int, payload: bytes = b"") -> None:
        self.ser.write(encode_request(cmd, payload))

    def send_raw_rc(self, channels: list[int]) -> None:
        self.ser.write(encode_set_raw_rc(channels))

    def poll(self) -> list[tuple[int, bytes]]:
        chunk = self.ser.read(4096)
        if chunk:
            self.buf.extend(chunk)
        # Cap buffer to prevent runaway on garbage stream.
        if len(self.buf) > 16384:
            self.buf = bytearray(self.buf[-2048:])
        return self._drain_frames()

    def _drain_frames(self) -> list[tuple[int, bytes]]:
        frames: list[tuple[int, bytes]] = []
        while True:
            f = self._try_parse_one()
            if f is None:
                return frames
            frames.append(f)

    def _try_parse_one(self) -> Optional[tuple[int, bytes]]:
        i = self._find_header()
        if i < 0:
            # Keep last 2 bytes in case they're a partial header.
            if len(self.buf) > 2:
                del self.buf[: len(self.buf) - 2]
            return None
        if i > 0:
            del self.buf[:i]
        if len(self.buf) < 6:
            return None
        size = self.buf[3]
        cmd = self.buf[4]
        total = 3 + 2 + size + 1
        if len(self.buf) < total:
            return None
        payload = bytes(self.buf[5 : 5 + size])
        chk = self.buf[5 + size]
        calc = size ^ cmd
        for b in payload:
            calc ^= b
        del self.buf[:total]
        if (calc & 0xFF) != chk:
            return self._try_parse_one()
        return cmd, payload

    def _find_header(self) -> int:
        b = bytes(self.buf)
        n = len(b)
        for i in range(n - 2):
            if b[i] == 0x24 and b[i + 1] == 0x4D and b[i + 2] in (0x3E, 0x21, 0x3C):
                return i
        return -1
