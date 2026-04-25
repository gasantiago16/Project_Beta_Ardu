"""BetaflightAdapter — FlightController implementation backed by MSP v1.

Owns the MspClient + FlightModeReader. Drives telemetry requests on a fixed
cadence (configurable; defaults to 10 Hz). Re-requests MSP_BOXNAMES until
primed so that ACRO detection works after FC reboots mid-flight.

Why a class and not a function: the adapter holds telemetry-stream state
(last_gps, last_rc, etc.) and the box-name map. main.py used to manage
these directly; the adapter pulls them into one cohesive type and lets the
caller treat the FC as opaque.

Snapshot contract: `tick()` returns a NEW TelemetrySnapshot each call. The
adapter mutates a private internal snapshot, then copies it before returning.
Callers may safely retain returned snapshots across ticks for diff/replay.
"""
from __future__ import annotations

import dataclasses
import logging
from typing import Optional

from .. import msp
from ..safety import FlightModeReader
from .protocol import TelemetrySnapshot

log = logging.getLogger("racer_companion.fc.betaflight")


# How often to request the rolling telemetry batch (seconds).
DEFAULT_TELEM_PERIOD_S = 0.1
# How often to re-request MSP_BOXNAMES while not yet primed.
BOXNAMES_RETRY_S = 5.0


class BetaflightAdapter:
    """FlightController implementation for Betaflight 4.5+ over MSP v1."""

    def __init__(self, client: msp.MspClient,
                 telemetry_period_s: float = DEFAULT_TELEM_PERIOD_S,
                 mode_reader: Optional[FlightModeReader] = None):
        self._client = client
        self._telem_period_s = telemetry_period_s
        self.mode_reader = mode_reader if mode_reader is not None else FlightModeReader()

        self._snapshot = TelemetrySnapshot()
        self._last_telem_request: float = -1e9
        self._last_boxnames_request: float = -1e9

    # ── Construction helpers ────────────────────────────────────────────

    @classmethod
    def open(cls, port: str, baud: int = 115200, *,
             record_path: Optional[str] = None, **kwargs) -> "BetaflightAdapter":
        """Open the underlying MSP transport and return an adapter wrapping it.

        `record_path` (optional): if set, every byte to/from the FC is teed
        to the given file path via `RecordingAdapter`. Use to capture a
        flight for offline replay/debug. The file is opened in append mode.
        """
        if record_path is None:
            client = msp.MspClient(port, baud)
        else:
            # Build the underlying transport explicitly so we can wrap it
            # in a RecordingAdapter before MspClient sees it.
            if port.startswith("tcp://"):
                from ..msp import _TcpSerialAdapter
                host_port = port[len("tcp://"):]
                host, p = host_port.rsplit(":", 1)
                inner = _TcpSerialAdapter(host, int(p))
            else:
                import serial  # lazy
                inner = serial.Serial(port, baud, timeout=0)
            from ..recorder import RecordingAdapter
            wrapped = RecordingAdapter(inner, record_path)
            client = msp.MspClient.from_adapter(wrapped)
        return cls(client, **kwargs)

    @classmethod
    def from_client(cls, client: msp.MspClient, **kwargs) -> "BetaflightAdapter":
        """Wrap a pre-built MspClient (useful for tests / replay)."""
        return cls(client, **kwargs)

    # ── FlightController Protocol ───────────────────────────────────────

    def tick(self, now: float) -> TelemetrySnapshot:
        # 1. Consume any bytes that have arrived since the last tick.
        for cmd, payload in self._client.poll():
            self._handle_frame(cmd, payload, now)

        # 2. Periodically request the rolling telemetry batch.
        if now - self._last_telem_request > self._telem_period_s:
            self._request_telemetry()
            self._last_telem_request = now

        # 3. Re-request box names until the mode reader is primed.
        if (self.mode_reader.box_names is None
                and now - self._last_boxnames_request > BOXNAMES_RETRY_S):
            self._client.send(msp.MSP_BOXNAMES)
            self._last_boxnames_request = now

        # Return a copy so callers can safely retain it across ticks.
        return dataclasses.replace(self._snapshot)

    def send_overrides(self, channels: list[int]) -> None:
        self._client.send_raw_rc(channels)

    def is_acro_active(self) -> bool | None:
        return self.mode_reader.is_acro_active()

    def close(self) -> None:
        self._client.close()

    # ── Internals ───────────────────────────────────────────────────────

    def _request_telemetry(self) -> None:
        c = self._client
        c.send(msp.MSP_RAW_GPS)
        c.send(msp.MSP_ATTITUDE)
        c.send(msp.MSP_ALTITUDE)
        c.send(msp.MSP_ANALOG)
        c.send(msp.MSP_RC)
        c.send(msp.MSP_STATUS_EX)

    def _handle_frame(self, cmd: int, payload: bytes, now: float) -> None:
        # Every decoder branch is wrapped: a single malformed frame from a
        # mid-flight glitch (radio interference, BF transient) must NOT take
        # the loop down. Pre-existing decoders raise ValueError on short
        # payloads; we swallow and log so the next valid frame recovers.
        try:
            if cmd == msp.MSP_RAW_GPS:
                self._snapshot.gps = msp.decode_raw_gps(payload)
            elif cmd == msp.MSP_ATTITUDE:
                self._snapshot.attitude = msp.decode_attitude(payload)
            elif cmd == msp.MSP_ALTITUDE:
                self._snapshot.altitude = msp.decode_altitude(payload)
            elif cmd == msp.MSP_ANALOG:
                self._snapshot.analog = msp.decode_analog(payload)
            elif cmd == msp.MSP_RC:
                self._snapshot.rc = msp.decode_rc(payload)
                self._snapshot.rc_received_at = now
            elif cmd == msp.MSP_STATUS_EX:
                self.mode_reader.update_status(msp.decode_status_ex(payload))
            elif cmd == msp.MSP_BOXNAMES:
                names = msp.decode_box_names(payload)
                if names:
                    was_primed = self.mode_reader.box_names is not None
                    self.mode_reader.update_box_names(names)
                    if not was_primed:
                        log.info("BF box names received (%d entries; angle_idx=%s, horizon_idx=%s)",
                                 len(names), self.mode_reader.angle_idx,
                                 self.mode_reader.horizon_idx)
        except ValueError as e:
            log.debug("Malformed MSP frame cmd=%d (%dB): %s", cmd, len(payload), e)
