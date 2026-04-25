"""FlightController Protocol + TelemetrySnapshot.

The Protocol describes the abstract interface main.py needs from any FC
implementation. BetaflightAdapter (in `fc/betaflight.py`) is the only
concrete implementation today.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from ..msp import (
    AltitudeReading,
    AnalogReading,
    AttitudeReading,
    GpsReading,
)


@dataclass
class TelemetrySnapshot:
    """One tick's worth of FC telemetry. Fields are `None` when the
    corresponding stream hasn't been received yet (cold start) or has dropped.
    Caller (main.py / safety.evaluate) is responsible for the staleness checks.

    Field naming matches the canonical MSP message names so a reader can
    cross-reference to BF source easily.
    """
    gps: GpsReading | None = None
    attitude: AttitudeReading | None = None
    altitude: AltitudeReading | None = None
    analog: AnalogReading | None = None
    rc: list[int] | None = None
    # Timestamp (monotonic seconds) at which `rc` was last refreshed.
    rc_received_at: float | None = None


class FlightController(Protocol):
    """Abstract interface for any flight-controller backend.

    A `tick()` is called once per main-loop iteration: it consumes whatever
    bytes are available on the wire, decodes them, and returns the latest
    snapshot of every telemetry stream the companion cares about.

    Implementations are expected to:
    - Be non-blocking on tick(): if no data is available, return the last
      snapshot unchanged.
    - Periodically request telemetry from the FC at their own cadence.
    - Send overrides only when `send_overrides()` is called by the loop —
      never autonomously.
    """

    def tick(self, now: float) -> TelemetrySnapshot:
        """Consume any pending bytes, return the latest snapshot."""
        ...

    def send_overrides(self, channels: list[int]) -> None:
        """Send 8-channel RC overrides to the FC (e.g., MSP_SET_RAW_RC)."""
        ...

    def is_acro_active(self) -> bool | None:
        """`True` if FC is currently in ACRO mode, `False` if ANGLE/HORIZON,
        `None` if flight-mode info hasn't been received yet."""
        ...

    def close(self) -> None:
        """Release the underlying transport (serial port / TCP socket)."""
        ...
