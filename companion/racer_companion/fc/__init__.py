"""Flight controller adapter package.

The companion runs against Betaflight today (locked-in decision #2 in
MEMORY.md). The abstraction here exists so a hypothetical future port to
INAV (same MSP variant, different boxnames) or ArduPilot (MAVLink) can
swap in a new implementation without touching nav.py / state.py / safety.py.

If you add a new FC implementation, follow the BetaflightAdapter shape:
- own the wire-protocol client
- ingest telemetry into a typed snapshot on `tick()`
- expose flight-mode introspection via `is_acro_active()`
- accept override commands via `send_overrides()`
"""
from .protocol import FlightController, TelemetrySnapshot

__all__ = ["FlightController", "TelemetrySnapshot"]
