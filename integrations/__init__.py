"""Bridges between the racer_companion stack and external simulators.

This subtree intentionally lives OUTSIDE `companion/` because it imports
simulator-specific runtime types (Pegasus / Isaac Sim) that the companion
itself must never depend on. The companion talks MSP to a Betaflight FC; it
does not know whether that FC is a real H7 board or a Docker SITL behind a
UDP physics bridge.

Current contents:
- `pegasus_betaflight_backend.py` — Pegasus Backend impl that exchanges
  fdm_packet / servo_packet UDP frames with our existing BF SITL Docker.
- `tools/fake_pegasus_loop.py` — exercises the bridge without Isaac Sim,
  using a synthesized hovering-quad State.
- `tests/` — unit + integration tests for the bridge.

Loaded into Pegasus orchestrator scripts via PYTHONPATH (per the v0.5 plan):
    sys.path.insert(0, str(Path(...) / "Project_Beta_Ardu" / "integrations"))
    from pegasus_betaflight_backend import BetaflightUdpBackend
"""
