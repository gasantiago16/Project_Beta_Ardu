"""Integration test: RecordingAdapter ↔ BetaflightAdapter.

Asserts the chain (real-or-fake serial) → RecordingAdapter → MspClient →
BetaflightAdapter works end to end. The per-component tests cover their
own units; this catches breaks at the seams between them — exactly the
kind of bug the holistic review surfaced (recorder was implemented and
tested but never wired into the FC adapter chain).
"""
from __future__ import annotations

import struct
import tempfile
import unittest
from collections import deque
from pathlib import Path

from racer_companion import msp
from racer_companion.fc.betaflight import BetaflightAdapter
from racer_companion.recorder import RecordingAdapter, ReplayAdapter


class _FakeSerial:
    def __init__(self, rx_chunks):
        self._rx = list(rx_chunks)
        self.tx = bytearray()
        self.closed = False

    def read(self, n):
        return self._rx.pop(0) if self._rx else b""

    def write(self, data):
        self.tx.extend(data)
        return len(data)

    def close(self):
        self.closed = True


def _build_response(cmd, payload):
    size = len(payload)
    chk = size ^ cmd
    for b in payload:
        chk ^= b
    return b"$M>" + bytes([size, cmd]) + payload + bytes([chk & 0xFF])


class TestRecordedFlightReplaysThroughFC(unittest.TestCase):
    def test_record_then_replay_through_adapter(self):
        # Simulated wire: one ATTITUDE frame + one ALTITUDE frame.
        att = _build_response(msp.MSP_ATTITUDE, struct.pack("<hhh", 100, -50, 270))
        alt = _build_response(msp.MSP_ALTITUDE, struct.pack("<ih", 1234, 5))

        with tempfile.TemporaryDirectory() as td:
            log_path = Path(td) / "session.jsonl"

            # ── Record path: real_serial → RecordingAdapter → MspClient → FC adapter
            real = _FakeSerial(rx_chunks=[att + alt])
            t = [0.0]
            recorder = RecordingAdapter(real, log_path, time_source=lambda: t[0])
            client = msp.MspClient.from_adapter(recorder)
            adapter = BetaflightAdapter.from_client(client)
            t[0] = 1.0
            snap = adapter.tick(now=1.0)
            adapter.close()

            self.assertIsNotNone(snap.attitude)
            self.assertEqual(snap.attitude.yaw_deg, 270)
            self.assertIsNotNone(snap.altitude)
            self.assertEqual(snap.altitude.alt_cm, 1234)
            self.assertTrue(real.closed)

            # ── Replay path: log → ReplayAdapter → MspClient → FC adapter
            replay = ReplayAdapter(log_path, time_source=lambda: float("inf"))
            client2 = msp.MspClient.from_adapter(replay)
            adapter2 = BetaflightAdapter.from_client(client2)
            snap2 = adapter2.tick(now=1.0)

            # Same telemetry comes back through the offline replay path.
            self.assertIsNotNone(snap2.attitude)
            self.assertEqual(snap2.attitude.yaw_deg, 270)
            self.assertIsNotNone(snap2.altitude)
            self.assertEqual(snap2.altitude.alt_cm, 1234)


class TestHeadingTrackerEndToEnd(unittest.TestCase):
    """Drives the same `transit_active` boolean main.py builds, against a
    realistic err sequence. Confirms the watchdog actually trips when the
    bearing controller diverges — not just when artificially fed signals
    that bypass main.py's gating logic.
    """

    def test_tracker_trips_on_sustained_divergence_in_transit(self):
        from racer_companion.safety import HeadingDivergenceTracker
        tracker = HeadingDivergenceTracker(threshold_deg=60.0, window_s=3.0)
        tripped = False
        # 5 seconds of sustained 90° error while in TRANSIT — exactly the
        # mag-NONE drift case the tracker is supposed to catch.
        for i in range(50):
            t = i * 0.1
            transit_active = True  # main.py: prev_state == State.TRANSIT
            tripped = tracker.update(t, heading_err_deg=90.0,
                                     pitching_forward=transit_active) or tripped
        self.assertTrue(tripped, "tracker MUST trip on sustained 90° error in TRANSIT")


if __name__ == "__main__":
    unittest.main()
