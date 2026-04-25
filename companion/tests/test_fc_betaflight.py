"""Tests for BetaflightAdapter — the only FlightController implementation today.

Drives the adapter against a fake MspClient (no real serial port). Validates:
- snapshot fields populate from the corresponding MSP frames
- box-name re-request retry until BOXNAMES is received
- ACRO check propagates from FlightModeReader
- send_overrides delegates to MspClient
"""
from __future__ import annotations

import struct
import unittest
from collections import deque

from racer_companion import msp
from racer_companion.fc import TelemetrySnapshot
from racer_companion.fc.betaflight import BetaflightAdapter, BOXNAMES_RETRY_S


class _FakeClient:
    """In-memory MspClient stand-in. Polls return queued frames; sends are
    captured for assertion."""

    def __init__(self):
        self.poll_queue: deque[tuple[int, bytes]] = deque()
        self.sent_cmds: list[int] = []
        self.sent_payloads: list[bytes] = []
        self.raw_rcs: list[list[int]] = []
        self.closed = False

    def poll(self):
        out = list(self.poll_queue)
        self.poll_queue.clear()
        return out

    def send(self, cmd: int, payload: bytes = b""):
        self.sent_cmds.append(cmd)
        self.sent_payloads.append(payload)

    def send_raw_rc(self, channels: list[int]):
        self.raw_rcs.append(list(channels))

    def close(self):
        self.closed = True


def _gps_payload(lat_e7: int = 370_000_000, lon_e7: int = -1_220_000_000,
                 fix: int = 1, sats: int = 12) -> bytes:
    return struct.pack("<BBiiHHH", fix, sats, lat_e7, lon_e7, 100, 0, 0)


def _attitude_payload(yaw_deg: int = 90) -> bytes:
    return struct.pack("<hhh", 0, 0, yaw_deg)


def _altitude_payload(alt_cm: int = 500, vario: int = 0) -> bytes:
    return struct.pack("<ih", alt_cm, vario)


def _analog_payload(vbat_dV: int = 155) -> bytes:
    return struct.pack("<BHHh", vbat_dV, 0, 80, 0)


def _rc_payload(channels: list[int]) -> bytes:
    return struct.pack(f"<{len(channels)}H", *channels)


def _status_ex_payload(flight_mode_flags: int) -> bytes:
    return struct.pack("<HHHIB", 100, 0, 0, flight_mode_flags, 0)


def _box_names_payload(names: list[str]) -> bytes:
    return (";".join(names) + ";").encode("ascii")


class TestSnapshot(unittest.TestCase):
    def setUp(self):
        self.client = _FakeClient()
        self.adapter = BetaflightAdapter.from_client(self.client, telemetry_period_s=0.1)

    def test_first_tick_requests_telemetry(self):
        self.adapter.tick(now=0.0)
        # Six standard requests: GPS, ATT, ALT, ANALOG, RC, STATUS_EX
        self.assertIn(msp.MSP_RAW_GPS, self.client.sent_cmds)
        self.assertIn(msp.MSP_ATTITUDE, self.client.sent_cmds)
        self.assertIn(msp.MSP_ALTITUDE, self.client.sent_cmds)
        self.assertIn(msp.MSP_ANALOG, self.client.sent_cmds)
        self.assertIn(msp.MSP_RC, self.client.sent_cmds)
        self.assertIn(msp.MSP_STATUS_EX, self.client.sent_cmds)

    def test_telemetry_period_throttles_requests(self):
        self.adapter.tick(now=0.0)
        n_initial = len(self.client.sent_cmds)
        # Tick again immediately — period not yet elapsed.
        self.adapter.tick(now=0.05)
        self.assertEqual(len(self.client.sent_cmds), n_initial)
        # After the period elapses, new batch goes out.
        self.adapter.tick(now=0.2)
        self.assertGreater(len(self.client.sent_cmds), n_initial)

    def test_snapshot_populates_from_frames(self):
        self.client.poll_queue.append((msp.MSP_RAW_GPS, _gps_payload(sats=14)))
        self.client.poll_queue.append((msp.MSP_ATTITUDE, _attitude_payload(yaw_deg=270)))
        self.client.poll_queue.append((msp.MSP_ALTITUDE, _altitude_payload(alt_cm=1000)))
        self.client.poll_queue.append((msp.MSP_ANALOG, _analog_payload(vbat_dV=160)))
        self.client.poll_queue.append((msp.MSP_RC, _rc_payload([1500] * 8)))
        snap = self.adapter.tick(now=10.0)
        self.assertIsInstance(snap, TelemetrySnapshot)
        self.assertIsNotNone(snap.gps)
        self.assertEqual(snap.gps.num_sat, 14)
        self.assertIsNotNone(snap.attitude)
        self.assertEqual(snap.attitude.yaw_deg, 270)
        self.assertIsNotNone(snap.altitude)
        self.assertEqual(snap.altitude.alt_cm, 1000)
        self.assertIsNotNone(snap.analog)
        self.assertAlmostEqual(snap.analog.vbat_v, 16.0, places=1)
        self.assertEqual(snap.rc, [1500] * 8)
        self.assertEqual(snap.rc_received_at, 10.0)

    def test_rc_timestamp_updates_only_on_rx(self):
        # First tick — no RC frame, rc_received_at stays None
        snap = self.adapter.tick(now=1.0)
        self.assertIsNone(snap.rc_received_at)
        # Second tick with RC frame
        self.client.poll_queue.append((msp.MSP_RC, _rc_payload([1500] * 8)))
        snap = self.adapter.tick(now=5.0)
        self.assertEqual(snap.rc_received_at, 5.0)
        # Third tick — no RC frame, timestamp stays at 5.0 (last known good)
        snap = self.adapter.tick(now=8.0)
        self.assertEqual(snap.rc_received_at, 5.0)


class TestBoxNamesRequest(unittest.TestCase):
    def setUp(self):
        self.client = _FakeClient()
        self.adapter = BetaflightAdapter.from_client(self.client)

    def test_requests_until_received(self):
        # First tick requests boxnames.
        self.adapter.tick(now=0.0)
        self.assertEqual(self.client.sent_cmds.count(msp.MSP_BOXNAMES), 1)
        # Within retry window — no additional request.
        self.adapter.tick(now=BOXNAMES_RETRY_S - 0.5)
        self.assertEqual(self.client.sent_cmds.count(msp.MSP_BOXNAMES), 1)
        # After retry window — re-request.
        self.adapter.tick(now=BOXNAMES_RETRY_S + 0.5)
        self.assertEqual(self.client.sent_cmds.count(msp.MSP_BOXNAMES), 2)
        # FC responds.
        self.client.poll_queue.append((msp.MSP_BOXNAMES, _box_names_payload(
            ["ARM", "ANGLE", "HORIZON", "BEEPER"])))
        self.adapter.tick(now=BOXNAMES_RETRY_S * 2 + 1)
        # After receipt — no further requests, however long we wait.
        before = self.client.sent_cmds.count(msp.MSP_BOXNAMES)
        self.adapter.tick(now=BOXNAMES_RETRY_S * 10)
        self.assertEqual(self.client.sent_cmds.count(msp.MSP_BOXNAMES), before)
        # Mode reader is primed.
        self.assertIsNotNone(self.adapter.mode_reader.box_names)


class TestAcroCheck(unittest.TestCase):
    def setUp(self):
        self.client = _FakeClient()
        self.adapter = BetaflightAdapter.from_client(self.client)

    def test_returns_none_before_box_names(self):
        self.adapter.tick(now=0.0)
        self.assertIsNone(self.adapter.is_acro_active())

    def test_angle_active_means_not_acro(self):
        self.client.poll_queue.append((msp.MSP_BOXNAMES, _box_names_payload(
            ["ARM", "ANGLE", "HORIZON", "BEEPER"])))
        # ANGLE is at index 1 → bit 1 set
        self.client.poll_queue.append((msp.MSP_STATUS_EX, _status_ex_payload(0b0010)))
        self.adapter.tick(now=0.0)
        self.assertEqual(self.adapter.is_acro_active(), False)

    def test_neither_angle_nor_horizon_is_acro(self):
        self.client.poll_queue.append((msp.MSP_BOXNAMES, _box_names_payload(
            ["ARM", "ANGLE", "HORIZON", "BEEPER"])))
        # Only ARM (bit 0) set
        self.client.poll_queue.append((msp.MSP_STATUS_EX, _status_ex_payload(0b0001)))
        self.adapter.tick(now=0.0)
        self.assertEqual(self.adapter.is_acro_active(), True)

    def test_malformed_status_ex_does_not_crash(self):
        self.client.poll_queue.append((msp.MSP_STATUS_EX, b"\x00" * 5))  # too short
        # Must not raise; the adapter swallows decode errors so a stray malformed
        # frame from a mid-flight glitch does not kill the loop.
        self.adapter.tick(now=0.0)

    def test_malformed_frames_all_decoders(self):
        # Every decoder branch must swallow ValueError, not just STATUS_EX.
        # Single short-payload frame for each of the 6 message types must not
        # raise out of tick().
        client = _FakeClient()
        adapter = BetaflightAdapter.from_client(client)
        for cmd in (
            msp.MSP_RAW_GPS, msp.MSP_ATTITUDE, msp.MSP_ALTITUDE,
            msp.MSP_ANALOG, msp.MSP_RC, msp.MSP_STATUS_EX,
        ):
            client.poll_queue.append((cmd, b"\x00"))  # short payload
        # Must not raise.
        adapter.tick(now=0.0)


class TestSnapshotIdentity(unittest.TestCase):
    def test_returns_distinct_snapshot_each_tick(self):
        # Caller must be able to retain a snapshot across ticks without
        # observing back-mutation. Each tick() returns an independent copy.
        client = _FakeClient()
        adapter = BetaflightAdapter.from_client(client)
        client.poll_queue.append((msp.MSP_ATTITUDE, _attitude_payload(yaw_deg=10)))
        s1 = adapter.tick(now=0.0)
        client.poll_queue.append((msp.MSP_ATTITUDE, _attitude_payload(yaw_deg=20)))
        s2 = adapter.tick(now=0.2)
        self.assertIsNot(s1, s2)
        # The earlier snapshot should be unchanged by the later tick.
        self.assertEqual(s1.attitude.yaw_deg, 10)
        self.assertEqual(s2.attitude.yaw_deg, 20)


class TestFcReboot(unittest.TestCase):
    def test_boxnames_re_request_after_reboot(self):
        # If the adapter's mode_reader is reset (simulating an FC reboot
        # mid-flight, after which we want to re-confirm the box-name map),
        # the retry timer fires another MSP_BOXNAMES request.
        client = _FakeClient()
        adapter = BetaflightAdapter.from_client(client)
        # Prime once.
        client.poll_queue.append((msp.MSP_BOXNAMES, _box_names_payload(
            ["ARM", "ANGLE", "HORIZON", "BEEPER"])))
        adapter.tick(now=0.0)
        first = client.sent_cmds.count(msp.MSP_BOXNAMES)
        # Simulate reboot: mode reader reset.
        adapter.mode_reader.box_names = None
        adapter.mode_reader.angle_idx = None
        adapter.mode_reader.horizon_idx = None
        adapter.tick(now=BOXNAMES_RETRY_S + 1)  # past retry window
        self.assertGreater(client.sent_cmds.count(msp.MSP_BOXNAMES), first)


class TestOverrideAndClose(unittest.TestCase):
    def test_send_overrides_delegates(self):
        client = _FakeClient()
        adapter = BetaflightAdapter.from_client(client)
        adapter.send_overrides([1500] * 8)
        self.assertEqual(client.raw_rcs, [[1500] * 8])

    def test_close_delegates(self):
        client = _FakeClient()
        adapter = BetaflightAdapter.from_client(client)
        adapter.close()
        self.assertTrue(client.closed)


if __name__ == "__main__":
    unittest.main()
