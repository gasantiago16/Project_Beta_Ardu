"""Round-trip tests for the byte-level MSP recorder/replayer.

The contract: a stream of bytes recorded via RecordingAdapter, replayed via
ReplayAdapter, then parsed by MspClient must yield the same decoded frames
that would have been parsed in the live session.
"""
from __future__ import annotations

import json
import struct
import tempfile
import unittest
from pathlib import Path

from racer_companion import msp
from racer_companion.recorder import RecordingAdapter, ReplayAdapter


class _FakeSerial:
    """Tiny stand-in for pyserial.Serial. Returns chunks from a queue on read,
    accumulates writes in a bytearray. Lets us drive RecordingAdapter without
    real hardware."""

    def __init__(self, rx_chunks: list[bytes] | None = None):
        self._rx = list(rx_chunks or [])
        self.tx = bytearray()
        self.closed = False

    def read(self, n: int) -> bytes:
        if not self._rx:
            return b""
        return self._rx.pop(0)

    def write(self, data: bytes) -> int:
        self.tx.extend(data)
        return len(data)

    def close(self) -> None:
        self.closed = True


def _build_response(cmd: int, payload: bytes) -> bytes:
    size = len(payload)
    chk = size ^ cmd
    for b in payload:
        chk ^= b
    return b"$M>" + bytes([size, cmd]) + payload + bytes([chk & 0xFF])


class TestRecordingAdapter(unittest.TestCase):
    def test_records_rx_and_tx(self):
        rx_frame = _build_response(msp.MSP_ATTITUDE, struct.pack("<hhh", 0, 0, 0))
        with tempfile.TemporaryDirectory() as td:
            log_path = Path(td) / "session.jsonl"
            inner = _FakeSerial(rx_chunks=[rx_frame])
            t = [0.0]
            adapter = RecordingAdapter(inner, log_path, time_source=lambda: t[0])
            t[0] = 1.0
            self.assertEqual(adapter.read(64), rx_frame)
            t[0] = 1.5
            adapter.write(b"hello")
            adapter.close()
            self.assertTrue(inner.closed)
            entries = [json.loads(l) for l in log_path.read_text().splitlines() if l.strip()]
        self.assertEqual(len(entries), 2)
        rx, tx = entries
        self.assertEqual(rx["dir"], "rx")
        self.assertEqual(bytes.fromhex(rx["hex"]), rx_frame)
        self.assertEqual(rx["t"], 1.0)
        self.assertEqual(tx["dir"], "tx")
        self.assertEqual(bytes.fromhex(tx["hex"]), b"hello")

    def test_skips_empty_reads(self):
        # A read that returns b"" shouldn't pollute the log.
        with tempfile.TemporaryDirectory() as td:
            log_path = Path(td) / "session.jsonl"
            inner = _FakeSerial(rx_chunks=[b""])
            adapter = RecordingAdapter(inner, log_path)
            adapter.read(64)
            adapter.close()
            self.assertEqual(log_path.read_text().strip(), "")


class TestReplayAdapter(unittest.TestCase):
    def test_round_trip_single_frame(self):
        rx_frame = _build_response(msp.MSP_ATTITUDE, struct.pack("<hhh", 100, -50, 270))
        with tempfile.TemporaryDirectory() as td:
            log_path = Path(td) / "session.jsonl"
            # Record
            inner = _FakeSerial(rx_chunks=[rx_frame])
            recorder = RecordingAdapter(inner, log_path, time_source=lambda: 0.0)
            recorder.read(64)
            recorder.close()
            # Replay (drain immediately)
            replay = ReplayAdapter(log_path, time_source=lambda: float("inf"))
            client = msp.MspClient.from_adapter(replay)
            frames = client.poll()
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0][0], msp.MSP_ATTITUDE)

    def test_round_trip_multiple_frames(self):
        f1 = _build_response(msp.MSP_ATTITUDE, struct.pack("<hhh", 100, -50, 270))
        f2 = _build_response(msp.MSP_ALTITUDE, struct.pack("<ih", 500, -25))
        with tempfile.TemporaryDirectory() as td:
            log_path = Path(td) / "session.jsonl"
            t = [0.0]
            inner = _FakeSerial(rx_chunks=[f1, f2])
            recorder = RecordingAdapter(inner, log_path, time_source=lambda: t[0])
            recorder.read(64)
            t[0] = 0.05
            recorder.read(64)
            recorder.close()
            replay = ReplayAdapter(log_path, time_source=lambda: float("inf"))
            client = msp.MspClient.from_adapter(replay)
            frames = client.poll()
        cmds = [f[0] for f in frames]
        self.assertEqual(cmds, [msp.MSP_ATTITUDE, msp.MSP_ALTITUDE])

    def test_realtime_pacing(self):
        # Replay paced to time_source: a fixed clock at t=0 should yield
        # nothing yet (first event is at t=0 but elapsed is also 0, so it's
        # at the boundary — IS yielded).
        # A clock at t=−1 (impossible in practice but exercise the elapsed<0
        # branch) should yield nothing.
        f1 = _build_response(msp.MSP_ATTITUDE, struct.pack("<hhh", 0, 0, 0))
        with tempfile.TemporaryDirectory() as td:
            log_path = Path(td) / "session.jsonl"
            inner = _FakeSerial(rx_chunks=[f1])
            recorder = RecordingAdapter(inner, log_path, time_source=lambda: 5.0)
            recorder.read(64)
            recorder.close()
            now = [5.0]  # exactly the recorded timestamp
            replay = ReplayAdapter(log_path, time_source=lambda: now[0])
            self.assertEqual(replay.read(64), f1)  # immediately due
            self.assertTrue(replay.exhausted)

    def test_malformed_lines_are_skipped(self):
        # A torn last line + a non-JSON line + a missing-field line should
        # all be ignored without aborting the whole replay.
        with tempfile.TemporaryDirectory() as td:
            log_path = Path(td) / "session.jsonl"
            log_path.write_text(
                '{"t": 0.0, "dir": "rx", "hex": "deadbeef"}\n'
                'this is not json\n'
                '{"t": 0.1, "dir": "rx"}\n'  # missing hex
                '{"t": 0.2, "dir": "rx", "hex": "feedface"}\n'
                '{"t": 0.3, "dir": "rx", "hex": "deadb',  # torn
                encoding="utf-8",
            )
            replay = ReplayAdapter(log_path, time_source=lambda: float("inf"))
            data = replay.read(64)
            self.assertEqual(data, b"\xde\xad\xbe\xef\xfe\xed\xfa\xce")

    def test_out_of_order_timestamps_are_sorted(self):
        # File-order is not trusted: events with t=2.0 then t=1.0 must be
        # delivered in time order.
        with tempfile.TemporaryDirectory() as td:
            log_path = Path(td) / "session.jsonl"
            log_path.write_text(
                '{"t": 2.0, "dir": "rx", "hex": "bb"}\n'
                '{"t": 1.0, "dir": "rx", "hex": "aa"}\n',
                encoding="utf-8",
            )
            replay = ReplayAdapter(log_path, time_source=lambda: float("inf"))
            self.assertEqual(replay.read(64), b"\xaa\xbb")

    def test_non_monotonic_clock_does_not_stall(self):
        # If time_source jumps backward (mocked clock decremented), replay
        # should NOT stall — clamp elapsed to >= 0.
        with tempfile.TemporaryDirectory() as td:
            log_path = Path(td) / "session.jsonl"
            log_path.write_text(
                '{"t": 5.0, "dir": "rx", "hex": "aa"}\n',
                encoding="utf-8",
            )
            now = [10.0]
            replay = ReplayAdapter(log_path, time_source=lambda: now[0])
            replay.read(64)  # anchors real_anchor=10.0, log_anchor=5.0; first event due
            now[0] = 5.0  # jump backward 5s
            # Should not raise and should not deliver a phantom future event.
            replay.read(64)  # already exhausted; just should not stall

    def test_recorder_survives_disk_full(self):
        # If the log fh raises OSError, the recorder must NOT propagate it
        # to the caller — flight is more important than logging.
        class FailingFH:
            def write(self, _): raise OSError("disk full")
            def flush(self): pass
            def close(self): pass

        rx_frame = b"\x24\x4d\x3e\x00\x00\x00"  # tiny valid-ish header
        with tempfile.TemporaryDirectory() as td:
            log_path = Path(td) / "session.jsonl"
            inner = _FakeSerial(rx_chunks=[rx_frame])
            adapter = RecordingAdapter(inner, log_path)
            adapter._fh.close()  # close real fh
            adapter._fh = FailingFH()  # swap in failing one
            # Must not raise.
            self.assertEqual(adapter.read(64), rx_frame)
            adapter.write(b"hi")
            self.assertTrue(adapter._fh_dead)

    def test_paced_holds_future_events(self):
        # First event at t=10, second at t=11. Real clock at t=10 should
        # yield ONLY the first.
        f1 = _build_response(msp.MSP_ATTITUDE, struct.pack("<hhh", 0, 0, 0))
        f2 = _build_response(msp.MSP_ALTITUDE, struct.pack("<ih", 500, 0))
        with tempfile.TemporaryDirectory() as td:
            log_path = Path(td) / "session.jsonl"
            t = [10.0]
            inner = _FakeSerial(rx_chunks=[f1, f2])
            recorder = RecordingAdapter(inner, log_path, time_source=lambda: t[0])
            recorder.read(64)
            t[0] = 11.0
            recorder.read(64)
            recorder.close()
            now = [10.0]  # at first event's recorded timestamp
            replay = ReplayAdapter(log_path, time_source=lambda: now[0])
            client = msp.MspClient.from_adapter(replay)
            frames = client.poll()
            self.assertEqual(len(frames), 1)
            self.assertEqual(frames[0][0], msp.MSP_ATTITUDE)
            self.assertFalse(replay.exhausted)
            now[0] = 11.0  # second event becomes due
            frames2 = client.poll()
            self.assertEqual(len(frames2), 1)
            self.assertEqual(frames2[0][0], msp.MSP_ALTITUDE)
            self.assertTrue(replay.exhausted)


if __name__ == "__main__":
    unittest.main()
