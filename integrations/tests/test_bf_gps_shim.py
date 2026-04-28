"""Unit tests for bf_gps_shim's thread-safe MSP forwarding.

The shim's `ClientForwarder` has two threads writing to the upstream
socket (BF): the d→u request-forwarder and the cache-refresh poller.
Without serialization, Python's `socket.sendall` is not atomic — two
concurrent calls can interleave bytes mid-MSP-frame, corrupting an
MSP_SET_RAW_RC frame BF then drops as bad RX. That's the latch
trigger we hunted down on Apr 27.

These tests verify the locks added to `_send_up` / `_send_down`
serialize concurrent writes such that no frame ever interleaves with
another.
"""
from __future__ import annotations

import sys
import threading
import time
import unittest
from pathlib import Path

_TOOLS = Path(__file__).resolve().parents[1] / "tools"
sys.path.insert(0, str(_TOOLS))

import bf_gps_shim as shim  # noqa: E402


class _SlowSocket:
    """Stand-in socket whose sendall artificially holds the GIL between
    the first and last byte of a payload — so any unprotected concurrent
    sendall would interleave deterministically. We record the payloads
    in the order they fully arrive."""

    def __init__(self, hold_s: float = 0.005):
        self.hold_s = hold_s
        self._lock_for_recording = threading.Lock()
        self.received: list[bytes] = []
        # Detect interleave: while a sendall is in progress, we mark
        # ourselves "busy"; if a second thread's sendall finds busy=True,
        # we record an interleave event.
        self._busy = False
        self.interleaves = 0

    def sendall(self, data: bytes) -> None:
        if self._busy:
            self.interleaves += 1
        self._busy = True
        time.sleep(self.hold_s)  # GIL releases during sleep
        self._busy = False
        with self._lock_for_recording:
            self.received.append(bytes(data))


class TestUpstreamSendLock(unittest.TestCase):
    def _make_forwarder(self, up: _SlowSocket) -> shim.ClientForwarder:
        cfg = shim.ShimConfig()
        integ = shim.PositionIntegrator(cfg)  # don't start its loop
        # ClientForwarder needs a downstream socket too; reuse a fake.
        fwd = shim.ClientForwarder(_SlowSocket(0.0), ("test", 0), integ, cfg)
        fwd.up = up
        return fwd

    def test_concurrent_send_up_does_not_interleave(self):
        up = _SlowSocket(hold_s=0.003)
        fwd = self._make_forwarder(up)

        n = 20
        payloads = [bytes([i]) * 32 for i in range(n)]  # distinguishable

        def worker(p: bytes) -> None:
            fwd._send_up(p)

        threads = [threading.Thread(target=worker, args=(p,)) for p in payloads]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        self.assertEqual(up.interleaves, 0,
                         "lock failed: sendall calls overlapped")
        self.assertEqual(len(up.received), n)
        # Every captured payload must be one of the originals — no
        # mid-frame splices.
        for got in up.received:
            self.assertIn(got, payloads)

    def test_send_up_returns_false_after_socket_error(self):
        # If sendall raises OSError, _send_up sets _stop and returns False.
        class _BrokenSocket:
            def sendall(self, _data):
                raise OSError("simulated")

        cfg = shim.ShimConfig()
        integ = shim.PositionIntegrator(cfg)
        fwd = shim.ClientForwarder(_BrokenSocket(), ("test", 0), integ, cfg)
        fwd.up = _BrokenSocket()
        ok = fwd._send_up(b"$M<\x00\x01\x01")
        self.assertFalse(ok)
        self.assertTrue(fwd._stop.is_set())

    def test_send_up_no_socket_returns_false(self):
        cfg = shim.ShimConfig()
        integ = shim.PositionIntegrator(cfg)
        fwd = shim.ClientForwarder(_SlowSocket(0.0), ("test", 0), integ, cfg)
        # fwd.up is None until run() connects upstream.
        self.assertIsNone(fwd.up)
        self.assertFalse(fwd._send_up(b"x"))


if __name__ == "__main__":
    unittest.main()
