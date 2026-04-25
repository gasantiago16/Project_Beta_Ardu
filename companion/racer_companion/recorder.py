"""Byte-level MSP recording + replay.

Use case: capture every byte to/from the FC during a flight (or a SITL run),
then re-feed the recording into the companion's parser for offline analysis
or regression testing. Field incidents become reproducible — "the drone did
X" turns into "here's the .jsonl, run replay.py and watch it happen."

Operates at the serial-adapter layer (read/write/close), so MspClient itself
is unchanged. Pass a `RecordingAdapter` or `ReplayAdapter` via
`MspClient.from_adapter(adapter)`.

Wire format: one JSON object per line (JSONL).
  {"t": <monotonic>, "dir": "rx" | "tx", "hex": "<hex bytes>"}

`rx` = bytes the FC sent us (we read from the wire).
`tx` = bytes we sent to the FC.
Timestamps are seconds from `time.monotonic()`; absolute origin doesn't
matter — only deltas. Replay anchors to the first `rx` event.
"""
from __future__ import annotations

import json
import math
import time
from pathlib import Path
from typing import Callable, IO, Optional, Protocol


class _SerialLike(Protocol):
    """The subset of pyserial.Serial that MspClient uses."""

    def read(self, n: int) -> bytes: ...
    def write(self, data: bytes) -> int: ...
    def close(self) -> None: ...


class RecordingAdapter:
    """Decorator over a real serial adapter. Tees every read/write byte to a
    JSONL log file. Use during a flight or SITL run; analyze afterward.

    Always opens the log in append mode so multiple sessions accumulate.
    Caller is responsible for closing — `close()` flushes the log fh too.
    """

    def __init__(self, inner: _SerialLike, log_path: str | Path,
                 time_source: Callable[[], float] = time.monotonic):
        self.inner = inner
        self._fh: IO[str] = open(log_path, "a", encoding="utf-8")
        self._time = time_source
        self._fh_dead = False

    def _emit(self, direction: str, data: bytes) -> None:
        if not data or self._fh_dead:
            return
        # Disk-full / pipe-closed during a flight must NOT take the FC link
        # down. Mark the recording dead and continue silently — the flight
        # is more important than the log.
        try:
            self._fh.write(json.dumps({
                "t": self._time(), "dir": direction, "hex": data.hex(),
            }) + "\n")
            # Flush every event so a hard crash mid-flight doesn't lose the tail.
            self._fh.flush()
        except OSError:
            self._fh_dead = True

    def read(self, n: int) -> bytes:
        data = self.inner.read(n)
        self._emit("rx", data)
        return data

    def write(self, data: bytes) -> int:
        self._emit("tx", bytes(data))
        return self.inner.write(data)

    def close(self) -> None:
        try:
            self.inner.close()
        finally:
            try:
                self._fh.close()
            except OSError:
                pass


class ReplayAdapter:
    """Serial-adapter facade that yields bytes from a JSONL recording, paced
    to the original recording timeline.

    `read(n)` returns at most `n` bytes that were recorded as `rx` and whose
    relative timestamp ≤ elapsed real-time since the first read. This makes
    replay run in real-time by default. Pass `time_source=` to fast-forward
    (e.g., for tests). `write()` is a no-op (don't echo back to the recording).

    Reading the entire log all-at-once requires a `time_source` that returns
    a monotonically-increasing value past the last recorded delta — easiest
    is `lambda: float('inf')`, which drains immediately.
    """

    def __init__(self, log_path: str | Path,
                 time_source: Callable[[], float] = time.monotonic):
        self._events: list[tuple[float, bytes]] = []
        with open(log_path, "r", encoding="utf-8") as fh:
            for line_no, raw in enumerate(fh, 1):
                raw = raw.strip()
                if not raw:
                    continue
                # Tolerate malformed lines: a torn last line from a
                # process-killed-mid-write is common in real flight logs.
                try:
                    d = json.loads(raw)
                except json.JSONDecodeError:
                    continue
                if d.get("dir") != "rx":
                    continue
                try:
                    self._events.append((float(d["t"]), bytes.fromhex(d["hex"])))
                except (KeyError, TypeError, ValueError):
                    continue
        # Sort by timestamp — file order is usually fine but a clock reset or
        # interleaved flush from multi-process writers could produce out-of-
        # order events. Sorting once is cheaper than defending against it
        # in every read() call.
        self._events.sort(key=lambda ev: ev[0])
        self._cursor = 0
        # Byte buffer: the previous `read()` may have returned fewer bytes
        # than the next event held; we carry the remainder forward.
        self._pending = bytearray()
        self._time = time_source
        self._real_anchor: Optional[float] = None
        self._log_anchor: Optional[float] = None

    def _maybe_anchor(self) -> None:
        if self._real_anchor is None and self._events:
            self._real_anchor = self._time()
            self._log_anchor = self._events[0][0]

    def _elapsed_log(self) -> float:
        if self._real_anchor is None:
            return 0.0
        elapsed = self._time() - self._real_anchor
        # If the time-source returns inf (drain mode), elapsed is nan
        # (inf-inf). Treat that as unbounded so all events drain. Otherwise,
        # clamp to >= 0 so a non-monotonic clock (skew, mocked clock running
        # backward) doesn't silently stall the replay.
        if math.isnan(elapsed):
            return math.inf
        return max(0.0, elapsed)

    def read(self, n: int) -> bytes:
        self._maybe_anchor()
        elapsed = self._elapsed_log()
        # Pull all events whose log-relative timestamp has been reached.
        while self._cursor < len(self._events):
            t, b = self._events[self._cursor]
            if (t - (self._log_anchor or 0)) > elapsed:
                break
            self._pending.extend(b)
            self._cursor += 1
        if not self._pending:
            return b""
        if len(self._pending) <= n:
            out = bytes(self._pending)
            self._pending.clear()
            return out
        out = bytes(self._pending[:n])
        del self._pending[:n]
        return out

    def write(self, data: bytes) -> int:
        # Replay is a one-way analysis tool — never echo back into the file.
        return len(data)

    def close(self) -> None:
        pass

    @property
    def exhausted(self) -> bool:
        """True once every recorded RX byte has been consumed."""
        return self._cursor >= len(self._events) and not self._pending
