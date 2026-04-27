"""MSP-layer GPS shim — synthesizes MSP_RAW_GPS for BF SITL builds without
USE_GPS compiled in (our 4.5.1 image's `feature GPS` returns "unavailable").

Architecture:

    mission_demo  --MSP TCP :5762-->  bf_gps_shim  --MSP TCP :5761-->  BF SITL
                                            |
                       polls MSP_ATTITUDE + MSP_ALTITUDE in background
                                            v
                             integrated position model
                          (lat/lon/alt + horizontal velocity)

For every MSP request from mission_demo:
  - cmd 106 (MSP_RAW_GPS) → synthesized response from integrated position
  - all other commands → forwarded transparently to BF, response returned

Position model (tilt-and-thrust quadrotor):
  a_body_fwd  = -g · tan(pitch)        # nose-down ⇒ forward accel
  a_body_rt   = +g · tan(roll)         # right-wing-down ⇒ rightward accel
  rotate body→world via yaw, integrate velocity with linear drag,
  then position. Vertical alt comes straight from MSP_ALTITUDE (BF's
  internal SITL physics already tracks it correctly — flight_profile.py
  proved that loop closes).

Lat/lon converted by flat-Earth from a configured origin (West Point
default to match BetaflightBackendConfig). Speed/course derived from
the integrated horizontal velocity.

Run:
  T1: docker compose -f sitl/docker-compose.yml up -d
  T2: python -m integrations.tools.bf_gps_shim
  T3: python -m integrations.tools.discover_bf_gps --bf-msp-port 5762   # smoke
  T4: python -m integrations.tools.mission_demo  --bf-port 5762         # fly
"""
from __future__ import annotations

import argparse
import logging
import math
import socket
import struct
import sys
import threading
import time
from dataclasses import dataclass

log = logging.getLogger("bf_gps_shim")

# ── MSP framing ────────────────────────────────────────────────────────────

MSP_HEADER_REQ = b"$M<"
MSP_HEADER_RESP = b"$M>"
MSP_RAW_GPS = 106
MSP_ATTITUDE = 108
MSP_ALTITUDE = 109
MSP_RC = 105
MSP_ANALOG = 110
MSP_BOXNAMES = 116
MSP_STATUS_EX = 150

# Commands the shim caches from BF and serves to clients from a local
# cache. The racer_companion's BetaflightAdapter polls these 6 cmds at
# 10 Hz (every 100 ms). At that rate, BF SITL's UDP/MSP thread interaction
# stalls — sim_loop's motor-packet RX freezes, ARM_SWITCH latches,
# RX_FAILSAFE flaps. By caching, the companion sees its production-rate
# responses while BF only handles ~2 Hz queries from the shim's refresh
# thread. Real BF firmware on STM32 doesn't have this thread coupling,
# so the cache is a sim-only mechanism that keeps the companion code
# path identical between sim and real flight.
CACHED_CMDS: frozenset[int] = frozenset({
    MSP_ATTITUDE,
    MSP_ALTITUDE,
    MSP_RC,
    MSP_ANALOG,
    MSP_BOXNAMES,
    MSP_STATUS_EX,
})

# ── Earth model ────────────────────────────────────────────────────────────

M_PER_DEG_LAT = 110_540.0
M_PER_DEG_LON_EQUATOR = 111_320.0
GRAVITY = 9.806_65


def _msp_request(cmd: int, payload: bytes = b"") -> bytes:
    pkt = bytes([len(payload), cmd]) + payload
    chk = 0
    for b in pkt:
        chk ^= b
    return MSP_HEADER_REQ + pkt + bytes([chk])


def _msp_response(cmd: int, payload: bytes = b"") -> bytes:
    pkt = bytes([len(payload), cmd]) + payload
    chk = 0
    for b in pkt:
        chk ^= b
    return MSP_HEADER_RESP + pkt + bytes([chk])


# ── Config ─────────────────────────────────────────────────────────────────


@dataclass
class ShimConfig:
    bf_host: str = "127.0.0.1"
    bf_port: int = 5761
    listen_host: str = "127.0.0.1"
    listen_port: int = 5762

    # GPS origin — matches BetaflightBackendConfig default (West Point).
    origin_lat_deg: float = 41.3915
    origin_lon_deg: float = -73.9560
    origin_alt_m: float = 100.0

    # Synthesized GPS metadata.
    sat_count: int = 12      # mission_demo's min_satellites is 8
    fix_state: int = 1       # 1 = 3D fix

    # Integrator tuning.
    integrate_hz: float = 50.0
    poll_period_s: float = 0.05         # 20 Hz BF state poll
    horizontal_drag: float = 0.5        # 1/s — Iris-ish settling time
    log_period_s: float = 5.0

    # MSP response cache refresh rate. The shim sends queries to BF at
    # this rate to keep the cache warm. Companion polls at 10 Hz are
    # served entirely from cache without hitting BF. Setting this too
    # high reproduces the BF SITL stall (>3 Hz starts to be risky on
    # this Windows/Docker setup).
    cache_refresh_hz: float = 2.0


# ── MSP response cache ─────────────────────────────────────────────────────


class ResponseCache:
    """Per-cmd cached MSP response frames (full $M> ... bytes).

    Populated by the forwarder's u→d pump as responses arrive from BF.
    Consumed by the d→u pump when the client asks for a CACHED_CMDS
    cmd — we return the cached frame directly instead of forwarding
    the request to BF. This keeps BF's MSP load at the shim's refresh
    rate (~2 Hz) regardless of how fast the client polls (10 Hz).
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._frames: dict[int, bytes] = {}
        self._timestamps: dict[int, float] = {}

    def store(self, cmd: int, frame: bytes) -> None:
        now = time.monotonic()
        with self._lock:
            self._frames[cmd] = frame
            self._timestamps[cmd] = now

    def get(self, cmd: int) -> bytes | None:
        with self._lock:
            return self._frames.get(cmd)

    def age(self, cmd: int) -> float:
        with self._lock:
            t = self._timestamps.get(cmd)
        return float("inf") if t is None else time.monotonic() - t


# ── Position integrator ────────────────────────────────────────────────────


class PositionIntegrator:
    """Integrates horizontal position from BF's reported attitude.

    PASSIVE — no MSP connection. The shim's `_up_to_down` pump pipes
    every parsed MSP response through `feed()`. We extract MSP_ATTITUDE
    + MSP_ALTITUDE values to drive the position model.

    Why passive: BF SITL routes MSP responses to whichever client most
    recently sent the request (or its TCP server has some other
    multi-client quirk). An integrator with its own MSP connection
    stole replies meant for the forwarded client, causing
    mission_demo / flight_profile to never receive their MSP_ALTITUDE
    responses → controllers misbehave on stale alt=0.

    BF angle conventions (verified against probe_attitude.py):
      yaw   — 0..360, 0 = north, 90 = east  (heading)
      pitch — degrees, positive = nose UP
      roll  — degrees, positive = right wing DOWN
    """

    def __init__(self, cfg: ShimConfig):
        self.cfg = cfg
        self._lock = threading.Lock()
        self._n_m = 0.0
        self._e_m = 0.0
        self._alt_m = cfg.origin_alt_m
        self._vn = 0.0
        self._ve = 0.0
        self._yaw_deg = 0.0
        self._roll_deg = 0.0
        self._pitch_deg = 0.0
        # Latest sensor reads (set by feed(), consumed by _loop()).
        self._latest_yaw_deg = 0.0
        self._latest_pitch_deg = 0.0
        self._latest_roll_deg = 0.0
        self._latest_alt_cm = 0
        self._stop = threading.Event()

    def start(self) -> None:
        threading.Thread(
            target=self._loop, daemon=True, name="bf-pos-integrator",
        ).start()

    def stop(self) -> None:
        self._stop.set()

    def feed(self, cmd: int, payload: bytes) -> None:
        """Called by the shim's u→d pump for each parsed MSP response.
        Updates the latest cached attitude/altitude — _loop() integrates
        on its own clock."""
        if cmd == MSP_ATTITUDE and len(payload) >= 6:
            rx10, py10, yh = struct.unpack("<hhh", payload[:6])
            with self._lock:
                self._latest_roll_deg = rx10 / 10.0
                self._latest_pitch_deg = py10 / 10.0
                self._latest_yaw_deg = float(yh)
        elif cmd == MSP_ALTITUDE and len(payload) >= 6:
            alt_cm, _vario = struct.unpack("<ih", payload[:6])
            with self._lock:
                self._latest_alt_cm = alt_cm

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "n_m": self._n_m,
                "e_m": self._e_m,
                "alt_m": self._alt_m,
                "vn": self._vn,
                "ve": self._ve,
                "yaw_deg": self._yaw_deg,
                "roll_deg": self._roll_deg,
                "pitch_deg": self._pitch_deg,
            }

    def lat_lon_alt(self) -> tuple[float, float, float]:
        s = self.snapshot()
        cos_lat = max(0.05, math.cos(math.radians(self.cfg.origin_lat_deg)))
        lat = self.cfg.origin_lat_deg + s["n_m"] / M_PER_DEG_LAT
        lon = self.cfg.origin_lon_deg + s["e_m"] / (M_PER_DEG_LON_EQUATOR * cos_lat)
        return lat, lon, s["alt_m"]

    def _loop(self) -> None:
        dt_target = 1.0 / self.cfg.integrate_hz
        last_log = time.monotonic()
        last_t = time.monotonic()

        while not self._stop.is_set():
            now = time.monotonic()
            actual_dt = max(0.0, now - last_t)
            last_t = now

            # Snapshot latest sensor reads.
            with self._lock:
                yaw = self._latest_yaw_deg
                pitch = self._latest_pitch_deg
                roll = self._latest_roll_deg
                alt_cm = self._latest_alt_cm

            # Integrate horizontal motion. Pitch < 0 (nose down) → forward.
            yaw_rad = math.radians(yaw)
            pitch_rad = math.radians(pitch)
            roll_rad = math.radians(roll)
            a_body_fwd = -GRAVITY * math.tan(pitch_rad)
            a_body_rt = GRAVITY * math.tan(roll_rad)
            cy = math.cos(yaw_rad)
            sy = math.sin(yaw_rad)
            a_n = a_body_fwd * cy - a_body_rt * sy
            a_e = a_body_fwd * sy + a_body_rt * cy

            with self._lock:
                self._vn += a_n * actual_dt
                self._ve += a_e * actual_dt
                drag = min(1.0, self.cfg.horizontal_drag * actual_dt)
                self._vn *= (1.0 - drag)
                self._ve *= (1.0 - drag)
                self._n_m += self._vn * actual_dt
                self._e_m += self._ve * actual_dt
                self._alt_m = self.cfg.origin_alt_m + (alt_cm / 100.0)
                self._yaw_deg = yaw
                self._roll_deg = roll
                self._pitch_deg = pitch

            if now - last_log >= self.cfg.log_period_s:
                last_log = now
                lat, lon, alt = self.lat_lon_alt()
                with self._lock:
                    n, e, vn, ve = self._n_m, self._e_m, self._vn, self._ve
                log.info(
                    "[pos] N=%+7.1fm E=%+7.1fm alt=%5.1fm | vN=%+5.2f vE=%+5.2f "
                    "| yaw=%5.1f° roll=%+5.1f° pitch=%+5.1f° | lat=%.6f lon=%.6f",
                    n, e, alt, vn, ve, yaw, roll, pitch, lat, lon,
                )

            sleep_for = dt_target - (time.monotonic() - now)
            if sleep_for > 0:
                time.sleep(sleep_for)


# ── MSP_RAW_GPS synth ──────────────────────────────────────────────────────


def synthesize_raw_gps(integrator: PositionIntegrator, cfg: ShimConfig) -> bytes:
    """Build an MSP_RAW_GPS response from the integrator's current state.

    Wire format (matches msp.decode_raw_gps in racer_companion):
      u8  fix
      u8  num_sat
      i32 lat × 1e7 (deg)
      i32 lon × 1e7 (deg)
      u16 alt (m, unsigned — clip to 0)
      u16 speed (cm/s)
      u16 course × 10 (deg)
    """
    lat, lon, alt = integrator.lat_lon_alt()
    s = integrator.snapshot()
    speed_m_s = math.sqrt(s["vn"] ** 2 + s["ve"] ** 2)
    course_deg = (math.degrees(math.atan2(s["ve"], s["vn"])) + 360.0) % 360.0
    payload = struct.pack(
        "<BBiiHHH",
        cfg.fix_state,
        cfg.sat_count,
        int(round(lat * 1e7)),
        int(round(lon * 1e7)),
        max(0, min(0xFFFF, int(round(alt)))),
        max(0, min(0xFFFF, int(round(speed_m_s * 100)))),
        int(round(course_deg * 10)) % 3600,
    )
    return _msp_response(MSP_RAW_GPS, payload)


# ── Per-client forwarder ───────────────────────────────────────────────────


class ClientForwarder:
    """One mission_demo connection ⇄ one BF connection. Three threads:
    - d→u: downstream→upstream, intercepts MSP_RAW_GPS (synth) and any
      cmd in CACHED_CMDS (served from cache) — never forwards those.
    - u→d: upstream→downstream, parses each $M> frame; cached cmds go
      to cache (absorbed), others forwarded.
    - cache_refresh: at cache_refresh_hz, sends queries upstream so the
      cache stays warm regardless of client polling rate."""

    def __init__(
        self,
        downstream: socket.socket,
        addr,
        integrator: PositionIntegrator,
        cfg: ShimConfig,
    ):
        self.down = downstream
        self.addr = addr
        self.integrator = integrator
        self.cfg = cfg
        self.up: socket.socket | None = None
        self.cache = ResponseCache()
        self._stop = threading.Event()

    def run(self) -> None:
        try:
            self.up = socket.create_connection(
                (self.cfg.bf_host, self.cfg.bf_port), timeout=5.0,
            )
        except OSError as e:
            log.error("[fwd %s] upstream connect failed: %s", self.addr, e)
            try:
                self.down.close()
            except OSError:
                pass
            return
        log.info("[fwd %s] connected → BF %s:%d (cache refresh %.1f Hz)",
                 self.addr, self.cfg.bf_host, self.cfg.bf_port,
                 self.cfg.cache_refresh_hz)
        t1 = threading.Thread(target=self._down_to_up, daemon=True,
                              name=f"d2u-{self.addr[1]}")
        t2 = threading.Thread(target=self._up_to_down, daemon=True,
                              name=f"u2d-{self.addr[1]}")
        t3 = threading.Thread(target=self._cache_refresh_loop, daemon=True,
                              name=f"refresh-{self.addr[1]}")
        t1.start()
        t2.start()
        t3.start()
        t1.join()
        t2.join()
        # cache thread is daemon and will exit when sockets close
        for s in (self.up, self.down):
            try:
                s.close()
            except OSError:
                pass
        log.info("[fwd %s] disconnected", self.addr)

    def _down_to_up(self) -> None:
        """Read MSP requests from mission_demo. Intercept MSP_RAW_GPS."""
        buf = bytearray()
        while not self._stop.is_set():
            try:
                chunk = self.down.recv(4096)
            except OSError:
                break
            if not chunk:
                break
            buf.extend(chunk)
            self._consume_requests(buf)
        self._stop.set()

    def _consume_requests(self, buf: bytearray) -> None:
        """Pull complete `$M<` frames out of buf; intercept MSP_RAW_GPS,
        forward all others. Anything before `$M<` (e.g. CLI bytes) flows
        through verbatim."""
        while True:
            i = buf.find(MSP_HEADER_REQ)
            if i < 0:
                if buf:
                    # No more headers — pass remaining bytes upstream as-is
                    # (could be CLI mode `#\n` or the start of a header that
                    # hasn't fully arrived). Keep last 2 bytes in case `$M`
                    # split across recv boundaries.
                    if len(buf) > 2:
                        try:
                            self.up.sendall(bytes(buf[:-2]))
                        except OSError:
                            self._stop.set()
                            return
                        del buf[:-2]
                return
            if i > 0:
                try:
                    self.up.sendall(bytes(buf[:i]))
                except OSError:
                    self._stop.set()
                    return
                del buf[:i]
            if len(buf) < 6:
                return
            size = buf[3]
            cmd = buf[4]
            total = 5 + size + 1
            if len(buf) < total:
                return
            frame = bytes(buf[:total])
            del buf[:total]
            if cmd == MSP_RAW_GPS:
                # Synthesized from integrator — never reaches BF.
                try:
                    self.down.sendall(synthesize_raw_gps(
                        self.integrator, self.cfg,
                    ))
                except OSError:
                    self._stop.set()
                    return
            elif cmd in CACHED_CMDS:
                # Serve from cache. The refresh thread keeps it warm.
                # Cache miss = silently drop the request — client will
                # retry on its next tick (~20-100 ms later) by which
                # time the refresh thread has populated the cache.
                cached = self.cache.get(cmd)
                if cached is not None:
                    try:
                        self.down.sendall(cached)
                    except OSError:
                        self._stop.set()
                        return
            else:
                # Non-cached cmd (e.g., MSP_API_VERSION, MSP_FC_VARIANT,
                # MSP_BOARD_INFO, MSP_SET_RAW_RC). Forward to BF as usual.
                try:
                    self.up.sendall(frame)
                except OSError:
                    self._stop.set()
                    return

    def _up_to_down(self) -> None:
        """Read BF responses, parse into frames, dispatch each frame:
          - cached cmds → store in cache, feed integrator, ABSORB
            (don't forward to client; the d→u pump serves from cache).
          - non-cached cmds → forward verbatim to client.
        We must parse before forwarding so we can decide per-frame; we
        can't just stream bytes through like the original implementation.
        """
        buf = bytearray()
        while not self._stop.is_set():
            try:
                chunk = self.up.recv(4096)
            except OSError:
                break
            if not chunk:
                break
            buf.extend(chunk)
            self._dispatch_responses(buf)
        self._stop.set()

    def _dispatch_responses(self, buf: bytearray) -> None:
        """Pull complete $M> frames out of buf and dispatch."""
        while True:
            i = buf.find(MSP_HEADER_RESP)
            if i < 0:
                # No header. Drop oldest bytes if buffer balloons (shouldn't
                # happen — BF only sends well-formed frames).
                if len(buf) > 4096:
                    del buf[:-256]
                return
            if i > 0:
                # Pre-header bytes — drop. (BF doesn't send free-form text.)
                del buf[:i]
            if len(buf) < 6:
                return
            size = buf[3]
            cmd = buf[4]
            total = 5 + size + 1
            if len(buf) < total:
                return
            frame = bytes(buf[:total])
            payload = bytes(buf[5: 5 + size])
            del buf[:total]

            # Always feed integrator (passive position model).
            self.integrator.feed(cmd, payload)

            if cmd in CACHED_CMDS:
                # Refresh-thread or client-triggered query response —
                # park in cache, don't forward.
                self.cache.store(cmd, frame)
            else:
                # One-off response (e.g., MSP_API_VERSION) — forward.
                try:
                    self.down.sendall(frame)
                except OSError:
                    self._stop.set()
                    return

    def _cache_refresh_loop(self) -> None:
        """Background: poll BF for cacheable cmds at cache_refresh_hz.
        BF responds, _dispatch_responses absorbs into cache. Companion
        polls (10 Hz) are served entirely from cache without BF ever
        seeing them — the entire point of the cache layer."""
        period = 1.0 / max(0.1, self.cfg.cache_refresh_hz)
        cmds = list(CACHED_CMDS)
        while not self._stop.is_set():
            for cmd in cmds:
                if self.up is None:
                    return
                try:
                    self.up.sendall(_msp_request(cmd))
                except OSError:
                    return
                # Tiny gap between burst queries so BF can interleave
                # processing without backlog.
                time.sleep(0.005)
            # Sleep to next refresh tick, in small chunks so we exit
            # promptly when the forwarder shuts down.
            slept = 0.0
            while slept < period and not self._stop.is_set():
                step = min(0.05, period - slept)
                time.sleep(step)
                slept += step


# ── Main ───────────────────────────────────────────────────────────────────


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--bf-host", default="127.0.0.1")
    p.add_argument("--bf-port", type=int, default=5761)
    p.add_argument("--listen-host", default="127.0.0.1")
    p.add_argument("--listen-port", type=int, default=5762)
    p.add_argument("--origin-lat", type=float, default=41.3915)
    p.add_argument("--origin-lon", type=float, default=-73.9560)
    p.add_argument("--origin-alt", type=float, default=100.0)
    p.add_argument("--sats", type=int, default=12)
    p.add_argument("--log-level", default="INFO")
    args = p.parse_args()

    logging.basicConfig(
        level=args.log_level.upper(),
        format="%(asctime)s %(levelname)s %(name)s %(message)s",
    )

    cfg = ShimConfig(
        bf_host=args.bf_host,
        bf_port=args.bf_port,
        listen_host=args.listen_host,
        listen_port=args.listen_port,
        origin_lat_deg=args.origin_lat,
        origin_lon_deg=args.origin_lon,
        origin_alt_m=args.origin_alt,
        sat_count=args.sats,
    )

    integrator = PositionIntegrator(cfg)
    log.info("starting position integrator (passive — observes "
             "attitude/altitude piped from forwarded clients)")
    integrator.start()

    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind((cfg.listen_host, cfg.listen_port))
    listener.listen(8)
    log.info("MSP shim listening on tcp://%s:%d (upstream BF tcp://%s:%d)",
             cfg.listen_host, cfg.listen_port, cfg.bf_host, cfg.bf_port)
    log.info("origin: lat=%.6f lon=%.6f alt=%.1fm  | synthesized sats=%d, fix=%d",
             cfg.origin_lat_deg, cfg.origin_lon_deg, cfg.origin_alt_m,
             cfg.sat_count, cfg.fix_state)
    log.info("Connect mission_demo with `--bf-port %d`", cfg.listen_port)

    try:
        while True:
            try:
                client_sock, addr = listener.accept()
            except KeyboardInterrupt:
                break
            fwd = ClientForwarder(client_sock, addr, integrator, cfg)
            threading.Thread(
                target=fwd.run, daemon=True, name=f"fwd-{addr[1]}",
            ).start()
    finally:
        integrator.stop()
        try:
            listener.close()
        except OSError:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
