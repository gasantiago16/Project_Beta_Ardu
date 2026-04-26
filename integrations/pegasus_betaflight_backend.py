"""Pegasus Backend that bridges Isaac Sim physics to Betaflight SITL via UDP.

Architecture:

    Pegasus Multirotor (Iris) -- ENU/FLU State 250 Hz -->  this backend
                                                                |
                                       fdm_packet (144 B) UDP  v
    +---------------------------------------------------------+
    | Docker: BF SITL 4.5.1 (project-beta-ardu-sitl)          |
    |   listens UDP 9003 (PORT_STATE — FDM sensor input)      |
    |   emits   UDP 9002 (PORT_PWM — motor output) → simulator_ip
    |   serves  TCP 5761 — MSP for racer_companion (separate) |
    +---------------------------------------------------------+
                                                                |
                                       servo_packet (16 B) UDP  v
    Pegasus Multirotor (Iris) <-- rotor ω (rad/s) --   this backend

The companion (`racer_companion.main`) connects independently to TCP 5761;
it does not interact with this bridge.

Wire format (verified against `betaflight/4.5.1/src/main/target/SITL/sitl.c`):
- fdm_packet: 18 little-endian doubles (144 B). All world-frame fields are
  NED, all body-frame fields are FRD.
    timestamp(1), ω_body_FRD(3) rad/s,
    accel_body_FRD(3) m/s² (BF reads gravity as +g on z-down at rest),
    q_world_to_body_NED_FRD[wxyz](4),
    v_NED(3) m/s (north, east, down),
    p_NED(3) m   (north, east, down — relative to home origin),
    pressure_Pa(1).
- servo_packet: 4 little-endian floats (16 B), normalized [0.0, 1.0].

BF source defines:
    PORT_STATE = 9003   // BF binds + reads FDM here
    PORT_PWM   = 9002   // BF sends motor packets to (simulator_ip, 9002)
The simulator_ip defaults to "127.0.0.1" but is overridable via argv[1] to
the betaflight_SITL binary. **For Docker on Win/Mac, sitl/start.sh passes
`host.docker.internal` so motor packets reach the host bridge process.**

Lockstep ordering (per Pegasus multirotor.py:113-141 — `input_reference()`
runs BEFORE `update(dt)`): in `update(dt)` we send FDM to (bf_host, 9003),
then blocking-recv motor on the local 9002 socket, cache the result.
`input_reference()` returns the cached values scaled to rad/s. First tick
runs before any motor packet — `start()` seeds the cache with zeros.
"""
from __future__ import annotations

import logging
import math
import socket
import struct
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation

log = logging.getLogger("racer_companion.integrations.pegasus_betaflight")

# Local frame helpers — duck-typed so we don't import Pegasus at module load.
# rot_FLU_to_FRD: pitch-axis 180° flip — y → -y, z → -z.
_FLU_TO_FRD = Rotation.from_matrix([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
# rot_ENU_to_NED: same shape (north/east swap + z flip), but for our purposes
# we mostly use the FLU→FRD body rotation; ENU→NED only enters via Pegasus's
# State.get_attitude_ned_frd helper (see _state_attitude_ned_frd below).

# Standard gravity (m/s²) — added to body-FRD kinematic acceleration so BF
# reads "+g on z" at rest, matching its accelerometer convention.
_GRAVITY = 9.806_65

# International Standard Atmosphere — used to map altitude to barometer
# pressure for the FDM packet. Accurate to ~0.5% up to a few km.
_ISA_SEA_LEVEL_PA = 101_325.0
_ISA_LAPSE_K_PER_M = 2.255_77e-5
_ISA_EXPONENT = 5.255_88


@dataclass
class BetaflightBackendConfig:
    """Caller-tunable config for the bridge.

    Defaults assume:
    - BF SITL Docker container running on the local host with port mappings
      from `Project_Beta_Ardu/sitl/docker-compose.yml` (9002/9003 udp).
    - Iris airframe (Pegasus default) — `rotor_max_omega` derived from a 880
      KV motor on 11.1 V × 2π/60. For a 5" race quad in Phase 5, override to
      ~3000 rad/s.
    - West Point default lat/lon origin so BF's GPS isn't fed (0, 0) which
      some FC firmwares reject.
    """
    bf_host: str = "127.0.0.1"
    # Sim→BF: BF listens here for FDM sensor data (BF source: PORT_STATE=9003).
    fdm_port: int = 9003
    # BF→Sim: BF sends motor commands here. We bind to receive (PORT_PWM=9002).
    motor_port: int = 9002

    # Iris hover at 880 KV × 11.1 V × (2π/60) ≈ 1023 rad/s. Multirotor
    # PWM-normalized → ω is `motor[i] * rotor_max_omega`.
    rotor_max_omega: float = 1023.0
    num_rotors: int = 4

    # Lockstep recv timeout. >>> physics_dt so we wait a few ticks before
    # giving up; small enough that a real failure still surfaces fast.
    recv_timeout_s: float = 1.0

    # Reference altitude (sea-level barometric pressure source). Pegasus
    # world (x, y, z=0) is treated as NED (0, 0, 0) at this altitude. The
    # FDM packet sends position as METERS NED — BF derives GPS lat/lon
    # internally from MAG_DECLINATION + a configured home, not from this
    # field directly.
    origin_alt_m: float = 100.0
    pressure_sea_level_pa: float = _ISA_SEA_LEVEL_PA

    # Phase-6 mag-NONE drift simulation (rad/s constant bias added to gyro
    # before sending to BF). Default 0 = noiseless. ~0.0003 ≈ 1°/min drift.
    gyro_bias_drift_rad_s: float = 0.0


# ── Wire-format helpers ─────────────────────────────────────────────────────

# 18 little-endian doubles. Field count comes from BF source
# `src/main/target/SITL/sitl.c` `fdm_packet` struct:
#   double timestamp;                       // 1
#   double imu_angular_velocity_rpy[3];     // 3 — body FRD rad/s
#   double imu_linear_acceleration_xyz[3];  // 3 — body FRD m/s²
#   double imu_orientation_quat[4];         // 4 — world→body NED-FRD wxyz
#   double velocity_xyz[3];                 // 3 — ENU world m/s
#   double position_xyz[3];                 // 3 — lon, lat, alt
#   double pressure;                        // 1 — barometer Pa
# Total: 1+3+3+4+3+3+1 = 18 doubles = 144 B. The earlier research summary
# claimed 13 doubles (104 B); that arithmetic was wrong, the field list
# itself was right.
_FDM_STRUCT = struct.Struct("<18d")
assert _FDM_STRUCT.size == 144, f"fdm_packet must be 144B, got {_FDM_STRUCT.size}"

_MOTOR_STRUCT = struct.Struct("<4f")
assert _MOTOR_STRUCT.size == 16, f"servo_packet must be 16B, got {_MOTOR_STRUCT.size}"


def _quat_pegasus_to_bf(q_xyzw_ned_frd):
    """Pegasus stores quats in [qx, qy, qz, qw] (xyzw); BF expects
    [qw, qx, qy, qz] (wxyz). Reorder explicitly — no implicit casts so a
    future refactor can't silently flip the convention."""
    qx, qy, qz, qw = q_xyzw_ned_frd
    return (qw, qx, qy, qz)


def _state_attitude_ned_frd(state) -> np.ndarray:
    """Return state.attitude in NED-FRD (xyzw). Use Pegasus's helper if
    available, else compute it from the (xyzw, ENU/FLU) attitude on the
    state. Duck-typed so we can pass a fake state object in tests."""
    if hasattr(state, "get_attitude_ned_frd"):
        return state.get_attitude_ned_frd()
    # Manual conversion: FLU body → ENU world is what state.attitude
    # encodes. NED-FRD is a pi-rotation about the north/east axis from ENU,
    # plus a pi-rotation about the body-x axis for FLU→FRD. The composite
    # under quaternion multiplication is most readable as scipy Rotation:
    rot_enu_to_ned = Rotation.from_matrix([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
    r_body_to_world_enu = Rotation.from_quat(state.attitude)
    r_body_to_world_ned = rot_enu_to_ned * r_body_to_world_enu * _FLU_TO_FRD
    return r_body_to_world_ned.as_quat()


def _state_angular_velocity_frd(state) -> np.ndarray:
    """Body-frame ω in FRD. Pegasus state stores it in FLU."""
    if hasattr(state, "get_angular_velocity_frd"):
        return state.get_angular_velocity_frd()
    return _FLU_TO_FRD.apply(state.angular_velocity)


def _state_linear_acceleration_body_frd(state) -> np.ndarray:
    """Body-frame acceleration in FRD matching BF's "+g on z-down at rest"
    accelerometer convention.

    Math: combine kinematic acceleration with the gravity vector in WORLD
    coordinates first, then rotate to body. Gravity in ENU is (0, 0, -g)
    (down = -z in ENU). Rotating the combined vector via R_world_to_body_FLU
    then FLU→FRD gives:
      - At rest level: (0, 0, +g) FRD ✓
      - Pitched 30° nose-up at rest: (+g·sin30, 0, +g·cos30) FRD
        — gravity tilts forward into +x as the drone tilts back
      - Accelerating forward 1 m/s² at level: (1, 0, +g) FRD

    Adding gravity in body AFTER rotation (the obvious-but-wrong approach)
    silently produces the level answer regardless of attitude.
    """
    accel_world_enu_plus_grav = (
        state.linear_acceleration + np.array([0.0, 0.0, -_GRAVITY])
    )
    r_world_to_body_flu = Rotation.from_quat(state.attitude).inv()
    accel_body_flu = r_world_to_body_flu.apply(accel_world_enu_plus_grav)
    return _FLU_TO_FRD.apply(accel_body_flu)


def _enu_to_ned_meters(
    x_e: float, y_n: float, z_u: float,
) -> tuple[float, float, float]:
    """Pegasus ENU (east, north, up) meters → BF NED (north, east, down) meters.
    Pure axis swap + z-flip; no Earth-radius math because BF's `position_xyz`
    is local meters from origin, not lon/lat/alt."""
    return y_n, x_e, -z_u


def _altitude_to_pressure_pa(alt_m: float, sea_level_pa: float = _ISA_SEA_LEVEL_PA) -> float:
    """ISA altimeter formula. Inverts to alt within ~0.5% over 0-5 km."""
    return sea_level_pa * (1.0 - _ISA_LAPSE_K_PER_M * alt_m) ** _ISA_EXPONENT


# ── Backend ─────────────────────────────────────────────────────────────────


class BetaflightUdpBackend:
    """Pegasus-compatible Backend (duck-typed; we don't import Pegasus's ABC
    here so the module can be loaded + tested without Pegasus installed).

    Pegasus calls (per `pegasus.simulator.logic.vehicles.multirotor.update`):
      1. `input_reference()`  — return desired rotor ω
      2. (forces applied)
      3. `update(dt)`         — chance to do I/O

    State and sensor callbacks (`update_state`, `update_sensor`) fire from
    the vehicle update loop on the SAME thread, so we don't need locks.

    Lifecycle:
      - `initialize(vehicle)` — one-time, store vehicle ref (we don't use it)
      - `start()` — open sockets, seed motor cache, set sim_time = 0
      - per physics step: `update_state()` → `input_reference()` → `update(dt)`
      - `stop()` — close sockets
      - `reset()` — reset clock, drain stale packets
    """

    def __init__(self, config: Optional[BetaflightBackendConfig] = None):
        self.config = config or BetaflightBackendConfig()
        self._vehicle = None
        self._sock: Optional[socket.socket] = None
        self._latest_state = None
        self._latest_motor: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
        self._sim_time: float = 0.0
        self._timeouts: int = 0
        self._packets_rx: int = 0
        self._packets_tx: int = 0
        # Cache the gyro bias as a vector so we don't reallocate per tick.
        self._gyro_bias = np.zeros(3)

    # ── Pegasus Backend interface ───────────────────────────────────────

    def initialize(self, vehicle) -> None:
        self._vehicle = vehicle

    def start(self) -> None:
        """Open the UDP socket and seed buffers. Safe to call multiple times."""
        if self._sock is not None:
            return
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # Bind to the well-known motor_port so BF SITL's auto-reply (or any
        # configured destination matching this port) reaches us.
        s.bind(("", self.config.motor_port))
        s.settimeout(self.config.recv_timeout_s)
        # Windows: by default an ICMP port-unreachable on a prior send
        # causes the next recvfrom to raise ConnectionResetError. The ioctl
        # below disables that. Best-effort — Python's socket.ioctl rejects
        # arbitrary control codes as "invalid ioctl command" on some Python
        # versions, and the ioctl doesn't exist on Linux/macOS at all.
        # Either way, our update() loop catches ConnectionResetError as a
        # fallback so this is a perf optimization, not a correctness gate.
        try:
            SIO_UDP_CONNRESET = 0x9800000C  # noqa: N806
            s.ioctl(SIO_UDP_CONNRESET, False)
        except (AttributeError, OSError, ValueError):
            pass
        self._sock = s
        self._latest_motor = (0.0, 0.0, 0.0, 0.0)
        self._sim_time = 0.0
        self._timeouts = 0
        self._packets_rx = 0
        self._packets_tx = 0
        self._gyro_bias = np.array([self.config.gyro_bias_drift_rad_s] * 3)
        log.info(
            "BetaflightUdpBackend started: bind 0.0.0.0:%d, send→%s:%d",
            self.config.motor_port, self.config.bf_host, self.config.fdm_port,
        )

    def stop(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
        log.info(
            "BetaflightUdpBackend stopped: tx=%d rx=%d timeouts=%d",
            self._packets_tx, self._packets_rx, self._timeouts,
        )

    def reset(self) -> None:
        self._sim_time = 0.0
        self._latest_motor = (0.0, 0.0, 0.0, 0.0)
        # Drain any stale packets that arrived during a paused world.
        if self._sock is not None:
            try:
                self._sock.setblocking(False)
                while True:
                    self._sock.recvfrom(64)
            except (BlockingIOError, OSError):
                pass
            finally:
                if self._sock is not None:
                    self._sock.settimeout(self.config.recv_timeout_s)

    def update_state(self, state) -> None:
        """Pegasus calls this once per physics step with the current vehicle
        state (ENU/FLU). We cache it for `update(dt)` to pack into FDM."""
        self._latest_state = state

    def update_sensor(self, sensor_type: str, data) -> None:
        """BF SITL gets its IMU/GPS from the FDM packet, not from Pegasus's
        sensor pipeline. Ignored here — but we keep the method so Pegasus's
        sensor dispatch doesn't crash on AttributeError."""
        return

    def update_graphical_sensor(self, sensor_type: str, data) -> None:
        return

    def update(self, dt: float) -> None:
        """Send FDM packet, block-receive motor packet, cache result."""
        if self._sock is None or self._latest_state is None:
            # No state yet (very first tick of cold start) or socket closed.
            return
        self._sim_time += dt
        try:
            fdm = self._pack_fdm(self._latest_state, self._sim_time)
            self._sock.sendto(fdm, (self.config.bf_host, self.config.fdm_port))
            self._packets_tx += 1
        except OSError as e:
            log.warning("FDM send failed: %s", e)
            return
        try:
            buf, _addr = self._sock.recvfrom(64)
            if len(buf) < _MOTOR_STRUCT.size:
                # Truncated packet — ignore, hold last command.
                return
            self._latest_motor = _MOTOR_STRUCT.unpack_from(buf, 0)
            self._packets_rx += 1
        except socket.timeout:
            # Hold last motor command. NEVER zero out — would crash a
            # hovering quad on a single dropped packet.
            self._timeouts += 1
            if self._timeouts == 50:
                log.warning("BF SITL motor packet timeout #50 — check container")
        except ConnectionResetError as e:
            # Windows fallback (in case SIO_UDP_CONNRESET ioctl was rejected):
            # treat ICMP-unreachable as a timeout, not a fatal error.
            self._timeouts += 1
            if self._timeouts == 1:
                log.debug("recvfrom raised ConnectionResetError "
                          "(SIO_UDP_CONNRESET ioctl may have failed): %s", e)
        except OSError as e:
            self._timeouts += 1
            log.warning("recvfrom OSError: %s", e)

    def input_reference(self) -> list[float]:
        """Return the most recently received rotor ω (rad/s). Called BEFORE
        `update(dt)` each tick (Pegasus multirotor.py:113), so the first
        tick returns the start()-seeded zeros."""
        scale = self.config.rotor_max_omega
        return [m * scale for m in self._latest_motor]

    # ── Internals ───────────────────────────────────────────────────────

    def _pack_fdm(self, state, sim_time: float) -> bytes:
        """Convert Pegasus State (ENU/FLU) to BF SITL fdm_packet bytes.

        Raises ValueError on a state with non-finite fields — better than
        silently feeding NaN to the FC.
        """
        omega_frd = _state_angular_velocity_frd(state) + self._gyro_bias
        accel_frd = _state_linear_acceleration_body_frd(state)
        q_xyzw_ned_frd = _state_attitude_ned_frd(state)
        qw, qx, qy, qz = _quat_pegasus_to_bf(q_xyzw_ned_frd)

        # Velocity: Pegasus ENU (east, north, up) → BF NED (north, east, down).
        v_n, v_e, v_d = _enu_to_ned_meters(
            state.linear_velocity[0], state.linear_velocity[1], state.linear_velocity[2],
        )
        # Position: same axis swap. NED meters from local origin (NOT lon/lat).
        p_n, p_e, p_d = _enu_to_ned_meters(
            state.position[0], state.position[1], state.position[2],
        )
        # Pressure derived from altitude above the configured sea-level
        # reference. Pegasus world z=0 is at origin_alt_m; positive z (up)
        # → higher altitude → lower pressure.
        altitude_m = self.config.origin_alt_m + state.position[2]
        pressure = _altitude_to_pressure_pa(altitude_m, self.config.pressure_sea_level_pa)

        values = (
            sim_time,
            float(omega_frd[0]), float(omega_frd[1]), float(omega_frd[2]),
            float(accel_frd[0]), float(accel_frd[1]), float(accel_frd[2]),
            qw, qx, qy, qz,
            v_n, v_e, v_d,
            p_n, p_e, p_d,
            pressure,
        )
        # 1 + 3 + 3 + 4 + 3 + 3 + 1 = 18, matches _FDM_STRUCT size (144 B).
        if not _all_finite(values):
            raise ValueError(f"non-finite values in FDM packet: {values}")
        return _FDM_STRUCT.pack(*values)


def _all_finite(values) -> bool:
    """True iff every value in `values` is a finite number."""
    for v in values:
        if not math.isfinite(v):
            return False
    return True
