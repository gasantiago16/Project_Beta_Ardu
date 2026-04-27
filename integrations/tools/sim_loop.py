"""Closed-loop Iris quadrotor sim for BF SITL.

BF SITL needs an FDM source for sensors to update — without it, MSP_ALTITUDE
and MSP_ATTITUDE return frozen values, so mission_demo's bearing controller
sees the drone as motionless even when motors are spinning. A static
fake_pegasus_loop hover-state isn't enough: the altitude / attitude must
reflect what the motors are actually doing.

This is a minimal 6-DOF Iris model that closes the loop:

  BF SITL -- motor[4] UDP 9002 --> motor_relay -- UDP 38500 --> sim_loop
                                                                    |
                                                            integrate physics
                                                                    v
  BF SITL <-- fdm_packet  UDP 9003 ---------------------- send via backend

Iris params (matching Pegasus's default airframe + BetaflightBackendConfig
rotor_max_omega=1023):
  mass         = 1.5 kg
  arm          = 0.13 m  (motor-to-center)
  I_xx, I_yy   = 0.029  kg·m²
  I_zz         = 0.055  kg·m²
  k_T          = 8.55e-6 N/(rad/s)²       (4·kT·ω_max² ≈ 36 N peak thrust)
  k_Q          = 1.6e-7  N·m/(rad/s)²
  drag_lin     = 0.10 / s         (linear translational damping)
  drag_ang     = 0.10 / s         (angular damping)

BF QuadX motor mapping (motor index 0-3 in the servo_packet matches BF's
internal order, NOT physical motor number):
  idx 0 (BF "M1"): back-right    — CCW   — body (x=-Lx, y=+Ly)
  idx 1 (BF "M2"): front-right   — CW    — body (x=+Lx, y=+Ly)
  idx 2 (BF "M3"): rear-left     — CW    — body (x=-Lx, y=-Ly)
  idx 3 (BF "M4"): front-left    — CCW   — body (x=+Lx, y=-Ly)
where Lx = Ly = arm·cos(45°). Roll/pitch/yaw torques computed from these.

Run:
  T1: docker compose -f sitl/docker-compose.yml up -d
  T2: python -m integrations.tools.sim_loop
  T3: python -m integrations.tools.bf_gps_shim
  T4: python -m integrations.tools.mission_demo --bf-port 5762
"""
from __future__ import annotations

import argparse
import logging
import math
import sys
import time
from dataclasses import dataclass, field

import numpy as np
from scipy.spatial.transform import Rotation

from integrations.pegasus_betaflight_backend import (
    BetaflightBackendConfig,
    BetaflightUdpBackend,
)

log = logging.getLogger("sim_loop")

GRAVITY = 9.806_65


@dataclass
class IrisParams:
    """Iris quadrotor physical params. Tuned so hover sits at throttle ≈
    1640 PWM (motor norm ≈ 0.66): T = m·g = 14.7 N, per motor = 3.68 N,
    needs ω = sqrt(3.68 / 8.55e-6) ≈ 656 rad/s ≈ 0.64·ω_max."""
    mass_kg: float = 1.5
    arm_m: float = 0.13
    inertia_xx: float = 0.029
    inertia_yy: float = 0.029
    inertia_zz: float = 0.055
    k_thrust: float = 8.55e-6
    k_torque: float = 1.6e-7
    omega_max: float = 1023.0
    drag_lin: float = 0.10
    drag_ang: float = 0.10
    # Ground constraint: don't let z drop below 0. World ENU.
    ground_z: float = 0.0


@dataclass
class SimState:
    """Pegasus-shaped state object that the backend can pack into FDM.
    All fields ENU world / FLU body."""
    position: np.ndarray = field(
        default_factory=lambda: np.array([0.0, 0.0, 0.0])
    )
    attitude: np.ndarray = field(
        default_factory=lambda: np.array([0.0, 0.0, 0.0, 1.0])  # xyzw, identity
    )
    linear_velocity: np.ndarray = field(
        default_factory=lambda: np.zeros(3)
    )
    linear_body_velocity: np.ndarray = field(
        default_factory=lambda: np.zeros(3)
    )
    angular_velocity: np.ndarray = field(
        default_factory=lambda: np.zeros(3)
    )
    linear_acceleration: np.ndarray = field(
        default_factory=lambda: np.zeros(3)
    )


class IrisSim:
    """6-DOF Iris quadrotor integrator. State in ENU/FLU."""

    def __init__(self, params: IrisParams):
        self.p = params
        self.state = SimState()
        # Inertia matrix + inverse, body frame.
        self._I = np.diag([params.inertia_xx, params.inertia_yy, params.inertia_zz])
        self._I_inv = np.linalg.inv(self._I)
        # Motor positions in body FLU. Each at distance arm/√2 along x and y.
        L = params.arm_m / math.sqrt(2.0)
        # FLU axes: +x forward, +y left, +z up. Convert from BF FRD layout:
        # In BF "M1 back-right", FRD x is forward, y is right, so back-right
        # is (x=-L, y=+L) in FRD = (x=-L, y=-L) in FLU (y flipped).
        # m1=BR, m2=FR, m3=RL, m4=FL.
        self._motor_pos_flu = np.array([
            [-L, -L, 0.0],   # m1 back-right
            [+L, -L, 0.0],   # m2 front-right
            [-L, +L, 0.0],   # m3 rear-left
            [+L, +L, 0.0],   # m4 front-left
        ])
        # Spin direction: +1 for CCW (positive z torque on body), -1 CW.
        # BF QuadX: m1 CCW, m2 CW, m3 CW, m4 CCW.
        self._motor_dir = np.array([+1.0, -1.0, -1.0, +1.0])

    def step(self, motor_norm: tuple[float, float, float, float], dt: float) -> None:
        """Advance one tick. motor_norm in [0, 1]."""
        omega = np.clip(np.array(motor_norm, dtype=np.float64), 0.0, 1.0) * self.p.omega_max
        thrust_per_motor = self.p.k_thrust * omega ** 2  # N, body +z direction
        total_thrust = float(thrust_per_motor.sum())

        # Torques from differential thrust (Mx, My) and reaction (Mz).
        # M_x = sum(thrust_i * y_i)   — roll torque
        # M_y = sum(-thrust_i * x_i)  — pitch torque (right-hand rule, +pitch=nose-up)
        # M_z = sum(direction_i * k_Q * ω_i²)
        Mx = float(np.sum(thrust_per_motor * self._motor_pos_flu[:, 1]))
        My = float(np.sum(-thrust_per_motor * self._motor_pos_flu[:, 0]))
        Mz = float(np.sum(self._motor_dir * self.p.k_torque * omega ** 2))
        torque_body = np.array([Mx, My, Mz])

        # Angular dynamics in body frame (FLU).
        omega_body = self.state.angular_velocity
        omega_dot = self._I_inv @ (
            torque_body - np.cross(omega_body, self._I @ omega_body)
            - self.p.drag_ang * (self._I @ omega_body)
        )
        omega_body_new = omega_body + omega_dot * dt

        # Update attitude via quaternion integration. attitude is body→world.
        rot = Rotation.from_quat(self.state.attitude)  # xyzw
        # Rotate omega_body by attitude to get world-frame rate? No — for
        # quat update we use body rates directly via the kinematic equation
        # q_dot = 0.5 * q ⊗ (0, ω_body)
        qx, qy, qz, qw = self.state.attitude
        wx, wy, wz = omega_body
        q_dot = 0.5 * np.array([
            qw * wx + qy * wz - qz * wy,
            qw * wy - qx * wz + qz * wx,
            qw * wz + qx * wy - qy * wx,
            -qx * wx - qy * wy - qz * wz,
        ])
        new_quat = self.state.attitude + q_dot * dt
        new_quat /= np.linalg.norm(new_quat)

        # Linear dynamics. Thrust acts along body +z; convert to world.
        thrust_body = np.array([0.0, 0.0, total_thrust])  # FLU: +z up
        thrust_world = rot.apply(thrust_body)
        # Gravity in ENU world: (0, 0, -g)
        gravity_world = np.array([0.0, 0.0, -self.p.mass_kg * GRAVITY])
        # Linear drag (proportional to velocity)
        drag_world = -self.p.drag_lin * self.state.linear_velocity * self.p.mass_kg

        force_world = thrust_world + gravity_world + drag_world
        accel_world = force_world / self.p.mass_kg
        new_vel = self.state.linear_velocity + accel_world * dt
        new_pos = self.state.position + new_vel * dt

        # Ground constraint: clamp z, kill downward velocity if on ground.
        if new_pos[2] < self.p.ground_z:
            new_pos[2] = self.p.ground_z
            if new_vel[2] < 0:
                new_vel[2] = 0.0
            # Damp angular rates a bit on ground contact (skids)
            omega_body_new *= 0.5

        # Body-frame velocity (used by some Pegasus state consumers).
        new_body_vel = rot.inv().apply(new_vel)

        # Commit.
        self.state.position = new_pos
        self.state.attitude = new_quat
        self.state.linear_velocity = new_vel
        self.state.linear_body_velocity = new_body_vel
        self.state.angular_velocity = omega_body_new
        self.state.linear_acceleration = accel_world


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--bf-host", default="127.0.0.1")
    p.add_argument("--fdm-port", type=int, default=9003)
    p.add_argument("--motor-port", type=int, default=38500)
    p.add_argument("--rate-hz", type=float, default=200.0,
                   help="Sim integration rate. 200 Hz keeps angular dynamics "
                        "stable; reduce only if CPU-bound.")
    p.add_argument("--log-period-s", type=float, default=2.0)
    p.add_argument("--log-level", default="INFO")
    args = p.parse_args()

    logging.basicConfig(
        level=args.log_level.upper(),
        format="%(asctime)s %(levelname)s %(name)s %(message)s",
    )

    cfg = BetaflightBackendConfig(
        bf_host=args.bf_host,
        fdm_port=args.fdm_port,
        motor_port=args.motor_port,
        recv_timeout_s=0.5,
    )
    backend = BetaflightUdpBackend(cfg)
    backend.start()

    sim = IrisSim(IrisParams())
    backend.update_state(sim.state)
    log.info("sim_loop started (%.0f Hz). BF→motor on UDP %d, sim→BF FDM on UDP %d",
             args.rate_hz, args.motor_port, args.fdm_port)

    dt = 1.0 / args.rate_hz
    last_log = time.monotonic()
    last_t = time.monotonic()
    next_tick = last_t

    try:
        while True:
            now = time.monotonic()
            sleep_for = next_tick - now
            if sleep_for > 0:
                time.sleep(sleep_for)

            actual_dt = max(1e-4, time.monotonic() - last_t)
            last_t = time.monotonic()

            # Read latest motor commands from BF via the backend's recv thread.
            with backend._rx_lock:
                motors = backend._latest_motor

            # Integrate.
            sim.step(motors, actual_dt)

            # Push new state to backend, send FDM packet.
            backend.update_state(sim.state)
            backend.update(actual_dt)

            now = time.monotonic()
            if now - last_log >= args.log_period_s:
                last_log = now
                p_ = sim.state.position
                v_ = sim.state.linear_velocity
                # Roll/pitch/yaw from quaternion (world-frame yaw).
                rot = Rotation.from_quat(sim.state.attitude)
                roll, pitch, yaw = rot.as_euler("xyz", degrees=True)
                stats = backend.stats()
                log.info(
                    "[sim] pos=(%+6.2f, %+6.2f, %+6.2f)m vel=(%+5.2f, %+5.2f, %+5.2f)m/s "
                    "att=(r%+5.1f° p%+5.1f° y%+5.1f°) "
                    "motors=(%.2f %.2f %.2f %.2f) | tx=%d rx=%d",
                    p_[0], p_[1], p_[2], v_[0], v_[1], v_[2],
                    roll, pitch, yaw,
                    motors[0], motors[1], motors[2], motors[3],
                    stats["tx"], stats["rx"],
                )

            next_tick += dt
            # Watchdog: if we drift more than 100 ms behind, reset schedule
            # so we don't burn CPU trying to catch up.
            if next_tick < time.monotonic() - 0.1:
                next_tick = time.monotonic()
    except KeyboardInterrupt:
        log.info("sim_loop stopped")
    finally:
        backend.stop()

    return 0


if __name__ == "__main__":
    sys.exit(main())
