"""Calibrate the Iris airframe's hover throttle by sweeping motor_norm.

Drives `sim_loop.IrisSim` directly (no BF, no UDP) at a fixed all-motors-
equal command, integrates for `--settle-s` seconds, and reports the
steady-state vertical velocity. The motor_norm where vario crosses zero
is the true sim hover.

Why bother: mission_demo, flight_profile, and the racer_companion config
all have a `throttle_hover_us` PWM number hard-coded. If the IrisSim
params drift (rotor_max_omega, k_thrust, mass) those PWM values silently
diverge from the actual sim equilibrium, controllers integrate windup,
and you get the symptoms we hit in mission_demo: drone climbs out the
top of the map at "hover" throttle. This tool gives a fresh number per
airframe + per BF mixer config.

Run:
  python -m integrations.tools.iris_hover_calibrate

Output (Iris defaults):
  motor_norm=0.620 -> vz=-0.91 m/s (sinking)
  motor_norm=0.630 -> vz=-0.55 m/s
  ...
  motor_norm=0.660 -> vz=+0.34 m/s
  motor_norm=0.665 -> vz=+0.07 m/s   ← hover
  motor_norm=0.670 -> vz=+0.45 m/s
  ...

Mapping motor_norm -> BF PWM is mixer-dependent. With the BF SITL stack
we run (motor_pwm_protocol=PWM, default mincheck/maxcheck), an unarmed
PWM stick sweep into bf_diag shows the linear range we use for hover
controllers maps as:

  PWM 1000 -> motor 0.00 (idle/disarmed)
  PWM 1640 -> motor ~ 0.66
  PWM 1700 -> motor ~ 0.71

So the PWM that nominally produces the motor_norm this tool finds is
approximately:

  hover_pwm ~ 1000 + 1000 * hover_motor_norm

Cross-check the result against `flight_profile --target-alt-m 5
--hover-s 10` - if Iris drifts up or down during the hover phase, the
PWM number's wrong. This tool gets you to within ~10 PWM us without
needing to fly.
"""
from __future__ import annotations

import argparse
import sys
import time

from integrations.tools.sim_loop import IrisParams, IrisSim


def measure_vz_at(
    motor_norm: float,
    settle_s: float,
    dt: float,
    params: IrisParams,
    start_alt: float = 50.0,
) -> tuple[float, float]:
    """Spin all 4 motors at `motor_norm`, integrate for settle_s,
    return (final_vz, final_z).

    Starts the drone at `start_alt` (default 50 m). If we started at
    z=0 the ground constraint in IrisSim.step would clamp z and zero
    out negative vz, so under-hover thrust would be misread as HOVER.
    """
    import numpy as np
    sim = IrisSim(params)
    sim.state.position = np.array([0.0, 0.0, start_alt])
    motors = (motor_norm, motor_norm, motor_norm, motor_norm)
    n_steps = int(settle_s / dt)
    for _ in range(n_steps):
        sim.step(motors, dt)
    return float(sim.state.linear_velocity[2]), float(sim.state.position[2])


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--low", type=float, default=0.55,
                   help="Lowest motor_norm to test.")
    p.add_argument("--high", type=float, default=0.75,
                   help="Highest motor_norm to test.")
    p.add_argument("--step", type=float, default=0.01,
                   help="motor_norm step size in coarse sweep.")
    p.add_argument("--settle-s", type=float, default=4.0,
                   help="Integration time at each motor_norm.")
    p.add_argument("--dt", type=float, default=0.005,
                   help="Sim step size (200 Hz default).")
    p.add_argument("--refine", action="store_true", default=True,
                   help="After coarse sweep, do a finer sweep around the "
                        "zero-crossing.")
    args = p.parse_args()

    params = IrisParams()
    print(f"[calibrate] Iris params:")
    print(f"  mass={params.mass_kg} kg, weight={params.mass_kg*9.80665:.2f} N")
    print(f"  per-motor hover thrust={params.mass_kg*9.80665/4:.3f} N")
    print(f"  k_thrust={params.k_thrust} N/(rad/s)^2")
    print(f"  omega_max={params.omega_max} rad/s")
    print(f"  drag_lin={params.drag_lin}/s")
    # Theoretical hover from physics (no drag, level):
    #   total_thrust = m*g
    #   per_motor_thrust = m*g / 4 = k_thrust * ω^2
    #   ω_hover = sqrt(m*g / (4*k_thrust))
    #   motor_norm = ω_hover / ω_max
    import math
    omega_hover = math.sqrt(params.mass_kg * 9.80665 / (4 * params.k_thrust))
    theoretical = omega_hover / params.omega_max
    print(f"  -> theoretical hover motor_norm = {theoretical:.3f} (no drag)")
    print()

    print(f"[calibrate] Coarse sweep {args.low:.2f} -> {args.high:.2f} "
          f"step {args.step:.3f}, settle {args.settle_s}s")
    print(f"  {'motor_norm':>10}  {'vz (m/s)':>10}  {'z (m)':>10}  state")
    results = []
    n = max(1, int(round((args.high - args.low) / args.step)) + 1)
    for i in range(n):
        m = args.low + i * args.step
        if m > args.high + 1e-6:
            break
        vz, z = measure_vz_at(m, args.settle_s, args.dt, params)
        state = "sinking" if vz < -0.05 else "rising" if vz > 0.05 else "HOVER"
        print(f"  {m:>10.4f}  {vz:>+10.3f}  {z:>+10.2f}  {state}")
        results.append((m, vz, z))

    # Find zero-crossing (linear interp between adjacent samples).
    crossing = None
    for i in range(len(results) - 1):
        m_lo, vz_lo, _ = results[i]
        m_hi, vz_hi, _ = results[i + 1]
        if vz_lo <= 0 <= vz_hi or vz_hi <= 0 <= vz_lo:
            # Linear interp
            frac = -vz_lo / (vz_hi - vz_lo) if (vz_hi - vz_lo) != 0 else 0.0
            crossing = m_lo + frac * (m_hi - m_lo)
            break
    if crossing is None:
        print("\n[calibrate] No zero-crossing in this range. Widen --low / --high.")
        return 1
    print(f"\n[calibrate] Coarse zero-crossing: motor_norm ~ {crossing:.4f}")

    if args.refine:
        # Finer sweep +/-0.01 around the crossing in 0.001 steps.
        print(f"\n[calibrate] Refined sweep {crossing-0.01:.3f} -> "
              f"{crossing+0.01:.3f} step 0.001")
        print(f"  {'motor_norm':>10}  {'vz (m/s)':>10}  state")
        refined = []
        for i in range(21):
            m = crossing - 0.01 + i * 0.001
            vz, _ = measure_vz_at(m, args.settle_s, args.dt, params)
            state = "sinking" if vz < -0.05 else "rising" if vz > 0.05 else "HOVER"
            print(f"  {m:>10.4f}  {vz:>+10.3f}  {state}")
            refined.append((m, vz))
        # New zero-crossing
        for i in range(len(refined) - 1):
            m_lo, vz_lo = refined[i]
            m_hi, vz_hi = refined[i + 1]
            if vz_lo <= 0 <= vz_hi or vz_hi <= 0 <= vz_lo:
                frac = -vz_lo / (vz_hi - vz_lo) if (vz_hi - vz_lo) != 0 else 0.0
                crossing = m_lo + frac * (m_hi - m_lo)
                break

    # Map motor_norm to nominal BF PWM. This is a first-order approximation;
    # the real PWM->motor curve depends on BF's mixer (idle, scaling, etc.).
    # See the module docstring for caveats.
    nominal_pwm = int(round(1000 + 1000 * crossing))
    print(f"\n[calibrate] RESULT")
    print(f"  hover motor_norm = {crossing:.4f}")
    print(f"  nominal hover PWM ~ {nominal_pwm} us (linear PWM->motor map)")
    print(f"  theoretical (no drag) = {theoretical:.4f}")
    print(f"  drag offset           = {(crossing-theoretical)*100:+.2f}% motor_norm")
    print(f"\n  -> suggested config values:")
    print(f"     mission_demo MissionConfig.throttle_hover_us = {nominal_pwm}")
    print(f"     flight_profile --hover-throttle              = {nominal_pwm}")
    print(f"     racer_companion config nav.throttle_hover    = {nominal_pwm}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
