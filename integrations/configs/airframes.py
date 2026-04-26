"""Per-airframe profiles for the BF SITL ↔ Pegasus integration.

A profile bundles the three pieces that change together when you swap
airframes:
  1. The USD asset (mesh + rigid-body + rotor positions).
  2. The Pegasus thrust-curve parameters (T = c_T·ω², counter-torque, etc).
  3. The BF SITL backend parameters (rotor_max_omega for PWM→ω scaling).

Items 1 + 2 only matter to the Pegasus orchestrator. Item 3 is consumed by
the bridge backend. Keeping them in one place prevents the silent drift
where someone swaps the URDF but forgets to update rotor_max_omega and
motors blow up in the first hover.

Two profiles ship today: `iris` (Pegasus default, slow F450-class — used
to validate the bridge end-to-end) and `racer5` (5" FPV race quad — used
once the bridge is proven). Adding a third is a copy-paste of one section.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional

# Sentinel: tell the orchestrator to fall back to Pegasus's bundled
# `ROBOTS["Iris"]` USD rather than a path under this repo.
USD_PEGASUS_DEFAULT = "__pegasus_default__"


@dataclass(frozen=True)
class AirframeProfile:
    """Static description of a multirotor airframe + its dynamics tuning.

    Pegasus reads the rigid-body parameters (mass, inertia, joint geometry)
    from the USD itself; we keep `mass_kg` etc. here only for documentation
    + sanity checks. The thrust/torque coefficients DO get plumbed into
    Pegasus's `MultirotorConfig` because Pegasus's force model lives outside
    the USD.
    """
    name: str
    usd_path: str            # absolute path or USD_PEGASUS_DEFAULT
    rotor_max_omega: float   # rad/s — passed to BetaflightBackendConfig
    num_rotors: int = 4

    # Documentation-only (rigid-body comes from USD):
    mass_kg: float = 0.0
    arm_length_m: float = 0.0

    # Thrust + torque model (Pegasus QuadraticThrustCurve):
    thrust_coeff: float = 0.0      # c_T in T = c_T·ω²  (N/(rad/s)²)
    torque_coeff: float = 0.0      # c_Q in Q = c_Q·ω²  (N·m/(rad/s)²)
    motor_time_constant_s: float = 0.025

    description: str = ""


# Path to the racer_5in USD on disk. The .usd is gitignored — the user
# regenerates it from racer_5in.urdf via Isaac Sim's importer. See
# assets/racer_5in/README.md.
_REPO = Path(__file__).resolve().parents[2]
_RACER_5IN_USD = str(_REPO / "assets" / "racer_5in" / "racer_5in.usd")


IRIS = AirframeProfile(
    name="iris",
    usd_path=USD_PEGASUS_DEFAULT,
    rotor_max_omega=1023.0,        # 880 KV × 11.1 V × 2π/60
    num_rotors=4,
    mass_kg=1.5,
    arm_length_m=0.255,
    # Match Pegasus's bundled QuadraticThrustCurve defaults for Iris so
    # `--airframe iris` exactly reproduces stock Iris flight.
    thrust_coeff=8.54858e-6,
    torque_coeff=1.0e-6,
    motor_time_constant_s=0.05,
    description=(
        "Pegasus default. F450-class slow heavy quad, T:W ≈ 2. "
        "Use to validate the BF↔Pegasus bridge end-to-end before swapping."
    ),
)

# Race-quad coefficients — picked so c_T·ω_max² × 4 ≈ T_max with realistic
# T:W ≈ 10 at 0.5 kg AUW, AND c_Q/c_T ≈ 0.012 (empirical 5"-prop ratio).
# Counter-agent caught the original (c_T=6e-7, c_Q=7.4e-6) producing
# T:W ≈ 3.8 (under-thrusted) and c_Q/c_T ≈ 12 (1000× over-yawed). Fix
# moves c_T up, c_Q down by 1e3, and adds the test that pins the ratio.
RACER_5IN = AirframeProfile(
    name="racer5",
    usd_path=_RACER_5IN_USD,
    rotor_max_omega=3000.0,        # 28 700 RPM band — 1300 KV × 6S equivalent
    num_rotors=4,
    mass_kg=0.50,
    arm_length_m=0.125,
    thrust_coeff=1.7e-6,           # T_max ≈ 4 × 1.7e-6 × 3000² ≈ 6.1 kgf, T:W ≈ 10
    torque_coeff=2.0e-8,           # c_Q/c_T ≈ 0.012, typical 5" tri-blade
    motor_time_constant_s=0.025,
    description=(
        "5\" FPV race quad. ~500 g AUW, T:W ≈ 10, race rates. URDF at "
        "assets/racer_5in/racer_5in.urdf; convert to USD per the asset README."
    ),
)


PROFILES: dict[str, AirframeProfile] = {
    IRIS.name: IRIS,
    RACER_5IN.name: RACER_5IN,
}


def get_profile(name: str) -> AirframeProfile:
    """Look up an airframe by name. Raises KeyError with a helpful message
    if the name is unknown (rather than the default cryptic dict KeyError)."""
    try:
        return PROFILES[name]
    except KeyError:
        known = ", ".join(sorted(PROFILES))
        raise KeyError(
            f"Unknown airframe profile {name!r}. Known: {known}. "
            f"Add a new entry in integrations/configs/airframes.py "
            f"if you've authored a new URDF."
        ) from None


def resolve_usd_path(profile: AirframeProfile, pegasus_iris_path: Optional[str] = None) -> str:
    """Resolve the profile's USD path. The Iris profile uses
    `USD_PEGASUS_DEFAULT` as a sentinel; the orchestrator passes
    `pegasus.simulator.params.ROBOTS["Iris"]` for that case. Race-quad
    profiles return their concrete on-disk path."""
    if profile.usd_path == USD_PEGASUS_DEFAULT:
        if pegasus_iris_path is None:
            raise ValueError(
                f"profile {profile.name!r} requires the orchestrator to pass "
                f"the Pegasus-bundled Iris path explicitly."
            )
        return pegasus_iris_path
    return profile.usd_path
