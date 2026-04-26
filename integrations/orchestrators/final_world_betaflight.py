"""Pegasus + Iris + Final_World + Betaflight SITL — full Phase 2 stack.

Canonical version lives here (in the Project_Beta_Ardu repo). The Pegasus
tree under `~/PegasusSimulator/` is a third-party checkout we don't track,
so the script needs to be runnable from this location too — Pegasus is
pip-installed (or otherwise on PYTHONPATH) once the Isaac Sim extension
manager has loaded `pegasus.simulator`.

Three independent processes, three terminals:

  T1>  cd Project_Beta_Ardu && docker compose -f sitl/docker-compose.yml up
  T2>  cd ~/PegasusSimulator
       python ~/Project_Beta_Ardu/integrations/orchestrators/final_world_betaflight.py
  T3>  (Phase 3) python -m racer_companion.main --config sim_waypoint.json
       OR (Phase 4) python -m tools.radio_to_bf

Note: T2's `cd ~/PegasusSimulator` matters only if Pegasus needs the cwd
to resolve its asset paths; if you run from elsewhere and asset loading
fails, that's the fix.

What this script does:
1. Pre-flight: TCP-connect to BF SITL's MSP port (5761) and bail with a
   clear message if not reachable. Most launch failures are "forgot to
   docker compose up" — make that obvious instead of letting Pegasus
   start, then time out 5 s later with a confusing socket error.
2. Loads Final_World_edge_clipped.scene.usd as a USD reference (proven
   pattern from teleop_final_world.py:147-149).
3. Spawns an Iris quadrotor at the proven road-junction spawn (40.7, 17.1,
   1.3) — 1 m above the SM_RoadJunction_Plus28 asphalt center.
4. Attaches a BetaflightUdpBackend (from Project_Beta_Ardu/integrations/)
   that exchanges fdm_packet (UDP 9003 → BF) and servo_packet (UDP 38500
   ← in-container socat relay forwarding BF's hardcoded 127.0.0.1:9002
   output — see sitl/start.sh for the why) with the BF SITL container.
5. Periodic stats logging every 5 s so it's visually obvious the lockstep
   is working (or stalling).

Without arming/throttling BF (via QGC joystick widget, racer_companion MSP
override, or radio_to_bf RC injection in Phase 4), the Iris will sit on
the asphalt — that's the expected Phase 2 success state. The point of this
script is to prove the wire is working; it's not yet a flight test.

Success signal: bridge stats (printed every --stats-interval-s seconds)
show steady non-zero `rx` and near-zero `timeouts`. The TCP-probe pre-flight
only proves the BF binary is up — it can pass while the UDP path is broken
(wrong simulator_ip, host firewall blocks UDP 9002 inbound, etc.). Watch
the bridge stats; rx going up is the real success signal.

Linux without Docker Desktop: pass `--bf-host <docker-bridge-IP>` (e.g.,
172.17.0.1) and rely on the `extra_hosts: host-gateway` mapping in
docker-compose.yml so BF SITL inside the container can resolve
`host.docker.internal` to the host bridge.

If the bridge stats show high timeouts: see `integrations/tools/fake_pegasus
_loop.py` module docstring for likely causes (port mismatch, simulator_ip
not reachable from inside Docker, etc.).
"""
import argparse
import os
import socket
import sys
import time
from pathlib import Path

# Windows console default codec is cp1252, which can't encode the Unicode
# arrows / em-dashes / Greek letters this script prints. Switch stdout +
# stderr to UTF-8 before any print() runs so we don't crash on the very
# first status line.
for stream in (sys.stdout, sys.stderr):
    if hasattr(stream, "reconfigure"):
        stream.reconfigure(encoding="utf-8", errors="replace")

from isaacsim import SimulationApp
# `carb` and friends are only importable AFTER SimulationApp() boots the
# Kit/Carb runtime; we defer those imports until after `simulation_app`
# is constructed below.

# ── CLI args (parse before SimulationApp init per Isaac Sim convention) ────

parser = argparse.ArgumentParser()
parser.add_argument(
    "--usd",
    default=os.path.expanduser(
        "~/OneDrive/Desktop/Collected_Final_World/Final_World_edge_clipped.scene.usd"
    ),
    help="Path to Final_World USD",
)
parser.add_argument(
    "--spawn", type=float, nargs=3, default=None,
    help="Quad spawn X Y Z (meters, ENU). Default depends on --corner; "
         "explicit --spawn overrides --corner.",
)
parser.add_argument(
    "--corner", default="ROADJUNCTION",
    choices=["ROADJUNCTION", "SW", "SE", "NE", "NW"],
    help="Named spawn point. ROADJUNCTION (default) = SM_RoadJunction_Plus28 "
         "+ 1m alt = (40.7, 17.1, 1.3) — proven open spot. SW/SE/NE/NW "
         "compute corners as ROADJUNCTION ± map-half-size on each axis "
         "(useful for the mission_demo which expects to start at a corner).",
)
parser.add_argument(
    "--map-half-size", type=float, default=80.0,
    help="Half-side of the square map in meters. Used to compute the "
         "SW/SE/NE/NW spawn points (not loaded into the USD).",
)
parser.add_argument(
    "--bf-host", default="127.0.0.1",
    help="Host where BF SITL is reachable (default 127.0.0.1 = host docker mappings).",
)
parser.add_argument(
    "--bf-msp-port", type=int, default=5761,
    help="BF MSP TCP port — used for the pre-flight reachability check only.",
)
parser.add_argument(
    "--skip-preflight", action="store_true",
    help="Skip the TCP MSP reachability check. Use if BF SITL is on a different host.",
)
parser.add_argument(
    "--integrations-dir",
    default=os.path.expanduser("~/Project_Beta_Ardu/integrations"),
    help="Path to Project_Beta_Ardu/integrations (where pegasus_betaflight_backend.py lives).",
)
parser.add_argument(
    "--airframe", default="iris",
    choices=["iris", "racer5"],
    help="Which airframe profile to use. `iris` (default) = Pegasus default, "
         "F450-class slow heavy quad — use to validate the bridge. "
         "`racer5` = 5\" FPV race quad — requires `assets/racer_5in/racer_5in.usd` "
         "to be generated first (see assets/racer_5in/README.md).",
)
parser.add_argument(
    "--stats-interval-s", type=float, default=5.0,
    help="Seconds between bridge stats prints. 0 disables.",
)
parser.add_argument(
    "--motor-port", type=int, default=0,
    help="Host UDP port the bridge listens on for BF motor packets. 0 = "
         "pick a fresh ephemeral high port (Windows Defender Firewall has "
         "been observed to silently block long-lived UDP ports on this "
         "machine; rotating each run sidesteps the heuristic). Whatever "
         "we pick is then pushed into the SITL container via "
         "SITL_HOST_MOTOR_PORT and the relay is restarted.",
)
args = parser.parse_args()


def _pick_motor_port() -> int:
    """Bind ephemeral UDP, get the assigned port, close the socket.
    The kernel picks a high-port-range slot we haven't used recently —
    much less likely to be in any Defender Firewall blocklist than a
    well-known port we've been re-binding for an hour."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.bind(("", 0))
        port = s.getsockname()[1]
        return port
    finally:
        s.close()


def _preflight_or_die() -> None:
    """Verify BF SITL is reachable on its MSP TCP port. Bail before opening
    Isaac Sim if not — Isaac startup is 30+ s and a missing SITL container
    is the most common reason this script fails to fly."""
    if args.skip_preflight:
        return
    print(
        f"[final_world_betaflight] Pre-flight: TCP-connect "
        f"{args.bf_host}:{args.bf_msp_port}...",
        flush=True,
    )
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(2.0)
    try:
        s.connect((args.bf_host, args.bf_msp_port))
    except (ConnectionRefusedError, socket.timeout, OSError) as e:
        print(
            f"[final_world_betaflight] FATAL: BF SITL not reachable at "
            f"{args.bf_host}:{args.bf_msp_port} ({type(e).__name__}: {e}).\n"
            f"  Did you `docker compose -f sitl/docker-compose.yml up`?\n"
            f"  Override with --bf-host or --skip-preflight if SITL is elsewhere.",
            file=sys.stderr,
        )
        sys.exit(2)
    finally:
        s.close()
    print("[final_world_betaflight] Pre-flight: BF SITL MSP up.", flush=True)


def _resolve_spawn_or_die() -> None:
    """Compute final spawn point. --spawn wins over --corner. Mutates args."""
    if args.spawn is not None:
        return
    rx, ry, rz = (40.7, 17.1, 1.3)  # SM_RoadJunction_Plus28
    h = args.map_half_size
    args.spawn = {
        "ROADJUNCTION": [rx, ry, rz],
        "SW": [rx - h, ry - h, rz],
        "SE": [rx + h, ry - h, rz],
        "NE": [rx + h, ry + h, rz],
        "NW": [rx - h, ry + h, rz],
    }[args.corner]
    print(f"[final_world_betaflight] Spawn from --corner={args.corner} "
          f"+ map_half_size={h}m → {tuple(args.spawn)}", flush=True)


def _validate_paths_or_die() -> None:
    if not os.path.exists(args.usd):
        print(
            f"[final_world_betaflight] FATAL: USD not found at {args.usd}",
            file=sys.stderr,
        )
        sys.exit(2)
    if not os.path.isdir(args.integrations_dir):
        print(
            f"[final_world_betaflight] FATAL: integrations dir not found at "
            f"{args.integrations_dir}\n"
            f"  Pass --integrations-dir <path> if Project_Beta_Ardu lives elsewhere.",
            file=sys.stderr,
        )
        sys.exit(2)


# Pre-flight runs BEFORE SimulationApp() so we don't pay the 30s Isaac startup
# just to discover BF isn't running.
_resolve_spawn_or_die()
_validate_paths_or_die()
_preflight_or_die()

# Pick the host motor port BEFORE Isaac warmup so we can re-launch the
# container's relay with the right target port if needed.
if args.motor_port == 0:
    args.motor_port = _pick_motor_port()
    print(
        f"[final_world_betaflight] Picked ephemeral host motor port: {args.motor_port}",
        flush=True,
    )
    # Reconfigure the SITL container's relay to forward to this port.
    # Uses `docker exec` to kill the existing relay and start a fresh
    # one — avoids a full container restart (~5 s vs ~30 s).
    try:
        import subprocess
        subprocess.run(
            ["docker", "exec", "project-beta-ardu-sitl", "bash", "-c",
             f"pkill -f motor_relay.py; sleep 0.3; "
             f"python3 /opt/sitl/motor_relay.py 192.168.65.254 {args.motor_port} >/dev/null 2>&1 &"],
            check=False, timeout=5,
        )
        print(
            f"[final_world_betaflight] Restarted in-container relay -> "
            f"192.168.65.254:{args.motor_port}",
            flush=True,
        )
        time.sleep(0.5)  # let the new relay bind before we send FDM
    except (FileNotFoundError, subprocess.TimeoutExpired) as e:
        print(
            f"[final_world_betaflight] WARN: could not auto-reconfigure "
            f"relay: {e}. Manually set SITL_HOST_MOTOR_PORT={args.motor_port}"
            f" + restart container.",
            file=sys.stderr,
        )
else:
    print(
        f"[final_world_betaflight] Using fixed motor port: {args.motor_port} "
        f"(must match SITL container's SITL_HOST_MOTOR_PORT env)",
        flush=True,
    )

# Make integrations/ importable.
sys.path.insert(0, args.integrations_dir)
# Also expose the Project_Beta_Ardu repo root so we can import
# `integrations.configs.airframes` as a proper package path.
sys.path.insert(0, str(Path(args.integrations_dir).parent))
# Bridge-side smoke import: catch ImportError BEFORE Isaac warm-up.
try:
    from pegasus_betaflight_backend import (  # noqa: E402
        BetaflightBackendConfig,
        BetaflightUdpBackend,
    )
    from integrations.configs.airframes import (  # noqa: E402
        USD_PEGASUS_DEFAULT, get_profile, resolve_usd_path,
    )
except ImportError as e:
    print(
        f"[final_world_betaflight] FATAL: cannot import bridge from "
        f"{args.integrations_dir}: {e}",
        file=sys.stderr,
    )
    sys.exit(2)

profile = get_profile(args.airframe)
print(f"[final_world_betaflight] Airframe: {profile.name} "
      f"({profile.description})", flush=True)
# Resolve race-quad USD existence pre-Isaac so failure surfaces in <1s.
if profile.usd_path != USD_PEGASUS_DEFAULT and not os.path.exists(profile.usd_path):
    print(
        f"[final_world_betaflight] FATAL: airframe USD not found at "
        f"{profile.usd_path}.\n"
        f"  Generate it from the URDF — see assets/racer_5in/README.md.",
        file=sys.stderr,
    )
    sys.exit(2)

# ── Isaac Sim warm-up (slow path starts here) ──────────────────────────────

simulation_app = SimulationApp({
    "headless": False,
    "width": 1920,
    "height": 1080,
})

import omni  # noqa: E402
import omni.timeline  # noqa: E402
# Isaac Sim 4.x: `omni.isaac.core.world.World`. Isaac Sim 5.x renamed to
# `isaacsim.core.api.World`. Try the modern path first; fall back so this
# orchestrator works on both 4.x (where the user's prior teleop_final_world.py
# was written) and 5.1 (current Isaac Sim release).
try:
    from isaacsim.core.api import World  # noqa: E402
except ImportError:
    from omni.isaac.core.world import World  # noqa: E402
from pxr import UsdGeom, UsdLux, Gf  # noqa: E402

from pegasus.simulator.params import ROBOTS  # noqa: E402
from pegasus.simulator.logic.vehicles.multirotor import (  # noqa: E402
    Multirotor, MultirotorConfig,
)
from pegasus.simulator.logic.interface.pegasus_interface import (  # noqa: E402
    PegasusInterface,
)
from pegasus.simulator.logic.thrusters.quadratic_thrust_curve import (  # noqa: E402
    QuadraticThrustCurve,
)


def main() -> int:
    print(f"[final_world_betaflight] USD:    {args.usd}", flush=True)
    print(f"[final_world_betaflight] Spawn:  {tuple(args.spawn)}", flush=True)
    print(f"[final_world_betaflight] BF host: {args.bf_host}", flush=True)

    timeline = omni.timeline.get_timeline_interface()
    pg = PegasusInterface()
    pg._world = World(**pg._world_settings)
    world = pg.world

    # Load Final_World as a USD reference (the proven workaround for local
    # USDs — Pegasus's load_environment() is hardwired to Nucleus paths).
    stage = omni.usd.get_context().get_stage()
    world_prim = stage.DefinePrim("/World/FinalWorld", "Xform")
    world_prim.GetReferences().AddReference(args.usd)
    print("[final_world_betaflight] Final_World loaded as reference", flush=True)

    # Supplement scene lighting so the quad is visible on first render.
    light = UsdLux.DistantLight.Define(stage, "/World/RenderLight")
    light.CreateIntensityAttr(2000.0)
    UsdGeom.Xformable(light.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-45, 30, 0))

    # Build the BF bridge. fdm_port=9003 (BF PORT_STATE), motor_port=38500
    # (in-container socat relay target — BF's hardcoded 9002 stays on
    # container loopback because Windows Defender Firewall blocks 9002).
    # rotor_max_omega is per-airframe (Iris ≈ 1023, race-quad ≈ 3000).
    # `recv_timeout_s=2.0` is used ONLY for the cold-start sync: the
    # bridge blocks on the very first tick until BF replies, then
    # switches to drain-only forever after. Pegasus's physics loop
    # runs at full rate, motor packets are consumed as they appear,
    # hold-last-command between. This is the right pattern for a real
    # flight controller: latency tolerance over hard lockstep.
    bf_cfg = BetaflightBackendConfig(
        bf_host=args.bf_host,
        motor_port=args.motor_port,
        rotor_max_omega=profile.rotor_max_omega,
        num_rotors=profile.num_rotors,
        recv_timeout_s=2.0,
    )
    bf_backend = BetaflightUdpBackend(bf_cfg)

    # Resolve which USD to load. Iris uses Pegasus's bundled path; race-quad
    # profiles point at our repo's assets/.
    usd_path = resolve_usd_path(profile, ROBOTS["Iris"])
    print(f"[final_world_betaflight] Loading airframe USD: {usd_path}", flush=True)

    # Plumb per-airframe thrust coefficients into Pegasus's force model.
    # WITHOUT this, only the USD changes when --airframe is swapped — the
    # thrust curve falls back to Iris-tuned defaults, which combined with
    # a race-quad's lower mass yields T:W ≈ 50 and instant divergence.
    n = profile.num_rotors
    thrust_curve = QuadraticThrustCurve(config={
        "num_rotors": n,
        "rotor_constant": [profile.thrust_coeff] * n,
        "rolling_moment_coefficient": [profile.torque_coeff] * n,
        "min_rotor_velocity": [0.0] * n,
        "max_rotor_velocity": [profile.rotor_max_omega] * n,
    })
    print(f"[final_world_betaflight] Thrust curve: c_T={profile.thrust_coeff:.2e} "
          f"c_Q={profile.torque_coeff:.2e} ω_max={profile.rotor_max_omega:.0f} rad/s",
          flush=True)

    config = MultirotorConfig()
    config.thrust_curve = thrust_curve
    config.backends = [bf_backend]
    Multirotor(
        "/World/quadrotor",
        usd_path,
        0,
        list(args.spawn),
        [0.0, 0.0, 0.0, 1.0],
        config=config,
    )

    world.reset()
    print(
        "[final_world_betaflight] World reset; starting timeline. "
        "BF lockstep begins on first physics tick.",
        flush=True,
    )
    timeline.play()

    last_stats_t = time.monotonic()
    last_stats = bf_backend.stats()

    try:
        while simulation_app.is_running():
            world.step(render=True)

            # Periodic bridge-health log. 0 disables.
            if args.stats_interval_s > 0:
                now = time.monotonic()
                if now - last_stats_t >= args.stats_interval_s:
                    s = bf_backend.stats()
                    dt_s = max(1e-6, now - last_stats_t)
                    delta_tx = s["tx"] - last_stats["tx"]
                    delta_rx = s["rx"] - last_stats["rx"]
                    pct = s["timeouts"] / max(1, s["tx"]) * 100.0
                    print(
                        f"[bridge] tx={s['tx']} rx={s['rx']} "
                        f"timeouts={s['timeouts']} ({pct:.2f}%) "
                        f"| Δ tx={delta_tx} rx={delta_rx} in {dt_s:.1f}s "
                        f"({delta_tx / dt_s:.0f} Hz)",
                        flush=True,
                    )
                    last_stats_t = now
                    last_stats = s
    except KeyboardInterrupt:
        print("[final_world_betaflight] Interrupted", flush=True)
    finally:
        timeline.stop()
        simulation_app.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
