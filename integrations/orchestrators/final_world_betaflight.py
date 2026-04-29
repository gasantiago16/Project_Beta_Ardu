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
import subprocess
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
parser.add_argument(
    "--respawn-signal-file", default=None,
    help="Path to the respawn signal file mission_demo writes when it "
         "aborts due to flip detection. The orchestrator polls for this "
         "file ~10 Hz; on detection calls `world.reset()` to return Iris "
         "to spawn so the operator can rerun mission_demo without "
         "restarting Pegasus. Default: tempdir/bf_respawn.signal — must "
         "match mission_demo's --respawn-signal-file value or the two "
         "processes silently disagree.",
)
parser.add_argument(
    "--fpv-camera", action="store_true",
    help="Enable FPV-style first-person camera mounted on the drone. "
         "Creates a Camera prim under /World/quadrotor that translates "
         "with the drone, simulating the racing-quad pilot view. "
         "Default off (third-person Isaac Sim viewport). Position/"
         "rotation tuned for Iris airframe; tweak in-source if mounting "
         "a different airframe.",
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


def _preflight_udp_or_die(motor_port: int) -> None:
    """Verify container→host UDP delivery on the chosen motor port.

    The TCP pre-flight only proves BF's MSP is reachable from the host. The
    return path (BF's motor packets coming OUT to the host) goes over a
    different transport — UDP, often through Docker Desktop's WSL2 backend
    — and has its own failure modes that look identical to the TCP path
    being fine:

      - VPN active on the Windows host: container→host UDP is blackholed
        even though the relay reports forwarding cleanly. Drop the VPN and
        re-run; this has burned an hour on the project before. See
        memory/project_beta_ardu_vpn_udp_blackhole.md.
      - Windows Defender Firewall heuristic block on certain ports
        (9002, 9012 verified blocked; 28000-35000 verified open). No
        pop-up, just silent drops — pick a different --motor-port.

    Same probe pattern as verify_sim_arms stage 1: bind UDP locally, fire
    5 datagrams from inside the container with `nc -u`, count receipts.
    """
    if args.skip_preflight:
        return
    print(
        f"[final_world_betaflight] Pre-flight: container→host UDP on "
        f":{motor_port}...",
        flush=True,
    )
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        rx.bind(("0.0.0.0", motor_port))
    except OSError as e:
        print(
            f"[final_world_betaflight] FATAL: cannot bind UDP :{motor_port} "
            f"on host ({type(e).__name__}: {e}). Another process is probably "
            f"holding it (look for a stale sim_loop / fake_pegasus_loop / "
            f"BetaflightUdpBackend). On Windows, "
            f"`Get-NetTCPConnection -LocalPort {motor_port}`.",
            file=sys.stderr,
        )
        sys.exit(2)
    rx.settimeout(0.3)

    # `host.docker.internal` resolves to the Windows host bridge from inside
    # the container — same address BF SITL uses for its motor packets. We
    # use 192.168.65.254 directly because the container's `getent ahostsv4`
    # path picks the Docker Desktop fixed IPv4 that BF inet_addr() also
    # uses (BF's own resolver path is documented in sitl/start.sh).
    triggered = False
    try:
        subprocess.run(
            [
                "docker", "exec", "project-beta-ardu-sitl", "bash", "-c",
                f"for i in 1 2 3 4 5; do "
                f"  echo X | nc -u -w 0 192.168.65.254 {motor_port}; "
                f"done",
            ],
            check=False, timeout=5,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        triggered = True
    except (subprocess.TimeoutExpired, FileNotFoundError) as e:
        print(
            f"[final_world_betaflight] WARN: could not invoke `docker exec` "
            f"to trigger UDP probe ({type(e).__name__}: {e}). Skipping UDP "
            f"pre-flight; you'll find out from bridge stats whether the path "
            f"is actually working.",
            file=sys.stderr,
        )

    if triggered:
        time.sleep(0.3)
        got = 0
        deadline = time.time() + 1.0
        while time.time() < deadline:
            try:
                rx.recvfrom(64)
                got += 1
            except socket.timeout:
                break
        rx.close()
        if got < 3:
            print(
                f"[final_world_betaflight] FATAL: container→host UDP on "
                f":{motor_port} delivered only {got}/5 probe packets.\n"
                f"  Most likely cause: VPN active on the Windows host\n"
                f"    (WSL2 + Docker Desktop blackholes UDP under a VPN —\n"
                f"     drop the VPN and re-run BEFORE other diagnostics).\n"
                f"  Next: Windows Defender Firewall blocking this port.\n"
                f"    Try --motor-port 28500 / 35000 (verified open),\n"
                f"    or --skip-preflight if you've manually verified the\n"
                f"    UDP path elsewhere.",
                file=sys.stderr,
            )
            sys.exit(2)
        print(
            f"[final_world_betaflight] Pre-flight: container→host UDP OK "
            f"({got}/5 packets).",
            flush=True,
        )
    else:
        rx.close()


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
        # The container is debian-slim — no pkill, no procps. Find the
        # PID by reading /proc and kill via the kill builtin instead.
        kill_relay = (
            "for f in /proc/[0-9]*/cmdline; do "
            "  if grep -q motor_relay.py \"$f\" 2>/dev/null; then "
            "    pid=$(echo $f | sed 's|/proc/||;s|/cmdline||'); "
            "    kill $pid 2>/dev/null; "
            "  fi; "
            "done; sleep 0.3"
        )
        # `setsid` detaches the new relay from the docker-exec session
        # so it doesn't get SIGHUPed when bash exits. Without setsid,
        # the new relay died the moment subprocess.run returned, and
        # the bridge would get rx=0 forever.
        subprocess.run(
            ["docker", "exec", "project-beta-ardu-sitl", "bash", "-c",
             f"{kill_relay}; "
             f"setsid python3 /opt/sitl/motor_relay.py 192.168.65.254 "
             f"{args.motor_port} </dev/null >/dev/null 2>&1 &"],
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

# UDP pre-flight: now that the relay is targeting the chosen motor port,
# verify packets actually arrive. Catches VPN-on-Windows and Defender
# Firewall blocks BEFORE Isaac Sim starts (saves ~30s of warmup).
_preflight_udp_or_die(args.motor_port)

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
import omni.usd  # noqa: E402  (used for in-sim marker prim creation)
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

    # ── In-sim visual UX (Apr 29) ────────────────────────────────
    # Static colored spheres at spawn + 4 corners + CENTER so the
    # operator has visible references in Isaac Sim. Plus a dynamic
    # "target bubble" that the main loop moves to mission_demo's
    # current waypoint each tick — lets the operator see whether
    # the drone is following the controller's intent or drifting
    # independently. Colors:
    #   spawn  = green
    #   SW     = blue
    #   SE     = magenta
    #   NE     = orange
    #   NW     = yellow
    #   CENTER = white
    #   target = red (dynamic)
    # Pegasus uses ENU (x=east, y=north, z=up) for spawn coords.
    # mission_demo's "corner" terminology is in N/E (y=north,
    # x=east), so SW = (-east, -north) etc. — convert here.
    stage = omni.usd.get_context().get_stage()
    sx, sy, sz = float(args.spawn[0]), float(args.spawn[1]), float(args.spawn[2])
    half = float(args.map_half_size)

    def _make_marker(name: str, x: float, y: float, z: float,
                     rgb: tuple[float, float, float], radius: float = 2.0):
        path = f"/World/markers/{name}"
        sphere = UsdGeom.Sphere.Define(stage, path)
        sphere.GetRadiusAttr().Set(radius)
        UsdGeom.XformCommonAPI(sphere.GetPrim()).SetTranslate(
            Gf.Vec3d(x, y, z),
        )
        sphere.GetDisplayColorAttr().Set([Gf.Vec3f(*rgb)])
        return sphere

    # Mission_demo's corners are N/E offsets from spawn (which becomes
    # HOME at INIT). In ENU: north→y, east→x.
    marker_alt = sz + 1.0  # slightly above spawn so spheres aren't buried
    # Spawn marker kept small (0.3 m) so it doesn't engulf an FPV
    # camera mounted on the drone — discovered Apr 29 when the drone-
    # parented camera at (0.5, 0, 0.1) body local was inside the
    # original 1.5 m-radius sphere at spawn world position, hence
    # all-black FPV.
    _make_marker("spawn",  sx,        sy,        sz,         (0.0, 1.0, 0.0), 0.3)
    _make_marker("SW",     sx - half, sy - half, marker_alt, (0.1, 0.1, 1.0), 2.5)
    _make_marker("SE",     sx + half, sy - half, marker_alt, (1.0, 0.1, 1.0), 2.5)
    _make_marker("NE",     sx + half, sy + half, marker_alt, (1.0, 0.5, 0.0), 2.5)
    _make_marker("NW",     sx - half, sy + half, marker_alt, (1.0, 1.0, 0.0), 2.5)
    _make_marker("CENTER", sx,        sy,        marker_alt, (1.0, 1.0, 1.0), 1.5)
    target_marker = _make_marker(
        "target", sx, sy, marker_alt, (1.0, 0.0, 0.0), 1.0,
    )
    target_marker_xform = UsdGeom.XformCommonAPI(target_marker.GetPrim())
    print(f"[final_world_betaflight] In-sim markers placed: spawn(green), "
          f"SW(blue), SE(magenta), NE(orange), NW(yellow), CENTER(white), "
          f"target(red, dynamic). map_half_size={half}m", flush=True)

    world.reset()
    print(
        "[final_world_betaflight] World reset; starting timeline. "
        "BF lockstep begins on first physics tick.",
        flush=True,
    )
    timeline.play()

    # ── FPV camera (Apr 29 side quest) ──────────────────────────
    # Two cameras, two debug paths:
    #   /World/debug_world_camera — NOT parented under drone, fixed
    #     in world. Isolates "viewport switch works" from "drone
    #     parenting works."
    #   /World/quadrotor/fpv_camera — parented under drone for the
    #     real FPV use. Top-down debug pose for now.
    # We log get_active_camera() before/after the switch so we can
    # see whether the API call took effect. Activates the WORLD
    # debug camera first; if operator sees the spawn area, viewport
    # API is fine and we can iterate the drone-parented camera.
    if args.fpv_camera:
        # World-fixed debug camera (NOT parented under drone)
        debug_path = "/World/debug_world_camera"
        debug_cam = UsdGeom.Camera.Define(stage, debug_path)
        UsdGeom.XformCommonAPI(debug_cam.GetPrim()).SetTranslate(
            Gf.Vec3d(sx, sy - 10.0, sz + 5.0),
        )
        # Look toward spawn — rotate camera so default -Z points +Y
        # (north, toward spawn). +90° around X tips the look from
        # straight-down toward forward-ish; tweakable.
        UsdGeom.XformCommonAPI(debug_cam.GetPrim()).SetRotate(
            Gf.Vec3f(75.0, 0.0, 0.0),
        )
        debug_cam.GetFocalLengthAttr().Set(24.0)
        debug_cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.05, 10000.0))

        # Drone-parented FPV camera. Iris spawn at (0, 0, 1.5) in
        # the chemical plant happens to be DIRECTLY UNDER A WALKWAY
        # — the proximate cause of the all-black FPV screenshots
        # we saw on Apr 29. Camera body-local (0.3 m forward, 3 m
        # up) puts it at world (0.3, 0, 4.5) at spawn — above the
        # walkway and looking forward. As the drone flies up, the
        # camera follows and the view changes naturally. Rotation
        # X=-15° pitch-down + Y=-90° yaw so default -Z look maps
        # to body +X (forward) with FPV-racer downtilt.
        fpv_path = "/World/quadrotor/fpv_camera"
        fpv = UsdGeom.Camera.Define(stage, fpv_path)
        UsdGeom.XformCommonAPI(fpv.GetPrim()).SetTranslate(
            Gf.Vec3d(0.3, 0.0, 3.0),
        )
        # USD XformCommonAPI XYZ rotation order, gimbal lock at
        # Y=-90: Z rotation locks with X about the view axis. Z=+90
        # rolled the image to body -Z (down) — wrong direction. Z=-90
        # rolls camera up to body +Z (up). Decomposed: at Y=-90,
        # increasing Z by 90° rotates camera-up CCW about body +X.
        # We need camera-up = body +Z, so Z=-90.
        UsdGeom.XformCommonAPI(fpv.GetPrim()).SetRotate(
            Gf.Vec3f(-15.0, -90.0, -90.0),
        )
        # 14 mm focal ≈ 105° HFOV (FPV racer typical).
        fpv.GetFocalLengthAttr().Set(14.0)
        fpv.GetClippingRangeAttr().Set(Gf.Vec2f(0.05, 10000.0))

        try:
            from omni.kit.viewport.utility import get_active_viewport
            viewport = get_active_viewport()
            before_path = (
                str(viewport.camera_path)
                if hasattr(viewport, "camera_path") else "?"
            )
            print(f"[final_world_betaflight] FPV-DEBUG before switch, "
                  f"viewport.camera_path = {before_path}", flush=True)
            # Activate the drone-parented FPV camera (top-down debug
            # pose — 1m above drone, looking straight down).
            viewport.set_active_camera(fpv_path)
            after_path = (
                str(viewport.camera_path)
                if hasattr(viewport, "camera_path") else "?"
            )
            print(f"[final_world_betaflight] FPV-DEBUG after switch to "
                  f"{fpv_path}, viewport.camera_path = {after_path}",
                  flush=True)
            print(f"[final_world_betaflight] FPV-DEBUG: world-fixed "
                  f"{debug_path} also created (proven working). To "
                  f"test it, switch in the viewport menu manually.",
                  flush=True)
        except (ImportError, Exception) as _e:
            print(f"[final_world_betaflight] FPV camera prim created "
                  f"but viewport switch failed: {_e}", flush=True)

        # Periodic auto-screenshot of the current viewport so the
        # human in the loop can pull a PNG and the assistant can
        # read it for diagnosis. Saved to project root every 5 s.
        # Disabled in production; only runs when --fpv-camera is
        # set.
        # Pump a few physics+render ticks so the parented camera
        # picks up its parent's resolved world transform before we
        # screenshot (otherwise we capture pre-physics-tick black).
        for _ in range(20):
            world.step(render=True)
        try:
            from omni.kit.viewport.utility import capture_viewport_to_file
            _fpv_screenshot_path = str(
                Path(__file__).resolve().parents[2] / "fpv_screenshot.png"
            )
            capture_viewport_to_file(viewport, _fpv_screenshot_path)
            print(f"[final_world_betaflight] Post-warmup viewport "
                  f"screenshot → {_fpv_screenshot_path}", flush=True)
        except Exception as _e:
            _fpv_screenshot_path = None
            print(f"[final_world_betaflight] viewport screenshot "
                  f"capture not available: {_e}", flush=True)
    else:
        _fpv_screenshot_path = None

    last_stats_t = time.monotonic()
    last_stats = bf_backend.stats()

    # State file path — same default as sim_loop / bf_gps_shim. The shim
    # reads this for ground-truth position when synthesizing MSP_ALTITUDE
    # responses. With Pegasus driving the physics here (instead of
    # sim_loop), we publish from bf_backend._latest_state.
    import tempfile
    state_file_path = os.path.join(tempfile.gettempdir(), "bf_sim_state.txt")
    print(f"[final_world_betaflight] Publishing sim state to: {state_file_path}",
          flush=True)
    last_state_write = 0.0
    # Respawn signal file. mission_demo writes this when it aborts
    # due to flip detection so the operator can iterate without
    # restarting Pegasus. We poll for it ~10 Hz; on detection,
    # call `world.reset()` (returns the drone to its spawn pose)
    # and delete the signal. The path defaults to a tempdir
    # convention that matches mission_demo's DEFAULT_RESPAWN_
    # SIGNAL_FILE so both processes agree out of the box.
    # Override via --respawn-signal-file when running mission_demo
    # with a non-default path so they stay in lockstep.
    respawn_signal_path = (
        args.respawn_signal_file
        if args.respawn_signal_file
        else os.path.join(tempfile.gettempdir(), "bf_respawn.signal")
    )
    # Pre-clear any stale signal from a prior run.
    try:
        os.remove(respawn_signal_path)
    except OSError:
        pass
    print(f"[final_world_betaflight] Watching respawn signal: "
          f"{respawn_signal_path}", flush=True)
    last_respawn_check = 0.0
    # Target state file the orchestrator polls to position the
    # red 'target bubble' marker. mission_demo writes to it every
    # tick: "n e alt label". Position is home-relative meters in
    # N/E/up. We convert to Pegasus ENU coords (east=x, north=y,
    # up=z) for the prim translate. ~10 Hz poll matches state file.
    target_state_path = os.path.join(
        tempfile.gettempdir(), "bf_target_state.txt",
    )
    print(f"[final_world_betaflight] Watching target state: "
          f"{target_state_path}", flush=True)
    last_target_check = 0.0

    try:
        while simulation_app.is_running():
            world.step(render=True)

            # Respawn signal poll (~10 Hz). When mission_demo aborts
            # via flip detection it touches the signal file. We
            # call world.reset() which returns dynamic prims to
            # their initial poses (Iris back to spawn, attitude
            # cleared) so the operator can rerun mission_demo
            # without restarting the whole sim. timeline.play()
            # after reset because world.reset() pauses the timeline
            # in some Pegasus versions.
            now_respawn = time.monotonic()
            if now_respawn - last_respawn_check >= 0.1:
                last_respawn_check = now_respawn
                if os.path.exists(respawn_signal_path):
                    print(f"[final_world_betaflight] respawn signal "
                          f"received → world.reset() (Iris returns to "
                          f"spawn {tuple(args.spawn)})", flush=True)
                    try:
                        os.remove(respawn_signal_path)
                    except OSError:
                        pass
                    try:
                        world.reset()
                        timeline.play()
                    except Exception as _e:
                        print(f"[final_world_betaflight] respawn reset "
                              f"failed: {_e}", flush=True)

            # Target bubble position update (~10 Hz). mission_demo
            # writes home-relative N/E/alt to target_state_path each
            # tick; we read and translate the red marker prim to
            # match. The home point IS the spawn (mission_demo locks
            # home from the GPS captured during INIT, which is
            # synthesized from sim_loop's published state — Pegasus's
            # spawn position).
            now_target = time.monotonic()
            if now_target - last_target_check >= 0.1:
                last_target_check = now_target
                try:
                    with open(target_state_path, "r") as f:
                        parts = f.read().strip().split(maxsplit=3)
                    if len(parts) >= 3:
                        n_m = float(parts[0])
                        e_m = float(parts[1])
                        alt_m = float(parts[2])
                        # ENU translate: east=x, north=y, up=z
                        target_marker_xform.SetTranslate(
                            Gf.Vec3d(sx + e_m, sy + n_m, sz + alt_m),
                        )
                except (OSError, ValueError):
                    pass

            # Publish ground-truth state for the shim to synthesize
            # MSP_ALTITUDE + MSP_RAW_GPS. ~10 Hz throttle.
            now_state = time.monotonic()
            if now_state - last_state_write >= 0.1:
                last_state_write = now_state
                st = bf_backend._latest_state
                if st is not None:
                    try:
                        # Pegasus state is ENU/FLU. ENU position:
                        # x=east, y=north, z=up. Shim wants n,e,alt,yaw.
                        # Yaw from quaternion (xyzw, body→world FLU).
                        from scipy.spatial.transform import Rotation as _R
                        rot = _R.from_quat(st.attitude)
                        _, _, yaw_deg = rot.as_euler("xyz", degrees=True)
                        with open(state_file_path, "w") as _f:
                            _f.write(
                                f"{st.position[1]:.3f} "
                                f"{st.position[0]:.3f} "
                                f"{st.position[2]:.3f} "
                                f"{yaw_deg:.2f}\n"
                            )
                    except (OSError, AttributeError):
                        pass

            # Periodic bridge-health log. 0 disables.
            if args.stats_interval_s > 0:
                now = time.monotonic()
                if now - last_stats_t >= args.stats_interval_s:
                    s = bf_backend.stats()
                    dt_s = max(1e-6, now - last_stats_t)
                    delta_tx = s["tx"] - last_stats["tx"]
                    delta_rx = s["rx"] - last_stats["rx"]
                    pct = s["timeouts"] / max(1, s["tx"]) * 100.0
                    motors = bf_backend.input_reference()
                    print(
                        f"[bridge] tx={s['tx']} rx={s['rx']} "
                        f"timeouts={s['timeouts']} ({pct:.2f}%) "
                        f"| Δ tx={delta_tx} rx={delta_rx} in {dt_s:.1f}s "
                        f"({delta_tx / dt_s:.0f} Hz) "
                        f"| motor w={motors[0]:.0f},{motors[1]:.0f},"
                        f"{motors[2]:.0f},{motors[3]:.0f} rad/s",
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
