# HITL sim — what works, what's flaky, what's next

This doc captures the state of the in-house BF SITL hardware-in-the-loop
sim for `racer_companion` (Project_Beta_Ardu) as of 2026-04-27. It's
written for whoever picks this back up — including future-me — so a
reader can decide whether to keep iterating in pure sim or move to a
real-hardware bench rig.

## The architecture (validated)

```
┌─────────────────────────┐
│  racer_companion / mission_demo
│  (UNCHANGED — same code as real flight)
│  - sends RC at 50 Hz via MSP_SET_RAW_RC
│  - polls 6-cmd telemetry batch at 10 Hz
└──────────────┬──────────┘
               │ MSP TCP :5762
               ▼
┌─────────────────────────────────────────────────┐
│  bf_gps_shim                                    │
│  ─────────────────────────                      │
│  • Synthesizes MSP_RAW_GPS  ← integrator        │
│  • Synthesizes MSP_ALTITUDE ← integrator        │
│  • Caches MSP_ATTITUDE/RC/ANALOG/STATUS_EX/     │
│    BOXNAMES; refreshes BF at 2 Hz so the FC's   │
│    UDP/MSP thread coupling can't be stalled by  │
│    the companion's 10 Hz polling                │
│  • Forwards RC writes + everything else to BF   │
│  • Position integrator pulls ground truth from  │
│    sim_loop's state file (tempfile path)        │
└──────────────┬──────────────────────────────────┘
               │ MSP TCP :5761                ▲
               ▼                              │ FDM UDP 9003
┌─────────────────────────────────────────────┴───┐
│  BF SITL Docker container (BF 4.5.1)            │
│  - Iris Pegasus airframe, ANGLE mode, AUX1 arm  │
│  - baro_hardware = VIRTUAL                      │
└──────────────┬──────────────────────────────────┘
               │ motor[4] UDP :9002 → relay → host:35000
               ▼
┌─────────────────────────┐
│  sim_loop (Python 6-DOF) │
│  - reads motor norms     │
│  - integrates Iris physics
│  - publishes ENU pos to    
│    state file (10 Hz)    │
│  - feeds FDM back to BF  │
└─────────────────────────┘
```

Sim-to-real correlation: **the companion code path is identical** in
sim and real flight. The shim's cache, GPS synth, and altitude synth
are sim-only mechanics that absorb BF SITL artifacts (concurrent UDP/MSP
thread coupling, VIRTUAL baro that doesn't actually convert FDM
pressure → alt). On real hardware those artifacts don't exist — STM32
BF runs single-core and has a real barometer.

## The validated bring-up procedure

```
T1>  cd Project_Beta_Ardu
T1>  SITL_HOST_MOTOR_PORT=35000 docker compose -f sitl/docker-compose.yml down
T1>  SITL_HOST_MOTOR_PORT=35000 docker compose -f sitl/docker-compose.yml up -d --build
T1>  sleep 15  # let BF fully boot

T1>  # smoke check that BF is actually receiving FDM and arming:
T1>  python -m integrations.tools.verify_sim_arms --motor-port 35000
     # PASS = `tx=1050 rx=1050 max_motor_norm≈0.706 samples_above_500_rad_s≥30`
     # If FAIL, see "Known shortfalls" below.

T2>  python -m integrations.tools.sim_loop --motor-port 35000
     # 6-DOF Iris physics; reads BF motor outputs, sends FDM back.

T3>  python -m integrations.tools.bf_gps_shim
     # Listens TCP :5762, proxies to BF :5761 with cache + synth.

T4>  python -m integrations.tools.mission_demo --bf-port 5762
     # Connects to the shim, flies the X-pattern + circles + handover demo.
```

`flight_profile` (the closed-loop altitude profile demo) and
`verify_sim_arms` (the canonical "is the wire working" test) both pass
cleanly when BF SITL is in a good state.

## Known shortfalls

These are documented because they bit me repeatedly. The fixes are
either bigger than this session's scope or sim-specific (no real-flight
impact).

### 1. BF SITL Phase-2 UDP-init bug (intermittent)

**Symptom**: `verify_sim_arms` passes some boots and fails others
(`max_motor_norm=0.000`, `rx=0` in sim_loop). When it fails, the BF
SITL container is up and MSP TCP works, but BF's UDP server thread
never accepts FDM packets. Without FDM, BF's gyro/acc/baro sensors
never update, ARMING_DISABLED bits never clear (BOOT_GRACE_TIME, LOAD,
CALIBRATING all stick), and motors stay at 0.

**Root cause**: BF SITL on BF 4.5.1 has a known issue (acknowledged in
`sitl/start.sh`'s own docstring) where the second BF spawn after a
`save`/systemReset doesn't re-init its SITL UDP server. The eeprom-bake
Dockerfile stage was supposed to dodge this by saving once at build
time so runtime is single-spawn, but the bug also triggers when BF
boots from a baked eeprom.

**Workarounds** (in priority order):
- **Always `down + up --build`**, never just `restart` — clears any
  stale state. After build, the FIRST `verify_sim_arms` may still fail
  once; re-restart and try again. ~50% success rate.
- **Run with sim_loop already streaming FDM** when the container
  starts — sometimes BF picks up UDP this way.
- Real fix would be patching BF SITL's UDP thread init, ~half-day of C
  debugging in `betaflight/src/main/target/SITL/sitl.c`.

### 2. ARM_SWITCH latch (real-flight-relevant — FIXED Apr 27)

**Symptom**: 30 s into a recorded flight, `arm_flags` flips from `0x0`
(ARMABLE) to `0x200008e` (FAILSAFE+RX_FAILSAFE+BAD_RX_RECOVERY+
THROTTLE+ARM_SWITCH bit 25). `fm` drops from `0x3` (ARM+ANGLE) to
`0x2` (ANGLE only). Drone disarms mid-mission and falls.

**Root cause** (two-part):
1. BF SITL on Windows-Docker periodically gaps the MSP RC stream long
   enough to trip BF's bad-rx state machine. Two contributing causes
   were patched:
   - `bf_gps_shim` had two threads writing to the upstream socket
     without synchronization. Concurrent `socket.sendall` calls can
     interleave bytes mid-MSP-frame, corrupting `MSP_SET_RAW_RC` —
     which BF treats as a bad RX frame, eventually tripping
     `BAD_RX_RECOVERY` (bit 3). Fixed in `pegasus-bridge` by
     serializing all up/down sendall through per-socket locks.
   - With `failsafe_delay = 200` (20 s) in `defaults.txt`, the
     longer-period RX_FAILSAFE no longer fires on transient gaps,
     just on sustained ones.
2. Once any disable bit set with AUX1 high, `ARM_SWITCH` (bit 25)
   latches and persists until AUX1 goes LOW. The naive recovery
   (drop AUX1 → raise AUX1) loops forever because at the LOW→HIGH
   transition BF re-evaluates arming with throttle still high
   (THROTTLE bit 7 set) and ARM_SWITCH re-latches. `mission_demo`
   now does a **two-stage recovery**:
   - **LOW stage**: AUX1=LOW + idle throttle until `(ARM_SWITCH |
     BAD_RX_RECOVERY)` clears.
   - **HOLD stage**: AUX1=HIGH + idle throttle for `RECOVERY_HOLD_S
     = 0.6 s` so BF lands the LOW→HIGH transition on a clean
     ARMABLE state.

**Empirical**: `mission4.png` shows the drone surviving 6 latches over
220 s, completing all four X-legs (timing out short of corners
because each recovery costs altitude), before landing in TO_CENTER.

**Real-flight impact**: the hardware path doesn't exhibit the shim
corruption (no Python relay) so the underlying RX-loss trigger is
sim-only. The recovery flow itself is real-flight-applicable as a
fallback against any transient that latches ARM_SWITCH mid-mission.
Mission_demo also still has the **wait-for-armable** check in INIT
that prevents the latch from happening on the *initial* arm.

**SITL gotcha**: BF SITL's `BOOT_GRACE_TIME` runs on sim-time, not
wall-time. If sim_loop / Pegasus hasn't started feeding FDM, sim
time doesn't advance, BOOT_GRACE_TIME never clears, mission_demo
waits forever in INIT. Make sure FDM is flowing before mission_demo
connects.

**Open**: even with the lock + recovery, BF still trips full failsafe
every ~18 s during sustained Isaac Sim flight. The 18 s interval
suggests the shim / Docker / Windows network path has a periodic
stall longer than ~2 s. Recovery handles it, but it makes legs
overshoot timeouts. Tracked as a follow-up; not blocking missions.

### 3. `baro_hardware = VIRTUAL` doesn't actually expose pressure → alt

**Symptom**: `set baro_hardware = VIRTUAL` is enabled (CLI confirms),
`status` reports `BARO=VIRTUAL`, but `MSP_ALTITUDE` returns 0 throughout
flight even when sim_loop is climbing the drone to 1 km. BF's altitude
estimator never receives values from the FDM `pressure` field.

**Workaround**: shim synthesizes `MSP_ALTITUDE` from sim_loop's
ground-truth altitude (published via state file). Real BF hardware
has a real baro chip, so this is sim-only.

### 4. Companion `msp.CH_THROTTLE` / `CH_YAW` are mis-mapped (REAL-FLIGHT BUG)

`msp.py` defines `CH_THROTTLE=3` and `CH_YAW=2`, but BF's default
`rcmap "AETR"` puts throttle at MSP slot 2 and yaw at slot 3. So
`rc[CH_THROTTLE]=1730` writes to BF's yaw slot. mission_demo works
around this with explicit `SLOT_THROTTLE=2 / SLOT_YAW=3` constants;
the production `racer_companion` still has the bug. Tracked as task
#42 in the project queue.

### 5. Stale Python processes hold ports across sessions

If `bf_gps_shim` or `sim_loop` doesn't exit cleanly (KeyboardInterrupt
to a `&` background, killing the parent shell), the Python process can
linger and hold its bound port. Subsequent runs silently get the OLD
process's behavior — including OLD code paths from before any edits.

**Diagnostic**: `Get-NetTCPConnection -LocalPort 5762` (PowerShell) and
`Get-Process -Id <PID>`. Kill stale processes between sessions.

### 6. Windows Defender Firewall heuristic blocks UDP ports

Some host UDP ports (9002, 9012, 38500) get blackholed by the firewall
without an admin pop-up. The container's motor relay forwards to
`SITL_HOST_MOTOR_PORT` which we've ended up moving across runs:
38500 → 28500 → 35000 as ports get blocked. If `verify_sim_arms` stage 1
fails ("0/5 packets arrived"), pick a different `--motor-port`
(28000–35000 range works) and set the matching `SITL_HOST_MOTOR_PORT`
env var on the container.

### 7. VPN + WSL2 + Docker Desktop blackholes container→host UDP

The whole motor packet path dies if a VPN is active on the Windows
host while Docker Desktop's WSL2 backend is in use. Symptom: container's
relay reports forwarding successfully, host's `verify_sim_arms` stage 1
gets 0/5. Drop the VPN.

## What's next

For higher-fidelity validation than this sim achieves:

- **Real-hardware bench HITL** — Speedybee F405 (~$30) flashed with
  BF 4.5.1, USB-serial to host, run `mission_demo` against the real
  chip. Sensors are static (drone sits on the bench), so the flight
  envelope isn't tested, but the FC firmware path, MSP wire timing,
  ARM_SWITCH semantics, and failsafe behavior are all real. Catches
  ~80% of what an outdoor flight would catch, none of the SITL
  flakiness.

- **Real-hardware HITL with sensor injection** — same FC, plus a
  carrier board that injects spoofed IMU/baro/GPS into the FC's
  sensor inputs. Lets sim_loop drive a real chip with fake-but-valid
  flight dynamics. Significant hardware investment but is the
  end-to-end gold standard.

- **Patch BF SITL's UDP thread init** — fix the Phase-2 bug
  upstream and re-pin our Dockerfile. Half-day of C work; benefits
  only the SITL path.

For the current sim setup, treat this as a pre-flight check — if
`verify_sim_arms` passes and `flight_profile` flies cleanly, you've
covered the wire path, motor mixer, ANGLE-mode response, and
altitude-PD basics. Use the bench rig (Path 1) for the rest before
real flight.

## File map

- `integrations/tools/sim_loop.py` — 6-DOF Iris physics integrator
- `integrations/tools/bf_gps_shim.py` — MSP proxy, GPS+alt synth, cache
- `integrations/tools/flight_profile.py` — closed-loop altitude demo
- `integrations/tools/mission_demo.py` — X-pattern + circles + handover
- `integrations/tools/verify_sim_arms.py` — canonical "is wire working"
- `integrations/tools/bf_diag.py` — live arming-flag inspector
- `sitl/Dockerfile`, `sitl/start.sh`, `sitl/defaults.txt` — BF SITL
  build + boot + config
- `integrations/pegasus_betaflight_backend.py` — wire-format helpers,
  used by `sim_loop` and `verify_sim_arms`

The arming-flag decoder (offset 17, u32) and bit-name map are inlined
in `mission_demo.py`'s `_handle_with_arming` — copy from there if you
need to instrument the companion or another tool.
