# MEMORY.md — Project handoff & context

> Read this if you're picking up the project cold. It captures the
> architectural decisions, sharp edges, and "what to do next" so you don't
> have to re-derive them from the code.

If you only read one section, read **§ Sharp edges** — that's where the
silent-failure landmines are buried.

## How to use this file

- **First time on the project:** read top to bottom (~10 minutes), then
  follow the links in §"Continuing the work."
- **Returning after time away:** read § Snapshot to see what's changed
  since you were last here. Skim § Open questions for anything that's been
  resolved or shifted.
- **Before a substantive change:** check § "Things explicitly NOT done" —
  if your idea is on that list, surface it before building. Reasons are
  load-bearing.

Append, don't reorganize. Date your additions if they materially change
project state.

---

## Snapshot — v0.6.1 (Apr 26 2026, in-Sim flight debug)

| Component | State |
|---|---|
| BF SITL container build + boot | ✅ Phase-1 save + Phase-2 respawn clean, eeprom persisted, `.config_applied` marker prevents re-apply |
| Bridge wire (host ↔ BF UDP via in-container relay) | ✅ Locked at ~67 Hz, 0 timeouts, rx ≈ tx in steady state. Background recv thread (`_rx_loop`) decouples motor RX from physics tick rate. Ephemeral motor port picked at orchestrator startup to dodge Windows Firewall heuristic. |
| Final_World scene + Iris spawn in Isaac Sim | ✅ Renders cleanly via `final_world_betaflight.py` |
| BF arming (motors actually spin) | ❌ `motor=0,0,0,0` even with `aux 0 0 0 1700 2100` + `msp_override_channels_mask=255` + AUX1=2000 from `hover_test`. Arming-disable flags say `RXLOSS` is stuck. |
| `mission_demo` flight | ❌ Blocked by arming + virtual-GPS plumbing not in place (BF SITL ignores `position_xyz` field of fdm_packet). |

### v0.6.1 changes — what landed in this session

- **`integrations/pegasus_betaflight_backend.py`** — background recv
  thread (`_rx_loop`), `SO_RCVBUF=1MiB`, `motor_port` default flipped
  to 19500 then 38500 then ephemeral (Windows Defender Firewall blocks
  long-lived UDP ports after enough rebinds). `recv_timeout_s` is now
  cold-start-only.
- **`integrations/orchestrators/final_world_betaflight.py`** — picks an
  ephemeral host motor port at startup, restarts the in-container relay
  to forward there. UTF-8 stdout reconfig before any print (Windows
  cp1252 console). `import carb` deferred until after
  `SimulationApp(...)` boots Kit/Carb. `[bridge]` stats line now
  includes the cached motor `w` in rad/s so 0,0,0,0 vs flight-rate
  values are obvious at a glance.
- **`sitl/start.sh`** — proper two-phase boot: Phase 1 applies
  defaults.txt + `save` (BF writes eeprom and exits with
  systemReset()), Phase 2 respawns BF with the saved eeprom. Without
  `save`, BF stays in CLI mode and the CLI arming-disable flag never
  clears. `.config_applied` marker prevents re-applying on container
  restart (BF creates a default eeprom on first boot, so eeprom
  existence alone isn't a useful signal).
- **`sitl/motor_relay.py`** (new) — single-process Python UDP relay
  127.0.0.1:9002 (BF's hardcoded output) → host:`SITL_HOST_MOTOR_PORT`.
  Replaces the previous `socat ... fork` which added 50-200 ms of
  per-packet latency and capped Pegasus's physics loop at ~2 Hz.
- **`sitl/defaults.txt`** — added `motor_pwm_protocol=PWM`
  (clears MOTOR_PROTO arming flag), `small_angle=180`,
  `min_check=1000`, `max_check=2000`,
  `runaway_takeoff_prevention=OFF`, `aux 0 0 0 1700 2100 0 0` (binds
  ARM mode to AUX1 high band). Bumped `msp_override_channels_mask` from
  15 to 255 (was only covering channels 1-4; AUX1 needed for the ARM
  box was being silently dropped at mask=15).
- **`integrations/tools/hover_test.py`** (new) — minimal MSP arm +
  throttle ramp tool that bypasses `mission_demo`'s GPS gate. Sends
  centered RC + AUX1 high + throttle ramp 1000→hover.
- **`integrations/tools/check_arming.py`** (new) — polls BF status
  flags via MSP CLI, used to debug RXLOSS in parallel with hover_test.

### v0.6.1 sharp edges

- **`save` reboots BF AND so does `exit`.** Both write/discard config
  and call `systemReset()`. The only way to leave CLI mode without
  rebooting is to NOT enter it in the first place. Once `#` is sent,
  the CLI arming-disable flag stays set until BF reboots. `start.sh`
  intentionally uses `save` (not `exit`) because save persists; Phase 2
  respawn loads the eeprom and BF starts NOT in CLI mode.
- **`host.docker.internal` resolves to IPv6 first** on Docker Desktop.
  BF's `inet_addr()` is IPv4-only and fails to INADDR_NONE, so motor
  packets sendto 255.255.255.255 and vanish. `start.sh` pre-resolves
  via `getent ahostsv4` before passing argv[1] to BF.
- **Windows Defender Firewall blocks inbound UDP on a heuristic port
  range.** 9002, 9012, 19500, 38500 all eventually got blocked after
  enough rebinds during a debug session. Workaround: orchestrator
  picks a fresh ephemeral port each launch and reconfigures the
  in-container relay to forward there.
- **BF SITL ignores `position_xyz` and `velocity_xyz` in fdm_packet**
  (verified against `betaflight/4.5.1/src/main/target/SITL/sitl.c` —
  `pkt->position_xyz` and `pkt->velocity_xyz` are referenced nowhere
  in updateState). BF only consumes timestamp, IMU, quat, pressure.
  No virtual GPS without a separate plumbing pass — that's task #39.
- **`msp_override_channels_mask=15` only overrides channels 1-4 and
  silently drops AUX1+.** Mission_demo + hover_test send AUX1=2000 to
  ARM, but mask=15 dropped that channel and BF never saw the ARM
  signal. Mask must be 255 for full 8-channel override.
- **CRLF in scripts copied into Linux container breaks shebangs.**
  Python scripts that touch `sitl/start.sh` or `sitl/motor_relay.py`
  via text-mode IO inject CRLF on Windows; the container's
  `#!/usr/bin/env bash\r` then fails with "bash\r: No such file".
  Always `f.read()` + `replace(b'\\r\\n', b'\\n')` + binary `f.write()`
  when editing those files programmatically.

### Continuing the work — open debug

Iris falls and sits on the asphalt because BF's motor outputs are
zero-pinned. Bridge wire is fully proven. Three concrete things to
try next session — see tasks #36-#40 + `docs/13_mission_demo.md`
"Open debug" section:

1. Open BF Configurator on `localhost:5761` while `hover_test` runs
   — Modes tab shows whether ARM flips green when AUX1 goes 1500→2000.
2. Lower `rx_min_usec = 750` (currently 885) + try
   `feature -RX_PARALLEL_PWM -RX_PPM` in defaults.txt.
3. Wire virtual GPS so mission_demo can navigate
   (BF SITL ignores position_xyz; mission needs MSP_RAW_GPS data).

## Snapshot — v0.6 (Apr 25 2026, HITL sim integration)

| Component | Status | Validated |
|-----------|--------|-----------|
| Companion Python library (`companion/`) | ✅ unchanged from v0.5 | 159 unit tests + 9 hypothesis on push CI |
| BF SITL ↔ Pegasus UDP bridge (`integrations/pegasus_betaflight_backend.py`) | ✅ shipped | 35 unit tests; integration test gated on real BF SITL Docker |
| Pegasus orchestrator (`integrations/orchestrators/final_world_betaflight.py`) | ✅ shipped | manual end-to-end (Final_World + Iris + BF SITL) |
| Pilot RC injector (`integrations/tools/radio_to_bf.py`) | ✅ shipped | 27 unit tests + fake-pygame integration |
| Airframe profile registry (`integrations/configs/airframes.py`) | ✅ shipped | 21 unit tests incl. URDF parse + T:W computation |
| 5" race-quad URDF (`assets/racer_5in/racer_5in.urdf`) | ✅ shipped, **USD regen needed locally** | URDF parses; rotor positions verified X-config |
| Sensor noise + mag-NONE drift (Phase 6) | ✅ shipped | 4 unit tests + standalone watchdog driver |
| Sim smoke procedure | ✅ docs/09..12 | Phase 3-6 docs each |
| CI: integrations job | ✅ added | runs `python -m unittest discover -s integrations/tests` |

**Branch `pegasus-bridge`** off `safety-defense-in-depth`. Three PR-pending phases. Test totals: ~159 companion + ~83 integrations + 11 Lua + 18 preflight + 12 tune-heading.

### v0.6 changes — HITL sim integration (Phases 1-6)

Goal: validate companion in PegasusSimulator + Final_World (TIAMAT chemical plant) before any real-hardware flight. "As close to real as possible" → BF SITL ↔ Pegasus UDP bridge (companion talks unmodified MSP), not a MAVLink shim.

- **Phase 1 — UDP bridge**: `integrations/pegasus_betaflight_backend.py`. fdm_packet (UDP 9003, 18 doubles, 144B) Sim→BF; servo_packet (UDP 9002, 4 floats, 16B) BF→Sim. Verified against actual BF source — counter-agent caught 5 wire-format bugs (ports reversed in Dockerfile comments, position is NED meters not lon/lat, accel is "+g on z-down" not specific-force, BF doesn't auto-reply). `host.docker.internal` plumbing added so motor packets reach the host from inside the container.
- **Phase 2 — orchestrator**: `integrations/orchestrators/final_world_betaflight.py`. Pre-flight TCP probe before paying Isaac's 30s warmup. Periodic bridge stats every 5s. Iris in chemical plant.
- **Phase 3 — companion wiring**: `companion/config/sim_waypoint.json`, `integrations/tools/discover_bf_gps.py`. No companion code changes. `min_vbat=0.0` is sim-only because BF SITL replies vbat=0.0.
- **Phase 4 — pilot RC**: `integrations/tools/radio_to_bf.py` + `integrations/configs/radiomaster_pocket.json`. Counter-agent caught BF's `rc_packet` is 40B (`<d16H`), not 16B as I'd assumed — same Phase-1-class bug. `--aux CH=PWM` for testing without a radio. `--sweep` mode.
- **Phase 5 — race-quad**: `assets/racer_5in/racer_5in.urdf` + `integrations/configs/airframes.py`. `--airframe iris|racer5` flag. Counter-agent caught two BLOCKERs: thrust curve wasn't actually wired (Iris-tuned defaults applied to race-quad → T:W~54), and c_Q was 1000× too high (yaw flip-spin on stick input). Now shipping with c_T=1.7e-6, c_Q=2.0e-8, T:W~10.
- **Phase 6 — sensor noise**: `BetaflightBackendConfig.gyro_bias_drift_rad_s`, `gyro_noise_std_rad_s`, `accel_noise_std_m_s2`, `noise_seed`. Standalone driver `integrations/tools/drive_heading_divergence.py` validates the v0.4 BUG-6 watchdog end-to-end with realistic ~1°/min mag-NONE drift.

### v0.6 sharp edges

- **BF SITL UDP wire format is verified against `betaflight/4.5.1/src/main/target/SITL/sitl.c` + `target.h`. Don't trust prior research summaries or Dockerfile comments — they had multiple errors. `fdm_packet` = 18 doubles / 144B; `servo_packet` = 4 floats / 16B; `rc_packet` = 1 double + 16 uint16 / 40B. BF strict-checks packet size; wrong size = silent drop.**
- **`MultirotorConfig.thrust_curve` MUST be set per-airframe.** Just changing the USD doesn't change the thrust math — Pegasus uses `MultirotorConfig` defaults if you don't override. Race-quad with Iris thrust = T:W 54 = instant divergence. Orchestrator now plumbs `QuadraticThrustCurve` from the profile registry.
- **`c_Q / c_T` ratio MUST be ~0.012 for 5" props.** Higher → flip-spin yaw; lower → no yaw response. `test_torque_to_thrust_ratio_in_band` pins it.
- **First-tick FDM dead-air, ~1 s.** `update_state` and `update(dt)` fire from different Pegasus callbacks, ordering not strict. Bridge bails on first tick when `_latest_state is None`. Visible as `[bridge] tx=0` for ~1 s after timeline play.
- **Race-quad USD must be regenerated locally** from URDF via Isaac Sim importer. `.gitignore` excludes `*.usd*` under `assets/`. See `assets/racer_5in/README.md`.

## Snapshot — v0.5 (Apr 25 2026, capability + quality pass)

| Component | Status | Validated |
|-----------|--------|-----------|
| Plan + architecture (`README.md`) | ✅ shipped | — |
| Companion Python library (`companion/`) | ✅ shipped | 159 unit tests + 9 hypothesis property tests on push CI |
| Betaflight CLI configs (`bf_config/`) | ✅ shipped, **no real BF flashed yet** | preflight_validator + `example_manifest.txt` |
| Per-phase build docs (`docs/`) | ✅ shipped | — |
| EdgeTX Lua scripts (`edgetx_scripts/`) | ✅ shipped, **no radio provisioned yet** | Lupa parse + 11 logic tests on push CI |
| Hardware BOM (`HARDWARE_BOM.md`) | ✅ shipped | — |
| Drill / incident / sign-off log templates | ✅ shipped, **empty** | — |
| BF SITL test rig (`sitl/`) | ✅ shipped, **container not yet built** | manual-trigger CI; readback asserts mask=15 + mag=NONE on boot |
| MAVLink publisher (`racer_companion/mavlink.py`) | ✅ shipped | HEARTBEAT + STATUSTEXT byte-for-byte + COMPANION_STATE round-trip vs pymavlink 2.4.49 |
| QGroundControl on phone setup | ✅ documented, **not yet field-verified** | — |
| Companion-side ACRO check (MSP_STATUS_EX) | ✅ shipped | unit tests; fail-open until BOXNAMES received (safelock.lua remains primary) |
| Heading-divergence watchdog | ✅ shipped | unit + integration tests; gate is `state==TRANSIT` (NO err filter — see Sharp edges) |
| FlightController abstraction (`fc/`) | ✅ shipped | Protocol + BetaflightAdapter (only impl); `is_acro_active`, `tick()→TelemetrySnapshot` |
| Byte-level MSP recorder (`recorder.py`) | ✅ shipped + wired | `--record <path>` on main.py; 11 round-trip tests + integration test |
| HOLD position controller (`nav.hold_command_us`) | ✅ shipped | body-frame P, NaN-guarded, heading=0/90/180/270 sign-tested |
| Pre-flight config validator (`scripts/preflight_validator.py`) | ✅ shipped | 18 tests; case-insensitive enums only, name fields strict |
| Heading-threshold tuning harness (`scripts/tune_heading_threshold.py`) | ✅ shipped, **needs first real flight log** | 12 tests; ASCII output; reports trip_edges + tripped_seconds |
| Property-based tests (`test_safety_properties.py`) | ✅ shipped | hypothesis>=6.100; FRESH_RECEIVED_AT strategy + lat/lon poles + antimeridian |

### v0.5 changes (capability + quality pass following 6-agent peer review + holistic review)
- **FC abstraction (`fc/`)**: FlightController Protocol + BetaflightAdapter wrapping MspClient + FlightModeReader. main.py now consumes a typed TelemetrySnapshot per tick. Future ports (INAV / ArduPilot) implement the Protocol; nav/state/safety stay unchanged.
- **Byte-level MSP recorder + replay**: `recorder.RecordingAdapter` tees every byte to a JSONL file; `tools.replay` (run as `python -m tools.replay`) re-feeds the bytes through the parser offline. Wire via `--record <path>` on main.py. Field incidents become reproducible.
- **HOLD wind correction (`nav.hold_command_us`)**: body-frame P controller with small gains (8 us/m, ±60us cap) replaces centered-sticks behavior. Drone now lazily opposes wind drift inside the hold disc; state.py still kicks back to TRANSIT past `arrival_radius_m × 1.5`.
- **Pre-flight validator (`scripts/preflight_validator.py`)**: diff a per-drone manifest against an FC `dump`, exit 1 on mismatch. `bf_config/per_drone/example_manifest.txt` defines the safety floor (`mask=15`, `mag_hardware=NONE`). Same idea as the SITL boot-time readback, but for real drones.
- **Heading-threshold tuning harness (`scripts/tune_heading_threshold.py`)**: replay a flight log through the divergence tracker across a grid of (threshold, window); report both trip_edges and tripped_seconds. ASCII output (Windows-console safe). Uses `state == TRANSIT` gate to match what main.py feeds the live tracker.
- **Property-based safety tests**: hypothesis-driven invariants on clamp, evaluate(), HeadingDivergenceTracker. `FRESH_RECEIVED_AT` strategy makes the `ok=True` branch reachable; lat/lon include poles and antimeridian.
- **Heading-tracker engagement gate corrected** (holistic review finding): the `pitching_forward` boolean used to additionally gate on `|heading_err| < 25°`, which combined with the 60° threshold meant the watchdog could NEVER trip. Now `transit_active = (state == TRANSIT)` only — feeds every TRANSIT-state sample to the tracker.

### v0.4 changes (defense-in-depth pass following ultrareview)
- **BUG-1**: `safety.evaluate` now blocks on `analog is None` (was silently OK; loose VBAT pad would pass). Same fix for stale_analog.
- **BUG-2**: state machine — HOLD now re-engages TRANSIT when drift > `arrival_radius_m × 1.5`. Wind in HOLD used to hold centered sticks while drifting.
- **BUG-3**: `safety.max_distance_from_target_during_transit_m` was a dead config field; now wired as `transit_runaway_*m` reason while in TRANSIT.
- **BUG-4**: `nav.compute_rc(climb_phase=True)` zeros roll/pitch/yaw during CLIMB until current alt ≥ 80% of target — kills the downhill-takeoff treeline hazard.
- **stale_rc**: companion now reports when MSP_RC stops arriving (was invisible).
- **heading_diverged**: sliding-window watchdog catches mag-NONE yaw drift before geofence does.
- **acro_active**: companion subscribes MSP_STATUS_EX + MSP_BOXNAMES, decodes ANGLE/HORIZON box bits, refuses overrides if BF is in ACRO. Defense-in-depth alongside `safelock.lua`.
- **Lua logic harness**: `scripts/test_lua_logic.py` drives `racestrt.lua`'s state machine and `safelock.lua`'s threshold logic via lupa with controlled time. Catches transition bugs that parse-only check missed.
- **SITL readback**: `sitl/start.sh` now hard-fails if `msp_override_channels_mask = 15` or `mag_hardware = NONE` is missing from the dump after defaults applied. Was best-effort with a misleading "WARNING" before.

**Tests now: 105 Python (102 always-run + 3 SITL env-gated) + 11 Lua logic.**

**No drones built yet.** All progress is software + documentation. Field
work begins with the first pilot doing Phase 0.

---

## Quick orient

```
Project_Beta_Ardu/
├── README.md           — start here, the architectural plan
├── ROADMAP.md          — what's shipped vs deferred + revisit triggers
├── HARDWARE_BOM.md     — concrete parts list with prices/links
├── MEMORY.md           — you are here
├── docs/               — phase-by-phase build instructions
├── companion/          — Python (runs on Pi Zero 2W)
├── bf_config/          — Betaflight CLI snippets
├── edgetx_scripts/     — Radio-side Lua
├── sitl/               — BF SITL test rig (Docker)
├── scripts/            — repo tooling (Lua syntax check, etc.)
└── .github/workflows/  — CI
```

**Build guide entry point:** [`docs/00_START_HERE.md`](docs/00_START_HERE.md).
**Cold-start setup for a new pilot:** Phase 0 in
[`docs/01_phase0_gps_rescue.md`](docs/01_phase0_gps_rescue.md). One evening,
GPS Rescue on every drone, gives lost-link RTL with zero new hardware
beyond a $25 GPS module.

---

## Architectural memory

### Locked-in decisions (re-deciding requires explicit conversation)

1. **100% native Betaflight stick feel during the race is non-negotiable.**
   Drove the entire architecture toward MSP-override + companion computer.
   Killed INAV (~90% feel) and ArduPilot (~90% feel) as alternatives.
2. **Single Betaflight FC + companion (Pi Zero 2W or ESP32) writing virtual
   sticks via `MSP_SET_RAW_RC` with `msp_override_channels_mask=15`.**
   Dual-FC architectures with ESC mux were considered and rejected — DShot
   is bidirectional with RPM telemetry, analog mux glitches it; cold-gyro
   handover is a 50–200 ms crash window at race speeds.
3. **Pi Zero 2W is the recommended companion for first build.** ESP32-S3
   port deferred — Pi gives real OS + journalctl + SSH debug, ~10g penalty.
4. **Betaflight 4.5.1 as version pin.** All `bf_config/` snippets, the SITL
   Dockerfile, the docs, are tied to 4.5.x semantics. Bumping = re-validate
   everything.
5. **MSP-override timeout (~500 ms) is the load-bearing safety primitive.**
   When companion stops sending MSP, BF reverts to RX values within this
   window. Three independent recovery paths (pilot AUX low, safety
   violation, companion crash) all converge on this same recovery.
6. **Mag (`mag_hardware`) is intentionally DISABLED.** Mag flyaways are the
   #1 reported GPS Rescue failure on race quads (carbon frames, PDB
   interference). Re-enabling requires dedicated calibration validation
   off the critical path.
7. **`ACRO + AUX-companion-HIGH = NEVER`.** Enforced by `safelock.lua` in
   software (forces companion AUX LOW any time mode is ACRO) AND by switch
   layout in hardware (mode + AUX-companion mapped to coordinates of one
   3-position switch with that combination unreachable). **Both** required;
   either alone is insufficient.
8. **MAVLink Phase 1 = QGroundControl on phone, not radio HUD.** Radio HUD
   integration deferred because the chain (companion → ELRS RX → ELRS TX →
   radio MAVLink display) is too hardware/firmware-variable to write once
   and have it work everywhere. Phone running QGC works on day one.

### Things explicitly NOT done — don't add without re-deciding

| Idea | Why we said no |
|------|----------------|
| Dual-FC + ESC mux | Nobody credible has shipped it. DShot bidirectional + cold-gyro handover unsolved. |
| INAV migration | Single firmware with waypoints + RTH, but ~90% race feel. Violates non-negotiable. |
| ArduPilot on race quads | Even with 4.7 + FSTRATE_ENABLE (4kHz rate loop on H7), zero racing community uses it. |
| BF master/alpha waypoint code | `pg/flight_plan.c` + `AUTOPILOT_MODE` exist but SITL-only and unproven. Re-evaluate annually. |
| MAVLink command ingest by BF | BF is MAVLink-tx-to-GCS only. Doesn't accept commands. |
| ML / learned controller | A P-controller is debuggable in 50 lines. Failure modes obvious. |
| Pre-commit hooks (ruff/black/lualint) | Adds friction before codebase has stabilized. Revisit after 5+ contributors. |
| CI MAVLink publisher integration test | pymavlink as runtime dep is heavy. Added as DEV dep (`requirements-dev.txt`) on 2026-04-25 for cross-validation tests only — companion runtime stays pyserial-only. |
| Pi 3B+/4 as companion | Too heavy and power-hungry. Pi Zero 2W = same compute, 1/3 the weight. |
| Beitian / no-name GPS modules | Tied to most BF Rescue flyaway reports. Stick to Matek / HGLRC / known-vendor M10. |

If you want to revisit any of these, surface it as an issue first. The
reasons above are real, not "we just didn't get to it."

---

## Sharp edges (silent failure landmines)

These are the things that compile cleanly, look right, and silently break
in flight. **Read this list before any change to the load-bearing files.**

### Wire-format
- **MAVLink `CRC_EXTRA` values are load-bearing.** Constants `{0:50, 253:83, 12500:42}` in `mavlink.py`. A wrong CRC_EXTRA produces frames that pass our internal checks but get silently dropped by every receiver. The `test_heartbeat_byte_for_byte_pymavlink` test catches this for HEARTBEAT — extend the same pattern if you add new message types.
- **MAVLink v2 field order is sorted by C-type size descending.** Not declaration order. Get this wrong and you'll see "received N msgs, 0 valid" on the receiver side. The encoder in `mavlink.py` already gets this right; preserve it if you refactor.
- **MSP v1 checksum is XOR of `size + cmd + payload`, not including header bytes.** If you add new MSP commands, double-check `encode_request` in `msp.py`.
- **Field bit ordering in `safety_reasons_bitmap`** must stay stable so radio-side decoders can rely on it. Defined in `mavlink.py SAFETY_BIT`. Add new reasons at higher bits, never reorder.

### Filesystem / build
- **EdgeTX `.lua` filenames must be ≤ 8 characters** (FAT 8.3 limit on B&W radios). All current names (`racestrt`, `safelock`, `racehud`) comply. Don't add a `racerecorder.lua` and expect it to load on TX12.
- **`pyserial` is imported lazily** in `msp.py` (inside `MspClient.__init__`). This lets the module load on a desktop without pyserial installed (for offline tests). Don't move the import to module top — you'll break the test suite on machines without serial hardware.
- **The 3 SITL tests are env-gated** with `@unittest.skipUnless(os.environ.get("SITL_TCP"), ...)`. Don't lose the decorator — without it, main CI tries to connect to a non-existent SITL and dies.
- **Line endings in committed files are LF.** Git on Windows warns about LF→CRLF on checkout, which is fine; the files in the repo stay LF. Don't `dos2unix` or `unix2dos` blindly.

### Operational
- **BF `mag_hardware = NONE` is mandatory** for Phase 0. Even if your GPS
  module has a built-in magnetometer (M10Q-5883 does), disable it in BF
  until calibration is independently validated.
- **`gps_rescue_throttle_hover` is per-drone** and must be measured, not
  copied. Wrong value = drone climbs forever or crashes hard. Procedure
  in `docs/01_phase0_gps_rescue.md`.
- **`msp_override_channels_mask = 15`** must be set, otherwise the companion
  silently has no effect (BF sends RX values regardless of MSP). Check
  `bf_config/per_drone/<drone>.txt` if behavior is wrong. SITL boot now
  reads back the dump and fails loudly if this isn't set.
- **Companion "active" gate is `state.is_active(state)`** — any state other
  than `CLIMB`/`TRANSIT`/`HOLD` returns `False` and the loop sends NO
  `MSP_SET_RAW_RC`. This is intentional — silence triggers BF's MSP
  timeout and RX takes over. Don't add MSP writes outside this gate.
- **First-tick timing in MAVLink sender:** `_last_heartbeat` and
  `_last_state` are initialized to `-1e9`, not `0.0`. This is intentional
  so the first call to `tick()` always emits both. Setting them to 0
  re-introduces the cold-start bug where no telemetry is sent until enough
  time has elapsed.
- **Telemetry-absent must trip safety, not just telemetry-stale.** Mirror
  the GPS pattern: `if foo is None: reasons.append("no_foo_telemetry")`,
  THEN check staleness, THEN check value bounds. The original code only
  checked stale_* and value bounds for analog → a loose VBAT pad silently
  passed. Same logic for any new telemetry stream.
- **Lua state-machine output is one-tick-delayed after a transition.**
  `racestrt.lua` sets outputs from the OLD state, then transitions; the
  NEW state's outputs appear on the next tick. ABORT is the exception
  (checked at the top of run() so it takes effect same-tick). The Lua
  logic harness encodes this; preserve it if you refactor.
- **`HeadingDivergenceTracker` resets when not pitching forward.** If you
  add code that pitches forward outside TRANSIT (e.g., a new "approach"
  state), update the `pitching_forward` predicate in `main.py` or the
  watchdog goes silent.
- **`FlightModeReader` is fail-open until `MSP_BOXNAMES` arrives.** Until
  the bit-index map is known, `acro_active` reads as `False`. `safelock.lua`
  (radio-side) is the primary defense; this is belt-and-suspenders. Don't
  rely on the companion-side check alone.
- **`HeadingDivergenceTracker` MUST NOT be gated on small heading error.**
  The tracker fires when the windowed mean of `|err|` exceeds threshold.
  If `main.py` only feeds it samples where `|err|` is already small (e.g.,
  the old `|err| < yaw_align_threshold_deg` gate), the tracker is silent
  dead code — it never sees the divergence it's supposed to catch. The
  caller-passed `pitching_forward` parameter is misnamed; it actually
  means "is the bearing controller active" and should be `state == TRANSIT`
  with no additional err filter. Holistic review found this on Apr 25 2026.
- **`is_acro_active()` does not consult ARM.** Neither-ANGLE-nor-HORIZON
  reads as ACRO even when the FC is disarmed. This is intentional fail-
  active for the safety use, but a future maintainer expecting an arm
  check (e.g., to suppress a warning while the FC is disarmed) won't get
  one. `safelock.lua` and the runbook gate the actual flight-mode discipline.
- **`tools/` is a sibling of `racer_companion/`, not a sub-package.**
  `python -m tools.replay` from `companion/` works; `python -m
  racer_companion.tools.replay` does NOT exist. Same convention as
  `tools.msp_loopback_test` and `tools.replay_synth`.
- **`TelemetrySnapshot` returned by `BetaflightAdapter.tick()` is a copy.**
  Each call returns a fresh `dataclasses.replace(...)` instance so callers
  may safely retain it across ticks for diff/replay. Don't change `tick()`
  to return the internal mutable snapshot — it's load-bearing for any
  diff-based consumer.
- **`RecordingAdapter._emit` swallows OSError on write.** A disk-full or
  pipe-closed during a flight must NOT take the FC link down. Once
  `_fh_dead` flips, no further log lines are emitted that session — the
  flight continues, the recording just stops mid-way. This is intentional;
  rotating the log mid-flight is out of scope.
- **`preflight_validator._normalize` is case-insensitive ONLY for uniformly-
  cased single-token alphanumeric values.** `none`/`NONE`/`MAX_ALT` match
  case-insensitively (BF emits enums uppercase but accepts either). Mixed-
  case values like `Alex1` or `craft_name = SteelEagle` stay strict — a
  wrong-case name in a name field would silently match otherwise.

### Test discipline
- **74 tests, 71 always-run + 3 SITL env-gated.** When you add a new module,
  add tests in the same commit — the codebase is small enough that bare
  modules without tests are visible from afar.
- **Cross-validate wire formats against canonical implementations** when
  possible. The MAVLink HEARTBEAT test does this against pymavlink. If you
  add a new MAVLink message type, hand-generate the ground truth via
  pymavlink (recipe in `docs/mavlink_setup.md`) and assert byte-for-byte.

---

## Continuing the work

Ranked by what unlocks the most for the next contributor.

### Tier 1 — get a drone in the air (highest priority)

1. **First pilot: do Phase 0 on one drone.** Order Matek M10Q-5883
   (~$28). Follow `docs/01_phase0_gps_rescue.md`. Bench-validate 5/5
   GPS Rescue drills. Commit your `diff all` output as
   `bf_config/per_drone/<pilot>_<drone>_phase0.txt`. Append to
   `docs/drill_log.md` and `docs/sign_offs.md`.
2. **Phase 0.5 audit spreadsheet.** Per `docs/02_phase05_hardware_audit.md`,
   one row per drone in the fleet. Tells you who needs FC upgrades, who
   can go straight to Phase 1+, and total BOM.
3. **Reference companion build.** First Phase 1+ pilot: provision Pi Zero 2W,
   `git clone` the repo, `pip install -e companion`, run the `tools/msp_loopback_test.py`
   against a real BF FC. This shakes out wiring and BF config issues
   before anyone tries to fly.

### Tier 2 — radio-side rollout (parallel with Tier 1)

4. **First buddy with EdgeTX:** copy `edgetx_scripts/SCRIPTS/` to the
   radio's SD card, follow `edgetx_scripts/model_setup_walkthrough.md`,
   do the props-off bench-test matrix (§ 6 of the walkthrough). If
   anything fails, file an issue and we'll fix the script.
5. **For new coders:** start with `edgetx_scripts/lua_for_beginners.md`.
   Make a small change (e.g., tone frequency in `racestrt.lua`), reload
   on the radio, hear it. That's your first commit.

### Tier 3 — verify the deferred infrastructure works

6. **Manually trigger `sitl.yml` once.** Validates the BF SITL container
   actually builds in CI. If it works, we have confidence; if it breaks,
   we know to look at SITL build vs main CI.
7. **First QGC verification.** Once the reference companion build is
   running, set `mavlink_publisher: udp://192.168.4.255:14550` in
   config, fire up QGC on a phone connected to the Pi's WiFi AP, confirm
   HEARTBEAT + COMPANION_STATE arrive.

### Tier 4 — improvements you might want eventually

8. Field photos for `docs/03_phase1_companion_wiring.md`. Add as you
   build.
9. ESP32-S3 port (~2 weekends, see `ROADMAP.md`).
10. SITL Phase 2 (Gazebo physics for closed-loop tuning, ~1–2 weekends).
11. MAVLink Phase 2 (radio HUD via ELRS-over-CRSF, ~1–2 weekends per radio
    variant).

---

## Streams + ownership

Three parallel workstreams from the original plan. Fill in owners as you
take responsibility — don't leave a blank stream unowned.

| Stream | Scope | Owner |
|--------|-------|-------|
| **A — Companion firmware** | `companion/` directory. Run `python -m unittest discover -s tests -t .` before commits. Build the deployable Pi image. | _unassigned_ |
| **B — BF config** | `bf_config/` directory. Per-drone Phase 0 GPS Rescue tuning. Maintain per-drone dumps. **Bench-validate stale-MSP failsafe behavior** on the chosen BF version (load-bearing safety check). | _unassigned_ |
| **C — Field testing** | Drill execution. Maintain `docs/drill_log.md`. Drill 4 (TX-kill failsafe) requires Stream C countersignature per drone before Phase 4. | _unassigned_ |

**Cross-stream sync points** (from main `README.md`):
- After Phase 0.5 audit → share spreadsheet, finalize BOM
- After Phase 1 bench tests → all owners review companion+BF wiring on reference rig
- After Phase 2 SITL → walk through controller behavior in sim
- Before Phase 3 field tests → group safety briefing, drill order locked
- Before Phase 4 race day → 100% drill pass rate per drone, signed off by Stream C

---

## Validated reference vectors

For maintainers cross-checking wire formats or wondering whether something
is supposed to be exactly that value:

| Vector | Value | Source |
|--------|-------|--------|
| HEARTBEAT frame, default fields, sysid=1 compid=191 seq=0 | `fd0900000001bf000000000000001208000403aec6` | pymavlink 2.4.49 |
| MAVLink CRC_EXTRA — HEARTBEAT (id 0) | 50 | MAVLink common.xml hash |
| MAVLink CRC_EXTRA — STATUSTEXT (id 253) | 83 | MAVLink common.xml hash |
| MAVLink CRC_EXTRA — COMPANION_STATE (id 12500, custom) | 42 | This project, fixed once and stable |
| Betaflight version pin (SITL Dockerfile, docs) | tag `4.5.1` | https://github.com/betaflight/betaflight/releases/tag/4.5.1 |
| MSP_SET_RAW_RC command ID | 200 | BF `src/main/msp/msp_protocol.h` |
| `msp_override_channels_mask` value for R/P/Y/T override | 15 (0b1111) | BF discussion #12615, BF source |
| Pi UART for companion (default) | `/dev/ttyAMA0` (PL011) | Raspberry Pi BCM serial doc |
| QGroundControl default UDP listen port | 14550 | QGC docs |
| BF MSP-override timeout | ~500 ms | BF source `src/main/rx/msp.c` (verify per BF version) |

---

## Open questions / things we punted

These are unresolved enough that they may shift the project. Re-check
periodically.

- **Will BF SITL build cleanly on later BF tags?** We pin to 4.5.1 but BF
  release cadence is roughly quarterly. Re-test SITL build before any pin
  bump.
- **Will ELRS MAVLink-over-CRSF support stabilize across receivers?** As
  of 2026 it works on some combos and not others. If the combo you have
  works, document the working setup in `docs/mavlink_setup.md` and we'll
  promote to Phase 2.
- **Is the F4 fleet really not viable for Phase 1+?** `HARDWARE_BOM.md`
  says F4 + Phase 0 only, F4 + Phase 1+ requires upgrade. If a pilot
  pushes back ("my F4 has free UARTs and headroom"), test on one F4
  before changing the recommendation.
- **MultiGP / DRL / racing-league rules.** Companion-driven autonomous
  launch likely violates most sanctioned-event rules. Talk to your event
  organizer before bringing this anywhere sanctioned. Documented in
  `docs/06_phase4_race_day.md`.
- **Per-drone variance in `nav.throttle_hover`.** Defaults to 1300 in
  `start_line.example.json` but real values measured 1275–1325 in the
  plan. After 5+ drones are tuned, see if it correlates with frame size /
  weight / battery cell count and add a guidance table.

---

## Conventions

### Code
- Python 3.10+ (uses `X | None` union syntax). Companion runs on whatever
  Python ships with the Pi OS image at the time, currently 3.11.
- Pyserial imported lazily — see § Sharp edges.
- Lua for EdgeTX is Lua 5.2 dialect (with EdgeTX-specific globals).
- Filenames in `edgetx_scripts/SCRIPTS/` ≤ 8 chars (FAT 8.3).
- Tests live next to code (`companion/tests/`). Run with `python -m
  unittest discover -s tests -t .`.

### Commits
- Conventional Commits not strictly enforced; commit messages should be
  descriptive about *why*, not just *what*.
- Co-author trailer (`Co-Authored-By: ...`) is fine and used by the
  initial commits.
- One logical concern per commit. Companion lib + 48 tests in one commit
  is fine; that's one "thing." Don't bundle docs + code if the docs are
  unrelated.

### Pull requests
- Issue / PR templates not yet created (deferred per ROADMAP). When 5+
  contributors are pushing PRs regularly, add them.

### Tests
- Add tests in the same commit as the code they cover. The repo is small
  enough that an untested module stands out.
- For wire-format / interop code, cross-validate against the canonical
  implementation when possible (e.g., MAVLink against pymavlink).

---

## Updating this file

- **Append new dated sections** if project state shifts substantially.
- **Update § Snapshot table** when components flip status (✅ deployed,
  field-validated, etc.).
- **Add to § Sharp edges** any time a silent-failure landmine bites and
  you fix it. The point of this section is to share the scar tissue.
- **Don't bloat.** If a topic grows past 30 lines, link to its own
  document instead. This file should stay scannable in under 10 minutes.

Last touched: Apr 25 2026, v0.5 (capability + quality pass + holistic-review fixes), branch `safety-defense-in-depth`.
