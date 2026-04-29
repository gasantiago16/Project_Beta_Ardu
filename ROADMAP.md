# Roadmap

What's shipped today, what's deferred, and the trigger that would make us
revisit each deferred item. Read [`README.md`](README.md) first for the
architectural picture.

## Shipped — v0.18 (Apr 29 afternoon 2026, UNTESTED)

- ⚠️ **Tilt-throttle FF gate in `_compute_waypoint_rc`.** FF now
  fires only when `err_m > 0` (drone below target). Above target,
  `tilt_boost = 0` so the kp term has full authority to command
  descent. Apr 29 fpvfix log analysis showed that without the gate,
  the FF + forward-stick formed a positive-feedback path that drove
  the drone 14 m → 49 m past a 15 m cruise target. **UNTESTED** —
  GPU-OOM blocks verify in this session. Verify as a stack with the
  v0.17 floor clamp on next session's first run.

## Shipped — v0.17 (Apr 29 noon 2026, UNTESTED)

- ⚠️ **Crash-floor safety net in `mission_demo`.** Vario tracking
  (LP-filtered finite-diff, tau=0.5 s) + altitude floor clamp. When
  in a flying phase AND `rel_alt < 3 m` AND `vario < -0.5 m/s`,
  forces `thr = hover + 200 µs`. Catches altitude-controller blow-ups
  before ground impact. Logs `[floor]` warning when active.
  **UNTESTED** — verify run blocked by Vulkan `OUT_OF_DEVICE_MEMORY`
  in this session. Verify checklist queued in MEMORY.md / TODO #16.

## Shipped — v0.16 (Apr 29 morning 2026)

- ✅ **`[phys-truth]` orchestrator log line.**
  `final_world_betaflight.py` prints `bf_backend._latest_state.position`
  every 5 s. Closed TODO #0 by proving shim altitude path is faithful
  end-to-end (peak phys-truth=+32.40 m, shim=132.4 m → rel=32.65 m,
  mission rel_alt=32.76 m — within ±0.4 m). Now the canonical
  altitude oracle. **DO NOT REMOVE.**
- ✅ **FPV camera rotation fix.** v0.15's body-local Euler XYZ
  `(-15, -90, -90)` rotated the image 90° AND sat exactly on the
  Y=-90 gimbal lock. Replaced with `(75, 0, -90)` — look=+X, up=+Z,
  15° downtilt, no gimbal lock at any yaw. Verified across yaws
  0/45/90/180/-90.

## Shipped — v0.15 (Apr 29 evening 2026)

- ✅ **FPV camera tracker.** `--fpv-camera` flag adds a top-level
  `/World/fpv_camera` whose world transform is set every tick from
  `bf_backend._latest_state.position` + attitude (composed via
  scipy `Rotation`). Tracks both position AND yaw — fixes v0.14's
  bug where the camera was parented under `/World/quadrotor`
  (which doesn't propagate physics-driven motion).
- ✅ **MP4 video recording.** `--fpv-video-fps` (default 5) +
  `--fpv-video-out-dir` (default `~/Desktop`). Per-tick PNG to a
  temp staging dir; orchestrator `finally:` block ffmpeg-encodes
  to `fpv_mission_<timestamp>.mp4`. **Default workflow** for
  confirming control + placement.
- ✅ **Shutdown signal.** Touch
  `{tempdir}/bf_orchestrator_shutdown.signal` to break the orch
  main loop gracefully — required because Stop-Process -Force on
  Windows skips Python's `finally:` so the encode would never fire.

## Shipped — v0.13 (Apr 29 afternoon 2026)

- ✅ **Tilt-throttle feedforward** in `mission_demo`. When forward
  pitch is commanded, throttle is boosted proportionally
  (`tilt_throttle_factor=0.15`) to counter cos(tilt) lift loss.
  Mission21 dropped X_LEG_3 from 41.8 m short → 14.1 m short.
  (Suspected co-cause of altitude oscillation per v0.17 analysis;
  TODO #17 will gate the FF to only fire when below target.)

## Shipped — v0.10–v0.12 (Apr 28 late night 2026)

- ✅ **v0.12 in-sim UX.** Spawn-position green sphere marker + live
  red target bubble. Operator can see drone vs commanded path.
- ✅ **v0.11 flip detection + respawn signal.** When roll>thresh OR
  pitch>thresh for N ticks, mission_demo writes
  `{tempdir}/bf_respawn.signal`; orch polls every 0.1 s and calls
  `world.reset()`. Operator can iterate without restarting Pegasus.
- ✅ **v0.10 lateral controller fix.** `pitch_kp_per_m` 2.0 → 4.0
  (matches `racer_companion` default). Mission15 X_LEG_2 ended
  10.3 m from SE corner (vs mission14's 34 m). Also shipped
  `--waypoint-trace-csv` for future tuning.

## Shipped — v0.8–v0.9 (Apr 28 evening 2026)

- ✅ **v0.9 altitude oscillation fix.** `throttle_kp_per_m`
  6.0 → 3.0 collapsed altitude amplitude from ±25 m to ±5 m
  AND eliminated all recovery cycles in mission13. Bonus: proved
  the bit-1 FAILSAFE pulses were aggressive-throttle artifacts,
  not a BF timer.
- ✅ **v0.8.2 recovery flow tuning.** Two-subphase HOLD with
  hover throttle in the second half (`RECOVERY_HOLD_S=1.5`)
  dropped recovery count 53 → 3 over 240 s.
- ✅ **v0.8 dual-write.** `mission_demo` sends RC via MSP TCP
  AND UDP 9004 every tick. Eliminated chronic RX_FAILSAFE bit-2
  latch (mission7=0 hits vs mission6=42).

## Shipped — v0.7 (Apr 27 evening 2026)

- ✅ **End-to-end Isaac Sim flight.** `final_world_betaflight.py`
  orchestrator + `bf_gps_shim` + `mission_demo` flying the
  X-pattern in the chemical-plant world. ARM_SWITCH latch fixed
  via shim send-lock + mission_demo two-stage recovery.

## Shipped — v0.1 (Apr 2026)

- ✅ Architectural plan + 6-phase rollout
- ✅ Python companion library (`companion/racer_companion/`) — MSP v1, P-controller, state machine, safety, 50Hz main loop
- ✅ 48 unit tests covering protocol, math, state, safety
- ✅ Bench tools — MSP loopback test, synthetic GPS replay
- ✅ Betaflight CLI snippets — Phase 0 (GPS Rescue) + Phase 1 (MSP override)
- ✅ Per-phase build docs (00–06) + safety runbook
- ✅ EdgeTX Lua scripts — race-start sequencer, safety lock, race HUD
- ✅ Beginner Lua tutorial + radio setup walkthrough
- ✅ CI on push (pytest + Lua syntax check)
- ✅ Hardware BOM (`HARDWARE_BOM.md`) + drill/incident/sign-off log templates
- ✅ Betaflight SITL test rig (Phase 1 — protocol-layer validation, no physics)

## Shipped — v0.3 (Apr 25 2026)

- ✅ MAVLink Phase 1 — companion-side publisher (`racer_companion/mavlink.py`)
  - Hand-rolled MAVLink v2 encoder for HEARTBEAT, STATUSTEXT, custom COMPANION_STATE
  - UDP + serial backends via `udp://` / `serial://` URIs in config
  - Byte-for-byte validation against pymavlink 2.4.49 in tests
  - Integrated into main loop with rate-limiting (1Hz heartbeat, 5Hz state, status-on-change)
  - QGroundControl on phone setup documented in `docs/mavlink_setup.md`

## Shipped — v0.2 (Apr 25 2026)

- ✅ `MspClient` accepts `tcp://host:port` URI — companion talks MSP-over-TCP
  to BF SITL with zero config changes beyond the port string
- ✅ `sitl/Dockerfile` builds BF 4.5.1 SITL target, applies Phase 0+1 defaults
  on container boot
- ✅ `sitl/docker-compose.yml` for one-line up
- ✅ `companion/tests/test_msp_tcp.py` — TCP adapter tests with embedded
  fake MSP server (always run)
- ✅ `companion/tests/test_sitl_smoke.py` — env-gated smoke test against
  real SITL (MSP_API_VERSION + MSP_FC_VARIANT + SET_RAW_RC round-trip)
- ✅ `.github/workflows/sitl.yml` — manual-trigger CI workflow

## Deferred

### ESP32-S3 MicroPython port

**Status:** deferred. **Trigger to revisit:** weight/power becomes a real constraint, OR Pi Zero 2W supply chain breaks again.

**Why deferred:** Pi Zero 2W is fine for the first build. Real OS, easy SSH debug, journalctl logging, full Python ecosystem. ESP32-S3 would save ~10g and ~300mA but requires:
- Porting `racer_companion/` to MicroPython (no `pyserial` — use `machine.UART`; no `dataclasses`; no `pathlib` — use `os`).
- Reimplementing the JSON-line logger to write to flash without wearing it out.
- Wireless config workflow (no SSH; need a web UI or BLE).
- Re-validating all 48 tests in MicroPython unittest variant.

**Effort estimate:** 2 weekends of focused work. Park until a pilot says "the Pi is too heavy."

### Betaflight SITL — closed-loop physics (Phase 2)

**Status:** Phase 1 (protocol layer) **shipped** in v0.2 — see `sitl/`. Phase 2 (closed-loop physics for tuning) deferred. **Trigger to revisit:** controller tuning becomes the bottleneck.

**What's shipped:** SITL container exposes BF MSP on TCP 5761. Companion connects via `tcp://`. Smoke test validates protocol-layer claims (override semantics, frame parsing). Catches BF-version drift.

**What's deferred:** Gazebo or RealFlight wiring to BF's UDP sensor/motor ports for actual quad physics. Lets us tune `nav.pitch_kp_per_m`/`yaw_kp` against simulated dynamics before the bench. Today's `tools/replay_synth.py` is a poor man's substitute (kinematic only, no real flight model).

**Effort estimate:** 1 weekend to wire Gazebo to SITL, ~1 more to harden + add to CI. Defer until someone says "I want to tune in sim."

### MAVLink — Phase 2 (radio HUD via ELRS-over-CRSF)

**Status:** Phase 1 (companion-side publisher + QGC support) **shipped** in v0.3 — see `docs/mavlink_setup.md`. Phase 2 (radio HUD integration) deferred. **Trigger to revisit:** pilots want companion state on the transmitter screen during flight (vs. on a phone running QGC).

**What's shipped:** Companion emits MAVLink v2 frames (HEARTBEAT @ 1Hz + custom COMPANION_STATE @ 5Hz + STATUSTEXT on safety changes) via configurable backend (`udp://host:port` or `serial:///dev/path[@baud]`). Hand-rolled encoder cross-validated byte-for-byte against pymavlink 2.4.49. Phone running QGroundControl on the Pi's WiFi AP works today.

**What's deferred:** Radio-side rendering. The complexity isn't on the companion side — it's the chain Pi → ELRS RX → ELRS TX → radio MAVLink display. Every link is hardware/firmware variable. ExpressLRS MAVLink-over-CRSF works on some radio + RX combos and fails on others; Yaapu telemetry script works on some EdgeTX color radios; mono radios don't have native MAVLink display.

**Effort estimate:** 1–2 weekends per radio variant. Defer until a pilot says "QGC on phone isn't enough."

**Workaround now:** racehud.lua already shows the race-start state via the in-radio global from racestrt.lua. That's the highest-value display state. Companion-side specifics (distance, safety reasons, vbat from companion's perspective) live on QGC.

### Multi-drone race coordination

**Status:** out of scope.

**Why:** every quad runs independently. Race director's countdown is voice + handheld. Trying to network the radios for synchronized launch is a different project.

### MultiGP / DRL conformance research

**Status:** ask first, build later. **Trigger:** pilot wants to fly this in a sanctioned event.

Most racing leagues require Betaflight-only on race quads. Companion-driven autonomous launch may violate rules. Talk to your event organizer before bringing this to a sanctioned race. Documented in `docs/06_phase4_race_day.md` "Don'ts" section.

### Hardware photos / wiring guides with images

**Status:** add as we build.

**Why:** real photos beat ASCII diagrams, but we don't have the hardware in hand yet. Plan: take photos during the first reference-rig build (Phase 1) and add them to `docs/03_phase1_companion_wiring.md`.

### Pre-commit hooks (ruff/black/lualint)

**Status:** deferred. **Trigger:** 5+ contributors regularly pushing PRs.

Auto-formatting prevents bikeshedding but adds friction. With 4 friends, code review by eyeball is fine. CI catches the regression risks.

### CHANGELOG.md

**Status:** not needed.

`git log --oneline` is the changelog at this scale.

## Things explicitly NOT on the roadmap

These were considered and rejected during the architecture phase:

- **Dual-FC + ESC mux** — DShot is bidirectional half-duplex with RPM telemetry; analog mux glitches it. Cold-gyro handover at race speeds is a 50–200ms crash window. Nobody credible has shipped it. Reject.
- **INAV migration** — single firmware with waypoints + RTH + decent ACRO, but ~90% of Betaflight feel. Violates the 100%-BF-feel requirement the team agreed on. Reject.
- **ArduPilot on race quads** — even with 4.7 + FSTRATE_ENABLE (4kHz rate loop on H7), it's ~90% of BF feel and zero racing community uses it. Wrong tool. Reject.
- **Betaflight master/alpha waypoint code** — `pg/flight_plan.c` + `AUTOPILOT_MODE` work in BF master is real but SITL-only and unproven on hardware. Not flyable. Reject for now (re-evaluate annually).
- **Companion as MAVLink ground station** — Betaflight is telemetry-only on MAVLink, doesn't ingest MAVLink commands. MSP is the only command channel. Reject.
- **ML-based controller** — for a hobby project, a P-controller is debuggable in 50 lines. Failure modes are obvious. Reject.

## How to propose changes to this roadmap

Open an issue (when issue templates exist) or just edit this file in a PR.
Roadmap items should answer: **what trigger makes this worth doing**, not
just "would be cool."
