# Roadmap

What's shipped today, what's deferred, and the trigger that would make us
revisit each deferred item. Read [`README.md`](README.md) first for the
architectural picture.

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

## Deferred

### ESP32-S3 MicroPython port

**Status:** deferred. **Trigger to revisit:** weight/power becomes a real constraint, OR Pi Zero 2W supply chain breaks again.

**Why deferred:** Pi Zero 2W is fine for the first build. Real OS, easy SSH debug, journalctl logging, full Python ecosystem. ESP32-S3 would save ~10g and ~300mA but requires:
- Porting `racer_companion/` to MicroPython (no `pyserial` — use `machine.UART`; no `dataclasses`; no `pathlib` — use `os`).
- Reimplementing the JSON-line logger to write to flash without wearing it out.
- Wireless config workflow (no SSH; need a web UI or BLE).
- Re-validating all 48 tests in MicroPython unittest variant.

**Effort estimate:** 2 weekends of focused work. Park until a pilot says "the Pi is too heavy."

### Betaflight SITL integration

**Status:** deferred. **Trigger to revisit:** field-test debug cycle becomes the bottleneck.

**Why deferred:** today the test loop is `change code → SCP to Pi → run on bench`. Acceptable for a small team. SITL would let us run companion against a simulated FC + drone in Gazebo entirely on the laptop.

**What it'd look like:** [Betaflight has an experimental SITL](https://betaflight.com/docs/development/autopilot/SITL_Autopilot_Testing_Gazebo) (Gazebo-based). Wire `racer_companion` to it via a virtual serial port. Replace `tools/replay_synth.py` (which fakes only GPS, ignores quad physics) with full closed-loop sim.

**Effort estimate:** 1 weekend to get hello-world working, 2–3 to make it useful. Punt until field iteration is painful.

### MAVLink-over-CRSF telemetry forward (companion → radio HUD)

**Status:** deferred. **Trigger to revisit:** pilots want to see companion state on the radio screen during flight.

**Why deferred:** today's `racehud.lua` reads BF-native telemetry (battery, sats, GPS, RSSI/LQ) but cannot read companion state — no plumbing from Pi → BF → ELRS/CRSF → radio.

**What it'd look like:** companion publishes a small set of MAVLink HEARTBEAT-like packets over MSP-MAVLink-bridge → BF forwards them via [ELRS MAVLink-over-CRSF](https://www.expresslrs.org/software/mavlink/) → radio's MAVLink module surfaces them as telemetry sensors → racehud.lua reads `getValue("Cstat")` etc.

**Effort estimate:** complex. ELRS MAVLink mode is bidirectional but every link in the chain needs config. 1–2 weekends, with risk of integration surprises.

**Workaround in v0.1:** racestrt.lua exposes `raceStartStateName` global that the HUD reads. That's radio-internal state, not companion state, but it's the highest-value thing to display anyway.

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
