# Betaflight + Autonomous Nav Integration for Racing Drones

## Context

A group of expert Betaflight racers want to add two autonomous capabilities to their racing quads without sacrificing race-day stick feel:

1. **Autonomous flight to a start-line waypoint**, then manual takeover for the race
2. **Reliable Return-To-Launch on lost RC link** that recovers, stabilizes, and flies home to a pre-set waypoint

The hard constraint is that the *race itself* must feel like Betaflight — full rate/acro, no leveling assist, no nav-stack mush. Deep research found that no single firmware delivers both ends cleanly: Betaflight has no flight-proven waypoint nav, INAV/ArduPilot give ~90% of Betaflight's race feel (a competitive racer notices within a lap), and a true dual-FC + ESC-mux design has never shipped credibly because DShot is bidirectional and cold-gyro handover is a crash window.

The shipped pattern that preserves 100% of Betaflight feel **and** adds real autonomy is **Betaflight + a small companion computer that writes virtual stick inputs over MSP**, with the pilot's transmitter switch handing control back instantly via `msp_override_channels_mask`. Native Betaflight GPS Rescue continues to own the lost-link path independently — even if the companion dies.

This plan proposes a 5-phase rollout starting with the lowest-risk safety win (GPS Rescue alone on the existing fleet) and ending with full autonomous-launch + race + failsafe-RTH.

**Confirmed scope:** 100% native Betaflight stick feel during the race is non-negotiable → companion-computer architecture is correct. Fleet hardware is mixed → Phase 0.5 audit is required before committing per-drone work. This is a group project → workstreams are explicitly parallelizable and called out at the end.

---

## Recommended Architecture

**Single Betaflight FC (H7, 4.5+) + Pi Zero 2W or ESP32-S3 companion + dedicated GPS/baro module.** No second flight controller, no ESC mux, no firmware swap. Companion talks to BF over UART using MSP. Pilot's existing radio is unchanged.

```
+----------------+        UART/MSP        +-----------------+
| Pi Zero 2W /   | <--------------------> | Betaflight FC   | -- DShot --> ESCs
| ESP32-S3       |  MSP_SET_RAW_RC        | (H7, 4.5+)      |
| (companion)    |  RC_OVERRIDE mask      |                 |
|                |                        | GPS Rescue OWNS |
| - GPS read     |                        | the failsafe    |
| - waypoint     |                        | path natively   |
|   logic        |                        +--------+--------+
| - virtual      |                                 |
|   sticks       |                                 | SBUS/CRSF
+----------------+                                 |
                                          +--------+--------+
                                          |   ELRS / Crossfire RX |
                                          +-----------------------+
```

**Why this design:**
- Race feel: 100% native Betaflight — pilot's race lap is identical to today's setup.
- Failsafe is independent: BF GPS Rescue triggers on RC link loss regardless of companion state.
- Handoff: pilot flips an AUX switch → companion sees the AUX state and clears the MSP override mask → real RC takes the sticks within one MSP frame (~20 ms).
- Reversible: pull the companion, you have a normal Betaflight quad.
- Hardware additions are cheap (~$30–50 per drone: Pi Zero 2W or ESP32-S3 + M10/M9N GPS + baro if FC lacks one).

---

## Phase 0 — Safety Win First (1 evening per drone)

Before *any* autonomy work, get every drone in the fleet onto **Betaflight 4.5+ with GPS Rescue properly configured**. This independently solves the lost-link requirement and validates the GPS/baro hardware that Phase 1 depends on.

**Per drone:**
1. Flash Betaflight 4.5 (or 4.6 if H7).
2. Add GPS module (Matek M10Q-5883 or BN-880 — note: prefer **no mag** or disable mag per Betaflight wiki; mag is the #1 cause of GPS Rescue flyaways).
3. Add barometer if FC doesn't have one (DPS310 / BMP388).
4. Configure GPS Rescue:
   - `failsafe_procedure = GPS_RESCUE`
   - `gps_rescue_min_sats = 8`
   - `gps_rescue_alt_mode = MAX_ALT` (climb to max of pilot-set altitude or current)
   - `gps_rescue_initial_climb = 10` m
   - `gps_rescue_descent_dist = 20` m
   - `gps_rescue_landing_alt = 4` m
   - `gps_rescue_throttle_hover` = tuned per quad (typically 1275–1325)
   - **Disable magnetometer** (`set mag_hardware = NONE`) until proven on the bench
5. **Bench validation drill:** Arm in safe location, fly out 50 m, manually flip TX off → quad must climb, rotate, return, descend, land. Repeat 5×.
6. Document `gps_rescue_throttle_hover` per quad in a shared spreadsheet.

**Critical files referenced (Betaflight master):**
- `src/main/flight/gps_rescue_multirotor.c` — state machine: `IDLE → INITIALIZE → ATTAIN_ALT → ROTATE → FLY_HOME → DESCENT → LANDING`
- `src/main/flight/failsafe.c` — `FAILSAFE_PROCEDURE_GPS_RESCUE` chain (lines ~280–340)
- `src/main/pg/gps_rescue.h` — all tunables

**Exit criterion:** Every drone passes 5 consecutive bench failsafe drills without flyaway, drift, or hard landing.

---

## Phase 0.5 — Fleet Hardware Audit (one shared spreadsheet, half a day)

Required because the fleet is mixed. Do this before anyone orders parts.

For each drone, log:
- FC model + MCU (F4 / F7 / H7) — read from BF Configurator → Setup tab
- Free UARTs (need at least one not already used by RX, ESC telemetry, VTX, OSD, RX SmartAudio)
- Onboard baro? (DPS310, BMP388, none)
- Existing GPS? Mag? (most race quads: no)
- ESC protocol (DShot300/600, BLHeli32 vs AM32) — informational only, no change needed
- Available 5V load capacity for adding companion + GPS

**Decision matrix per drone:**
- **H7 + free UART + baro present** → ready for full plan (Phases 1–4)
- **H7 + free UART + no baro** → add baro module before Phase 1
- **F7 + free UART** → Phase 0 GPS Rescue works fine; Phase 1+ companion may run but tight on CPU/UART. Test on one F7 drone before committing fleet.
- **F4** → Phase 0 (GPS Rescue) only. Recommend FC upgrade to H7 before companion work.
- **No free UART** → FC upgrade required. Don't try to share a UART with VTX or RX.

**Output:** shared spreadsheet, one row per drone, with a "Phase target" column (0 / 0+1 / full). Drives BOM ordering and per-pilot work plan.

---

## Phase 1 — Companion Hardware Bring-up (1 weekend)

Wire the companion computer to a spare drone (don't touch the fleet yet).

**BOM per drone:**
- Pi Zero 2W (~$15) or ESP32-S3 DevKit (~$8). Pi gives you a real OS for logging/telemetry; ESP32 is lighter and lower power. **Recommendation: Pi Zero 2W** for the first build (easier debug), migrate to ESP32 once code is stable.
- 5V BEC if not already on FC
- 4-pin JST to FC UART (TX/RX/GND/5V)
- Vibration-isolated mount (3D-printed TPU is fine)

**Wiring:**
- Companion UART → free UART on FC (commonly UART6 or UART4 on H7 boards)
- Set in BF: `feature RX_MSP` is **not** what we want. Instead, keep `serialrx` on the real RX UART, and on the companion UART set `set msp_override_channels_mask = 15` (binary 1111 = roll/pitch/yaw/throttle overrideable).
- Verify `msp_override_channels_mask` is non-zero only when an AUX switch is high (companion-active mode).

**Companion software stub (Pi):**
- Python with `pyMultiWii` or `yaMSP` or raw pyserial.
- Library/protocol reference: Betaflight `src/main/msp/msp_protocol.h` — `MSP_SET_RAW_RC = 200`, payload is 8× `uint16` (1000–2000 µs per channel).
- Loop at 50 Hz writing center sticks (1500/1500/1500/1000) to prove pipe works.
- Read `MSP_RAW_GPS` and `MSP_ATTITUDE` for closed-loop later.

**Bench validation:**
- Props OFF. Arm BF in ACRO. Pull TX sticks to extremes; confirm BF logs the real RC values.
- Flip AUX switch HIGH (companion-active). Companion sends 1500 center sticks.
- Verify: BF rate-output is now zero regardless of TX stick position. AUX-switch LOW → real RC restored within ~50 ms.

**Critical files referenced:**
- Betaflight `src/main/msp/msp_protocol.h` — MSP command IDs
- Betaflight discussion #12615 (Offboard Control with Companion Computer) — canonical pattern
- Betaflight issue #13374 — known caveat: companion-active state must NOT trigger RX failsafe; verify behavior on chosen 4.x branch

---

## Phase 2 — Position Controller on Companion (1 weekend)

Implement a minimal "fly-to-waypoint" controller on the Pi.

**Controller design (intentionally simple — this is not a research autopilot):**
- Inputs: GPS lat/lon/alt (from companion's own GPS module — do **not** rely on FC's GPS for the autonomous controller; keep them isolated so a sensor fault on one doesn't sink both), target lat/lon/alt, current heading from MSP_ATTITUDE.
- Bearing-to-target: standard great-circle math.
- Distance-to-target: haversine.
- Yaw command: P-controller on (target_bearing − current_heading), clamp to ±200°/s rate.
- Pitch command: P-controller on distance, capped at ~15° equivalent stick (mapped to ~1650 µs forward).
- Roll command: 1500 µs (no lateral, just yaw-to-bearing then pitch-forward).
- Throttle: hold-altitude PID using MSP_ALTITUDE, hover throttle from Phase 0 spreadsheet.
- Arrival: distance < 3 m for 2 s → switch to "hold" state (zero pitch, throttle = hover, yaw locked).

**Target waypoint loading:**
- Simple JSON file on Pi: `start_line.json = {"lat": ..., "lon": ..., "alt_m": 5}`
- Loaded at boot. For race day, can be set from a phone over Pi's WiFi AP.

**Bench/SITL validation:**
- Run controller against simulated GPS data (replay a CSV of GPS coords moving toward target). Confirm computed stick outputs are sane.
- Tethered hover test: tie quad to fence post via 3 m line, set waypoint 5 m to one side, watch it pull toward the line.

---

## Phase 3 — Field Integration & Handoff Drills (1 weekend, low-altitude grass field)

This is where it has to actually fly. Stage it:

1. **Hover test:** Arm in ACRO, manual takeoff to 3 m, flip companion AUX HIGH. Quad should hold position. Flip AUX LOW, manual landing.
2. **Short hop:** Set waypoint 20 m away at 5 m altitude. Hand-launch into companion mode. Quad flies to waypoint, holds. Pilot flips AUX LOW → manual flyback.
3. **Full launch sequence:** Place quad at takeoff pad. Companion mode armed. Companion: throttle ramp to hover → hold 3 s → fly to start-line waypoint → hold. Pilot flips AUX LOW, flies the race lap manually.
4. **Failsafe drill mid-companion-flight:** During step 3, mid-transit, kill the TX. **GPS Rescue must trigger natively** (not via companion) and bring the quad home. This is the load-bearing safety test — verify the companion's MSP traffic does NOT mask the RX failsafe trigger. Reference: BF issue #13374.
5. **Companion-death drill:** During companion-controlled flight, pull the companion's power. BF must detect stale MSP, drop the override (real RX takes over), and pilot recovers manually OR — if also TX failsafe — GPS Rescue fires. **Bench-validate stale-MSP behavior on the chosen BF version before this test in air.**

**Exit criteria:**
- 10 consecutive successful auto-launch → handoff → manual race laps
- 3 consecutive successful TX-kill drills with native GPS Rescue recovery
- 3 consecutive successful companion-power-cut drills with manual recovery

---

## Phase 4 — Race-day Rollout (per event)

- Set start-line waypoint from a phone connected to Pi WiFi AP, walk to start grid, place quad, arm.
- Race director countdown: pilots flip AUX HIGH → all quads auto-launch and hold at start gate.
- "GO" → pilots flip AUX LOW → race begins with native Betaflight feel.
- Crash / lost link → GPS Rescue brings the quad to pre-set home waypoint.

---

## Critical Files to Modify / Create

**Betaflight side (CLI config only — no firmware fork needed):**
- Per-drone `bf_dump.txt` (output of `diff` CLI command), versioned in git, including:
  - GPS Rescue parameters (Phase 0)
  - `msp_override_channels_mask` and AUX-conditioned activation (Phase 1)
  - UART assignment for companion link

**Companion side (new repo, e.g., `racer_autonav/`):**
- `companion/main.py` — main loop at 50 Hz
- `companion/msp.py` — thin MSP_SET_RAW_RC / MSP_RAW_GPS / MSP_ATTITUDE / MSP_ALTITUDE wrapper (use `pyMultiWii` if it works on current BF, otherwise raw pyserial against `msp_protocol.h`)
- `companion/nav.py` — bearing/distance math + P controllers
- `companion/state.py` — state machine: IDLE → CLIMB → TRANSIT → HOLD → RELEASED
- `companion/config/start_line.json` — waypoint
- `companion/systemd/racer.service` — auto-start on Pi boot

**Documentation deliverable:**
- `racer_autonav/README.md` — wiring diagram, BF CLI dump template, calibration procedure, drill checklists

---

## What This Plan Explicitly Does NOT Do (and why)

- **No dual-FC + ESC mux.** Research confirmed nobody credible has shipped this. DShot is bidirectional half-duplex with RPM telemetry; an analog switch glitches it. Cold-gyro handover at race speeds is a 50–200 ms crash window. Skip it.
- **No INAV migration.** Single-firmware INAV is the alternative path. It works (waypoints + RTH + decent ACRO), but a competitive Betaflight racer will feel the difference within one lap. Race feel is non-negotiable for this group.
- **No ArduPilot Copter on the race quad.** Even with 4.7 + `FSTRATE_ENABLE` (4 kHz rate loop on H7), it's ~90% of Betaflight feel and zero racing community uses it. Wrong tool.
- **No Betaflight master/alpha waypoint code.** The `pg/flight_plan.c` + `AUTOPILOT_MODE` work in BF master is real but SITL-only and unproven on hardware. Not flyable today.
- **No MAVLink command ingest.** Betaflight only sends MAVLink as telemetry; it does not ingest MAVLink commands. MSP is the only command channel.

---

## Group Project Workstreams (parallelizable from Phase 0.5 onward)

These three streams can run in parallel once the audit is done. Assign one owner per stream.

### Stream A — Companion firmware (1 owner, software-leaning)
- Set up `racer_autonav/` repo
- Write/port MSP wrapper (`pyMultiWii` or raw pyserial against `msp_protocol.h`)
- Implement `nav.py` (bearing/distance + P controllers) + `state.py` (state machine)
- SITL/replay test harness with synthetic GPS data
- systemd service for Pi auto-start
- **Deliverable:** working companion image (Pi SD card or ESP32 firmware) that any team member can flash and use

### Stream B — Betaflight config + per-drone tuning (1 owner, BF-expert-leaning)
- Write canonical `bf_dump.txt` template (GPS Rescue params + companion UART + `msp_override_channels_mask` + AUX-switch mapping)
- Per-drone Phase 0 GPS Rescue tuning (especially `gps_rescue_throttle_hover`)
- Document per-drone CLI diffs in shared repo
- Bench-validate stale-MSP failsafe behavior on the chosen BF version (load-bearing safety check — see Phase 3 step 5)
- **Deliverable:** versioned per-drone config + a "flash + dump" runbook

### Stream C — Field testing + safety drills (1 owner, pilot-leaning)
- Build the Phase 3 drill checklist into a printable card
- Identify safe field locations with GPS lock + open emergency-land area
- Run drills on the reference rig first, then each drone
- Maintain a drill log (date / drone / drill / pass-fail / notes)
- **Deliverable:** signed-off go-live gate per drone before race day

### Sync points
- After Phase 0.5 audit → share spreadsheet, finalize BOM
- After Phase 1 bench tests → all owners review companion+BF wiring on reference rig
- After Phase 2 SITL → group walks through controller behavior in sim
- Before Phase 3 field tests → group safety briefing, drill order locked
- Before Phase 4 race day → 100% drill pass rate per drone, signed off by Stream C owner

---

## Contingency (if hardware audit kills the companion path on too many drones)

If Phase 0.5 reveals the fleet is mostly F4 / no-free-UART, FC upgrades may be cost-prohibitive. **Fallback path:** keep Phase 0 (GPS Rescue) on the existing fleet for the safety win, and migrate only the upgrade-willing pilots to the full companion architecture. Do **not** fall back to INAV (single firmware, ~90% feel) — that violates the "100% BF feel non-negotiable" requirement the team agreed on. INAV stays off the table for this group.

---

## Verification — End-to-End Test Protocol

**Bench (props off):**
- [ ] BF arms in ACRO, real RC controls all axes
- [ ] AUX HIGH + companion sending: real RC inputs are masked, companion stick values appear at output
- [ ] AUX LOW: real RC restored within 50 ms
- [ ] Companion power cut while AUX HIGH: BF reverts to real RC (or failsafe per `msp_override` stale behavior — version-specific, must verify)
- [ ] TX power off: BF triggers GPS Rescue regardless of AUX state

**Tethered hover (3 m fence-post tether):**
- [ ] AUX HIGH: holds position over launch point
- [ ] Waypoint 5 m N: drifts north until tether stops it
- [ ] Throttle hold maintains altitude within ±0.5 m

**Open field, low altitude (≤ 10 m):**
- [ ] Auto-launch to 5 m hover
- [ ] Auto-transit to 50 m waypoint
- [ ] Hold for 5 s
- [ ] Pilot AUX LOW handoff → manual control feels identical to baseline Betaflight
- [ ] TX-kill mid-transit → GPS Rescue brings home to within 5 m of takeoff
- [ ] Companion-power-cut mid-transit → pilot recovers manually

**Race-rep dress rehearsal:**
- [ ] 10 consecutive auto-launch → handoff → manual lap → land cycles, zero anomalies
- [ ] 3 consecutive intentional TX failsafes during race lap, GPS Rescue recovery within 30 s

**Go-live gate:** all bench + tethered + field tests pass; one full dress rehearsal with all participating pilots' drones; lost-link recovery success rate 100% over ≥10 trials per drone.

---

## Decisions Locked Before Implementation

- **Race feel: 100% native Betaflight is non-negotiable** → companion-computer architecture, not INAV migration.
- **Fleet hardware: mixed/unknown** → Phase 0.5 audit gates Phase 1 BOM ordering and per-drone work.
- **Build mode: group project** → three parallel workstreams (companion firmware / BF config / field testing) with named owners and explicit sync points.
- **Safety floor: every drone gets Phase 0 (GPS Rescue) regardless of whether it goes on to Phase 1+.** No drone leaves the bench without a passing failsafe drill.

---

## References

Research that backed this plan:

**Betaflight**
- [Betaflight 4.5 Release Notes](https://betaflight.com/docs/wiki/release/Betaflight-4-5-Release-Notes)
- [Betaflight GPS Rescue Wiki](https://github.com/betaflight/betaflight/wiki/GPS-rescue-mode)
- [Betaflight Magnetometer docs](https://betaflight.com/docs/wiki/guides/current/Magnetometer)
- [Issue #12692 — 4.4.1 GPS lag flyaway](https://github.com/betaflight/betaflight/issues/12692)
- [Issue #14191 — Magnetometer 4.6](https://github.com/betaflight/betaflight/issues/14191)
- [Discussion #12615 — Offboard Control with Companion Computer](https://github.com/betaflight/betaflight/discussions/12615)
- [Issue #12790 — MSP_SET_RAW_RC with serial RX](https://github.com/betaflight/betaflight/issues/12790)
- [Issue #13374 — MSP override + failsafe](https://github.com/betaflight/betaflight/issues/13374)
- [`msp_protocol.h`](https://raw.githubusercontent.com/betaflight/betaflight/master/src/main/msp/msp_protocol.h)

**ArduPilot (evaluated, not selected)**
- [Acro Mode — Copter docs](https://ardupilot.org/copter/docs/acro-mode.html)
- [Aggressive Rate Loop Tuning — Copter docs](https://ardupilot.org/copter/docs/high-loop-rate-tuning.html)
- [Unlocking Faster Attitude Rates — ArduPilot Blog](https://discuss.ardupilot.org/t/unlocking-the-potential-of-faster-attitude-rates-in-copter-control/120743)
- [`mode_acro.cpp`](https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/mode_acro.cpp)

**INAV (evaluated, not selected)**
- [INAV Navigation.md](https://github.com/iNavFlight/inav/blob/master/docs/Navigation.md)
- [INAV Failsafe.md](https://github.com/iNavFlight/inav/blob/master/docs/Failsafe.md)
- [INAV Discussion #10299 — Compass interference](https://github.com/iNavFlight/inav/discussions/10299)

**Companion-computer pattern**
- [FPV Autonomous Operation with Betaflight + RPi](https://medium.com/illumination/fpv-autonomous-operation-with-betaflight-and-raspberry-pi-0caeb4b3ca69)
