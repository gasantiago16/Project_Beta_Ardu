# Phase 3 — Field Drills (1 weekend, low-altitude grass field)

## Goal

This is where it has to actually fly. Stage tests bottom-up: hover → short
hop → full launch sequence → failsafe drills → companion-death drills. Each
drill must pass before the next is attempted.

## Pre-flight checklist (every session)

- [ ] Open field, ≥100m clear in all directions, no people downrange
- [ ] Battery alarm threshold set on TX, audible
- [ ] Fresh battery on quad, fresh battery on TX
- [ ] Spotter present (one observer, eyes on quad at all times)
- [ ] Quad in BF **ANGLE mode** — verify on Configurator → Modes tab before disconnecting
- [ ] AUX-companion switch set to LOW
- [ ] GPS lock confirmed in OSD (≥8 sats)
- [ ] Print this drill checklist; sign each line as you go

## Drill 1 — Hover test (props on, low risk)

1. Arm in ACRO/ANGLE. Manual takeoff to ~3m.
2. Switch BF to ANGLE mode (if not already).
3. Flip AUX-companion HIGH.
4. **Expected:** quad holds position. Slight wobble OK; significant drift = abort.
5. Flip AUX-companion LOW. Manual control returns within ~500ms.
6. Manual landing.

✅ Pass = 3 consecutive holds with <2m drift, clean handoff both directions.

## Drill 2 — Short hop (20m waypoint)

1. Set waypoint 20m N of takeoff in `config.json` (use a phone GPS app to find
   the lat/lon, edit the file, restart `racer-companion` service).
2. Arm. Manual takeoff to ~3m. Switch to ANGLE.
3. Flip AUX-companion HIGH.
4. **Expected:** quad climbs to `climb_target_alt_m` (5m default), yaws to
   bearing N, transits forward, holds at waypoint.
5. Flip AUX-companion LOW. Pilot manual flyback and land.

✅ Pass = 3 consecutive successful hops with arrival within `arrival_radius_m`
of waypoint. If overshoots: lower `nav.pitch_max_us`. If undershoots: raise.

## Drill 3 — Full auto-launch sequence

1. Place quad at takeoff pad. Disarm. Power FC.
2. Wait for GPS lock (OSD ≥8 sats).
3. Pilot pre-arm: AUX-companion LOW, ANGLE mode armed.
4. Arm via BF arm AUX.
5. Flip AUX-companion HIGH.
6. **Expected:** companion ramps throttle to hover, holds 3s, climbs to
   target altitude, transits to start-line waypoint, holds.
7. Pilot flips AUX-companion LOW.
8. Pilot flies a manual race lap in ACRO (switch BF mode AUX to ACRO before
   the race starts).
9. Land.

✅ Pass = 5 consecutive auto-launches → handoff → manual lap → land cycles.

## Drill 4 — Failsafe drill (LOAD-BEARING SAFETY TEST)

This is the most important drill. **All Phase 0 GPS Rescue work was for this
moment** — verifying that the companion's MSP traffic does NOT mask the
RX-loss failsafe trigger.

1. Set up as Drill 3.
2. AUX-companion HIGH, quad mid-transit.
3. **Power off the radio (full power-cycle, not AUX kill).**
4. **Expected:** within ~4s (BF `failsafe_delay`), GPS Rescue triggers
   natively. Quad climbs, yaws home, transits, descends, lands within 5m of
   takeoff.
5. Power radio back on, disarm via switch.

✅ Pass = 3 consecutive successful TX-kill recoveries during companion-active
mid-transit. **Failure here means companion MSP traffic is masking the RX
failsafe — fix BF config or upgrade BF version before continuing. Do not fly
Phase 4 race day with this failing.** Reference: BF
[issue #13374](https://github.com/betaflight/betaflight/issues/13374).

## Drill 5 — Companion-death drill

What happens if the Pi crashes mid-flight or the BEC fails?

1. Set up as Drill 3.
2. AUX-companion HIGH, quad mid-transit.
3. **Pull the companion's power** (yank the JST or pull the SD card — disable
   it however your hardware allows).
4. **Expected paths (one of):**
   - **(a)** Companion stops sending MSP → BF MSP-override timeout (~500ms) →
     RX values flow through. Pilot still has TX in hand → can take over
     manually OR flip TX off to trigger GPS Rescue.
   - **(b)** If MSP_RC stops being read by companion, companion stops sending
     SET_RAW_RC. Pilot regains control via real RX within ~500ms.
5. Pilot must demonstrate they can land manually after companion death.

✅ Pass = 3 consecutive companion-power-cuts with manual recovery and safe
landing.

## Drill log

Maintain a log file in this repo (`docs/drill_log.md`, create on first
session). One line per drill attempt:

```
2026-04-26 14:32 | gabriel | Mr. Slippery | drill_4 | PASS | recovery 3.8s, 4m short of takeoff
2026-04-26 14:38 | gabriel | Mr. Slippery | drill_4 | PASS | recovery 4.1s, 6m short
2026-04-26 14:45 | gabriel | Mr. Slippery | drill_4 | FAIL | quad continued autonomous, no rescue. Companion MSP didn't release. Investigating.
```

## Exit criterion

✅ Per drone, all 5 drills passed with required consecutive-success counts.
Drill log signed off by Stream C owner. Per-drone `bf_config/per_drone/...txt`
re-captured with any tuning changes.

→ Continue to [Phase 4 — Race day](06_phase4_race_day.md).
