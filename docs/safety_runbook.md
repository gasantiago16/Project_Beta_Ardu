# Safety Runbook

Read before any field test. This is the cross-cutting safety doctrine that
underwrites every phase.

## Three independent recovery layers

A safe build has three orthogonal ways to bring the drone back. Each must
work without the others:

1. **Pilot stick override.** Pilot drops AUX-companion → companion goes
   silent → BF MSP-override timeout (~500ms) → real RX values flow → pilot
   has the sticks. Owns: intentional handoff, "this is going wrong" abort.
2. **Native BF GPS Rescue.** TX power off → BF detects RX loss → after
   `failsafe_delay` (4s default), GPS Rescue triggers regardless of
   companion state. Owns: lost RC link, pilot incapacitated.
3. **Companion safety lockout.** Stale telemetry / low sats / low vbat /
   geofence breach → state machine forces RELEASED → companion silent →
   pilot or RX takes over. Owns: companion sensor or logic failure.

**Invariant:** any one layer alone must safely return the drone. **Never
fly a build where any layer is degraded.**

## Mode discipline

| BF mode | Companion AUX | Allowed? |
|---------|---------------|----------|
| ANGLE | LOW | ✅ Manual flying, leveling assist |
| ANGLE | HIGH | ✅ Companion-active. **The only autonomous mode.** |
| ACRO | LOW | ✅ Race mode |
| ACRO | HIGH | ❌ **NEVER.** Controller assumes ANGLE; in ACRO it commands rates and the drone will diverge. |
| GPS_RESCUE | any | (auto on failsafe — pilot doesn't select directly) |

The pilot's mode-switch panel layout should physically prevent the
ACRO + AUX-HIGH combination — e.g., put mode and AUX-companion on the same
3-position switch with mappings:
- DOWN: ANGLE + AUX-LOW (manual practice)
- MID: ANGLE + AUX-HIGH (autonomous)
- UP: ACRO + AUX-LOW (race)

This makes ACRO + AUX-HIGH unreachable in any single switch position.

## Pre-flight gate (every session)

Before propellers go on:
- [ ] BF GPS Rescue bench-validated (Phase 0 sign-off in `bf_config/per_drone/`)
- [ ] MSP loopback test passed (Phase 1 sign-off)
- [ ] Companion-death recovery passed (Phase 3 Drill 5 sign-off)
- [ ] TX failsafe configured to drive AUX-companion LOW (so failsafe takes
      AUX out of HIGH automatically — verify in BF Receiver tab with TX off)
- [ ] Battery alarm armed on TX
- [ ] Spotter assigned
- [ ] Field cleared downrange

## Stop conditions (any → land immediately)

- Companion log shows `safety_reasons` repeatedly
- Drone wobbles or oscillates during companion HOLD
- Heading or altitude diverges during companion TRANSIT
- Pi `journalctl` shows repeated `Restart=on-failure` entries
- Any unexplained behavior

Land manually, recover quad, debug on bench. **Do not "try one more time"
in the field.**

## Post-incident protocol

If the drone behaves unexpectedly or crashes:
1. **Don't immediately re-fly.** Pull the SD card from the Pi.
2. Copy `/var/log/racer-companion.jsonl` to dev machine.
3. `jq 'select(.safety_ok == false)' racer-companion.jsonl` — find safety
   trips.
4. Cross-reference with BF Blackbox log if enabled.
5. Root-cause documented in `docs/incident_log.md` (create on first incident).
6. Fix the cause. Re-run the Phase 3 drill that should have caught it.
7. Only after re-passing Phase 3 do you fly again.

## Things that aren't in the safety stack (and why)

- **No automatic emergency landing.** If something goes wrong, we want
  control to revert to humans (pilot via TX or BF GPS Rescue), not for the
  companion to attempt a recovery procedure of its own. The companion is
  intentionally a one-trick pony.
- **No machine-learning anything.** Controller is a P-controller you can
  read in 50 lines. Failure modes are obvious from the code.
- **No over-the-air updates.** Companion software changes require physical
  SD card swap or wired SSH. Prevents bad-deploy mid-race-day.

## Regulatory note (US, 2026)

- FAA Remote ID enforcement is live (since March 2024). Hybrid architecture
  doesn't change RID — your existing RID module broadcasts regardless of
  which side commands motors. Verify it on every drone before flying.
- FRIA / AMA fields exempt RID for racing — fly there if your hardware
  doesn't have RID.
- This system is for hobby / club use. Do not use for commercial operations
  without checking 14 CFR Part 107 implications.
