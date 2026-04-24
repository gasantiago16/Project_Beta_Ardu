# Start Here

This is the build guide for adding autonomous flight to a Betaflight racing
quad without sacrificing race feel. Read the top-level [`README.md`](../README.md)
first for the architecture decision and rationale (why companion-computer over
INAV / ArduPilot / dual-FC).

## Repo layout

| Path | Purpose |
|------|---------|
| `README.md` | The architectural plan (what + why) |
| `docs/` | Phase-by-phase build instructions (you are here) |
| `companion/` | Python software for the Pi/ESP32 companion |
| `bf_config/` | Betaflight CLI snippets and per-drone dumps |

## Build order

1. **[Phase 0 — GPS Rescue](01_phase0_gps_rescue.md)** — every drone, even
   the ones not getting autonomy, gets BF GPS Rescue. Independently solves
   the lost-link RTL requirement. ~1 evening per drone.
2. **[Phase 0.5 — Hardware audit](02_phase05_hardware_audit.md)** — audit the
   fleet before ordering parts. ~half a day, one shared spreadsheet.
3. **[Phase 1 — Companion wiring](03_phase1_companion_wiring.md)** — wire the
   Pi to the FC, prove MSP comms props-off. 1 weekend per reference rig.
4. **[Phase 2 — Controller install + tune](04_phase2_controller.md)** — install
   the companion software, tune the P-controller in dry-run + replay sim.
5. **[Phase 3 — Field drills](05_phase3_field_drills.md)** — incremental field
   testing with mandatory drills. Sign-off per drone before race day.
6. **[Phase 4 — Race day](06_phase4_race_day.md)** — the actual procedure.

Cross-cutting: **[safety runbook](safety_runbook.md)** — read before any
field test. Stop-conditions, abort signals, post-incident protocol.

## Group ownership

Three parallel workstreams (assign one owner each — see
[main README](../README.md#group-project-workstreams-parallelizable-from-phase-05-onward)):

- **Stream A — Companion firmware:** `companion/` directory. Owner runs the
  pytest suite, builds the deployable Pi image.
- **Stream B — BF config:** `bf_config/` directory. Owner runs Phase 0 tuning
  per drone, maintains per-drone dumps.
- **Stream C — Field testing:** drill execution and sign-off. Owner maintains
  `docs/drill_log.md` (created on first field session).

## Don'ts

- **Don't skip Phase 0.** Even drones never going on to Phase 1 should fly
  GPS Rescue. It's the lowest-risk safety win in the whole project.
- **Don't fly a Phase 1+ build until the bench tests in `companion/README.md`
  pass.** Props-off MSP loopback is mandatory.
- **Don't enable mag** unless you've validated calibration with a dedicated
  off-critical-path flight. Mag flyaways are the #1 GPS Rescue failure mode.
- **Don't run companion-active mode in BF ACRO.** The controller assumes
  ANGLE-mode stick mapping. See [safety runbook](safety_runbook.md).
- **Don't push per-drone dumps that contain receiver bind keys.** Strip those
  lines first.
