# Mission demo — full HITL flight in Final_World

Autonomous Iris flight that exercises every piece of the v0.6 sim stack:
climb → X-pattern across the map → 2× 50 m circles at center → handover
to pilot → simulated loss-of-signal → autonomous landing in the opposite
corner. Designed as a single end-to-end test of the bridge + companion-
adjacent infrastructure.

## Mission sequence

| Phase | Goal | Stops sending MSP? |
|---|---|---|
| `INIT` | Wait for GPS fix + ≥ 8 sats. | n/a (idle) |
| `CLIMB` | Take off + climb to 75 m at SW corner. | no |
| `X_LEG_1` | SW → NE diagonal (first leg of X). | no |
| `X_LEG_2` | NE → SE edge (sets up second diagonal). | no |
| `X_LEG_3` | SE → NW diagonal (completes X). | no |
| `TO_CENTER` | Fly to map center. | no |
| `CIRCLE_1` | First 50 m radius circle at center (~30 s). | no |
| `CIRCLE_2` | Second 50 m circle. | no |
| `HANDOVER` | **MSP off** for 10 s — pilot has channels 1-4 via UDP 9004. | **yes** |
| `LOS_TEST` | Operator should kill `radio_to_bf`; BF will failsafe ~4 s in. | **yes** |
| `LANDING_APPROACH` | Mission resumes MSP, flies to NE (opposite of SW). | no |
| `LANDING_DESCENT` | Throttle below hover at NE; descend until alt < 1 m. | no |
| `DONE` | Mission complete; script exits. | n/a |

The X-pattern visits 4 of the 5 corners (SW start, NE, SE, NW) — that's
two crossing diagonals, hence "X". After circles, the drone parks in
the geographically *opposite* corner (NE) for the autoland test.

## Prerequisites

- Phases 1-6 of `pegasus-bridge` shipped (this branch).
- Final_World USD at the path the orchestrator expects (Phase 2).
- Iris USD bundled with Pegasus (no Phase 5 USD conversion needed unless
  you want to swap to `--airframe racer5`).
- BF SITL Docker container builds + boots cleanly.

## Run — four-terminal workflow

> **Windows pre-flight**: kill any stale Python listener on UDP 19500
> before T2 (PowerShell: `Get-Process python | Stop-Process -Force`).
> Stale listeners with `SO_REUSEADDR` can split-receive motor packets
> and cause the bridge to think it's getting nothing.

```bash
# T1: BF SITL container.
cd ~/Project_Beta_Ardu
docker compose -f sitl/docker-compose.yml up
# Wait for "[sitl] Ready. SITL pid=N" + "Phase echoes confirmed".

# (optional) Confirm the wire path before launching Isaac Sim:
#   python -m integrations.tools.fake_pegasus_loop --duration 3
#   Expect tx=150 rx≈75 in ~38 s — 50% drop is BF main-loop / FDM
#   rate mismatch on a disarmed FC, not a wire bug.

# T2: Pegasus + Iris in Final_World, spawning at SW corner.
#     Bridge auto-binds host UDP 19500 (relay target) for motor RX.
cd ~/PegasusSimulator
python ~/Project_Beta_Ardu/integrations/orchestrators/final_world_betaflight.py \
    --corner SW

# T3 (optional but recommended for the HANDOVER phase): pilot RC.
cd ~/Project_Beta_Ardu
python -m integrations.tools.radio_to_bf
# Or, no radio: --sweep --aux 7=1100 to keep companion-active LOW.

# T4: the mission.
cd ~/Project_Beta_Ardu
python -m integrations.tools.mission_demo
```

## What you should see

In **T2** (Pegasus): `[bridge] tx=N rx=N timeouts=…` ticks up at ~250 Hz
once the bridge is talking to BF SITL. The Iris should sit on the
asphalt of the SW corner.

In **T4** (mission): state machine prints transitions:

```
[mission] INIT → CLIMB
[mission] CLIMB → X_LEG_1
[mission] arrived NE (dist=4.7m)
[mission] X_LEG_1 → X_LEG_2
...
[mission] CIRCLE_1 → CIRCLE_2
[mission] CIRCLE_2 → HANDOVER
[mission] simulating LOSS OF SIGNAL — kill radio_to_bf now if it's running.
[mission] HANDOVER → LOS_TEST
[mission] resuming control after LOS test
[mission] LOS_TEST → LANDING_APPROACH
[mission] arrived NE (dist=3.1m)
[mission] LANDING_APPROACH → LANDING_DESCENT
[mission] LANDED at NE corner (alt=0.85m)
[mission] LANDING_DESCENT → DONE
```

In Isaac Sim's GUI: the Iris flies the X over Final_World's chemical
plant, traces two circles at center, then descends at the opposite
corner. ~3-4 minutes wall-clock.

## Tuning

The mission ships with conservative gains because Iris dynamics + BF's
default PIDs aren't a perfect pair. If the quad oscillates, overshoots,
or stalls in flight, edit `MissionConfig` defaults in
`integrations/tools/mission_demo.py`:

| Symptom | Fix |
|---|---|
| Quad never reaches cruise altitude | Increase `throttle_kp_per_m` or `throttle_max_offset` |
| Hover throttle wrong (Iris lifts off too aggressively / not at all) | Adjust `throttle_hover_us` (default 1500) |
| Slow horizontal travel | Increase `pitch_kp_per_m` and/or `pitch_max_us` |
| Oscillates around waypoint | Reduce `pitch_kp_per_m`, increase `arrival_radius_m` |
| Yaw never catches up to bearing | Increase `yaw_kp` |
| Mission times out before reaching corner | Increase `leg_timeout_s` |

## Map size

`--map-half-size 80` on the orchestrator means corners are ±80 m from
the road junction (160 × 160 m square). The mission script reads its
own `--map-half-size` to compute the same corners relative to wherever
GPS reports HOME. If the orchestrator's spawn corner doesn't match the
mission's "SW", flight will look weird — keep both in sync.

If your scene is bigger or smaller, pass matching `--map-half-size` to
both. Don't let one default to 80 while the other says something else.

## Loss-of-signal test

The `LOS_TEST` phase **demonstrates** that BF SITL's failsafe
(`failsafe_procedure = GPS_RESCUE` per `sitl/defaults.txt`) fires when
RC stops arriving. It does NOT prove the mission cleanly preempts GPS
Rescue afterward — that depends on BF version (4.5.1 holds the rescue
state machine until its own `LANDED` / `EXITED` transition; later
versions may surrender control to incoming RC sooner).

To watch the failsafe path for real:

1. Have `radio_to_bf` running in T4 BEFORE the mission reaches HANDOVER.
2. When the mission prints `simulating LOSS OF SIGNAL — kill radio_to_bf
   now`, kill T4 (Ctrl-C).
3. BF detects the lost RC after `failsafe_delay = 4 s` and starts GPS
   Rescue: climbs `gps_rescue_initial_climb=10m`, heads home (the
   **SW corner**, where the drone first armed — NOT the NE landing
   target), then descends per `gps_rescue_landing_alt`.
4. At second 7 the mission resumes sending MSP_SET_RAW_RC. **In BF
   4.5.1 this does NOT immediately preempt the rescue** — sticks are
   written to `rcData[]` but the rescue state machine continues until
   it auto-completes. Most likely outcome: the drone lands at SW
   (where rescue brought it home), the mission then tries to push the
   landed/disarmed drone toward NE during LANDING_APPROACH, the
   altitude controller saturates, and the mission's descent timeout
   fires the DONE state.

If you want a clean autoland-at-NE without rescue interference, the
fastest fix is to swap `failsafe_procedure = GPS_RESCUE` for
`failsafe_procedure = DROP` in a sim-only `defaults.txt` override.
That way LOS_TEST observes simple RC-stale failsafe (drop), then the
mission resumes and flies to NE. Real-flight `defaults.txt` keeps
GPS_RESCUE — that's the safety primitive every real drone needs.

For the demo as-shipped (with GPS_RESCUE on), expect: rescue brings
drone back to SW corner, mission fights briefly, landing finishes
near SW, runbook is over-promising. **This is a known limitation —
fix is in the runbook above, not in code.**

## Known issues

- **ANGLE mode is mapped via `aux 0 0 0 900 2100` in `sitl/defaults.txt`.**
  This is sim-only — ANGLE is forced on regardless of AUX1 value so the
  controller (which assumes ANGLE per `nav.py`) doesn't diverge in BF's
  default ACRO. Real-flight builds remove this and bind ANGLE to a pilot
  switch. If you delete the `aux` line by accident, expect immediate
  divergence after CLIMB.
- **Iris hover throttle = 1300 µs** to match
  `sitl/defaults.txt`'s `gps_rescue_throttle_hover`. Earlier drafts used
  1500 — that saturated the climb-throttle hard cap and Iris would blow
  past 75 m. If first flight overshoots cruise, re-confirm this value
  matches your BF SITL build.
- **No mission-status MAVLink output.** The mission just logs to stdout.
  Future: emit COMPANION_STATE-style messages so QGC can show progress.
- **Circle direction is fixed (CCW from above).** The script flies the
  circle moving target counterclockwise. Reverse by flipping the sign on
  `east_off` in `_circle_target`.
- **No abort.** Ctrl-C kills the mission immediately. The drone just stops
  receiving MSP, BF takes over via failsafe — usually safe, but document
  before pilots watch a demo.
