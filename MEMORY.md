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

## Snapshot — v0.8 (Apr 28 PM 2026, dual-write kills chronic RX_FAILSAFE)

| Component | State |
|---|---|
| `mission_demo` UDP 9004 dual-write (TODO #1a) | ✅ shipped + verified. Same RC values flow to both MSP TCP 5761 and UDP 9004 every tick. Default ON; `--no-udp-rc` to disable. |
| Chronic RX_FAILSAFE bit-2 latch | ✅ **eliminated**. mission7 = 0 RX_FAILSAFE hits in 240 s of `[rc]` logs. mission6 (yesterday, same setup minus dual-write) = 42 hits. Architecturally clean fix — single source of truth for RC means no precedence fight. |
| `bf_rc_keepalive` standalone tool | ⚠️ obsoleted by dual-write but kept in tree for HANDOVER-phase pilot-input testing. Wire format pin still useful. |
| Remaining bit-1 FAILSAFE pulses | ⚠️ open. Mission7 had ~10 s recovery cadence vs mission6's ~25-30 s — actually faster, with `flags=0x2000080` (THROTTLE+ARM_SWITCH) and `0x2000082` (FAILSAFE+THROTTLE+ARM_SWITCH) showing. Source unknown — separate from RX state machine. Recovery flow handles each. |
| X-pattern flight quality | comparable to mission6. Drone alive 240 s, climbs to 27 m peak, sustained 5-17 m through CIRCLE_2 phase. Legs still time out 33-42 m short of corners. |

### v0.8 changes — what landed in this session (Apr 28 PM)

- **`integrations/tools/mission_demo.py`** — added UDP 9004 dual-write
  alongside existing MSP_SET_RAW_RC. New CLI args:
  `--udp-rc-host`, `--udp-rc-port`, `--no-udp-rc`. Default is dual-
  write ON. The same `rc[]` list `compute_rc()` produces is sent on
  both paths every tick — identical 8 channels via MSP, padded to 16
  via UDP. Send errors counted, never raised. Socket `connect()`
  pattern matches `radio_to_bf` / `bf_rc_keepalive` (Windows ICMP
  unreachable handling).
- Counter-agent reviewed; 4 NITs found (none blockers). Applied:
  - HANDOVER/LOS_TEST gating doc note added to constants block
  - Pre-existing duplicate `rel = ...` line cleaned up
  - Skipped: lift-out-of-snap-gate optimization (premature) and
    BF-restart-reconnect logic (not blocking)

### v0.15 — FPV camera tracks the drone, MP4 video recording (Apr 29 late evening)

User asked: "Is the fpv able to move with the drone?" — and v0.14
empirically failed: screenshots at drone z=5 m and z=78 m looked
identical, proving Pegasus's `/World/quadrotor` parent prim doesn't
inherit physics-driven motion. Pegasus moves a deeper rigid-body
prim, not the named root.

**Fix.** Moved the FPV camera to top-level `/World/fpv_camera`
(unparented). Per-tick in the orchestrator main loop, read
`bf_backend._latest_state` and set the camera's world transform to:

- world position = drone_pos + R(drone_attitude) @ body_offset
  where body_offset = (0.3, 0, 3.0) in body FLU
- world rotation = drone_attitude * fpv_body_local_rot
  where fpv_body_local_rot is the (-15, -90, -90) Euler that maps
  default USD camera (look -Z, up +Y) to body (look +X, up +Z)

Both composed via scipy `Rotation`, output Euler XYZ to USD.

Verified by re-running with periodic PNG captures: at drone (5 m
alt, near spawn) the FPV view shows the chemical-plant walkway
from above; at drone (~80 m alt) the view is mostly sky with the
ground markers as tiny dots. Camera tracks position AND orientation.

**MP4 video recording.** User request: "save the recordings in
new file my desktop, do that from now on." Added `--fpv-video-fps`
(default 5) + `--fpv-video-out-dir` (default `~/Desktop`). Each
tick captures a numbered PNG to a temp staging dir; on graceful
shutdown the `finally:` block runs ffmpeg (bundled via
`imageio_ffmpeg`) to encode `fpv_mission_<timestamp>.mp4`. Frames
are deleted on success, preserved if encode fails.

**Shutdown signal mechanism.** Stop-Process -Force ==
TerminateProcess on Windows; Python's `finally:` does NOT run, so
the encode would never fire after a force-kill. Added a polling
mechanism: orchestrator watches
`{tempdir}/bf_orchestrator_shutdown.signal`. Touch the file → main
loop breaks naturally → finally runs → MP4 lands on Desktop.

**Stale-frames recovery.** If the orchestrator crashes hard
without running `finally:`, the staged frames live in
`{tempdir}/fpv_video_frames_*/` and can be encoded manually:

```
ffmpeg=$(python -c "import imageio_ffmpeg as i; print(i.get_ffmpeg_exe())")
"$ffmpeg" -framerate 5 -i frame_%06d.png -c:v libx264 \
  -pix_fmt yuv420p out.mp4
```

**Caveat.** This run's mission flight was bad (drone climbed to
~100 m, throttle below hover at 1602 µs but altitude rising). The
shim's altitude reading appears to be ~half of true physics
altitude — controller fights itself. Separate from FPV work.
v0.14's tilt feedforward + altitude tuning still need iteration.

### v0.14 — FPV camera side quest (Apr 29 evening)

User asked: "could we put the camera in fpv mode so we can see what
the drone sees as it flies around?" Added `--fpv-camera` flag to the
orchestrator. Two cameras created when set:

1. `/World/debug_world_camera` — world-fixed at (sx, sy-10, sz+5)
   looking at spawn. Isolates "viewport switch works" from
   "drone parenting works." Operator-switchable from the viewport
   menu if the drone-parented camera misbehaves.
2. `/World/quadrotor/fpv_camera` — parented under the Iris, body
   local (0.3, 0, 3.0) — 3 m above the drone so the view clears
   the chemical-plant walkway directly above the spawn point.

**Three things broke and how they got fixed:**

- **All-black image** at body-height camera positions. Not occlusion
  by the drone mesh — the Iris in this scene spawns DIRECTLY UNDER
  A WALKWAY in the chemical plant. Any camera within ~2 m of body
  height was inside the walkway structure. Confirmed by lifting
  the camera to (0, 0, 50): clear top-down view. Settled on Z=3.0.
- **Rotation order gimbal lock.** With Y=-90 (yaw camera to look
  body +X), Z and X rotations co-axial about the view axis. Z=+90
  was the wrong sign of roll; Z=-90 gives camera-up = body +Z.
  Final rotation: `(-15, -90, -90)` for 15° pitch-down FPV.
- **Spawn marker engulfing camera.** v0.12's spawn-marker sphere
  was 1.5 m radius, centered exactly on the camera at low offsets.
  Shrunk to 0.3 m so it doesn't render over the FPV view.

**Programmatic screenshot capture.** Added a one-shot
`capture_viewport_to_file` after 20 warm-up `world.step(render=True)`
ticks (so the parented camera resolves its world transform before
capture). Saves to `fpv_screenshot.png` at project root. Lets the
assistant `Read` the rendered PNG to verify orientation without the
human in the loop — was the unblocking move for the rotation
debugging.

### v0.13 — tilt-throttle feedforward (Apr 29 afternoon)

Picked up TODO #11 with the right hypothesis this time: mission16's
trace showed altitude sagging to 2-3 m during pitched flight because
`cos(tilt)` reduces vertical thrust faster than the pure-P altitude
controller can react. Added a feedforward term that boosts throttle
proportionally to commanded forward pitch — anticipates the lift loss
instead of waiting for the alt-error feedback to catch up. New
`MissionConfig.tilt_throttle_factor: 0.15`. Throttle authority is
the existing P term + feedforward, then clamped to ±200 µs.

**mission20 (factor=0.25) vs mission21 (factor=0.15) vs mission16:**

| | mission16 (no FF) | mission20 (0.25) | mission21 (0.15) |
|---|---|---|---|
| X_LEG_1 short | 11.3 m | 13.3 m | 15.0 m |
| X_LEG_2 short | 12.6 m | **8.5 m** | 18.0 m |
| X_LEG_3 short | 33.7 m | 31.3 m | **14.1 m** |
| recoveries / 240 s | 2 | 19 | 10 |
| max altitude peak | 27 m | 43 m | 33 m |

**Why 0.15 wins on net.** mission20 with 0.25 saw the drone climb
to 43 m and crash to 0 m three times — feedforward + alt-P
both pushing throttle up when below target = throttle saturated, big
overshoot. 0.15 reduces the boost so the alt-P term stays in charge
overall. The closest result on any single leg (X_LEG_2 at 8.5 m, just
3.5 m from arrival) came at 0.25 but cost altitude stability and
3× the recoveries; 0.15 trades that single best for **far better
X_LEG_3 (the long diagonal) and noticeably calmer altitude**.

**Trace tooling paid off again.** The mission16 decile data ruled out
yaw-gate and pitch-saturation, pointed at altitude sag during tilt as
the real problem. Without the per-tick CSV from v0.10's
`--waypoint-trace-csv`, we'd have iterated more variables blindly.

3 new tests in `TestTiltThrottleFeedforward`: hover-no-pitch baseline,
forward-pitch-adds-proportional-boost, factor=0 disables. All 25
targeted tests pass.

### v0.12 — in-sim visual UX (Apr 29 mid-morning)

User asked for two operator-visibility improvements while watching
mission19 in Isaac Sim:
> Different color Shapes on the swan point and the way points it
> needs to land on in the sim and also have the sim spwan a waypoint
> bubble where the sim wants it ro go every 25 m so we can see if
> we're contoling or just guessing

Shipped:

1. **Static colored marker spheres** placed at orchestrator startup:
   spawn (green), SW (blue), SE (magenta), NE (orange), NW (yellow),
   CENTER (white). Positioned via Pegasus ENU coords from `args.spawn`
   ± `args.map_half_size`. Lives at `/World/markers/{name}` USD
   prims with `displayColor` attribute set per marker.

2. **Dynamic "target bubble"** at `/World/markers/target` (red, 1m
   sphere). Orchestrator polls `tempfile.gettempdir()/bf_target_
   state.txt` ~10 Hz; mission_demo writes to it every tick from
   `_publish_target_state()` containing home-relative N/E/alt for
   the current waypoint. Orchestrator translates target marker prim
   to match. Operator can SEE whether drone is following commanded
   target or drifting.

The "every 25 m" trail of breadcrumbs the user mentioned isn't shipped
yet — current target bubble is a single-prim live position. If we
need a trail, add a deque of older targets and render N spheres from
that. Marked as TODO #12.

Helper added: `Mission._active_target(now)` — unifies waypoint-vs-
circle target lookup so both `_publish_target_state()` and future
callers don't need to know which phase uses which.

CLI flag `--target-state-file` added to `mission_demo` (default
matches `DEFAULT_TARGET_STATE_FILE`); the orchestrator reads from
the same default. Empty string disables.

### v0.11 — flip detection + respawn signal (Apr 29 morning)

Today's defensive day. Three failed experiments + two real ships:

**What we tested and reverted (one-variable-at-a-time):**

| run | change | result | reverted? |
|-----|--------|--------|-----------|
| mission16 | `pitch_max_us 200→300` | X_LEG_3 41.8→33.7 m short, but stalled at 33.7 m for 30 s (drone hit obstacles at low alt) | yes (back to 200) |
| mission17 | `cruise_alt_m 15→25` | X_LEG_3 73 m short, drone got stuck at SE-area obstacle for 115 s | yes (back to 15) |
| mission19 | `--map-half-size 15` (smaller X-pattern) | drone climbed to 26 m then **flipped** during pitched flight, crashed | n/a (CLI flag, not default) |

The trace decile data on mission16 surfaced the real coupling:
drone tilts forward → cos(tilt) lift loss → altitude drops to 2-3 m
during pitched flight → drone hits chemical-plant obstacles. Higher
cruise alt (mission17) just changed which obstacles. Smaller map
(mission19) made the drone flip on hard-pitched commands.

**What we shipped:**

1. **Flip detection in `mission_demo.Mission.update()`.** Tracks
   `_flip_tick_count`. If `|roll|>90°` or `|pitch|>90°` AND phase
   not in {INIT, ARM, DONE}, increment counter. After
   `FLIP_REQUIRED_TICKS = 25` (0.5 s at 50 Hz) consecutive flipped
   ticks, log + force `Phase.DONE`. Counter resets on any tick
   with normal attitude. Avoids false positives from transient
   gimbal-lock readings during INIT/ARM transitions.

2. **Respawn signal protocol.** When flip-induced DONE fires,
   `mission_demo` writes `tempfile.gettempdir()/bf_respawn.signal`.
   The Pegasus orchestrator polls the path ~10 Hz; when present
   it calls `world.reset()` (returns dynamic prims to initial
   poses → Iris back at spawn, attitude cleared) then
   `timeline.play()`, then deletes the signal. Lets the operator
   iterate mission_demo runs without restarting Pegasus + Iris
   Sim. `world.reset()` is safe — the BetaflightUdpBackend's
   `reset()` drains stale UDP packets without closing the socket,
   so BF SITL in Docker is unaffected.

3. **`--respawn-signal-file` CLI on both `mission_demo` and
   `final_world_betaflight.py`** so the path can be overridden in
   lockstep. Default matches; mismatch silently disables the
   respawn loop.

5 new tests in `TestFlipDetection` (brief flip OK, sustained →
DONE, counter resets, pitch flip also detected, INIT phase
skipped). All 22 targeted tests pass.

**Counter-agent reviewed; 2 SHOULD-FIX items applied:**
- Orchestrator now reads its own `--respawn-signal-file` flag (was
  hardcoded; would silently disagree with mission_demo's CLI value).
- mission_demo pre-clears the signal at startup (was: only
  orchestrator pre-cleared, leaving a window for ghost-signal-
  triggered respawns).

### v0.10 — drone flies the X-pattern; X_LEG_2 within 10 m of corner (Apr 28 late night)

Diagnose-before-tune via counter-agent recipe. Added `--waypoint-
trace-csv PATH` flag to `mission_demo` that dumps `(t, phase,
rel_alt, distance_m, h_err, yaw_aligned, pitch_us, throttle_us,
target)` per tick from `_compute_waypoint_rc`. mission14 trace
showed yaw_aligned 90-98 % of ticks (not the bottleneck) and
pitch_us never saturated (avg 1568-1575 µs of max 1700) — both
counter-agent hypotheses ruled out cleanly.

Real bottleneck: pitch was deliberately weak. `pitch_kp_per_m =
2.0 × distance=37 m = +74 µs` forward stick — too gentle for Iris
to make headway. Drone stalled at 31-34 m from corner.

**Fix:** `MissionConfig.pitch_kp_per_m: 2.0 → 4.0` (matches
`racer_companion/nav.py::NavTuning` default — mission_demo's 2.0
was the outlier).

**mission14 (kp=2) vs mission15 (kp=4)** end-of-leg distances:

| leg | mission14 end-dist | mission15 end-dist |
|-----|--------------------|--------------------|
| X_LEG_1 (toward NE)   | ~32 m | **18.8 m** |
| X_LEG_2 (toward SE)   | ~34 m | **10.3 m** ← 5 m would be "arrived"! |
| X_LEG_3 (toward NW)   | ~32 m | 41.8 m (long diagonal, saturates pitch) |

Recovery count still 0; altitude stable 8-11 m. Mission15
trajectory plot shows actual X-pattern shape — the drone visits
NE area (peak ~26 m N), traverses south near SE, swings northwest.
Lateral excursions ~25-30 m vs mission13's 7 m.

**What's next (TODO #11):** X_LEG_3 is the long diagonal SE→NW
(~85 m). With pitch_kp=4 and pitch_max_us=200, pitch saturates at
distance ≥ 50 m, so the drone has +200 µs forward stick for the
first 35 m — but the leg is still long and the drone struggles.
Bumping `pitch_max_us` from 200 to 300 (gives +300 µs at full
saturation) is the obvious next single-variable test.

### v0.9 — altitude controller gentled, latch chain ELIMINATED (Apr 28 night)

`throttle_kp_per_m` lowered from 6.0 to 3.0 in `MissionConfig`.
`max_offset` stays at 200 (so saturation still gives full ±200 µs
authority at err=±67 m, plenty of climb headroom).

**mission13 vs mission12** (one-variable change):

| | recoveries / 240 s | RX_FAILSAFE | altitude amplitude | phase reached |
|---|---|---|---|---|
| mission12 (kp=6.0) | 3 | 0 | ±25 m around 15 m | TO_CENTER |
| mission13 (kp=3.0) | **0** | 0 | **±5 m around 12 m** | CIRCLE_1 |

The aggressive kp=6 throttle commands were apparently the trigger
for the residual bit-1 FAILSAFE pulse we'd been chasing since
v0.8 (TODO #2 → mitigated v0.8.2 → no recoveries observed v0.9
on n=1 240 s run). Hypothesis: gentler throttle → fewer state-
machine perturbations in BF → no failsafe trip → no ARM_SWITCH
latch → no recovery cycles. Needs 3-5 reruns to confirm (mission7's
lucky single-run lesson — see v0.7.1 sharp edge).

The recovery flow itself (LOW + 0.3 s idle HOLD + 1.2 s hover HOLD)
stays in place as a safety net — it just doesn't fire anymore in
typical Iris flight.

**What didn't change:** legs still time out 28-49 m short of the
30 m corners. The drone is flying stably but the X_LEG bearing /
pitch controller doesn't push hard enough to reach corners within
the 60 s leg timeout. Different problem; queued as TODO #10.

**Companion JSON configs still hardcode kp=6.0** in
`companion/config/start_line.example.json` and
`companion/config/sim_waypoint.json`. They feed a different code
path (`racer_companion/nav.py::NavTuning`, not `mission_demo.
MissionConfig`), so this kp=3.0 ship is correctly scoped — but
anyone wiring the racer_companion to drive this same Iris+BF SITL
stack will trip on the same oscillation. Not fixing in v0.9
because it's outside the mission_demo scope; flag for whoever
takes the racer_companion sim-bring-up pass next.

### v0.8.2 — recovery flow tuned (Apr 28 evening)

After v0.8.1's failed `failsafe_throttle_low_delay` experiment, two
local tweaks to `mission_demo.compute_rc()`'s recovery flow gave
real per-tick wins without changing BF settings:

1. **`RECOVERY_HOLD_S` 0.6 → 1.5 s.** mission11 vs mission10:
   recovery count dropped 53 → 5 over 240 s. Longer HOLD lets BF
   fully settle internal failsafe state before normal flow resumes,
   so the relatch doesn't fire 1.5-3 s later.
2. **Two-subphase HOLD** (new `RECOVERY_HOLD_IDLE_S = 0.3 s`). First
   0.3 s of HOLD keeps throttle idle (preserves the THROTTLE bit
   clear at the LOW→HIGH AUX1 transition). Remaining 1.2 s raises
   throttle to hover so the drone doesn't free-fall through HOLD.
   AUX1 stays HIGH the whole HOLD — no new transition, no fresh
   ARM_SWITCH-latch opportunity. mission12: 3 recoveries / 240 s,
   peak altitudes 36-40 m (drone stays high through cycles).

Both changes shipped in `integrations/tools/mission_demo.py` plus
two new tests in `test_mission_demo.py::TestArmSwitchRecovery`
(`test_hold_idle_subphase_keeps_throttle_low`,
`test_hold_hover_subphase_keeps_drone_at_altitude`).

**New bug exposed by the fix:** mission12's altitude trace shows
40 m peak / -16 m trough oscillation with ~50 s period during
normal flight (independent of the recovery flow). Lateral progress
is still limited because the wild altitude swings starve the X_LEG
yaw/pitch controller. Tracked as TODO.md item 9. Likely root cause:
mission_demo's CLIMB altitude controller has `max(40, ...)` clamp
that makes it climb-only — once above target, no descent thrust.
The X_LEG waypoint controller does have bidirectional altitude
control but apparently can't catch up.

### v0.8.1 follow-up — Apr 28 PM cautionary tale

Same-day attempt to close the residual bit-1 FAILSAFE pulse via
`set failsafe_throttle_low_delay = 200` in defaults.txt. Hypothesis:
the ~10 s cadence matched the setting's default (100 × 0.1 s).
**Empirically WORSE**: mission8 had recovery firing every ~3 s and
the drone never lifted off ("CLIMB timeout AND rel_alt 0.00m < 5.0m").
mission8 was killed mid-run, so only `mission8_md.log` exists — no
png/csv artifact for that one. After reverting, mission9 also
unexpectedly failed (drone got to 2.14 m before timeout — likely a
flake from the docker-compose down/build cycle). mission10 (clean
container restart) confirmed v0.8 baseline behavior persists (53
recoveries / 240 s, drone progresses through TO_CENTER but crashes
during CIRCLE_1). Mission7's clean reach to CIRCLE_2 was a
particularly lucky run, not the typical case.

DO NOT add `set failsafe_throttle_low_delay = 200`. The setting's
behavior in BF 4.5.1 SITL doesn't match the docs (or my
understanding of them). Either the units are different, or it
controls something other than "throttle-low timeout." Walk BF source
`betaflight/4.5.1/src/main/flight/failsafe.c` before guessing again.

### v0.8 sharp edges

- **Two bit-1 FAILSAFE triggers exist in BF SITL.** The original
  RX_FAILSAFE → STAGE2 path (bit 2 → bit 1 after 20 s) is dead now
  — dual-write keeps bit 2 quiet. But mission7 still saw bit 1 fire
  on a ~10 s cycle without bit 2 ever appearing. Source unknown
  pending next session's investigation. Don't assume "no bit 2 = no
  bit 1"; check both. Possible candidates: `failsafe_throttle_low_
  delay`, BOXFAILSAFE, or some BF SITL-specific timer.
- **Wire format `<d16H` 40 B is now pinned in THREE places** —
  `bf_rc_keepalive.RC_PACKET`, `radio_to_bf.UDP_RC_PACKET`, and
  `mission_demo._UDP_RC_PACKET`. All three must agree or BF silently
  drops. The `assert _UDP_RC_PACKET.size == 40` lines are
  load-bearing; do not remove them in a refactor.
- **`should_send_msp()` now gates UDP too.** During HANDOVER/LOS_TEST,
  mission_demo stops sending BOTH MSP and UDP. That's the right
  semantic (radio_to_bf owns UDP 9004 during HANDOVER; LOS_TEST
  needs RC silence to trigger BF's failsafe). If a future maintainer
  wants UDP keepalive even during HANDOVER, they'll need a separate
  gate.

## Snapshot — v0.7.1 (Apr 28 2026, RX-stall diagnostic + keepalive parked)

| Component | State |
|---|---|
| Diagnostic timing in `bf_gps_shim` (--debug-timing) | ✅ shipped. Tracks RC-forward gaps, state-file read times, cache-refresh bursts. Off in production. |
| RX-stall ROOT CAUSE | ⚠️ identified, not fixed. NOT state-file IO (only 50-68ms reads observed, well under 200ms threshold). Real cause: BF SITL's RX state machine never sees MSP overrides as "real RX", so `arm=0x4[RX_FAILSAFE]` is set chronically. After `failsafe_delay = 200` (20s) of bit-2 set, BF transitions to STAGE2 → bit 1 FAILSAFE fires → with AUX1 high, ARM_SWITCH latches. |
| `bf_rc_keepalive` UDP 9004 RX heartbeat tool | ✅ shipped, **parked**. Sends 40-B `rc_packet` to UDP 9004 at 50 Hz. Verified BF clears RX_FAILSAFE when it flows (`arm=0x0[ARMABLE]` confirmed via direct MSP query). But integration with mission_demo's MSP overrides is broken (see below). |
| `msp_override_failsafe = ON` experiment | ❌ tried, reverted. Setting name suggests "MSP overrides always-active including failsafe" but empirically the OPPOSITE: ON disables MSP overrides during normal flight, drone unarmable. Reverted to default OFF. Documented in defaults.txt comment. |
| `mission6.png` flight | ✅ same as v0.7 mission4. 240s alive, 5 climb-fall cycles to ~27m, completes through CIRCLE_2 phase. Recovery handles each ~25-30s latch. Drone moves around the map but legs time out short of corner waypoints. |

### v0.7.1 changes — what landed in this session

- **`integrations/tools/bf_gps_shim.py`** — added opt-in `--debug-timing`
  flag plus three instrumentation points: RC-forward gap detection in
  `_consume_requests`, state-file read timing in `PositionIntegrator
  ._loop`, cache-refresh burst timing in `_cache_refresh_loop`. All
  gated by `cfg.debug_timing` so production remains a no-op. Counter-
  agent reviewed; one dead-line cleanup applied.
- **`integrations/tools/bf_rc_keepalive.py`** — new tool, 40-B UDP RC
  heartbeat at 50 Hz to BF SITL's port 9004. Wire format pinned in
  `test_bf_rc_keepalive.py` (matches `radio_to_bf.py`). Tool works
  standalone (BF reports ARMABLE) but coexistence with mission_demo
  MSP overrides is unresolved. Counter-agent caught a BLOCKER (AUX1
  default arming the box prematurely), a SHOULD-FIX (Windows ICMP
  unreachable latching the socket — fixed via connect()+send()
  pattern matching radio_to_bf), and an arm-band test pin. All
  applied. Default flipped twice during the session as we tried
  different precedence stories.

### v0.7.1 sharp edges

- **`msp_override_failsafe = ON` BREAKS MSP overrides during normal
  flight.** Despite the setting name, ON apparently means "MSP
  overrides are ONLY effective during failsafe" — opposite of what
  the docs imply. Drone became unarmable. Stay at default OFF.
  Defaults.txt now has a "DO NOT" comment to prevent re-tripping
  this footgun.
- **Keepalive vs MSP override on AUX1 — precedence is unclear.** With
  keepalive AUX1=LOW + mission_demo MSP override AUX1=HIGH, the
  keepalive value won and BF never armed (counter-intuitive; with
  mask=255 + msp_override_failsafe=OFF the override SHOULD apply).
  With keepalive AUX1=HIGH from boot, BF latched ARM_SWITCH on the
  initial AUX1 LOW→HIGH transition (BAD_RX_RECOVERY briefly set).
  No safe coexistence found in this session. Tracked as TODO #1.
- **Phase-2 UDP-init flake worsens with port reuse.** Today verify_
  sim_arms repeatedly failed on the default port 38500 after several
  container restarts (Windows Firewall heuristic blocks high-traffic
  UDP ports after enough rebinds). Restart-and-retry-with-different-
  port works around it but is annoying. The orchestrator's ephemeral-
  port picker (`_pick_motor_port`) handles this correctly.
- **Recovery flow remains the load-bearing safety net.** Even with
  the diagnostic + keepalive infrastructure, the actual flight
  reliability still comes from `mission_demo.py`'s two-stage
  ARM_SWITCH recovery (LOW + HOLD idle throttle, RECOVERY_HOLD_S =
  0.6 s). DO NOT remove this code path.

## Snapshot — v0.7 (Apr 27 2026, end-to-end Isaac Sim flight)

| Component | State |
|---|---|
| End-to-end mission in Isaac Sim (Iris + BF SITL + Pegasus + shim + mission_demo) | ✅ Flying. `mission4.png` shows climb-X_LEG_1-X_LEG_2-X_LEG_3-TO_CENTER over 220 s, surviving 6 mid-flight latches via two-stage recovery |
| ARM_SWITCH latch — terminal mid-flight disarm | ✅ Fixed. Two complementary patches landed. |
| `bf_gps_shim` MSP-frame corruption (concurrent sendall) | ✅ Fixed. Per-socket `threading.Lock`s on `_send_up` / `_send_down`. |
| `mission_demo` two-stage ARM_SWITCH recovery (LOW + HOLD idle throttle) | ✅ Fixed. `RECOVERY_HOLD_S = 0.6` so LOW→HIGH AUX1 transition lands on clean ARMABLE. |
| `failsafe_delay = 200` (20 s) in `defaults.txt` | ✅ Bakes via `eeprom_bake` Dockerfile stage; closes the longer-period RX_FAILSAFE path. |
| Live state-file publishing from orchestrator | ✅ Pegasus's `_latest_state` now writes `n e alt yaw` at 10 Hz so `bf_gps_shim` synthesizes MSP_ALTITUDE/RAW_GPS from real Pegasus physics. |
| Recurring ~18 s RX-stall under sustained Isaac Sim load | ⚠️ Open. Recovery handles it cleanly, but it makes leg arrivals overshoot timeouts. See TODO.md item 1. |
| Phase-2 UDP-init flake at SITL boot | ⚠️ Unchanged from v0.6.1; container restart unblocks. |

### v0.7 changes — what landed in this session

- **`integrations/tools/bf_gps_shim.py`** — `ClientForwarder` now has
  `_up_send_lock` and `_down_send_lock`; all `up.sendall` / `down.sendall`
  call sites go through `_send_up()` / `_send_down()` helpers which hold
  the appropriate lock. Two threads (the d→u request forwarder and the
  cache-refresh poller) used to race on the upstream socket, interleaving
  bytes mid-MSP-frame and corrupting `MSP_SET_RAW_RC`. BF treated the
  corrupted frames as bad RX, eventually tripping `BAD_RX_RECOVERY` (bit
  3) and latching `ARM_SWITCH` while AUX1 was still HIGH.
- **`integrations/tools/mission_demo.py`** — two-stage recovery in
  `compute_rc`. When the latch mask `(ARM_SWITCH | BAD_RX_RECOVERY)` is
  set during a flying phase, **LOW stage** holds AUX1=LOW + idle throttle
  until the bits clear. Then **HOLD stage** keeps idle throttle (with
  AUX1=HIGH) for `RECOVERY_HOLD_S = 0.6 s` so BF lands the LOW→HIGH
  transition on a clean ARMABLE state — preventing the THROTTLE bit from
  re-latching ARM_SWITCH on the same instant. After the hold expires,
  per-phase compute resumes.
- **`integrations/orchestrators/final_world_betaflight.py`** — publishes
  Pegasus's vehicle state to the same `bf_sim_state.txt` file
  `bf_gps_shim` reads, at 10 Hz. Replaces the prior workaround where
  `sim_loop` had to run alongside Pegasus.
- **`sitl/defaults.txt`** — `set failsafe_delay = 200` (vs 15 default).
  Changes the long-window RX_FAILSAFE timer to 20 s; combined with the
  shim lock fix, real RX stalls under 20 s no longer trip the full
  failsafe. Note: defaults.txt edits require `docker compose build
  --no-cache betaflight-sitl` because `eeprom_bake` is a build-time stage
  and Docker layer-cache hits despite source changes.
- **Tests**: 8 new (`integrations/tests/test_bf_gps_shim.py` × 3,
  `integrations/tests/test_mission_demo.py::TestArmSwitchRecovery` × 7
  including HOLD-stage assertions).
- **Artifacts**: `mission2.png`, `mission3.png`, `mission4.png` in repo
  root capture the flight progression: latch-fatal → latch-fatal →
  latch-survivable.

### v0.7 sharp edges

- **`.config_applied` marker is baked into the Docker image** at line 130
  of `sitl/Dockerfile`, not created at runtime. `defaults.txt` is applied
  at the `eeprom_bake` build stage, then runtime `start.sh` only re-applies
  if the marker is missing — which it never is in a built image. Edit
  `defaults.txt` → must `docker compose build --no-cache betaflight-sitl`.
  `docker compose restart` alone uses the cached eeprom.
- **`armingDisableFlags` bit numbers — bit 3 is BAD_RX_RECOVERY, bit 7 is
  THROTTLE.** Got these reversed in the first patch attempt; the fix
  needs the real bit-3 constant to detect bad-rx pre-emptively. Reference
  the names list in `mission_demo.py:_decode_arming` — that order matches
  BF's `armingDisableFlags_e` enum.
- **Docker compose `up` after `down` does NOT rebuild eeprom_bake even
  with `build:` directive** — Docker layer cache bites. Use `--no-cache`
  on the build command if defaults.txt actually needs to land.
- **`failsafe_off_delay` is intentionally LEFT AT DEFAULT 10**. Setting
  it to 200 (matching `failsafe_delay`) breaks arming entirely — BF main
  loop stalls. Documented in `defaults.txt` already; preserve the asymmetry.
- **`RECOVERY_HOLD_S = 0.6 s` is empirical.** Shorter values risk BF
  re-latching on transient throttle bits during the LOW→HIGH transition.
  Longer values cost more altitude per recovery. 0.6 s held up across
  6 recoveries in `mission4`; if you see ARM_SWITCH re-latching
  immediately after HOLD ends, bump to 0.8 s.

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
