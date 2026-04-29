# TODO

Open items as of v0.17 (Apr 29 noon 2026). See `MEMORY.md` § Snapshot
for the context behind each.

## High — sim flight quality

0. ~~Shim altitude vs physics altitude divergence.~~ **CLOSED Apr 29
   morning (v0.16 verify run).** Re-ran full X-pattern with new
   `[phys-truth]` instrumentation (`final_world_betaflight.py` logs
   `bf_backend._latest_state.position` every 5 s). Peak values from
   the same flight: phys-truth z=**+32.40 m**, shim alt=**132.4 m**
   (rel = 32.65 m after subtracting origin 99.75), mission_demo
   `rel_alt`=**32.76 m**. Three sources within ±0.4 m end-to-end. The
   Apr 28 regression note's "rel_alt=36 m / physics z=78 m" almost
   certainly came from reading `/World/quadrotor` translate in the
   Isaac Sim stage browser — the v0.14 bug (that prim doesn't track
   physics; physics moves a deeper rigid-body prim). Logs preserved
   as `verify_orch.log` (768 lines, 65 phys-truth samples),
   `verify_shim.log`, `verify_mission.log`. Instrumentation is
   load-bearing going forward — DO NOT remove the `[phys-truth]`
   log line; it's our canonical altitude oracle now.


1. ~~Chronic RX_FAILSAFE bit-2 latch.~~ **CLOSED Apr 28 PM** by the
   `mission_demo` UDP 9004 dual-write (yesterday's 1a). Verified
   mission7 has zero `[rc]` log hits with RX_FAILSAFE, vs mission6's
   42. See MEMORY.md v0.8.

2. ~~Bit-1 FAILSAFE / ARM_SWITCH cycles every ~3-10 s.~~ **MITIGATED
   Apr 28 evening.** Two-subphase HOLD with hover throttle in the
   second half (mission_demo.RECOVERY_HOLD_IDLE_S=0.3,
   RECOVERY_HOLD_S=1.5) dropped recovery count from 53 (mission10)
   to 3 (mission12) over 240 s. The trigger is still there but
   fires far less often AND no longer costs full altitude per cycle.
   Root cause of the bit-1 trigger remains unknown — see TODO #9.
   - **Apr 28 PM failed experiment** (preserved as cautionary tale):
     hypothesis was `failsafe_throttle_low_delay` (default 100, in
     0.1 s units = 10 s) matched the cadence, so bumping to 200
     should stretch it. mission8 with this setting baked: drone
     NEVER lifted off, recovery fired ~every 3 s. Either BF 4.5.1
     semantics for `failsafe_throttle_low_delay` differ from the
     docs, or the bit-1 trigger is elsewhere entirely. Reverted in
     `sitl/defaults.txt` with a "DO NOT" comment.
   - **Real diagnostic now needed**: walk BF source
     `betaflight/4.5.1/src/main/flight/failsafe.c` and identify
     every entry into `FAILSAFE_RX_LOSS_DETECTED` state without an
     RX_LOSS prerequisite. The `[rc]` flag pattern in the next
     run's log will show which BF transition fires; cross-ref to
     source.
   - **Less cheap mitigation**: detect ARM_SWITCH latch in
     `mission_demo.compute_rc` more aggressively and avoid throttle
     transitions across the LOW→HIGH AUX1 edge. Possibly extend the
     HOLD stage to longer (e.g. `RECOVERY_HOLD_S = 2.0` instead of
     0.6) so the drone has time to settle before throttle ramps up
     again.

3. **Phase-2 UDP-init flake at SITL boot.** ~50% of `docker compose up`
   results in BF receiving FDM but not sending motors back
   (`tx=N rx=0`). Workaround is `docker compose restart`. Real fix is
   in `betaflight/src/main/target/SITL/sitl.c` — see
   `HITL_SIM_HANDOFF.md` § 1 for the speculation.

## Medium — known but not blocking

4. **`flight_record` task wrapper buffers all stdout until subprocess
   exit.** When mission_demo + flight_record are piped through `sed |
   tail`, the harness's task `.output` file stays empty until something
   kills both children. Workaround: redirect mission_demo to a
   per-flight log file (`mission4_md.log`) and read that directly.

5. **Recovery costs altitude.** Each LOW+HOLD cycle (~2 s) drops the
   drone by 5–10 m at hover-and-fall rates. After 4–5 recoveries in
   one flight, it's effectively starting each leg from 5 m AGL instead
   of cruise. Could pre-emptively boost climb throttle on HOLD exit,
   but that re-introduces the relatch race; leave alone until item 2
   is solved.

6. **Real BF on STM32 with a real RX should not exhibit any of this.**
   The shim corruption + recovery flow are sim artifacts. When we
   move to bench HITL with a real Speedybee F405 (per
   `HITL_SIM_HANDOFF.md` Path 1, ~$30), validate that the latch
   recovery path doesn't trigger spuriously on a clean RC stream.

## Low — polish

7. **`integrations/tests/test_mission_demo.py` has 11 pre-existing test
   failures** in `TestComputeRc` / `TestMissionStateTransitions` from
   commit `373d5bd` ("Mission demo: X-pattern + circles + handover…").
   Unrelated to v0.7 work (verified by git-stashing the patch and
   re-running). Worth a separate sweep before next release —
   `test_default_hover_matches_bf_default` expects `1300` but the
   constant is `1641` (per the iris hover calibration commit), etc.

8. **mission_demo's loop has no STATUS_EX-fresh check.** If the
   shim/BF ever returns stale arming flags from cache, the recovery
   branch could fire on outdated data. Today's runs trust the cache
   refresh thread (2 Hz) — adequate but not bullet-proof.

9. ~~Altitude oscillation: mission12 peaks at 36-40 m / dips to -16 m
   on a ~50 s period.~~ **CLOSED v0.9 (Apr 28 night).** Lowered
   `throttle_kp_per_m` from 6.0 to 3.0. mission13: amplitude
   collapsed to ±5 m around 12 m mean. Bonus: the same change
   eliminated ALL recovery cycles too (mission13 = 0 recoveries
   over 240 s), so the bit-1 FAILSAFE pulse was apparently
   triggered by aggressive kp throttle commands, not by a BF timer.

10. ~~Lateral controller doesn't reach corners — legs time out
    28-49 m short.~~ **MITIGATED v0.10 (Apr 28 late night).** Per-
    tick trace ruled out yaw-gate (97% aligned) and pitch saturation
    (never hit). Real cause: `pitch_kp_per_m = 2.0` × 37 m = only
    +74 µs forward stick. Bumped to 4.0; mission15 X_LEG_2 ended
    10.3 m from SE corner (vs mission14's 34 m). Diagnostic CSV
    shipped behind `--waypoint-trace-csv` flag for future tuning.

11. ~~X_LEG_3 long diagonal stalls~~ **MITIGATED v0.13 (Apr 29
    afternoon).** Tilt-throttle feedforward (`tilt_throttle_factor =
    0.15`): commanded forward pitch boosts throttle proportionally,
    countering cos(tilt) lift loss. mission21 dropped X_LEG_3 from
    41.8 m short → 14.1 m short. Trade-off: X_LEG_2 regressed to
    18 m short (was 10.3 m). Net better on the long-diagonal
    bottleneck. Vario-Kd term still queued as a future improvement
    if we want tighter altitude hold during pitched flight.

12. **In-sim visualization breadcrumbs (v0.12 follow-up).** User
    asked for "every 25 m" markers to show where the controller
    intended the drone to be. Current implementation only renders
    the live target bubble. Add a deque of last N target positions
    rendered as fading-color spheres so the operator sees the
    commanded path. Cheap: USD prim creation/translate already
    proven in v0.12.

13. ~~FPV camera side quest.~~ **DONE v0.15 (Apr 29 late evening).**
    v0.14 added `--fpv-camera` but the camera was parented under
    `/World/quadrotor` which DOES NOT track physics — Pegasus moves
    a deeper rigid-body prim. v0.15 moved camera to top-level
    `/World/fpv_camera` and updates its world transform every tick
    from `bf_backend._latest_state.position` + attitude. Verified:
    z=5 m capture vs z=80 m capture render radically different
    views.

14. ~~FPV mission video recording.~~ **DONE v0.15.** `--fpv-video-fps`
    (default 5) + `--fpv-video-out-dir` (default `~/Desktop`)
    capture per-tick PNGs to a temp staging dir, encode to MP4 via
    `imageio_ffmpeg` in the orchestrator's `finally:` block.
    Shutdown signal at
    `{tempdir}/bf_orchestrator_shutdown.signal` lets external
    processes ask for graceful exit (Stop-Process -Force skips
    Python's finally on Windows). User direction: use video for
    confirming control + placement going forward, instead of just
    static screenshots.

15. ~~FPV camera 90° tilt + gimbal lock.~~ **DONE v0.16 (Apr 29
    morning).** v0.15's body-local Euler XYZ `(-15, -90, -90)`
    silently mapped camera "up" into the horizontal plane (image
    rotated 90° from natural) AND sat exactly on the Y=-90 gimbal
    lock (every frame triggered scipy `Gimbal lock detected`).
    Replaced with `(75, 0, -90)`: look=+X, up=+Z, 15° downtilt at
    drone-yaw=0, no gimbal lock at any yaw. Verified visually and
    by zero gimbal warnings across the v0.16 verify run. MP4
    `Desktop/fpv_mission_20260429_104652.mp4` is the proof.

## High — flight quality (open)

16. **Crash-floor clamp — verify after fresh boot (v0.17 UNTESTED).**
    Apr 29 noon: shipped vario tracking + crash-floor clamp in
    `mission_demo` (`rel_alt < 3 m AND vario < -0.5 m/s` in a flying
    phase forces `thr = hover + 200 µs`). Could not verify in this
    session — three Isaac Sim launches hit Vulkan
    `OUT_OF_DEVICE_MEMORY`, fourth launch's BF stuck in
    `BOOT_GRACE_TIME` because orch FDM rate fell to ~30 Hz. Patch
    is on `pegasus-bridge` as `3255ae1`. **Next session:** fresh
    boot, close other GPU apps, re-run X-pattern. Confirm
    `[floor]` log fires when expected and no regression vs v0.13's
    X_LEG_3 14.1 m short. If clamp never fires AND drone reaches
    corners, that's also valid — clamp is dormant safety net.

17. **Altitude PID is underdamped — add Kd term keyed to vario.**
    Apr 29 fpvfix run: drone overshot 15 m cruise target to 49 m
    (230% overshoot — original "87%" misread the cruise as 30 m;
    actual default cruise is 15 m), then dove uncontrolled to
    `rel_alt = -7 m`, tumbling on yaw, flipped on impact.
    Underlying control is kp-only on rel_alt error plus a tilt-FF
    feedforward — no derivative term, classic underdamped behavior.
    - **Tilt-FF gate** (the cheap part): SHIPPED v0.18 (UNTESTED).
      Gate FF to fire only when `err_m > 0`. Per fpvfix log analysis,
      this kills the dominant positive-feedback path: drone went 14m
      → 49m past target because FF kept adding throttle as drone
      climbed past target. With the gate, FF stops at `err_m=0` so
      kp regains descent authority. Verify on next session's run.
    - **Kd term** (the real fix, queued): `t_kd = -kd_per_m_s *
      vario`. Vario is already tracked on `Mission` (added v0.17 for
      the floor clamp). Start at `kd_per_m_s = 30 µs` per m/s so a
      +1 m/s climb rate at zero error yields −30 µs throttle. Only
      worth doing AFTER gate is verified — the gate may be enough
      alone, in which case Kd adds complexity for no gain.

18. **Sim hygiene — pre-warm orch FDM stream so BF clears
    BOOT_GRACE_TIME.** After a `docker compose restart` BF SITL
    holds `BOOT_GRACE_TIME=0x200` until orch FDM reaches roughly
    50 Hz steady. If Isaac Sim warmup is GPU-pressured (third
    launch in a session), bridge can stall at 30-40 Hz and BF
    never arms. Cheap mitigation: in `final_world_betaflight.py`,
    run N=200 `world.step()` ticks BEFORE bf_backend starts
    publishing FDM, so the FDM stream comes up at full rate from
    the first packet BF sees. Or even simpler: just wait until
    `[bridge] tx >= 70 Hz` for 10 s before logging "ready" so the
    operator knows when to launch mission_demo.

## Not doing — rejected ideas

- **Disable BAD_RX_RECOVERY entirely via `failsafe_recovery_delay = 0`.**
  Doesn't fix the real path; ARM_SWITCH still latches whenever any
  disable bit appears at the LOW→HIGH AUX1 transition. The two-stage
  recovery is the structural fix.

- **Skip the shim and have mission_demo talk MSP directly to BF on
  5761.** Loses the synthesized MSP_ALTITUDE / MSP_RAW_GPS that
  mission_demo's state machine depends on (BF SITL's VIRTUAL baro
  doesn't track FDM pressure). The shim is load-bearing for the
  navigation loop.
