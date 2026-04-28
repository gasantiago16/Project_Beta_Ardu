# TODO

Open items as of v0.8 (Apr 28 PM 2026). See `MEMORY.md` § Snapshot for
the context behind each.

## High — sim flight quality

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

10. **Lateral controller doesn't reach corners — legs time out
    28-49 m short.** v0.9 has the drone flying stably but lateral
    progress is unchanged from earlier missions. The X_LEG bearing
    controller in `_compute_waypoint_rc` has `pitch_kp_per_m = 2.0`
    + `yaw_align_threshold_deg = 25°`. Drone reaches center area
    fine but can't push out toward corners.
    - **Diagnostic next step:** record per-tick (n, e, target_n,
      target_e, h_err, pitch_us) and see whether yaw_align gate is
      keeping pitch=0 most of the time.
    - **Cheap fixes to try (one at a time, like the kp=6→3 ship):**
      (a) raise `pitch_kp_per_m` to 4.0 — note `racer_companion/
      nav.py::NavTuning` already defaults to 4.0; mission_demo's
      2.0 is the outlier; (b) lower `yaw_align_threshold_deg` to
      10° (only push pitch when really aligned); (c) bump
      `pitch_max_us` from 200 to 300; (d) extend `leg_timeout_s`
      from 60 to 120 — *try this last; it masks rather than fixes a
      slow controller*.

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
