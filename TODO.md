# TODO

Open items as of v0.8 (Apr 28 PM 2026). See `MEMORY.md` § Snapshot for
the context behind each.

## High — sim flight quality

1. ~~Chronic RX_FAILSAFE bit-2 latch.~~ **CLOSED Apr 28 PM** by the
   `mission_demo` UDP 9004 dual-write (yesterday's 1a). Verified
   mission7 has zero `[rc]` log hits with RX_FAILSAFE, vs mission6's
   42. See MEMORY.md v0.8.

2. **Bit-1 FAILSAFE pulses on a ~10 s cycle, without bit 2.**
   New problem surfaced by closing #1. mission7 recovery events
   showed `flags=0x2000082` (FAILSAFE+THROTTLE+ARM_SWITCH) and
   `flags=0x2000080` (THROTTLE+ARM_SWITCH) — bit-1 FAILSAFE without
   the usual bit-2 RX_FAILSAFE precursor. So this is a different BF
   trigger, not the standard STAGE2 transition we already fixed.
   Recovery handles each, but cadence is faster (~10 s vs yesterday's
   25-30 s) so legs still time out 33-42 m short of corners.
   - **Diagnostic next step**: walk BF source `failsafeUpdateState()`
     entry conditions. Plausible candidates:
     - `failsafe_throttle_low_delay` (default 100 = 10 s) — fires
       when throttle < min_check too long. mission_demo's INIT/ARM
       phases keep throttle at 950 µs for 7 s, but X_LEG phases
       have throttle 1731. Could the timer not reset?
     - BOXFAILSAFE (bit 4) on a transient mode bit during recovery.
     - BF SITL-specific timer unrelated to the real-firmware paths.
   - **Cheap test:** read BF's CLI `get failsafe_throttle_low_delay`
     and try setting it to 200 (20 s) to see if the cadence stretches.

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
