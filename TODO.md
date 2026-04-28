# TODO

Open items as of v0.7 (Apr 27 2026). See `MEMORY.md` § Snapshot for the
context behind each.

## High — sim flight quality

1. **~18 s recurring RX-stall under sustained Isaac Sim load.**
   Even with the shim send-lock + two-stage recovery, BF trips full
   failsafe (`flags=0x200008e`) every ~18 s during a steady X-leg
   transit. Recovery handles it (the drone keeps flying), but the
   2 s recovery cycle costs altitude and forward progress, so legs
   time out 27–42 m short of corners instead of arriving cleanly.
   - Suspected cause: shim's d→u request thread occasionally blocks
     for 1–2 s under combined Pegasus 250 Hz physics + Windows-Docker
     network jitter, gapping the MSP RC stream past BF's hard-coded
     ~200 ms rx-loss detector.
   - Diagnostic next step: instrument `_consume_requests` with
     per-tick wall-time deltas — find the >100 ms gaps and trace them
     back to the GIL holder (likely `_dispatch_responses` parsing a
     big STATUS_EX response, or the integrator lock).
   - Possible fixes: (a) move RC frames to a bypass path that
     `sendall`s without parsing, (b) split the shim into separate
     processes for u-pump vs d-pump so the GIL doesn't matter,
     (c) raise BF's rx-loss detector via a custom 4.5.1 patch.

2. **Phase-2 UDP-init flake at SITL boot.** ~50% of `docker compose up`
   results in BF receiving FDM but not sending motors back
   (`tx=N rx=0`). Workaround is `docker compose restart`. Real fix is
   in `betaflight/src/main/target/SITL/sitl.c` — see
   `HITL_SIM_HANDOFF.md` § 1 for the speculation.

## Medium — known but not blocking

3. **`flight_record` task wrapper buffers all stdout until subprocess
   exit.** When mission_demo + flight_record are piped through `sed |
   tail`, the harness's task `.output` file stays empty until something
   kills both children. Workaround: redirect mission_demo to a
   per-flight log file (`mission4_md.log`) and read that directly.

4. **Recovery costs altitude.** Each LOW+HOLD cycle (~2 s) drops the
   drone by 5–10 m at hover-and-fall rates. After 4–5 recoveries in
   one flight, it's effectively starting each leg from 5 m AGL instead
   of cruise. Could pre-emptively boost climb throttle on HOLD exit,
   but that re-introduces the relatch race; leave alone until item 1
   is solved.

5. **Real BF on STM32 with a real RX should not exhibit any of this.**
   The shim corruption + recovery flow are sim artifacts. When we
   move to bench HITL with a real Speedybee F405 (per
   `HITL_SIM_HANDOFF.md` Path 1, ~$30), validate that the latch
   recovery path doesn't trigger spuriously on a clean RC stream.

## Low — polish

6. **`integrations/tests/test_mission_demo.py` has 11 pre-existing test
   failures** in `TestComputeRc` / `TestMissionStateTransitions` from
   commit `373d5bd` ("Mission demo: X-pattern + circles + handover…").
   Unrelated to v0.7 work (verified by git-stashing the patch and
   re-running). Worth a separate sweep before next release —
   `test_default_hover_matches_bf_default` expects `1300` but the
   constant is `1641` (per the iris hover calibration commit), etc.

7. **mission_demo's loop has no STATUS_EX-fresh check.** If the
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
