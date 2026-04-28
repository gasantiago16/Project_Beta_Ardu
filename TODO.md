# TODO

Open items as of v0.7.1 (Apr 28 2026). See `MEMORY.md` § Snapshot for
the context behind each.

## High — sim flight quality

1. **~25-30 s recurring RX-failsafe latch.** UPDATED Apr 28 from prior
   "18 s" hypothesis after instrumentation showed it's NOT the
   shim's GIL/IO. Real cause: BF SITL's RX state machine doesn't
   recognize MSP overrides as "real RX", so `arm=0x4[RX_FAILSAFE]`
   is chronically set during normal flight. After `failsafe_delay`
   (20 s, max) of bit 2, BF transitions to STAGE2 → bit 1 FAILSAFE
   fires → with AUX1 HIGH, ARM_SWITCH latches.
   - **Today's finding** (Apr 28 diag run): only 8 state-file reads
     over 120 s exceeded 50 ms (max 68 ms), zero RC-forward gaps over
     100 ms, zero cache-refresh bursts over 100 ms. The shim is fine.
   - **Today's failed fix attempts** (preserved as cautionary tales
     in MEMORY.md v0.7.1):
     - `set msp_override_failsafe = ON` — DON'T do this; it disables
       MSP overrides during normal flight (despite the name).
     - `bf_rc_keepalive.py` UDP 9004 RX heartbeat — works in
       isolation (BF clears RX_FAILSAFE) but conflicts with
       mission_demo's MSP override on AUX1 in ways we don't yet
       understand. AUX1 LOW from keepalive prevents arming; AUX1
       HIGH from boot latches ARM_SWITCH on initial RX-arrival.
   - **Next session ideas (in order of plausibility):**
     - (a) Have `mission_demo` send its computed RC values to BOTH
       MSP TCP 5761 AND UDP 9004 simultaneously (same values, no
       fight). BF's RX state machine sees the UDP path as "real RX",
       MSP overrides are unnecessary, the entire override-precedence
       question disappears.
     - (b) Find a serialrx_provider config that tricks BF SITL into
       considering its (nonexistent) UART as a "valid RX" by default.
     - (c) Patch BF SITL source so the failsafe state machine
       considers MSP override frames as RX-eligible.

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
