# HANDOFF — Project_Beta_Ardu HITL flight (Apr 29 update)

**Audience:** Jeremy (or anyone picking this up cold).
**Read time:** 5 minutes. Then 10 minutes to reproduce the working stack.

> **Read order for incoming:** this status table → `MEMORY.md` Snapshot
> v0.17 → `TODO.md` items #16/#17 → `ROADMAP.md` for the version timeline.
> The Apr 27 status table further below is HISTORICAL — kept for the
> diagnostic walkthrough on what was broken when, but superseded by
> the Apr 29 table.

---

## Status table (Apr 29 12:30 — current)

| Layer | State | Evidence |
|---|---|---|
| BF SITL Docker container | ✅ boots clean | `docker logs project-beta-ardu-sitl` → `[sitl] Ready. SITL pid=12` |
| Bridge wire | ✅ ~80 Hz steady, 0 timeouts | `[bridge] tx=… rx=… (80 Hz) motor w=…` once BF is armed and FDM rate is stable |
| `mission_demo` X-pattern flight | ✅ flies, but flips during legs | Today's runs: mission flew CLIMB → X_LEG_1 → X_LEG_2 → X_LEG_3 → TO_CENTER, FLIP'd on ground impact during X_LEG_3 / CIRCLE_1 |
| FPV camera + MP4 recording | ✅ shipped + verified | `Desktop/fpv_mission_20260429_104652.mp4` shows horizon level, sky on top, walkway below; zero gimbal lock warnings |
| `[phys-truth]` altitude oracle | ✅ shipped | Closed TODO #0; shim altitude path proven faithful end-to-end (±0.4 m at peak across all three sources) |
| Crash-floor clamp (v0.17) | ⚠️ shipped UNTESTED | Verify run blocked by Vulkan OOM. Patch on `pegasus-bridge` as `3255ae1`. See TODO #16. |
| **Altitude PID oscillation** | ⚠️ open | Today's fpvfix run: drone overshot 30 m target to 49 m, dove to rel_alt = -7 m, flipped on ground impact. Real fix is Kd term (TODO #17). |
| **Sim hygiene — BOOT_GRACE_TIME quirk** | ⚠️ open | After `docker compose restart`, BF holds `0x200` until orch FDM ≥ 50 Hz. Pre-warm orch FDM before mission_demo connects. TODO #18. |
| Companion bug audit (Apr 27) | ⚠️ unchanged | `CH_THROTTLE/CH_YAW` issue from old table is still open; not touched this session. |

**Run cadence today:** v0.16 verify (10:18), v0.16 FPV-fix verify
(10:46), v0.17 floor-clamp verify (12:08, blocked by GPU OOM).
Three Isaac Sim launches per session is the ceiling we hit.

---

## Status table (Apr 27 13:00 — HISTORICAL, superseded)

| Layer | State | Evidence |
|---|---|---|
| BF SITL Docker container | ✅ Builds + boots clean | `docker logs project-beta-ardu-sitl` shows `[sitl] Phase 2 MSP up` + `[sitl] Ready` |
| Bridge wire (host ↔ BF over UDP via in-container relay) | ✅ Locked at ~67 Hz, 0 timeouts, rx ≈ tx in steady state | `[bridge] tx=336 rx=334 in 5.0s (67 Hz)` |
| Final_World scene + Iris spawn | ✅ Renders cleanly in Isaac Sim | One window, road junction asphalt visible, Iris visible at spawn |
| **BF arming → motors actually spin** | ✅ **WORKING.** `motor w = 722.2 rad/s, raw=0.706` at hover throttle | `bf_diag` shows `flags=0x00000000 [(none — armable)]`, then `NOT_DISARMED` after AUX1 high |
| `hover_test` end-to-end | ⚠️ Companion's `BetaflightAdapter.send_overrides` doesn't currently arm BF in this stack — use `bf_diag --mode arm` instead. Bug being chased under task #42. | hover_test rc[3] never reaches BF (BF stays in failsafe channels) |
| **`bf_diag --mode arm` end-to-end through Pegasus → Iris in Isaac Sim** | ✅ **VERIFIED VISIBLY FLYING** | Bridge `motor w=522,522,521,521 rad/s` at hover_throttle=1500; user confirms "moved up and stopped" |
| `mission_demo` flight | ❌ Blocked by virtual GPS not wired | mission_demo hangs on INIT phase (≥8 GPS sats; BF SITL has no GPS provider) — task #39 |
| **Companion `CH_THROTTLE/CH_YAW` real-flight bug** | ❌ **NEW BUG FOUND** — see "Real-flight risks" below | task #42 |

**110/110 integration tests + 159/159 companion tests pass.** Tree is clean at
`pegasus-bridge` branch (`gasantiago16/Project_Beta_Ardu`).

---

## What flipped from "blocker" to "working"

Two layered bugs in `sitl/defaults.txt` + `integrations/tools/hover_test.py`,
both diagnosed via the new `integrations/tools/bf_diag` tool:

1. **`min_check = 1000`** (which I'd set thinking it relaxed throttle).
   BF code is `if (rcData[THROTTLE] < mincheck) THROTTLE_LOW`. With
   `mincheck=1000` and `throttle=1000`, `1000 < 1000` is FALSE → THROTTLE
   flag stays set → ARM_SWITCH flag (derived) fires when AUX1 goes high
   → motors stay at 0. Reverted to BF default 1050; `hover_test` now
   sends throttle=950 at idle (well below 1050) so flag clears.
2. **MSP_SET_RAW_RC slot order**. BF parses `rcChannelLetters="AERT..."`
   with default rcmap `"AETR"` — meaning MSP slot 2 → `rcData[THROTTLE]`
   and slot 3 → `rcData[YAW]`. `hover_test`'s slot order is
   `[Roll, Pitch, Throttle, Yaw, AUX1...]` (T at slot 2, Y at slot 3) —
   correct. (I briefly "fixed" it the other way and broke arming; the
   commit history shows the back-and-forth.)
3. **`small_angle = 180`** isn't enough — BF's IMU-uninitialized state
   sets ANGLE flag for the first ~7 s after FDM starts flowing. Idle
   phase before AUX1-high needs to be ≥ 10 s (`hover_test --ramp-s 2`
   has only 1 s idle; `bf_diag --idle-s 10` is what reproduces flight).

After all three: bf_diag at `t=10s` shows `flags=0x00000000 (armable)` →
AUX1 → 2000 → throttle ramp → motors output **0.706 normalized = 706 us
above min, 70% throttle**.

---

## Canonical "is the sim working?" check

**Always run this first, before re-deriving setup from memory.**

```powershell
docker compose -f $PROJ\sitl\docker-compose.yml up -d
& $PY -m integrations.tools.verify_sim_arms --motor-port 28500
```

PASS criteria:
- Stage 1: ≥3/5 UDP packets delivered (container→host wire is live)
- Stage 2: tx≈1000, rx>100, max_motor_norm>0.4, samples>500 rad/s ≥5

If Stage 1 fails: try a different `--motor-port` (28500 / 35000 / 19077),
drop the VPN, or restart Docker Desktop.

If Stage 2 fails: arming-disable flags pin the bug. Run
`python -m integrations.tools.bf_diag --duration-s 25 --interval-s 1.0
--mode arm --idle-s 12` and read the `flags=` line.

If Stage 1 passes but Stage 2 fails with rx=0: relay isn't running on
the port, or got killed by a previous `docker exec`. Restart it:

```powershell
docker exec -d project-beta-ardu-sitl bash -c `
  "cd /opt/sitl && python3 motor_relay.py 192.168.65.254 28500"
```

## Reproduce the blocker (4 commands)

Windows / `isaaclab311` conda env. Linux notes inline.

```powershell
# Setup
$PY   = "C:\miniconda3\envs\isaaclab311\python.exe"     # Linux: just `python`
$PROJ = "C:\Users\Gabriel Santiago\Project_Beta_Ardu"   # adjust to your path

# 1. SITL container up (~10 s)
docker compose -f $PROJ\sitl\docker-compose.yml up -d
# Wait for `[sitl] Ready` in `docker logs project-beta-ardu-sitl`.

# 2. Pegasus + Iris in Isaac Sim (~30 s warmup, then opens window)
& $PY "$PROJ\integrations\orchestrators\final_world_betaflight.py" --corner ROADJUNCTION
# Wait for `[final_world_betaflight] World reset; starting timeline`.

# 3. Hover test — connects MSP, sends arm sequence, ramps throttle
$env:PYTHONPATH = "$PROJ\companion"
& $PY -m integrations.tools.hover_test --duration-s 25 --hover-throttle 1700 --ramp-s 2.0
```

In the orchestrator terminal you'll see:

```
[bridge] tx=336 rx=334 timeouts=0 (0.00%) | Δ tx=336 rx=334 in 5.0s (67 Hz) | motor w=0,0,0,0 rad/s
```

`tx ≈ rx`, 0 timeouts ⇒ **wire is healthy**.
`motor w=0,0,0,0` ⇒ **BF isn't arming despite the RC override**.

Iris falls 1.3 m to the asphalt and sits there. That's the bug.

> **Linux Docker host:** in `sitl/docker-compose.yml` set
> `SITL_USE_MOTOR_RELAY=0`, override
> `BetaflightBackendConfig.motor_port=9002`, and add `9002:9002/udp`
> back to compose's `ports`. Windows Defender Firewall is the only
> reason the in-container Python relay exists — Linux doesn't need it.

---

## What we know about the blocker

BF status query (CLI `status`) during a hover_test run shows:

```
Arming disable flags: RXLOSS CLI
```

**`CLI`** is set because the status query enters CLI mode — ignore it.
**`RXLOSS`** is the real blocker: BF doesn't think it's receiving
fresh RC, so it refuses to arm.

What's already configured (verified via `get` queries):
- `motor_pwm_protocol = PWM` ✅ clears `MOTOR_PROTO` flag
- `msp_override_channels_mask = 255` ✅ all 8 channels override-able
- `aux 0 0 0 1700 2100 0 0` ✅ ARM mode bound to AUX1 [1700,2100]
- `feature` enabled list includes `RX_MSP` ✅
- `small_angle = 180`, `min_check = 1000`, `max_check = 2000`,
  `runaway_takeoff_prevention = OFF` ✅ (sim-only safety relaxations)

What we haven't verified:
- Does `feature RX_MSP` actually clear RXLOSS when MSP_SET_RAW_RC is
  pumping at 50 Hz? Or do we need a different feature combination?
- Does `msp_override_channels_mask` feed the override values into
  BF's RC subsystem (the thing that updates `lastRxFrameTime`)?
  Or is it a stage-2 override that stacks on top, leaving RXLOSS
  driven by a still-stale `lastRxFrameTime`?
- What does BF's Receiver tab in BF Configurator show for our 8
  channels while hover_test runs? (See task #36.)

---

## Next-session priority order (3 hours max)

Tasks #36 → #41 in the Claude todo list, summarized:

1. **(1 hour)** Open BF Configurator (Chrome app, free) — connect
   to `tcp://localhost:5761`, watch the **Modes** tab while
   hover_test runs. Does the `ARM` row flip green when AUX1 goes
   1500→2000? Does the **Receiver** tab show channel values
   matching what hover_test sends? **Task #36.**

2. **(45 min)** Edit `sitl/defaults.txt`:
   ```
   set rx_min_usec = 750
   feature -RX_PARALLEL_PWM
   feature -RX_PPM
   ```
   then `docker compose -f sitl/docker-compose.yml up -d --build` (the
   `--build` is critical — `.config_applied` marker only goes away on a
   fresh image build). **Task #37.**

3. **(30 min)** While hover_test is running, open a parallel TCP MSP
   connection to `localhost:5761` and poll MSP_RC (msg ID 105) every
   500 ms. Print the response. If channel values match hover_test's
   overrides → RXLOSS is a separate failsafe-phase issue. If they
   don't → MSP override isn't actually feeding the RX subsystem on
   this BF build. **Task #38.**

If none of those work in 3 hours, **stop** and consider the bigger
question: does BF SITL's RX path support pure-MSP arming at all on
4.5.1? It might require a real serial RX provider with a faked
serial port.

---

## What changed this session (so you don't re-derive it)

- **Background recv thread** in `pegasus_betaflight_backend.py`
  decouples motor RX from physics tick rate. Before: each `update()`
  blocked on `recvfrom`, throttling Pegasus to ~2 Hz. Now: thread
  pulls every motor packet the moment it arrives, lock-protected.
- **Ephemeral motor port**, picked by the orchestrator at startup.
  Windows Defender Firewall blocks long-lived UDP ports after enough
  rebinds (we burned through 9002 → 9012 → 19500 → 38500). The
  orchestrator now picks a fresh port and reconfigures the in-
  container relay via `docker exec`.
- **Two-phase boot in `sitl/start.sh`**. Phase 1 applies defaults +
  `save` (BF writes eeprom and exits via `systemReset()`); Phase 2
  respawns BF with the saved eeprom. Without `save`, BF stays in CLI
  mode and the `CLI` arming flag never clears. `.config_applied`
  marker in `/opt/sitl/` prevents re-applying on container restart
  (BF creates a default eeprom on first boot, so eeprom existence
  alone isn't a useful signal).
- **Python relay replaces socat**. `sitl/motor_relay.py` —
  single-process Python loop that forwards 127.0.0.1:9002 (BF's
  hardcoded output) to host:`SITL_HOST_MOTOR_PORT`. Replaced
  `socat ... fork` which added 50-200 ms per-packet latency and
  capped Pegasus at ~2 Hz.
- **`motor_pwm_protocol=PWM`** — clears the `MOTOR_PROTO` arming
  flag. SITL has no real ESCs; default `DISABLED` doesn't work.
- **`msp_override_channels_mask` 15 → 255** — was silently dropping
  AUX1+ overrides; mask=15 only covers channels 1-4.
- **`aux 0 0 0 1700 2100 0 0`** — binds ARM mode to AUX1 high band.
  Without this, even with mask=255, BF has nothing watching AUX1
  to flip the ARM box.
- **MEMORY.md "v0.6.1" snapshot section** has the full list of new
  sharp edges discovered.

---

## ANGLE mode tuning (Apr 27 update)

ANGLE flight mode is now bound in `sitl/defaults.txt`
(`aux 1 1 0 900 2100`). With ANGLE active + bridge frame conversion
verified correct (probe_attitude shows roll/pitch round-trip cleanly),
Iris hovers stably with motors at ~700-740 rad/s and 4-6% per-motor
variance (BF's PID corrections, normal).

**Hover throttle empirics:**
| throttle | result |
|----------|--------|
| 1500 | Iris doesn't lift off cleanly; motors split 329-751 trying to right ground bounce |
| 1700 | Motors at 700-740 rad/s, Iris climbs continuously (above hover equilibrium) |
| ~1640 (estimated) | True hover; need closed-loop altitude control or careful ramp |

For the **mission demo**, `mission_demo.py` already has a closed-loop
altitude controller (`throttle_hover_us=1300` baseline + climb/descent
offsets). For `hover_test` and `bf_diag` smoke tests, use 1700 to
confirm flight and accept that Iris will climb. Real altitude hold
is task #47.

## ⚠️ Real-flight risks discovered

### Companion CH_THROTTLE/CH_YAW vs MSP slot mapping (task #42)

**Bug**: `companion/racer_companion/msp.py` defines `CH_YAW=2,
CH_THROTTLE=3` and `main.py` does `rc[msp.CH_THROTTLE] = throttle_hover`,
which puts throttle at MSP frame slot 3. **But BF default rcmap "AETR"
reads slot 3 as YAW** (because rcChannelLetters in BF source is
`"AERT12345..."` and rcmap[YAW=2]=3, rcmap[THROTTLE=3]=2). So:

- Companion's throttle command → BF's yaw input
- Companion's yaw command → BF's throttle input

If a real drone took off with the companion-active path, "throttle up"
would yaw it instead of climbing, and the constant centered-yaw command
would hold mid-throttle. Quad would spin without lifting off.

**Why this hasn't been caught**: the companion has 159 unit tests but
they all use mocks that don't simulate BF's rcmap. And no real drone
has ever flown with this code (project is software-only so far). The
SITL bridge would have caught it but never did because the bridge
didn't reach the "armed motors actually spinning" state until today.

**Fix options** (decide before next real-flight prep):
- (A) Swap msp.py constants: `CH_THROTTLE=2, CH_YAW=3`. Smallest diff,
  matches MSP frame layout under default AETR rcmap. Risks: any test
  that hardcoded `rc[3]` for throttle needs updating.
- (B) Apply rcmap translation in `encode_set_raw_rc`. More principled,
  works with non-default rcmaps (e.g. TAER). Risks: more code to test.

**Counter-agent the fix** before merging. 159 unit tests need to stay
green plus a fresh `hover_test` run.

## Real-flight safety reminder

Several settings in `sitl/defaults.txt` are **DANGEROUS on real
hardware**:

- `small_angle = 180` — lets BF arm at any orientation, including
  upside-down (real default is 25°)
- `runaway_takeoff_prevention = OFF` — disables the safety that
  cuts motors on uncommanded takeoff
- `motor_pwm_protocol = PWM` — wrong for real ESCs (need DSHOT600)

These are correctly isolated to `sitl/`. Real-flight config is
governed by `bf_config/per_drone/*.manifest` + the pre-flight
validator (`scripts/preflight_validator.py`), which enforces the
safety floor (`mask=15`, `mag_hardware=NONE`). **Don't move
`sitl/defaults.txt` settings into `bf_config/`**.

The companion (`companion/racer_companion/`) was **NOT modified**
this session — sim-to-real surface area didn't grow. 159/159
companion tests still green.

---

## ⚠️ Windows VPN + WSL2 + Docker = container→host UDP blackhole

If you have a VPN client running on Windows (NordVPN, OpenVPN, Cisco
AnyConnect, anything that adds a TUN/TAP adapter), Docker Desktop's
vpnkit container→host UDP forwarding gets intermittently captured by
the VPN's routing table. **Symptom:** `bf_diag` works for one run, then
the next run gets `rx=0` on the bridge even though `[fake_pegasus]`
shows tx-only. Looks identical to Windows Defender Firewall heuristic
blocking — but isn't.

**Fix:** drop the VPN before running the SITL stack. Verified empirically
today (Apr 27): same setup blackholed UDP under VPN, immediately worked
when VPN dropped, and the bridge captured 0.706 motor values during armed
flight.

If you genuinely need the VPN up, options:
- Run the whole stack inside WSL2 (no vpnkit hop). Linux Docker works
  natively without the container→host hypervisor crossing.
- Use a VPN that supports split-tunnel and exclude `192.168.65.0/24`.

## If you get stuck

- **Bridge stays at `tx=N rx=0`:** the wire is broken, not the arming.
  Read `memory/project_beta_ardu_arming_blocker_apr26.md` § "What
  works" — the relay or port may need restarting.
- **Container won't come up:** `docker compose down` then
  `docker compose up -d --build`. The `--build` rewrites the image
  and clears the `.config_applied` marker.
- **Stale Python listener blocks the bridge** (Windows): `Get-Process
  python | Stop-Process -Force` in PowerShell. Multiple Python
  binds to the same UDP port via `SO_REUSEADDR` split traffic
  non-deterministically.
- **Defaults.txt edit not taking effect:** the `.config_applied`
  marker means start.sh skipped the apply. Force-rebuild:
  `docker compose -f sitl/docker-compose.yml up -d --build`.
- **Windows console crashes on Unicode:** `sys.stdout.reconfigure
  (encoding="utf-8")` is already in the orchestrator. If a new tool
  hits the same issue, copy that snippet to its top.

---

## Files to know

| Path | What it is |
|---|---|
| `HANDOFF.md` | this file |
| `MEMORY.md` § v0.6.1 | full project state + sharp edges |
| `docs/13_mission_demo.md` | mission demo runbook (top has Status table; bottom has Open debug section) |
| `sitl/start.sh` | container entrypoint, two-phase boot |
| `sitl/defaults.txt` | sim-only BF config (DO NOT propagate to real flight) |
| `sitl/motor_relay.py` | in-container UDP relay, BF output → host bridge |
| `sitl/Dockerfile` | container image |
| `sitl/docker-compose.yml` | compose env vars (`SITL_HOST_MOTOR_PORT`, `SITL_USE_MOTOR_RELAY`, `SITL_SIM_HOST`) |
| `integrations/pegasus_betaflight_backend.py` | the bridge (Pegasus Backend duck-type) |
| `integrations/orchestrators/final_world_betaflight.py` | Pegasus + Isaac Sim launcher, picks ephemeral motor port |
| `integrations/tools/hover_test.py` | bypass-GPS arm + throttle test |
| `integrations/tools/check_arming.py` | dump BF status flags via MSP CLI |
| `integrations/tools/fake_pegasus_loop.py` | bridge wire smoke test (no Isaac Sim) |
| `companion/racer_companion/` | unchanged from v0.5; real-flight code path |
| `bf_config/per_drone/example_manifest.txt` | real-flight BF config (the safety floor) |

Good luck. Wire's solid; arming's the last gate.
