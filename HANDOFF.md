# HANDOFF — Project_Beta_Ardu HITL flight (Apr 27 update)

**Audience:** Jeremy (or anyone picking this up cold).
**Read time:** 5 minutes. Then 10 minutes to reproduce the working stack.

---

## Status table (Apr 27 13:00 — UPDATED)

| Layer | State | Evidence |
|---|---|---|
| BF SITL Docker container | ✅ Builds + boots clean | `docker logs project-beta-ardu-sitl` shows `[sitl] Phase 2 MSP up` + `[sitl] Ready` |
| Bridge wire (host ↔ BF over UDP via in-container relay) | ✅ Locked at ~67 Hz, 0 timeouts, rx ≈ tx in steady state | `[bridge] tx=336 rx=334 in 5.0s (67 Hz)` |
| Final_World scene + Iris spawn | ✅ Renders cleanly in Isaac Sim | One window, road junction asphalt visible, Iris visible at spawn |
| **BF arming → motors actually spin** | ✅ **WORKING.** `motor w = 722.2 rad/s, raw=0.706` at hover throttle | `bf_diag` shows `flags=0x00000000 [(none — armable)]`, then `NOT_DISARMED` after AUX1 high |
| `hover_test` end-to-end | ✅ Iris should lift off at hover throttle 1500 (Iris airframe) | Bridge raw=0.706 = ~706 us motor PWM ≈ 70% throttle |
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
