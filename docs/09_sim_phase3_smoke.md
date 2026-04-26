# Phase 3 — racer_companion in the loop, BF SITL ↔ Pegasus, no real flight

This is the procedure for the first end-to-end smoke flight against the
TIAMAT chemical plant in Isaac Sim. Companion code is unchanged — Phase 3
is purely a config + wiring exercise.

## Prerequisites

- Phases 1 + 2 already shipped (PR `pegasus-bridge`).
- `Project_Beta_Ardu/sitl/` Docker stack builds + runs.
- `~/PegasusSimulator/` set up + Iris USD asset present.
- `Final_World_edge_clipped.scene.usd` at the path the orchestrator expects.
- `pip install -e companion` (so `python -m racer_companion.main` works).

## Three-terminal smoke run

The companion expects all three pieces talking simultaneously. Use three
shells. **You MUST run the calibration step (between T2 and T3) before
launching the companion**, or the placeholder waypoint will trigger
`transit_runaway_*` (a ~7000 km error vs the 80 m runaway cap) on the
first state transition out of CLIMB.

### T1 — BF SITL container

```bash
cd ~/Project_Beta_Ardu
docker compose -f sitl/docker-compose.yml up
# Watch for the readback assertions in start.sh:
#   [sitl] OK: msp_override_channels_mask = 15
#   [sitl] OK: mag_hardware = NONE
#   [sitl] Ready. SITL pid=...
```

### T2 — Pegasus + Iris in Final_World, BF backend attached

```bash
cd ~/PegasusSimulator
python ~/Project_Beta_Ardu/integrations/orchestrators/final_world_betaflight.py
# Watch for: [final_world_betaflight] Pre-flight: BF SITL MSP up.
#            ... Isaac startup (~30 s) ...
#            [bridge] tx=... rx=... timeouts=... (...%)  <-- every 5 s
# Iris will sit idle on the asphalt. That's expected.
```

### CALIBRATION (REQUIRED before T3)

Companion's nav uses haversine on lat/lon. The waypoint in
`sim_waypoint.json` ships as `(0.0, 0.0)` PLACEHOLDER. We pick the real
waypoint by reading whatever lat/lon BF SITL is reporting at the
chemical-plant spawn — that depends on BF's internal GPS-from-NED logic
and isn't knowable until we observe.

```bash
# In a fourth, ephemeral shell (or pause T3 and run here):
cd ~/Project_Beta_Ardu
python -m integrations.tools.discover_bf_gps --duration 15
```

Output looks like:

```
  time   fix  sats           lat           lon  alt_m
   0.5  True    11    41.3915000   -73.9560000     100
   ...
[discover_bf_gps] Last good GPS: lat=41.3915000 lon=-73.9560000 alt=100m sats=11

  Pick a waypoint near these coords. Quick recipes:
  ~30 m north:   lat=41.3918000 lon=-73.9560000
  ~30 m east:    lat=41.3915000 lon=-73.9563356
```

Edit `companion/config/sim_waypoint.json`:

```json
"waypoint": {
  "lat_deg": 41.3918000,
  "lon_deg": -73.9560000,
  "alt_m":   5.0
}
```

The 30 m nudge is conservative — the SM_RoadJunction_Plus28 is 21.8 × 21
m, so 30 m off-center may put the waypoint over a building. Adjust as you
explore.

### T3 — companion (after calibration)

```bash
cd ~/Project_Beta_Ardu/companion
python -m racer_companion.main --config config/sim_waypoint.json --log-level INFO
```

## Smoke procedure

Once the waypoint is calibrated:

1. **Arm + take off** in T2 — open QGroundControl, connect to UDP 14550
   (or use the Pegasus joystick widget) — manually take the Iris up to
   ~5 m altitude and let it hover.
2. **Check the companion is talking** in T3 — look for "Home set:" log
   line. That fires on the first GPS reading meeting `min_satellites=8`.
3. **Engage AUX-companion-active**: flip the radio AUX channel
   (`aux_channel_index=6`, i.e., AUX3) to high (>= 1700 µs). Three ways:
   - **With radio (Phase 4)**: `python -m tools.radio_to_bf` from
     `companion/`, then flip AUX3 high on the Radiomaster.
   - **No radio, force AUX3 high** (Phase 4): `python -m tools.radio_to_bf
     --sweep --aux 7=1900` — sticks idle, AUX3 forced high.
   - **No radio at all**: arm + manual fly via QGC's joystick widget,
     then craft an MSP_SET_RAW_RC manually. Phase 4 is the cleaner path.
4. **Watch state transitions** in T3:
   ```
   IDLE → CLIMB → TRANSIT → HOLD
   ```
   - CLIMB: throttle commands ramp up; horizontal sticks centered (climb
     gate active until alt ≥ 80 % of `climb_target_alt_m=5 m`).
   - TRANSIT: forward pitch + yaw to bearing; alt held at 5 m.
   - HOLD: small body-frame roll/pitch corrections (the v0.5 wind-correction
     controller). Should park the quad within `arrival_radius_m=3 m`
     of the waypoint.
5. **Drift check**: in HOLD, observe the position over 30 s. The drift
   should stay under `arrival_radius_m × 1.5 = 4.5 m` from the waypoint;
   exceed that and the state machine kicks back to TRANSIT (the v0.4 BUG-2
   fix).
6. **Pilot takeover**: flip AUX-companion-active LOW. Companion stops
   sending MSP_SET_RAW_RC; BF reverts to RX values within ~500 ms; pilot
   has full control again. Verify by yawing the quad with the radio.

## Expected gotchas

- **`heading_diverged` safety reason should NOT fire.** Iris IMU is
  noiseless in sim and gyro_bias_drift_rad_s defaults to 0, so the
  watchdog (BUG-6 fix from v0.4) sees zero error and stays quiet. If it
  fires anyway, something is wrong with the bridge frame conversions —
  investigate before flying anything real.
- **vbat: `low_vbat_0.0`, NOT `no_analog_telemetry`.** BF SITL replies
  to MSP_ANALOG with a zeroed payload (vbat=0.0), so the companion's
  low-vbat check trips, not the absent-telemetry check. The shipped
  `sim_waypoint.json` works around this with `safety.min_vbat = 0.0`
  (sim-only). If you instead see `no_analog_telemetry`, BF stopped
  responding — that's a bridge problem.
- **`stale_*` safety reasons during Isaac startup.** Pegasus's first
  physics tick lands ~30 s after Isaac launch. Companion polling MSP
  earlier sees no telemetry until then — totally expected; clears once
  T2 finishes warming up.
- **First-tick FDM dead-air, ~1 s.** The bridge needs `update_state` to
  fire before `update(dt)` can pack an FDM packet, and Pegasus's
  callbacks aren't strictly synchronized. So `[bridge] tx=0 rx=0` is
  expected for the first 1-2 seconds, then jumps to ~250 Hz. If it
  stays at 0 past 5s, something's actually broken — see fake_pegasus_loop.
- **`no_rc_telemetry` if you didn't start `radio_to_bf` (or QGC joystick).**
  BF SITL doesn't synthesize MSP_RC values without UDP 9004 input. Either
  start `radio_to_bf` first, OR connect QGC's joystick widget, OR set
  `safety._skip_rc_check` in a future config (deferred).
- **`transit_runaway_*` if you skipped calibration.** With the placeholder
  waypoint at (0,0), distance to target is ~7000 km. Once state ticks past
  CLIMB → TRANSIT, the v0.4 BUG-3 runaway guard fires and forces RELEASED.
  Fix is to re-run the calibration step above; this is the guard working
  as designed, not a sim bug.
- **High lockstep timeouts on first run.** `[bridge] timeouts=N (X.XX%)`
  in T2's stats. > 0.5 % over 30 s of warm-up is concerning; see
  `integrations/tools/fake_pegasus_loop.py` module docstring for likely
  causes (port mismatch, simulator_ip not reachable, etc.).

## Success criteria

- Companion log shows `state` transitioning IDLE → CLIMB → TRANSIT → HOLD
  within 30 s of AUX-companion-active going high.
- HOLD position controller maintains < 4.5 m drift (the BUG-2 hysteresis
  bound) over 30 s of nominal hover.
- Pilot AUX-low immediately restores stick control (verify by visible quad
  response within 500 ms).
- `companion/tests/` still passes (unchanged): `python -m unittest discover -s tests -t .` reports 159 ok.
- Bridge stats settle at ~250 Hz tx + rx, < 0.5 % timeouts.

## What still doesn't work in Phase 3

- **No real pilot input** until Phase 4 (Radiomaster Pocket → UDP 9004).
  T3's smoke uses QGC's joystick widget for arming + manual flight.
- **Iris dynamics are not race-quad dynamics.** Phase 5 swaps in the 5"
  URDF.
- **Sensor noise is unrealistic.** Phase 6 adds gyro bias to validate
  the heading-divergence watchdog in motion.
- **Some companion safety reasons fire by design** because BF SITL doesn't
  simulate everything. Document each one as you encounter it; don't
  silence them globally.

## Companion log captures

When the run completes, save the companion's stdout (or the JSONL log if
`companion.log_path` was non-empty) and any unusual `[bridge]` lines from
T2. These are the artifacts for the next phase's tuning + the eventual
Phase 6 sensor-noise calibration.
