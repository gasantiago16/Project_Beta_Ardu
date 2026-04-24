# Phase 2 — Controller Install + Tuning (1 weekend)

## Goal

Install the companion software's main loop, tune the P-controller against a
synthetic GPS replay, and validate behavior in dry-run mode (no MSP traffic
sent). At the end, the Pi is ready to fly — but you do **not** fly until
Phase 3 drills.

## Step 1 — Per-drone config

```bash
sudo mkdir -p /etc/racer-companion
sudo cp /opt/racer-companion/companion/config/start_line.example.json \
        /etc/racer-companion/config.json
sudo nano /etc/racer-companion/config.json
```

Set per-drone values:

| Key | How to set |
|-----|------------|
| `companion.serial_port` | Usually `/dev/ttyAMA0` on Pi Zero 2W |
| `companion.aux_channel_index` | 0-based index of the AUX channel that activates companion mode. Pilot decides; default 6 = AUX3 |
| `companion.aux_active_us_min` | ≥1700 means "switch high" — adjust to match your TX setup |
| `companion.climb_target_alt_m` | Initial autonomous climb altitude (e.g., 5m) |
| `nav.throttle_hover` | From Phase 0 hover measurement |
| `safety.geofence_radius_m` | Hard cap on distance from launch home (e.g., 100m) |
| `safety.min_vbat` | Per battery chemistry. 4S LiPo typical: 14.4V (3.6V/cell) |
| `waypoint.lat_deg`, `lon_deg`, `alt_m` | Set on race day; placeholder OK now |

## Step 2 — Dry-run smoke test

```bash
cd /opt/racer-companion/companion
.venv/bin/python -m racer_companion.main --config /etc/racer-companion/config.json --dry-run --log-level DEBUG
```

Expected: logs on stdout, no serial port opened, no MSP traffic. Process
loops indefinitely. `Ctrl-C` to stop. If you see a config-loading error, fix
the JSON.

## Step 3 — Replay-sim controller behavior

```bash
.venv/bin/python -m tools.replay_synth --config /etc/racer-companion/config.json \
                                       --start-distance-m 50 \
                                       --start-bearing 180 \
                                       --current-heading 0 \
                                       --ticks 500
```

This generates a fake GPS trajectory 50m due south of the waypoint, with the
quad initially heading north. The controller's outputs are printed each tick.
Look for:

- **Yaw command** changes to align heading to bearing (initially the bearing
  is 0° = north and quad is heading 0°, so yaw stays at 1500). Try
  `--current-heading 90` to see yaw correction.
- **Pitch command** climbs from 1500 (centered) when heading_err is large,
  rises when heading is aligned, decreases as distance shrinks.
- **Throttle** stays near `nav.throttle_hover` because target altitude == current.
- **ARRIVED** appears within 200–400 ticks for default tuning at 30–50m start.

If `REACHED TICK LIMIT WITHOUT ARRIVAL` — increase `nav.pitch_kp_per_m` or
`nav.pitch_max_us`. If oscillating — decrease `nav.yaw_kp`.

## Step 4 — Bench test with real telemetry

Battery in, props OFF, FC powered. Companion on FC's 5V BEC. Outdoors with
GPS lock.

```bash
.venv/bin/python -m racer_companion.main --config /etc/racer-companion/config.json --log-level INFO
```

Expected log lines:
```
INFO Home set: 37.xxxxxxx, -122.xxxxxxx
INFO MSP client open on /dev/ttyAMA0 @ 115200
```

With pilot AUX-companion **LOW**, log should show `state=IDLE` repeatedly,
`safety_ok=True`, `rc_sent=null`. **No MSP_SET_RAW_RC should be transmitted.**

With AUX-companion **HIGH**, log should immediately switch to `state=CLIMB`,
`rc_sent=[1500, 1500, 1500, <hover>+offset, ...]`. Throttle command should be
near `nav.throttle_hover` + climb correction.

Flip back: AUX LOW → state returns to IDLE, no MSP. ✓

## Step 5 — Logging path

Set `companion.log_path` to `/var/log/racer-companion.jsonl`. Each tick is
one JSON line with `t`, `state`, `aux_active`, `safety_ok`, `safety_reasons`,
`rc_sent`, `alt_m`, `heading_deg`, `distance_m`, `home`. Useful for post-flight
analysis with `jq`.

```bash
sudo touch /var/log/racer-companion.jsonl
sudo chown racer:dialout /var/log/racer-companion.jsonl
```

## Step 6 — systemd auto-start

Once Step 4 runs cleanly:

```bash
sudo cp /opt/racer-companion/companion/systemd/racer-companion.service \
        /etc/systemd/system/
sudo useradd -r -G dialout racer || true
sudo chown -R racer:dialout /opt/racer-companion
sudo systemctl daemon-reload
sudo systemctl enable --now racer-companion
journalctl -u racer-companion -f
```

Service should report identical behavior to manual run.

## Tuning order (when you graduate to Phase 3 field tests)

Tune one parameter at a time, log per change, and never tune mid-session:

1. `nav.throttle_hover` — measured on the bench in Phase 0; verify altitude
   hold within ±0.5m in tethered hover.
2. `nav.yaw_kp` — start at 4.0. Increase until heading aligns briskly without
   visible overshoot. Stop ≤10.0.
3. `nav.pitch_kp_per_m` and `nav.pitch_max_us` — start at 4.0 / 150. Increase
   `pitch_max_us` (caps forward stick) for faster transit; increase
   `pitch_kp_per_m` for crisper response near target.
4. `nav.throttle_kp_per_m` and `nav.throttle_kd_per_cms` — increase Kp until
   altitude hold is tight; add Kd if it oscillates.
5. `nav.arrival_radius_m` and `companion.arrival_dwell_s` — trade between fast
   arrival lock and stable hold (smaller radius = tighter hold but slower commit).

## Exit criterion

✅ Dry-run, replay-sim, and bench-with-telemetry all behave correctly.
systemd service starts automatically on boot. Logging works.

→ Continue to [Phase 3 — Field drills](05_phase3_field_drills.md).
