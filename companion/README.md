# Companion Software (`racer_companion`)

Betaflight + autonomous nav companion. Reads MSP telemetry from the FC,
runs a P-controller toward a waypoint, writes virtual sticks back via
`MSP_SET_RAW_RC`. Stays silent unless an AUX channel is HIGH; pilot's
flick of that AUX hands control back to the real RX within ~500ms (BF
MSP-override timeout).

## Install on the Pi (Raspberry Pi OS Lite, 64-bit)

```bash
sudo apt update && sudo apt install -y python3-venv git
git clone https://github.com/gasantiago16/Project_Beta_Ardu.git /opt/racer-companion
cd /opt/racer-companion/companion
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
.venv/bin/pip install -e .
sudo cp systemd/racer-companion.service /etc/systemd/system/
sudo mkdir -p /etc/racer-companion /var/log
sudo cp config/start_line.example.json /etc/racer-companion/config.json
sudo useradd -r -G dialout racer || true
sudo chown -R racer:dialout /opt/racer-companion /var/log/racer-companion.jsonl 2>/dev/null || true
```

Edit `/etc/racer-companion/config.json` — at minimum, set the waypoint
lat/lon and the per-drone `nav.throttle_hover`.

Enable/start:

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now racer-companion
journalctl -u racer-companion -f
```

## Bench test (props OFF) before flying

```bash
# 1. Verify MSP comms
.venv/bin/python -m tools.msp_loopback_test --port /dev/ttyAMA0

# 2. Run controller in dry-run (no serial), confirm logic
.venv/bin/python -m racer_companion.main --config /etc/racer-companion/config.json --dry-run

# 3. Replay a synthetic trajectory through the controller (no FC needed)
.venv/bin/python -m tools.replay_synth --config /etc/racer-companion/config.json
```

## Run the test suite (development machine)

```bash
cd companion
python3 -m venv .venv
.venv/bin/pip install -r requirements-dev.txt
.venv/bin/pip install -e .
.venv/bin/pytest tests/ -v
```

## Architecture

```
main loop @ 50Hz
  ├── poll MSP responses (GPS, ATTITUDE, ALTITUDE, ANALOG, RC)
  ├── @ 10Hz: request all 5 telemetry messages
  ├── compute aux_active = (last_rc[aux_channel_index] >= 1700)
  ├── safety.evaluate(...) → SafetyStatus(ok, reasons)
  ├── nav.compute_rc(...) → 8-channel RC array + debug
  ├── state.step(...) → State (IDLE / CLIMB / TRANSIT / HOLD / RELEASED)
  └── if state.is_active(state):
        rc_safe = safety.clamp_rc(rc)
        msp.send_raw_rc(rc_safe)
      else:
        # silent — BF reverts to RX values within ~500ms
```

**Safety invariant.** The companion sends `MSP_SET_RAW_RC` ONLY when the
state machine returns `is_active() == True`. Pilot AUX low → IDLE → silent
→ RX takes over. Safety violation → RELEASED → silent → RX takes over.
Companion process dies → no MSP traffic → BF MSP-override timeout fires →
RX takes over. All three failure modes converge on the same recovery path.

## Files

- `racer_companion/msp.py` — MSP v1 wire protocol, encode + decode
- `racer_companion/fc/` — FlightController abstraction (Protocol + BetaflightAdapter)
- `racer_companion/nav.py` — bearing/distance + P controllers + HOLD position controller
- `racer_companion/state.py` — state machine
- `racer_companion/safety.py` — bounds, geofence, telemetry watchdog, divergence trackers
- `racer_companion/recorder.py` — byte-level MSP record (RecordingAdapter) + replay (ReplayAdapter)
- `racer_companion/mavlink.py` — MAVLink v2 publisher (UDP / serial)
- `racer_companion/config.py` — JSON config loader
- `racer_companion/main.py` — main 50Hz loop. Pass `--record <path>` to
  capture every MSP byte to a JSONL file for offline replay.
- `tools/msp_loopback_test.py` — bench test against a real BF FC
- `tools/replay_synth.py` — offline controller replay (synthetic input)
- `tools/replay.py` — replay a recorded byte log; run with
  `python -m tools.replay --log path.jsonl` from `companion/`
- `tests/test_*.py` — unit tests for protocol + math + state + safety
- `systemd/racer-companion.service` — auto-start on Pi boot

## Tuning

Defaults in `config/start_line.example.json` are conservative starting points.
Tune in this order:
1. `nav.throttle_hover` — per-drone, measure with manual hover at known altitude.
2. `nav.yaw_kp` — increase until quad rotates briskly toward bearing without overshoot.
3. `nav.pitch_kp_per_m` and `nav.pitch_max_us` — start low, increase until transit speed is acceptable.
4. `nav.throttle_kp_per_m` and `nav.throttle_kd_per_cms` — increase Kp until altitude hold is tight, increase Kd if it oscillates.
5. `nav.arrival_radius_m` and `companion.arrival_dwell_s` — trade between fast arrival and stable hold.
