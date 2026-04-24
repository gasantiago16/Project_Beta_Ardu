# Phase 0 — GPS Rescue (per drone, ~1 evening)

## Goal

Every drone in the fleet flies home automatically on lost RC link. No autonomy
yet — just the safety floor.

## Hardware shopping list

- **GPS module:** Matek M10Q-5883 (~$25) or BN-880. **Disable the mag** in
  Phase 0 — it's the #1 cause of GPS Rescue flyaways per the BF wiki.
- **Barometer:** if your FC has none built in, add a DPS310 or BMP388 module.
  Check Phase 0.5 audit row first.
- **4-pin JST cable** to wire GPS to a free FC UART (TX/RX/GND/5V).
- A patch of frame top to mount the GPS, away from VTX antenna and battery
  leads.

## Wiring

1. GPS 5V ← FC 5V pad. GPS GND ← FC GND.
2. GPS TX → FC UART RX. GPS RX → FC UART TX. (Crossed.)
3. Mount GPS on the frame top, ideally with a 1cm standoff. Antenna upward.
4. Verify GPS LED blinks on power (no fix yet) and goes solid after a fix
   outdoors.

## Betaflight setup

1. Flash Betaflight 4.5+ (4.6 if H7) using Configurator's flash tab.
2. **Ports tab:** enable GPS on the GPS UART, baud 115200.
3. **Configuration tab:** enable GPS feature.
4. **Failsafe tab:** Stage 2 procedure → **GPS Rescue**.
5. **CLI tab:** paste the contents of
   [`bf_config/phase0_gps_rescue.diff`](../bf_config/phase0_gps_rescue.diff).
   The file ends with `save` — FC will reboot.
6. Reconnect, verify under Configurator → Setup tab that GPS shows sat count
   when outdoors.

## Per-drone tuning: hover throttle

Critical setting. GPS Rescue uses `gps_rescue_throttle_hover` to hold altitude
on the way home.

1. Outdoor location, props on, battery in, arm, manual hover at ~1m.
2. Look at OSD throttle %.
3. Convert: `gps_rescue_throttle_hover = (hover_pct * 10) + 1000`.
   - Example: hovers at 30% → set to **1300**.
4. CLI: `set gps_rescue_throttle_hover = 1300; save`.
5. Record per drone in shared spreadsheet (Phase 0.5 captures this).

## Bench validation drill (REQUIRED before sign-off)

**Prerequisite:** outdoor location, ≥50m radius clear airspace, no people
downrange. Battery fresh.

For each drone, run **5 consecutive successful drills** before signing off:

1. Arm. Fly out 50m, altitude ~10m, hover.
2. **Power off the radio** (not the AUX failsafe switch — actually power-cycle
   it).
3. Quad must:
   - Climb to `gps_rescue_initial_climb` altitude (10m default).
   - Yaw to face home.
   - Fly home along bearing-to-launch.
   - Descend.
   - Land within ~5m of takeoff.
4. Power radio back on, disarm via switch.
5. Log pass/fail per drone in shared sheet.

If any of 5 drills fails → fix, re-tune, restart count from zero.

## Common failures

| Symptom | Likely cause | Fix |
|---------|--------------|-----|
| Flyaway (drone goes the wrong way) | Mag interference / bad mag cal | `set mag_hardware = NONE` |
| Lands hard | `gps_rescue_throttle_hover` too high | Lower by 25, retest |
| Climbs forever | `gps_rescue_throttle_hover` way too high | Lower by 50–100 |
| Won't activate | Sat count < `gps_rescue_min_sats` | Wait for fix, check antenna mount |
| Drifts on landing | GPS jitter (normal within 3–5m) | Acceptable |
| Yaws wildly | No mag, but trying to yaw at low groundspeed | Increase climb altitude |

## Exit criterion

✅ 5/5 consecutive bench failsafe drills pass per drone, no flyaways, all
landings within 10m of takeoff. Hover-throttle value recorded in shared sheet.
Per-drone `diff all` output committed to `bf_config/per_drone/<pilot>_<drone>_phase0.txt`.

## References

- [Betaflight GPS Rescue Wiki](https://github.com/betaflight/betaflight/wiki/GPS-rescue-mode)
- BF source: `src/main/flight/gps_rescue_multirotor.c`
- BF source: `src/main/flight/failsafe.c` (lines ~280–340 for the rescue chain)
- [Oscar Liang — GPS Rescue setup](https://oscarliang.com/setup-gps-rescue-mode-betaflight/)
