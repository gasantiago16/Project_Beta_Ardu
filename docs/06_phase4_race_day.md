# Phase 4 — Race Day Procedure

## Goal

Use the system in anger: autonomous launch, manual race, automatic recovery
on link loss. Repeatable across multiple races and pilots.

## Pre-event setup (~30 minutes)

For each participating drone:

1. Power Pi via FC BEC. Verify `racer-companion.service` running:
   `journalctl -u racer-companion -n 20 --no-pager`.
2. Verify GPS lock in OSD (≥8 sats).
3. Set the start-line waypoint:
   - Walk to the start grid spot for this drone.
   - On phone, get lat/lon (e.g., from `https://www.gps-coordinates.net/`).
   - SSH into Pi and edit `/etc/racer-companion/config.json` `waypoint`
     section. Or use the Pi's WiFi AP form (TODO if implemented).
   - `sudo systemctl restart racer-companion`.
4. Verify the new waypoint in journal: `journalctl -u racer-companion -n 5`
   shows `waypoint=<lat>,<lon> alt=Xm`.
5. AUX-companion LOW. Mode AUX → ANGLE. Arm AUX → DISARMED.

## Race-start countdown procedure

Race director calls phases over loudspeaker:

| Phase | Director call | Pilot action | Expected drone behavior |
|-------|---------------|--------------|--------------------------|
| 30s warning | "30 seconds" | Pre-flight: GPS lock OK, battery fresh, props secure | Idle, MSP silent |
| 10s warning | "10 seconds" | Mode AUX → ANGLE, Arm AUX → ARMED, AUX-companion → HIGH | Companion → CLIMB → TRANSIT → HOLD |
| 5s warning | "5 seconds" | Hands clear of throttle | Drone holding at start gate |
| GO | "GO!" | AUX-companion → LOW, Mode AUX → ACRO | Drone in pilot's hands, native BF feel |

## During the race

Companion is silent (state = IDLE). 100% native Betaflight, no autonomy
interference.

If pilot crashes or loses link mid-race:
- **Crash:** disarm via TX, walk to recover.
- **Lost link:** BF GPS Rescue (Phase 0) climbs to `gps_rescue_initial_climb`,
  yaws home, transits, lands within ~5m of takeoff. Companion stays silent
  the whole time (it sees AUX-companion = LOW because MSP_RC is reporting
  the RX-failsafe values, which are configured to drive AUX-companion to
  LOW per the BF Failsafe tab).

## Post-race

1. Pilot lands manually. Disarm.
2. Companion AUX should already be LOW from race-start.
3. Pull battery.
4. Optional: pull `/var/log/racer-companion.jsonl` from Pi for post-flight
   review.

## Multi-pilot start

If 4 pilots race simultaneously:
- Each pilot's AUX-companion is on their own TX → independent.
- Each Pi has its own waypoint at its own start gate.
- Race director countdown drives all pilots simultaneously.
- "GO" — all 4 pilots flip AUX-companion LOW within their own reaction time.
- Race begins; the staggered "GO" reactions are ~100ms variance, smaller than
  the 500ms MSP-override timeout, so each drone hands off cleanly.

## Failure modes during race day

| Symptom | Likely cause | On-the-spot fix |
|---------|--------------|-----------------|
| Companion never engages on AUX HIGH | service not running, no GPS lock | `systemctl status racer-companion`, check OSD |
| Drone drifts during HOLD | wind > controller gain | accept drift inside arrival_radius, abort if >5m |
| Drone won't release on AUX LOW | MSP-override timeout misbehaving | TX off → GPS Rescue brings it home |
| Drone lands itself unexpectedly | safety violation (low vbat, geofence, etc.) — RELEASED state | check journal for `safety_reasons` |

## Don'ts

- **Don't use this in a sanctioned MultiGP / DRL race without organizer
  approval.** Most racing leagues require Betaflight only — companion-driven
  autonomous launch may violate rules.
- **Don't change tuning at the field.** Tune in Phase 2/3 dev sessions, race
  with what you tuned.
- **Don't ignore a failed Drill 4.** If the failsafe-during-companion drill
  ever regressed, return to Phase 3 before next race day.

## Exit criterion (per event)

✅ All participating drones complete the race with no flyaways and no manual
recovery interventions caused by autonomous behavior. Drill log updated with
race-day observations.
