# Model Setup Walkthrough — Radiomaster Pocket (EdgeTX)

This walks through wiring the scripts to a model on a Radiomaster Pocket
running EdgeTX 2.10. The same flow works on TX16S, Boxer, Zorro, T-Pro, and
T-Lite — only the menu nav buttons differ.

For Radiomaster TX16S / Boxer (with touchscreen), tap to navigate; the
menu structure is identical.

## 0. Prerequisites

- EdgeTX 2.10+ installed.
- SD card prepared with `/SCRIPTS/MIXES/racestrt.lua`,
  `/SCRIPTS/MIXES/safelock.lua`, `/SCRIPTS/TELEMETRY/racehud.lua`
  (copy from this repo's `SCRIPTS/` directory).
- Receiver bound (ELRS / Crossfire) and channels mapped to your FC's RX
  protocol.

## 1. Create the model

1. Power on radio. Long-press **MENU** to enter MODEL menu.
2. Page to MODEL SELECT. Highlight an empty slot. Press **ENT** to enter,
   choose **Create model**.
3. Name it (e.g., `RACE_QUAD`). Set internal/external module per your RX.
4. Bind the receiver.

## 2. Wire the scripts

1. From MODEL menu, page to **Custom Scripts** (a.k.a. LUA, Lua scripts).
2. Press **ENT** on the first slot. Pick `racestrt.lua` from the list.
3. EdgeTX shows the input/output configuration screen. Set:

   | Input | Source |
   |-------|--------|
   | Trig  | SH (momentary, configured as toggle if needed) |
   | Go    | SI (momentary) |
   | Abrt  | SF (2-pos) |
   | Mode  | SA (3-pos) |
   | Comp  | SB (2-pos) |

   Outputs (RsArm, RsMode, RsComp) appear automatically.

4. Press **ENT** on the second slot. Pick `safelock.lua`. Set:

   | Input | Source |
   |-------|--------|
   | Mode  | RsMode (output of racestrt) |
   | Comp  | RsComp (output of racestrt) |

   Output: SfComp.

## 3. Wire the channels

1. From MODEL menu, page to **Mixers** (sometimes "Mixes").
2. Find the channel you've assigned in BF for **arm** (commonly CH5).
   Set its source to **RsArm**. Weight 100, offset 0.
3. Find the channel for **mode** (commonly CH6). Source = **RsMode**.
4. Find the channel for **companion AUX** (commonly CH7). Source = **SfComp**.
   **Important:** use SfComp (the safelock output), NOT RsComp directly.
5. Save.

## 4. Verify channel output

1. From MODEL menu, page to **Channels** (or **Channel Monitor**).
2. With all switches in idle position (SA down=ACRO, SB down=LOW, SH off,
   SI off, SF off):
   - CH5 should show ~ -1024 (arm OFF)
   - CH6 should show ~ -1024 (ACRO)
   - CH7 should show ~ -1024 (companion LOW) — because safelock forces LOW
     when mode is ACRO

3. Flip SA up (Mode → GPS_RESCUE), SB up (Comp → HIGH):
   - CH6 → +1024
   - CH7 → +1024 (no longer suppressed because mode isn't ACRO)

4. Now SA up, SB up, SH press-and-hold for 2 seconds:
   - You should hear: tone (CONFIRM) → after 2s, three-beep + "3, 2, 1"
   - State should advance to AUTO_LAUNCH (mode forced to ANGLE, comp HIGH,
     arm ON)

5. Tap SI (Go):
   - tone (transition to RACING)
   - CH6 → -1024 (ACRO), CH7 → -1024 (companion LOW)

6. Flip SF up (Abort):
   - low tone + haptic
   - CH6 → +1024 (GPS_RESCUE), CH7 → -1024 (companion LOW), arm OFF

## 5. Add the telemetry HUD

1. From MODEL menu, page to **Display** (a.k.a. Telemetry Screens).
2. Pick a screen slot. Type = **Script**. Script = **racehud.lua**.
3. Save.
4. Exit to the main screen. From main screen, page to telemetry view (PAGE
   button on Pocket; long-press PAGE to choose screen).
5. With telemetry alive, you should see RSSI, battery, sats, and the
   current race state.

## 6. Bench test (props OFF)

This is the most important step. Do this on the bench before any flight.

| Test | Expected |
|------|----------|
| Power on, all switches idle | State=IDLE, CH7 (comp)=LOW regardless of SB |
| SA up (RESCUE), SB up | CH7=HIGH (no safety lock here, mode isn't ACRO) |
| SH press-hold 2s | Audio cues fire, State=LAUNCH, CH5=ON, CH6=ANGLE, CH7=HIGH |
| Wait 8s | State=HOLD, channels unchanged |
| Tap SI | State=RACE, CH6=ACRO, CH7=LOW |
| In RACE: try SB up (manual comp HIGH) | CH7 stays LOW — safelock forces it |
| SF up (Abort) | State=ABORT, CH6=RESCUE, CH7=LOW, ARM=OFF |
| SF down + tap SH (or wait) | State=IDLE |

If any test fails, check the Lua Console (Radio > Tools > Lua Console) for
errors.

## 7. First flight

Only after every step in §6 passes:

1. Props on, battery in, GPS lock confirmed in OSD.
2. Manual takeoff in ACRO. Get a feel for the build.
3. Land. Disarm.
4. Run the Phase 3 drills from `docs/05_phase3_field_drills.md` — drill 1 first.
5. Do not skip drill 4 (the failsafe drill). The whole project's safety
   model assumes it works.

## Troubleshooting

**Scripts don't appear in Custom Scripts list.**
- Filename too long for FAT 8.3. Confirm `racestrt.lua`, `safelock.lua`.
- Script in wrong directory: must be `/SCRIPTS/MIXES/`.
- Power-cycle the radio after copying.

**State machine appears stuck in CONFIRM.**
- SH source mis-assigned. Check that SH is reading >0 when pressed in the
  Inputs page on the radio.
- `CONFIRM_HOLD_S` is set high. Default 2.0s.

**Audio cues silent.**
- Speaker volume is zero (Radio > Sound > Speaker Volume).
- Sound disabled per-model.
- For `playNumber` to work, EdgeTX language pack must be installed.

**HUD shows STATE: ?.**
- racestrt.lua isn't running, so the global `raceStartStateName` is nil.
- Check the Custom Scripts list — is racestrt assigned and not erroring?

**HUD shows STATE: IDLE forever even after SH press.**
- racestrt.lua input "Trig" not assigned to a real switch source.
