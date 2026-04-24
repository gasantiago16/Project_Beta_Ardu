# EdgeTX Lua Scripts for Project_Beta_Ardu

Radio-side scripts for **EdgeTX** transmitters (Radiomaster TX16S / Pocket / Boxer / Zorro / TX12, Jumper T-Pro / T-Lite, FrSky X9D / Tandem, etc.). These run on your TRANSMITTER, not the drone.

## What's in here

| File | Type | Purpose |
|------|------|---------|
| [`SCRIPTS/MIXES/racestrt.lua`](SCRIPTS/MIXES/racestrt.lua) | Mix script | Race-start sequencer. One-button auto-launch → hold-at-start → "GO" releases to manual race. Eliminates the fumble-the-dual-flip failure mode. |
| [`SCRIPTS/MIXES/safelock.lua`](SCRIPTS/MIXES/safelock.lua) | Mix script | Belt-and-suspenders: forces AUX-companion LOW any time mode = ACRO. Implements the `safety_runbook.md` "ACRO+AUX-HIGH = NEVER" rule in software so a stuck switch can't kill you. |
| [`SCRIPTS/TELEMETRY/racehud.lua`](SCRIPTS/TELEMETRY/racehud.lua) | Telemetry script | Race HUD — battery, sats, RSSI/LQ, GPS distance/altitude, race state. Color radios get a richer layout; mono falls back to text. |
| [`lua_for_beginners.md`](lua_for_beginners.md) | Tutorial | Lua syntax + EdgeTX API + how to debug + suggested first edits. For someone new to coding. |
| [`model_setup_walkthrough.md`](model_setup_walkthrough.md) | Tutorial | Step-by-step radio setup (Radiomaster Pocket as example, transferable). |

## Compatibility

- **EdgeTX 2.10+** primary target (works on 2.9 with minor caveats noted in script comments).
- **OpenTX 2.3+** mostly compatible — `playTone` and `getValue` work the same. Untested on OpenTX in 2026.
- All filenames ≤ 8 chars + `.lua` so monochrome radios with FAT 8.3 limit still load them.

## Install

1. Power off radio. Pull SD card. Mount on computer.
2. Copy the `SCRIPTS/` directory in this repo over the SD card's root `SCRIPTS/` directory. (Merge — don't replace; you may have other Lua scripts there already.)
3. Eject SD, replace in radio, power on.
4. Follow [`model_setup_walkthrough.md`](model_setup_walkthrough.md) to wire scripts to your model.

## Switch layout (recommended)

The scripts assume this physical layout — you can change it in the model setup, but this layout is what `safety_runbook.md` recommends:

| Switch | Function | Positions |
|--------|----------|-----------|
| **SA** (3-pos) | Manual mode pass-through | Down=ACRO, Mid=ANGLE, Up=GPS_RESCUE |
| **SB** (2-pos toggle) | Manual companion AUX pass-through | Down=LOW, Up=HIGH |
| **SD** (arm switch) | Arm | Down=disarmed, Up=armed |
| **SH** (momentary) | Race-start TRIGGER | Press & hold 2s to engage sequence |
| **SI** (momentary) | "GO" trigger | Tap to release from auto-hold to manual race |
| **SF** (2-pos toggle) | ABORT | Up=abort (forces GPS_RESCUE) |

**Why this layout works:** `safelock.lua` forces AUX-companion LOW whenever Mode is ACRO. So even if SA goes to ACRO and SB stays HIGH (the dangerous combo), the actual companion AUX channel sent to the FC is LOW. The pilot can't physically reach the unsafe state.

## How the scripts chain together

```
Physical switches (SA, SB, SD, SH, SI, SF)
        │
        ▼
┌──────────────────┐
│  racestrt.lua    │  reads physical switches
│                  │  state machine: IDLE → CONFIRM → AUTO_LAUNCH →
│                  │   HOLDING → RACING (or → ABORT)
│                  │  outputs: RsArm, RsMode, RsComp
└──────────┬───────┘
           │
           ▼
┌──────────────────┐
│  safelock.lua    │  reads RsMode + RsComp
│                  │  forces SfComp = LOW if RsMode = ACRO
│                  │  output: SfComp
└──────────┬───────┘
           │
           ▼
   Mixer channels:
     CH5 ← RsArm     (your arm AUX channel in BF)
     CH6 ← RsMode    (your mode AUX channel in BF)
     CH7 ← SfComp    (your companion AUX channel in BF)
        │
        ▼
   RX → Betaflight FC → companion (Pi) sees AUX state
```

## Customization

Every tunable is at the top of the script as a named constant. Don't dig through the state machine to change a timeout — change `CONFIRM_HOLD_S` and you're done. See `lua_for_beginners.md` for what's safe to change.

## Troubleshooting

**Scripts don't appear in the menu.**
- Filename longer than 8 chars (B&W radios). Rename.
- File on wrong path (`/SCRIPTS/MIXES/` vs `/SCRIPTS/MIX/` — depends on EdgeTX version).
- Reboot radio after copying.

**Lua error popup on boot.**
- Press EXIT to dismiss, then Radio → Tools → Lua Console for the error.
- Common: missing input source assignment in script setup.

**State machine seems stuck.**
- Check `racehud.lua` HUD — it shows current state.
- Try the ABORT switch — that always returns to a safe state.
- Power-cycle the radio if all else fails (script state resets on init).

**Audio cues silent.**
- Volume up. EdgeTX `playTone` requires non-zero volume.
- Check Speaker Volume: Radio → Sound → Speaker Volume.

## Don't

- **Don't run racestrt.lua without safelock.lua.** The sequencer is reasonably safe alone, but safelock is the floor that catches script bugs.
- **Don't change the input order in the script declaration without updating the radio's input source assignments** — they're positional.
- **Don't share your `models/` folder publicly** — it includes radio bind keys.
