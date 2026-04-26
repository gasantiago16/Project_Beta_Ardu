# Phase 4 — Radiomaster Pocket pilot RC injection

After Phase 3's smoke flow, the racer_companion drives channels 1-4 (when
AUX-companion-active is HIGH) and the pilot drives channels 5-8. Phase 4
gets the *pilot* side wired in for sim, via a USB joystick → UDP packet
injector that talks BF SITL's `PORT_RC` (UDP 9004) directly.

## Setup

1. Plug the Radiomaster Pocket via USB.
2. EdgeTX → Model Setup → USB Mode → **USB Joystick (HID)**. Confirm
   Windows sees it as a generic 8-axis HID gamepad (`joy.cpl`).
3. Run Windows joystick calibration once (`joy.cpl` → Properties →
   Settings → Calibrate). Without this, axis 2 (throttle) may report
   0..+1 instead of -1..+1, breaking the `invert: true` mapping.

## Run

```bash
cd ~/Project_Beta_Ardu
python -m integrations.tools.radio_to_bf
```

Default config at `integrations/configs/radiomaster_pocket.json` —
EdgeTX AETR layout. Pitch + throttle are inverted to match SDL's
stick-up = -y convention.

Useful flags:
- `--list-joysticks` — enumerate connected HIDs and exit. Use the index
  with `--joystick-index N` if the Pocket isn't first.
- `--dry-run` — print PWM, don't send. For calibration / debugging.
- `--sweep` — synthetic sine sweep on every channel; no radio required.
  Pair with `--aux 7=1900` to force AUX-companion-active high.
- `--aux 7=1900` — override channel 7 (AUX3 by default) to PWM 1900.
  Useful for forcing companion-takeover without touching a switch.

## Validation

In T1's docker compose log, look for the BF SITL `[bridge] rx=N`-style
counter ticking up — once `radio_to_bf` is running, BF should be
seeing pilot RC packets at 50 Hz.

In T3 (companion), `aux_active` flips True when AUX3 crosses 1700 µs
(default `aux_channel_index=6, aux_active_us_min=1700`). State machine
transitions IDLE → CLIMB.

If `radio_to_bf` is NOT running, expect:
- Companion's `no_rc_telemetry` safety reason fires (BF SITL doesn't
  synthesize MSP_RC values without UDP 9004 input).
- Bridge tx counter still climbs but rx may stay low.

## Known issues

- **Throttle mapping on a stock Windows install.** EdgeTX outputs the
  throttle stick as -1 to +1 in joystick mode. Without Windows
  calibration, the OS may rescale to 0..+1, breaking `invert: true`.
  Always run `joy.cpl` calibration after first plug-in.
- **AUX channels output ±1, not 0/+1.** A 2-position switch with
  `deadband: 0.0` reports either 1000 or 2000 µs, never 1500. That's
  fine for arming/mode bits but means a centered "neutral" position is
  not 1500 unless the radio config explicitly outputs 0.
