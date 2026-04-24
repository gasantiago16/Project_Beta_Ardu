# Phase 1 — Companion Wiring & MSP Bring-up (1 weekend per reference rig)

## Goal

Wire the companion computer to one drone (the **reference rig** — don't touch
the rest of the fleet yet). Prove props-off that the companion can read BF
telemetry over MSP and that BF accepts MSP RC override on the AUX-active flag.

## Hardware

- **Companion:** Raspberry Pi Zero 2W (~$15). Recommended over ESP32 for the
  reference rig because Linux gives you real logging and SSH debugging.
- **SD card:** 32GB Class 10. Flash Raspberry Pi OS Lite (64-bit) with
  rpi-imager — set hostname, user, WiFi, SSH key in the imager's advanced
  options before writing.
- **5V supply:** ~500mA from FC's 5V BEC. Verify FC BEC can sustain.
- **JST 4-pin** cable: 5V, GND, FC UART TX → Pi UART RX, FC UART RX → Pi UART TX.
- **TPU mount** for the Pi (3D-print one or buy generic).

## Wiring diagram

```
FC UART_N (free)              Pi Zero 2W
  TX  ─────────────────────►  RX  (GPIO15, pin 10)
  RX  ◄─────────────────────  TX  (GPIO14, pin 8)
  GND ─────────────────────   GND (any GND pin)
  5V  ─────────────────────►  5V  (pin 2 or 4)
```

Pi serial is `/dev/ttyAMA0` (PL011) by default. Disable serial console:
```bash
sudo raspi-config nonint do_serial_hw 0       # enable hardware serial
sudo raspi-config nonint do_serial_cons 1     # disable login over serial
sudo reboot
```

## Apply Phase 1 BF config

1. Edit `bf_config/phase1_msp_companion.diff` — replace the literal
   `UART_N` with the real UART number you wired (e.g., `UART4` or `UART6`).
2. Paste into Configurator → CLI tab. FC reboots.
3. Verify in CLI: `dump | grep msp_override` should show
   `set msp_override_channels_mask = 15`.
4. Verify in Configurator → Ports tab that MSP is enabled on the chosen UART
   at 115200.

## Install companion software

On the Pi (SSH in):

```bash
sudo apt update && sudo apt install -y python3-venv git
sudo git clone https://github.com/gasantiago16/Project_Beta_Ardu.git /opt/racer-companion
cd /opt/racer-companion/companion
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
.venv/bin/pip install -e .
```

Don't enable the systemd service yet — Phase 1 is bench-only.

## Bench test (props OFF — verify before installing props)

**Test 1 — MSP loopback (verify wire + protocol).** Battery in, props OFF.
Plug the Pi via USB power for the test (don't use FC BEC yet — easier debug).

```bash
cd /opt/racer-companion/companion
.venv/bin/python -m tools.msp_loopback_test --port /dev/ttyAMA0 --duration 5
```

Expected output:
```
Opened /dev/ttyAMA0 @ 115200
  MSP_API_VERSION: protocol=0 api=1.45    # or similar
  MSP_FC_VARIANT: b'BTFL'
Sending centered MSP_SET_RAW_RC for 5.0s (props OFF only)...
  MSP_RC echo: [1500, 1500, 1500, 1000, ...]
```

**If MSP_RC echo matches sent values, override is working.** If not:
- AUX channel that activates `msp_override` is LOW (pilot's switch). Flip it.
- `msp_override_channels_mask` not set to 15.
- Wrong UART or no MSP function enabled on that UART.

**Test 2 — RC override toggles cleanly.** Same setup, props still OFF:
1. Pilot pulls real TX sticks to extreme positions (e.g., full forward).
2. With companion AUX HIGH and the loopback test running, observe:
   `MSP_RC echo` shows centered values (1500), not the pilot's stick values.
3. Pilot flips companion AUX LOW. Within ~500ms, `MSP_RC echo` shows the
   pilot's actual stick positions again.
4. Repeat 5×. Both directions must work cleanly every time.

## Failure-mode bench check (the load-bearing safety test)

**Test 3 — Companion-process death recovery.**
1. Companion AUX HIGH, loopback test running, props still OFF.
2. `Ctrl-C` the loopback test. Within 500ms, `MSP_RC echo` (now from Configurator's Receiver tab — keep it open) must show real pilot stick values.
3. If BF stays stuck on the last MSP frame indefinitely, you have a BF version
   without MSP-override timeout — see BF
   [issue #13374](https://github.com/betaflight/betaflight/issues/13374).
   **Do not proceed past this test until resolved** — either upgrade BF or
   change strategy.

## Exit criterion

✅ Tests 1, 2, 3 all pass on the reference rig. Companion mounts cleanly
on frame. `bf_config/per_drone/<pilot>_<drone>_phase1.txt` committed with
new `diff all` output. Power moved from USB to FC 5V BEC and tests re-run.

→ Continue to [Phase 2 — Controller install + tune](04_phase2_controller.md).
