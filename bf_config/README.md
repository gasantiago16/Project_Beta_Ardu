# Betaflight CLI Configs

Drop-in CLI snippets for each phase. Apply via Betaflight Configurator → CLI tab:

1. Power up FC, connect over USB.
2. Open Configurator, choose your COM port, **Connect**.
3. Click **CLI** tab.
4. Paste the contents of the relevant `.diff` file. **Do NOT** type `save`
   yourself — the file ends with it.
5. After save, FC reboots. Reconnect.

## Files

- `phase0_gps_rescue.diff` — Phase 0 minimum: GPS Rescue on RC link loss.
  Required on every drone before any autonomy work.
- `phase1_msp_companion.diff` — Phase 1: MSP override on a free UART for the
  companion computer. Apply only after `phase0_gps_rescue.diff` is signed off.
- `per_drone/<pilot>_<drone>.txt` — versioned `diff all` output per drone
  (drift detector — if a setting changes unexpectedly, git tells you).

## Per-drone tuning

These two values MUST be set per-drone, not copied:

- `set gps_rescue_throttle_hover` — measured at hover (see Phase 0 doc)
- `serial UART_N 1 115200 ...` — `N` = the free UART you wired the companion to

`phase1_msp_companion.diff` ships with `UART_N` as a literal placeholder.
Substitute the real number (`UART4`, `UART6`, etc.) before pasting.

## Capturing per-drone state

After applying configs and tuning per-drone:

```
# In BF Configurator CLI:
diff all
```

Copy the output to `bf_config/per_drone/<pilot>_<drone>.txt` and commit.
This becomes the source of truth for "what's actually on this quad" and
makes drift visible in `git log`.

## Safety note

`phase1_msp_companion.diff` enables MSP RC override. **The override only
takes effect when MSP traffic is actively arriving** — when the companion
stops sending (silent state, crash, power cut), Betaflight reverts to the
real RX values within ~500ms. That timeout is the load-bearing safety
primitive. Bench-validate it on your BF version before flying — see BF
[issue #13374](https://github.com/betaflight/betaflight/issues/13374).
