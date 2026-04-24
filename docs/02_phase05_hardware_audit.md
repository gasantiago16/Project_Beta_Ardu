# Phase 0.5 — Fleet Hardware Audit (½ day, one shared spreadsheet)

Required because the fleet is mixed. **Do this before anyone orders parts**
for Phase 1+, so you know which drones are eligible and which need an FC
upgrade first.

## Spreadsheet template (copy into Google Sheets)

| Pilot | Drone name | FC model | MCU | Free UARTs | Onboard baro? | GPS already? | ESC protocol | Phase target | Notes |
|-------|------------|----------|-----|-----------|---------------|--------------|--------------|--------------|-------|
| Gabriel | Mr. Slippery | Kakute H7 | H7 | UART4, UART6 | DPS310 | none | DShot600 | full | needs M10Q + Pi |
| Brad | The Wiggler | SpeedyBee F405 v3 | F4 | UART3 | BMP280 | M8N | DShot300 | 0 only | F4 → upgrade for full |
| ... | | | | | | | | | |

## How to fill each column

- **FC model + MCU:** Connect to BF Configurator → Setup tab. The board
  identifier is at the top. MCU = look up board → STM32F405 (F4),
  STM32F722/F745 (F7), STM32H743 (H7).
- **Free UARTs:** Configurator → Ports tab. Count UARTs that have NO function
  enabled (not RX, not GPS, not SmartAudio/IRC Tramp, not ESC telemetry).
  Need ≥1 free for Phase 1.
- **Onboard baro:** Configurator → Setup tab → Sensors. If "Baro" is listed
  with a chip name (DPS310, BMP388, BMP280, MS5611), you have one. None? add
  a module in Phase 0.
- **GPS already:** Same Sensors panel — GPS shows sat count when outdoors.
- **ESC protocol:** Configurator → Configuration tab → ESC/Motor → ESC protocol.
  Informational; no change needed.

## Decision matrix

| Hardware profile | Phase target | Action |
|-----------------|--------------|--------|
| H7 + free UART + baro present | **full** | Ready for Phases 0 → 4 as written |
| H7 + free UART + no baro | **full** | Add baro module before Phase 1 |
| F7 + free UART + baro | **full (test first)** | Phase 0 fine. Phase 1 may run but tight on CPU/UART — test on one F7 first before committing fleet |
| F4 + free UART | **0 only** | Phase 0 GPS Rescue works fine. Recommend FC upgrade to H7 before Phase 1 |
| Any + no free UART | **0 only** | Don't share UART with VTX/RX. FC upgrade required for Phase 1 |

## BOM driven by audit

After audit, sum the per-row needs:
- Number of GPS modules (count drones without GPS)
- Number of baro modules (count drones without onboard baro AND in "full")
- Number of Pi Zero 2W (count drones in "full")
- Number of FC upgrades (count "0 only" drones whose pilots want full)

## Output

Shared spreadsheet, one row per drone, "Phase target" column filled in for
all drones. Drives the BOM order and per-pilot work plan.

## Exit criterion

✅ Every drone has a row, every row has a Phase target. BOM compiled and
ordered. Sync the team on who's getting full vs Phase-0-only before parts
arrive.
