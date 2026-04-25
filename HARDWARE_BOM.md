# Hardware Bill of Materials

All prices USD as of April 2026, ballpark. Order from your usual race-quad
vendor (RaceDayQuads, GetFPV, Pyrodrone, AliExpress for budget). Links below
are to canonical product pages, not affiliate.

## Per-drone — Phase 0 only (GPS Rescue safety floor)

| Item | Part | Qty | Price | Notes |
|------|------|-----|-------|-------|
| GPS module | [Matek M10Q-5883](https://www.mateksys.com/?portfolio=m10q-5883) | 1 | $28 | M10 chipset, fast lock. Built-in mag (we **disable** it — Phase 0 doc) |
| Barometer | [BMP388 module](https://www.mateksys.com/?portfolio=baro-bmp388) | 0 or 1 | $8 | Skip if FC has built-in. Check with `Setup` tab in Configurator |
| JST-SH cable | 4-pin pre-wired | 1 | $2 | For GPS-to-FC UART |
| **Subtotal Phase 0** | | | **~$30–40** | per drone |

## Per-drone — Phase 1+ (companion computer)

Add to Phase 0 above:

| Item | Part | Qty | Price | Notes |
|------|------|-----|-------|-------|
| Companion SBC | [Raspberry Pi Zero 2W](https://www.raspberrypi.com/products/raspberry-pi-zero-2-w/) | 1 | $15 | Real OS, easy debug. Recommended for first build |
| SD card | SanDisk 32GB A1 | 1 | $8 | 32GB plenty; A1 is fastest random IO |
| 5V BEC | [Matek BEC-1S/3A](https://www.mateksys.com/?portfolio=bec-1s) | 0 or 1 | $5 | Only if FC's onboard 5V can't sustain Pi (~500mA) |
| JST-SH cable | 4-pin pre-wired | 1 | $2 | Pi UART to FC UART |
| TPU mount | print yourself or [Etsy generic](https://www.etsy.com/search?q=raspberry+pi+zero+2w+drone+mount) | 1 | $5 | Vibration-isolated mount for Pi |
| **Subtotal Phase 1+** | | | **~$35** | per drone, on top of Phase 0 |

**Alternative companion:** [ESP32-S3 DevKitC](https://www.espressif.com/en/products/devkits/esp32-devkitc) — $8, lighter, lower power. But: requires MicroPython port (see [`ROADMAP.md`](ROADMAP.md)). Not recommended for the first build.

## Per-drone — FC upgrade (only if Phase 0.5 audit shows F4 / no free UART)

| Item | Part | Qty | Price | Notes |
|------|------|-----|-------|-------|
| H7 stack | [Holybro Kakute H7](http://www.holybro.com/product/kakute-h7/) | 1 | $70 | Stock racing FC, multiple free UARTs, onboard baro |
| H7 stack alt | [Mamba H743 v4](https://www.diatone.us/products/mamba-h743-v4-flight-controller-stack) | 1 | $55 | Cheaper, still has bdshot + free UARTs |
| H7 stack alt | [SpeedyBee F405 V4 → upgrade to V4 H7](https://www.speedybee.com/) | 1 | $50–80 | If pilot already has SpeedyBee ecosystem |
| 30A BLHeli32 ESCs | (any 30A 4-in-1) | 1 | $40–60 | If swapping FC, often easier to swap ESC stack too |
| **Subtotal FC upgrade** | | | **~$55–130** | only as needed |

## Per-pilot — radio side (if running EdgeTX scripts)

Most pilots already have these. Listed for completeness.

| Item | Notes |
|------|-------|
| Radio running EdgeTX 2.10+ | Radiomaster TX16S / Pocket / Boxer / Zorro / TX12, Jumper T-Pro, FrSky X9D |
| ELRS or Crossfire RX bound | For low-latency RC link. ELRS is the modern default |
| 6 free switches | SA (3-pos), SB (2-pos toggle), SD (arm), SH (momentary), SI (momentary), SF (2-pos toggle). See `edgetx_scripts/README.md` for layout |

## Total per build

| Build profile | Phase 0 | Phase 1+ | FC upgrade | Total |
|---------------|---------|----------|------------|-------|
| Existing H7 + free UART + baro | $30 | $35 | — | **~$65** |
| Existing H7 + free UART, no baro | $40 | $35 | — | **~$75** |
| Existing F4 + free UART, gets full | $40 | $35 | $70 | **~$145** |
| Existing F4, Phase 0 only | $30 | — | — | **~$30** |

## Where to buy

- **US:** [RaceDayQuads](https://www.racedayquads.com/), [GetFPV](https://www.getfpv.com/), [Pyrodrone](https://pyrodrone.com/)
- **EU:** [Drone-FPV-Racer](https://www.drone-fpv-racer.com/), [Phaser FPV](https://phaserfpv.com/)
- **Budget:** [Banggood](https://www.banggood.com/), [AliExpress](https://aliexpress.com/) — cheaper, slower, occasional QC issues. Fine for spares; not for the first build where you want to debug.

## Don't substitute

- **Don't use a Beitian / random no-name GPS.** Most BF Rescue flyaway reports
  trace to bad GPS modules with corrupt firmware. Stick to Matek / HGLRC /
  M8N / M10 from a known vendor.
- **Don't use BN-220.** Older M8 chipset, slower lock, weaker signal in
  carbon-frame quads. The extra $10 for an M10 is worth it.
- **Don't use a Pi 3B+ or Pi 4.** Too heavy and too power-hungry. Pi Zero 2W
  has the same compute as a Pi 3 in 1/3 the weight.
