# Drill Log

Append one row per drill attempt, per drone. Stream C owner reviews before
sign-off. Format is fixed — fill in the columns, don't restructure.

## Format

```
YYYY-MM-DD HH:MM | <pilot> | <drone> | <drill> | <PASS|FAIL> | <notes>
```

- **drill:** `phase0` (GPS Rescue) | `drill_1` (hover) | `drill_2` (short hop) | `drill_3` (full launch) | `drill_4` (TX-kill failsafe) | `drill_5` (companion death)
- **notes:** anything that helps post-mortem — recovery time, drift distance, failure mode, weather

## Sign-off thresholds (per drone)

| Drill | Required consecutive passes |
|-------|----------------------------|
| phase0 | 5 |
| drill_1 | 3 |
| drill_2 | 3 |
| drill_3 | 5 |
| drill_4 | 3 |
| drill_5 | 3 |

A FAIL resets the counter. Sign off in [`docs/sign_offs.md`](sign_offs.md)
once threshold reached.

## Example rows (delete when starting your real log)

```
2026-04-26 14:32 | gabriel | mr_slippery   | phase0  | PASS | landed 2.4m short of takeoff, clean
2026-04-26 14:38 | gabriel | mr_slippery   | phase0  | PASS | 3.1m short, slight east drift
2026-04-26 14:45 | gabriel | mr_slippery   | phase0  | FAIL | flew north, recovered manually. Mag was re-enabled by mistake. RESET counter.
2026-04-26 15:02 | gabriel | mr_slippery   | phase0  | PASS | mag disabled, 1.8m short, clean
```

---

## Real log

<!-- Append below this line. Newest at the bottom. -->
