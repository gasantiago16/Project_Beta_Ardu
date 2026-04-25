# Incident Log

Anything unexpected — flyaway, hard landing, near-miss, weird companion
behavior, mid-air glitch, gear failure. The point is to root-cause, not
blame. **Don't omit incidents because they were "embarrassing"** — the team
needs the data.

## Format

```markdown
## YYYY-MM-DD HH:MM — <one-line summary>

- **Pilot:** name
- **Drone:** name
- **Phase / drill:** which phase or drill was running
- **Outcome:** crash / drift / safe-landing / no-damage / equipment-loss
- **Root cause:** short factual description (or "unknown" if not yet diagnosed)
- **Evidence:** path to companion log, BF blackbox, video, photos
- **Fix applied:** what was changed
- **Drill re-run after fix:** which drill, what counter

## YYYY-MM-DD HH:MM — next incident
...
```

## Rules

- **Land + recover the quad** before writing the incident. Don't fly again
  until the entry exists.
- **Pull the SD card** and copy the companion log before re-flying. The log
  is the primary evidence.
- **Re-run the drill that should have caught it** after the fix. Reset the
  pass counter from zero.

## Example (delete when starting your real log)

```markdown
## 2026-04-27 11:47 — Companion held HOLD past 60s timeout, no abort

- **Pilot:** brad
- **Drone:** the_wiggler
- **Phase / drill:** drill_3 (full auto-launch)
- **Outcome:** safe-landing (manual override at 90s)
- **Root cause:** STATE_TIMEOUT_S in racestrt.lua was hardcoded but the script
  state machine ALSO has a separate timeout in companion/main.py — only the
  Lua side fired. Investigation: BF AUX-companion was still HIGH at 90s, but
  companion saw aux_active=True only because MSP_RC echo was lagged.
- **Evidence:** /var/log/racer-companion.jsonl from drone, photo of TX state
  at the moment, racehud screenshot
- **Fix applied:** added MSP_RC freshness check to safety.py — if last MSP_RC
  is >2s old, force aux_active=False. Bumped tests to cover this.
- **Drill re-run after fix:** drill_3, 0/5 pass count, restarted Apr 28
```

---

## Real log

<!-- Append below this line. Newest at the BOTTOM. -->
