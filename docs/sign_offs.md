# Sign-offs

One section per drone. Sign each line as the drill reaches its required
consecutive-pass threshold. Stream C owner countersigns.

## Sign-off thresholds

(Mirrors `docs/drill_log.md` — kept here for reference.)

| Drill | Required consecutive passes | Stream C owner countersignature required |
|-------|----------------------------|------------------------------------------|
| phase0 | 5 | Yes |
| drill_1 | 3 | No |
| drill_2 | 3 | No |
| drill_3 | 5 | Yes |
| drill_4 | 3 | **Yes — load-bearing safety drill** |
| drill_5 | 3 | Yes |

## Example drone

```markdown
## mr_slippery (gabriel, Kakute H7)

- [x] phase0    — PASS 5/5 — 2026-04-26, signed: gabriel  | countersigned: <stream-C>
- [ ] drill_1
- [ ] drill_2
- [ ] drill_3
- [ ] drill_4   ⚠ DO NOT FLY PHASE 4 UNTIL THIS PASSES
- [ ] drill_5
```

---

## Real sign-offs

<!-- One section per drone. -->
