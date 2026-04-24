# Per-drone Betaflight dumps

Each pilot commits a `<pilot>_<drone>.txt` file here containing the output of
`diff all` from the Betaflight Configurator CLI after applying the phase
configs and tuning the drone-specific values.

## Why

- **Drift detection.** If a setting changes between sessions (deliberate or
  not), `git log` and `git diff` show exactly what.
- **Recovery.** If an FC bricks, a fresh flash + paste of this file gets you
  back to a known-flying state in minutes.
- **Per-drone parameter sharing.** When pilot A's tune works, others can see
  the actual values rather than guessing.

## Naming convention

`<pilot_handle>_<frame_or_FC>_<phase>.txt`

Examples:
- `gabriel_kakuteh7_phase0.txt`
- `gabriel_kakuteh7_phase1.txt`
- `brad_speedybeev3_phase0.txt`

## Procedure

1. Apply the phase diff(s) from `../phase*.diff`.
2. Tune per-drone parameters (see `docs/01_phase0_gps_rescue.md` etc.).
3. In Configurator → CLI tab: `diff all`.
4. Copy the entire output, paste into a new file here, commit.

## Don't

- Do not commit dumps that include radio bind keys (some receivers expose
  these via CLI). Strip those lines before committing.
- Do not commit dumps from a drone that hasn't passed Phase 0 bench drills.
