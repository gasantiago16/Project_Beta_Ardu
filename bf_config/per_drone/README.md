# Per-drone Betaflight dumps

Two file types live here:

- **`<pilot>_<drone>_phase*.txt`** — the output of `diff all` from the
  Betaflight Configurator CLI. The state of record. Used for drift
  detection, recovery, and parameter sharing.
- **`<pilot>_<drone>_manifest.txt`** — a SHORT subset of `set` lines the
  pre-flight validator (`scripts/preflight_validator.py`) checks against
  a fresh `dump`. See `example_manifest.txt` for the floor every drone
  must satisfy.

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

## Pre-flight validation

Run before each flight day to catch a partial CLI apply or a forgotten
`save`:

```
# 1. SCP the dump from the drone (or paste from Configurator CLI tab):
scp pi@dronefpv:/tmp/dump.txt /tmp/your_drone_dump.txt

# 2. Validate against manifest (exit 0 = pass, 1 = mismatch, 2 = setup error):
python scripts/preflight_validator.py \
    --manifest bf_config/per_drone/example_manifest.txt \
    --dump     /tmp/your_drone_dump.txt
```

`example_manifest.txt` is the minimum every drone must satisfy. Copy it
to `<pilot>_<drone>_manifest.txt` and extend with drone-specific safety
settings as needed.
