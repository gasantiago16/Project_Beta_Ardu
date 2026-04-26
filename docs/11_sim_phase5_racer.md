# Phase 5 — 5" race-quad URDF

Replaces Pegasus's bundled Iris (F450-class, slow, T:W ~2) with a
realistic 5" FPV race quad (~500 g AUW, T:W ~10, race rates).

## Setup (one-time)

The race-quad USD is regenerated from URDF — `assets/racer_5in/*.usd*`
is in `.gitignore`. Do this once:

1. Open Isaac Sim.
2. `Isaac Examples → URDF Importer` (or `File → Import` → URDF).
3. Source: `~/Project_Beta_Ardu/assets/racer_5in/racer_5in.urdf`.
4. Output: `~/Project_Beta_Ardu/assets/racer_5in/racer_5in.usd`.
5. Settings:
   - **Merge Fixed Joints**: OFF (we need joint0..joint3 exposed)
   - **Fix Base**: OFF (free-flying)
   - **Self Collision**: OFF
6. Import + save.

Full procedure (incl. scriptable Python option) in
`assets/racer_5in/README.md`.

## Run

```bash
cd ~/PegasusSimulator
python ~/Project_Beta_Ardu/integrations/orchestrators/final_world_betaflight.py \
    --airframe racer5
```

Expect:
- Iris is replaced by the 5" airframe in the chemical-plant scene.
- Bridge stats print same as Phase 2 — UDP rx/tx ~250 Hz.
- BF flight envelope feels like a race quad: T:W ~10, agile rates.

## Validation

Hover stability is the key first test. With the companion + radio
running (T3 + T4), arm + slowly throttle up. The race-quad should:
- Reach hover at ~30 % throttle (vs Iris ~60 %) — T:W ~10 means it
  takes way less throttle to suspend mass.
- Roll-rate test: max roll-stick should produce ~800-1000 deg/s. Iris
  caps around 200-400.
- Climb rate: full throttle should produce ≥ 15 m/s climb.

If the quad rockets straight up on first arm or yaw-flips on first
stick input, the thrust/torque coefficients in
`integrations/configs/airframes.py` are wrong for this BF tune. Scale
`thrust_coeff` / `torque_coeff` in the `RACER_5IN` profile down + retest.

## Known issues

- **Iris-tuned BF PIDs may be wildly wrong on race-quad dynamics.**
  Default `bf_config/per_drone/example_manifest.txt` doesn't override
  PIDs — BF SITL boots with stock defaults which are tuned for a
  generic medium quad. Expect oscillation on hover; tune in BF
  Configurator if needed.
- **No race-quad-specific defaults.txt.** If you find a stable PID
  set, save it as `bf_config/per_drone/sim_racer5.txt` and reference
  it in the orchestrator's `start.sh` for repeatability.
