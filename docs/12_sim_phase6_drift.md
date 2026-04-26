# Phase 6 — sensor noise + mag-NONE drift

Validates the racer_companion's heading-divergence watchdog (BUG-6 fix
from v0.4) end-to-end in motion. Real flight: mag is disabled
(`mag_hardware = NONE` per the safety doctrine), so attitude estimate
relies on gyro integration alone, which drifts. The watchdog catches
sustained drift before it becomes a runaway.

## How

`BetaflightBackendConfig` now has noise fields (default 0 = noiseless,
matches earlier phases bit-identically):

| Field | Realistic value | What it does |
|---|---|---|
| `gyro_bias_drift_rad_s` | 0.0003 (~1°/min) | Constant per-axis bias added to gyro before sending FDM. Drives heading drift. |
| `gyro_noise_std_rad_s` | 0.005 | White-noise σ on gyro. Realistic MEMS floor. |
| `accel_noise_std_m_s2` | 0.05 | White-noise σ on accelerometer. Mostly cosmetic. |
| `noise_seed` | 42 (or any int) | Pin RNG for reproducible scenarios. |

In the orchestrator, edit `BetaflightBackendConfig(...)` to set these,
or pass them via a future CLI flag.

## Standalone harness — no Isaac required

```bash
docker compose -f sitl/docker-compose.yml up -d
python -m integrations.tools.drive_heading_divergence \
    --gyro-bias 0.0003 --gyro-noise 0.005 --duration 180
```

The harness:
1. Configures the bridge with the given bias + noise.
2. Synthesizes a hovering quad's State at 250 Hz against real BF SITL.
3. Runs the v0.4 `HeadingDivergenceTracker` against the same drift.
4. Reports when (or if) the watchdog trips.

Expected: at 1°/min drift with the watchdog's 60° threshold, trip
should fire around t=180 s. If it never fires, either the bias is too
low, the watchdog gate predicate is broken, or the tracker's window
is set too long.

## Companion-side validation in motion

With Phase 5's race-quad airframe + Phase 4's pilot + bias = 0.0003:

1. Arm and engage AUX-companion-active.
2. Companion enters CLIMB → TRANSIT.
3. After ~2-3 minutes of forward TRANSIT flight, the companion's
   `safety.HeadingDivergenceTracker` should observe `|heading_err|`
   exceed 60° for 3 s (its window) → safety reason
   `heading_diverged` fires → state machine forces RELEASED.
4. BF takes over via the documented MSP-override timeout (~500 ms).

## Known issues

- **Bias-only drift accumulates linearly; real drift is higher-order.**
  This phase models the *minimum* drift case (constant bias only).
  Real MEMS gyros also have temperature, vibration, and integration
  errors. The watchdog is tuned for the worst case; this phase only
  proves the best-case detection works.
- **No accel bias, only noise.** Bias on accel would also cause
  attitude drift. Future phase if we want belt-and-suspenders.
