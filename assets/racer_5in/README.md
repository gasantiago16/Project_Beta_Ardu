# 5-inch FPV Race-Quad Asset

Replaces Pegasus's bundled Iris (F450-class, slow, T:W ~2) with a
realistic 5" race-quad airframe (~500 g AUW, T:W ~7, race rates).

## Files

| File | Purpose |
|---|---|
| `racer_5in.urdf` | Source-of-truth structure: links, joints, inertia. Edit here. |
| `racer_5in.usd`  | Generated from the URDF via Isaac Sim's importer. **Not version-controlled** — regenerate locally. |

## One-time URDF → USD conversion

Pegasus loads USD assets, not URDFs directly. Isaac Sim's
`omni.importer.urdf` extension handles the conversion. Run it once after
cloning the repo or after editing the URDF; the output `.usd` is
gitignored.

### Option A — Isaac Sim GUI (easiest)

1. Open Isaac Sim.
2. `Isaac Examples → URDF Importer` (or `File → Import` → URDF).
3. Source: `~/Project_Beta_Ardu/assets/racer_5in/racer_5in.urdf`.
4. Output: `~/Project_Beta_Ardu/assets/racer_5in/racer_5in.usd`.
5. Settings:
   - Merge Fixed Joints: **OFF** (we need joint0..joint3 exposed)
   - Fix Base: **OFF** (free-flying robot)
   - Self Collision: **OFF**
   - Convex Decomposition: ON for collision meshes
   - Joint Drive Type: position (rotor visual spin only)
6. Import. Save the resulting USD.

### Option B — Python (scriptable, repeatable)

```python
# Run from inside an Isaac Sim Python environment.
from isaacsim import SimulationApp
sim = SimulationApp({"headless": True})

from omni.importer.urdf import _urdf
import omni.kit.commands

cfg = _urdf.ImportConfig()
cfg.merge_fixed_joints = False
cfg.fix_base = False
cfg.self_collision = False

omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path="/path/to/racer_5in.urdf",
    import_config=cfg,
    dest_path="/path/to/racer_5in.usd",
)

sim.close()
```

## Validation after conversion

Open the USD in Isaac Sim. Confirm:

- The articulation root prim is `/racer_5in` (or whatever name you used).
- Four rotor prims `/rotor0`, `/rotor1`, `/rotor2`, `/rotor3` exist.
- Four joints `joint0` … `joint3` are continuous (revolute).
- Body mass is 0.5 kg + 4 × 0.02 = 0.58 kg (close enough to the real
  ~500 g target — the rotors carry their own mass).
- Inertia diagonals match what's in the URDF (use Property panel).

## Flight-dynamics parameters (NOT in the URDF)

Pegasus's thrust + torque models live in `MultirotorConfig`, not the
URDF. The URDF only supplies the rigid-body skeleton.

The race-quad-tuned values ship in
`integrations/configs/airframes.py` as `RACER_5IN`:

| Parameter | URDF | Pegasus config | Reason |
|---|---|---|---|
| Mass + inertia | ✓ | (read from USD) | rigid-body |
| Rotor positions | ✓ (joint origins) | — | Pegasus reads from USD |
| Thrust coefficient `c_T` | — | ✓ | dynamics: T = c_T·ω² |
| Torque coefficient `c_Q` | — | ✓ | counter-torque |
| Motor time constant | — | ✓ | first-order rotor lag |
| Max ω | — | (in BetaflightBackendConfig) | PWM normalization |

## Reference values (5" 6S race quad, T-Motor F40 Pro IV-class)

```
mass:                 0.50 kg
arm_length:           0.125 m   (motor-to-center)
motor_pos (X-config): (±0.0884, ±0.0884, 0)
I_xx = I_yy:          0.005 kg·m²
I_zz:                 0.008 kg·m²
c_T (thrust):         6e-7 N/(rad/s)²
c_Q (torque):         7.4e-6 N·m/(rad/s)²
motor_time_const:     0.025 s
max_omega:            ~3000 rad/s   (≈28 700 RPM at 6S × 2300 KV idle)
```

These are flight-validated reference points, not hardware-specific. For a
specific build, replace `c_T` / `c_Q` / `max_omega` with bench-measured
values (thrust stand + voltmeter at full throttle).

## Why we don't track the .usd

USD is binary, regenerated from URDF, large (a few MB), and any change to
the URDF should produce a fresh USD anyway. The `.gitignore` excludes
`*.usd` under this directory; commit only the `.urdf` source.
