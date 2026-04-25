# Betaflight SITL — desktop test rig

Lets you run the companion against a real Betaflight binary without a flight
controller. Boots a Linux build of BF 4.5.1's SITL target, applies our Phase
0 + Phase 1 config, exposes MSP on TCP 5761.

## Run it

```bash
cd sitl
docker compose up --build
```

First run takes ~5 minutes (BF compile). Subsequent runs are instant.
TCP MSP available on `localhost:5761`. Logs visible via `docker compose logs`.

## Point companion at it

The `MspClient` now accepts a `tcp://` URI. In your config JSON:

```json
{
  "companion": {
    "serial_port": "tcp://localhost:5761",
    ...
  }
}
```

No socat, no pty bridge — the companion talks MSP-over-TCP directly.

## Run the smoke test

```bash
# In a second shell
cd companion
SITL_TCP=localhost:5761 python -m unittest tests.test_sitl_smoke -v
```

The smoke test:
1. Connects via the TCP adapter
2. Requests MSP_API_VERSION + MSP_FC_VARIANT — confirms BF responds
3. Sends MSP_SET_RAW_RC for a few seconds — confirms BF accepts override
4. Reads MSP_RC echo back — confirms override taking effect

If `SITL_TCP` env var is not set, the test is skipped (so it never breaks
local dev or main CI).

## Manual config via Configurator

```
Configurator → Connect → choose "tcp://localhost:5761" (custom server)
                          OR `nc localhost 5761` for raw CLI access
```

## Caveats

- **No physics.** SITL doesn't simulate quad dynamics by default. MSP_RAW_GPS
  returns whatever defaults BF SITL ships with; MSP_ATTITUDE shows level.
  This is enough to validate the protocol layer (override semantics, frame
  parsing, state machine). For closed-loop tuning you need a sim like
  Gazebo wired to the UDP ports — see ROADMAP.md.
- **No RX.** SITL doesn't simulate the RC link. MSP_RC reports whatever the
  test driver writes. This means AUX-active edge detection has to be
  validated by injecting fake MSP_RC frames in the test, not by flipping
  a real switch.
- **Build pinning.** BF tagged versions are stable; we pin to 4.5.1. Bump
  `BETAFLIGHT_REF` in Dockerfile + docker-compose.yml when validating
  against a newer BF.
- **SITL build can drift.** If `make TARGET=SITL` fails after a BF release,
  check the BF GitHub for SITL-related issues — the target name has been
  renamed in the past (`SITL` → `sitl` etc.).

## CI

`.github/workflows/sitl.yml` is a manual-trigger workflow (`workflow_dispatch`)
that builds the SITL container and runs the smoke test. Not on push trigger
yet because:
1. ~5 min build adds a lot to per-push CI time.
2. SITL has historical flakiness on cold containers.

Promote to push trigger once we have 30+ successful manual runs.
