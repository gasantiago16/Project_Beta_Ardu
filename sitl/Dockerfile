# Betaflight SITL container for Project_Beta_Ardu testing.
#
# Builds the BF SITL target, applies our Phase 0 + Phase 1 config defaults
# (GPS Rescue + msp_override_channels_mask=15), and exposes MSP on TCP 5761.
#
# Usage:
#   docker compose -f sitl/docker-compose.yml up
# Then point companion at tcp://localhost:5761 (the MspClient now accepts
# tcp:// scheme).
#
# Pinned to a known-good BF tag. Bump BETAFLIGHT_REF when re-validating
# against a newer BF.

FROM debian:bookworm-slim AS build

ARG BETAFLIGHT_REF=4.5.1

RUN apt-get update && apt-get install -y --no-install-recommends \
        git make gcc g++ libc6-dev ruby ca-certificates \
        gcc-arm-none-eabi \
    && rm -rf /var/lib/apt/lists/*
# BF's mk/tools.mk performs an `arm-none-eabi-gcc --version` pre-check even
# on the SITL target (which actually compiles with host gcc). Without
# gcc-arm-none-eabi, the makefile aborts before deciding the SITL target
# doesn't need ARM. The package is ~200 MB of toolchain we never use; the
# alternative `make arm_sdk_install` downloads ~50 MB of binaries from BF's
# CDN — slower + adds a network dep at build time.

WORKDIR /opt
RUN git clone --depth 1 --branch ${BETAFLIGHT_REF} https://github.com/betaflight/betaflight.git

WORKDIR /opt/betaflight
# SITL build. We override GCC_REQUIRED_VERSION so the makefile's pre-check
# accepts whatever arm-none-eabi-gcc Debian shipped (the SITL target uses
# host gcc, not ARM gcc, so the actual version doesn't matter — just the
# pre-check). Capture the installed version dynamically; if the env var is
# unset BF rejects 12.2.1 because it expects 10.3.1.
# If your BF version renamed the target, edit the line below
# (historical names: SITL, sitl, SITL_X86_64).
RUN GCC_VER=$(arm-none-eabi-gcc -dumpversion) && \
    echo "Using arm-none-eabi-gcc ${GCC_VER} (host gcc does the actual SITL compile)" && \
    make TARGET=SITL GCC_REQUIRED_VERSION="${GCC_VER}" || \
    (echo "SITL build failed — see Dockerfile comments" && exit 1)

# ── eeprom_bake: pre-apply defaults.txt at build time so the runtime
# container never has to do the save+respawn dance.
#
# Why: at runtime, applying defaults via CLI requires `save`, which
# triggers BF's systemReset() → exit(0). start.sh would then respawn
# BF Phase 2, but the second BF instance silently fails to re-init the
# SITL UDP server (`[SITL] start UDP server @9003` never logs). With
# UDP dead, BF doesn't receive FDM, sim time doesn't advance,
# BOOT_GRACE_TIME never clears, motors stuck at 0 — verified live via
# `verify_sim_arms` returning `max_motor_norm=0.000`.
#
# Fix: do the apply+save here, in a build stage. The eeprom.bin that
# falls out captures the saved config. Runtime then single-spawns BF
# with the baked eeprom — no save, no reset, UDP server alive.
FROM debian:bookworm-slim AS eeprom_bake

RUN apt-get update && apt-get install -y --no-install-recommends \
        netcat-openbsd ca-certificates \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/sitl
COPY --from=build /opt/betaflight/obj/main/betaflight_SITL.elf ./betaflight_SITL
COPY defaults.txt ./defaults.txt
RUN chmod +x ./betaflight_SITL

# Spawn BF, drip defaults+save over CLI TCP, wait for systemReset
# exit, verify eeprom.bin came out. argv[1]=127.0.0.1 means BF
# motor packets just go to /dev/null inside this build container —
# we don't care about motors here.
RUN set -e ; \
    ./betaflight_SITL 127.0.0.1 > /tmp/sitl.log 2>&1 & \
    SITL_PID=$! ; \
    echo "[bake] spawned BF (pid=$SITL_PID); waiting for MSP..." ; \
    for i in $(seq 1 30); do \
      nc -z localhost 5761 && break ; \
      sleep 1 ; \
    done ; \
    if ! nc -z localhost 5761 ; then \
      echo "[bake] FATAL: MSP never came up" ; \
      cat /tmp/sitl.log ; \
      exit 1 ; \
    fi ; \
    echo "[bake] applying defaults.txt + save..." ; \
    { \
      printf '\n#\n' ; sleep 0.5 ; \
      while IFS= read -r line || [ -n "$line" ]; do \
        printf '%s\n' "$line" ; sleep 0.05 ; \
      done < defaults.txt ; \
      printf 'save\n' ; sleep 1.5 ; \
    } | nc -q 3 localhost 5761 || true ; \
    echo "[bake] waiting for BF to exit (systemReset)..." ; \
    wait $SITL_PID 2>/dev/null || true ; \
    if [ ! -f eeprom.bin ] ; then \
      echo "[bake] FATAL: eeprom.bin not produced" ; \
      cat /tmp/sitl.log ; \
      exit 1 ; \
    fi ; \
    echo "[bake] eeprom.bin baked: $(stat -c%s eeprom.bin) bytes"

FROM debian:bookworm-slim AS runtime

RUN apt-get update && apt-get install -y --no-install-recommends \
        netcat-openbsd ca-certificates socat python3-minimal \
    && rm -rf /var/lib/apt/lists/*
# python3-minimal: powers `motor_relay.py`, a single-process UDP relay
# from BF's hardcoded 127.0.0.1:9002 output to host:38500. socat is kept
# only for ad-hoc debugging — its fork-per-packet model adds 50-200 ms
# of latency that throttles Pegasus's physics loop down to ~2 Hz.
# Windows Defender Firewall blocks inbound UDP on a range of ports
# (9002, 9012 verified blocked; 38500/28000/35000 verified open) and we
# can't whitelist without admin, so the relay always runs in the default
# Windows topology. Linux/Mac users skip it via SITL_USE_MOTOR_RELAY=0.

WORKDIR /opt/sitl
COPY --from=build /opt/betaflight/obj/main/betaflight_SITL.elf ./betaflight_SITL
COPY --from=eeprom_bake /opt/sitl/eeprom.bin ./eeprom.bin
COPY defaults.txt ./defaults.txt
COPY start.sh ./start.sh
COPY motor_relay.py ./motor_relay.py
RUN chmod +x ./start.sh ./betaflight_SITL ./motor_relay.py
# Pre-create the .config_applied marker so start.sh skips its
# defaults-apply path (we already did that work in the eeprom_bake
# stage). BF runtime spawns once with the baked eeprom.bin, no
# CLI save → no systemReset → SITL UDP server stays initialized
# and FDM packets are received.
RUN touch .config_applied

# BF SITL ports (per BF 4.5.1 src/main/target/SITL/sitl.c):
#   5761 — MSP / CLI TCP
#   9001 — PORT_PWM_RAW UDP (BF → sim, raw PWM output, unused by us)
#   9002 — PORT_PWM     UDP (BF → sim, normalized motor output 0..1)
#   9003 — PORT_STATE   UDP (sim → BF, FDM sensor input)
#   9004 — PORT_RC      UDP (sim → BF, pilot RC injection — Phase 4)
# BF sends to (simulator_ip, 9002); simulator_ip is argv[1] to the SITL
# binary, default "127.0.0.1". For Docker we pass `host.docker.internal`
# (or override via the SITL_SIM_HOST env var) in start.sh so motor packets
# reach the host process across the container boundary.
EXPOSE 5761 9003 9004
# 9002 is intentionally NOT exposed: BF's motor packets stay on container
# loopback in the default (relay) topology — see start.sh + docker-compose.yml.

CMD ["./start.sh"]
