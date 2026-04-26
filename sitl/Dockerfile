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

FROM debian:bookworm-slim AS runtime

RUN apt-get update && apt-get install -y --no-install-recommends \
        netcat-openbsd ca-certificates socat \
    && rm -rf /var/lib/apt/lists/*
# socat: in-container UDP relay. Windows Defender Firewall blocks inbound
# UDP on a range of ports — 9002, 9012, and (after a few binds) 19012
# all got blocked, while 9050/9077/19077/19500/28000/35000 stayed open.
# Without admin we can't whitelist. start.sh tells BF to send motor
# packets to 127.0.0.1:9002 (container loopback), and socat forwards
# every packet to host.docker.internal:19500 — high enough to dodge
# the heuristic + a port nothing else commonly uses.

WORKDIR /opt/sitl
COPY --from=build /opt/betaflight/obj/main/betaflight_SITL.elf ./betaflight_SITL
COPY defaults.txt ./defaults.txt
COPY start.sh ./start.sh
RUN chmod +x ./start.sh ./betaflight_SITL

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
