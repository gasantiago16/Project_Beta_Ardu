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
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt
RUN git clone --depth 1 --branch ${BETAFLIGHT_REF} https://github.com/betaflight/betaflight.git

WORKDIR /opt/betaflight
# SITL build. May produce warnings; build itself succeeds on 4.5.x.
# If your BF version renamed the target, edit the line below
# (historical names: SITL, sitl, SITL_X86_64).
RUN make TARGET=SITL || (echo "SITL build failed — see Dockerfile comments" && exit 1)

FROM debian:bookworm-slim AS runtime

RUN apt-get update && apt-get install -y --no-install-recommends \
        netcat-openbsd ca-certificates \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/sitl
COPY --from=build /opt/betaflight/obj/main/betaflight_SITL.elf ./betaflight_SITL
COPY defaults.txt ./defaults.txt
COPY start.sh ./start.sh
RUN chmod +x ./start.sh ./betaflight_SITL

# BF SITL ports (defaults; verify in your BF version):
#   5761 — MSP / CLI TCP
#   9002 — sensor input UDP (sim → BF)
#   9003 — motor output UDP (BF → sim)
EXPOSE 5761 9002 9003

CMD ["./start.sh"]
