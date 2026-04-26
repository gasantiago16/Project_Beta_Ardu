#!/usr/bin/env bash
# Boot Betaflight SITL, then apply our defaults via the CLI TCP port.
#
# Single-phase boot: spawn BF, drip CLI commands, NEVER call `save`. Config
# is applied in-memory and lives for the lifetime of the process. The
# container is ephemeral (compose `restart: unless-stopped` re-runs this
# whole script), so flash persistence isn't needed for sim.
#
# Why not save+respawn? Tested on BF 4.5.1: `save` triggers systemReset()
# → exit(0), and the second BF instance silently fails to re-init the
# SITL UDP server (no `[SITL] start UDP server @9003` print, no FDM
# packets received from the bridge). Whatever the second-spawn path
# touches in eeprom.bin, the SITL UDP threads don't come back. Single-
# phase boot avoids the whole problem.
set -e

RAW_SIM_HOST="${SITL_SIM_HOST:-host.docker.internal}"
HOST_MOTOR_PORT="${SITL_HOST_MOTOR_PORT:-19500}"
USE_RELAY="${SITL_USE_MOTOR_RELAY:-1}"

# BF SITL parses argv[1] with inet_addr() — IPv4-dotted only. On Docker
# Desktop the /etc/hosts entry "host.docker.internal" lists BOTH the IPv6
# fdc4:... AND IPv4 192.168.65.254 addresses; the default getent path
# returns the IPv6, BF inet_addr() rejects it (INADDR_NONE), and motor
# packets silently sendto 255.255.255.255 → never arrive at the bridge.
# Pre-resolve to IPv4 here so the relay socat targets an address Linux
# IPv4 routing can use directly.
if [[ "$RAW_SIM_HOST" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
  HOST_IPV4="$RAW_SIM_HOST"
else
  HOST_IPV4=$(getent ahostsv4 "$RAW_SIM_HOST" 2>/dev/null | awk 'NR==1 {print $1}')
  if [ -z "$HOST_IPV4" ]; then
    echo "[sitl] FATAL: could not resolve $RAW_SIM_HOST to IPv4"
    exit 1
  fi
  echo "[sitl] Resolved $RAW_SIM_HOST -> $HOST_IPV4 (IPv4)"
fi

SITL_PID=
RELAY_PID=
cleanup() {
  if [ -n "$RELAY_PID" ]; then
    kill -TERM "$RELAY_PID" 2>/dev/null || true
  fi
  if [ -n "$SITL_PID" ]; then
    echo "[sitl] Shutting down BF (pid=$SITL_PID)"
    kill -TERM "$SITL_PID" 2>/dev/null || true
    wait "$SITL_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

# Two motor-path topologies, depending on SITL_USE_MOTOR_RELAY.
#
# Direct (Linux/Mac): BF sends motors straight to ${HOST_IPV4}:9002.
#   Requires the host bridge to bind UDP 9002 AND for nothing on the host
#   to block UDP 9002 inbound. Set SITL_USE_MOTOR_RELAY=0 to use this.
#
# Relayed (Windows, default): BF sends to 127.0.0.1:9002 inside the
#   container, where socat catches each datagram and forwards to
#   ${HOST_IPV4}:${HOST_MOTOR_PORT}. Why:
#     - Windows Defender Firewall on the dev host empirically blocks
#       inbound UDP on a heuristic port range that includes 9002 and
#       9012, while 9050/9077/19500/28000 stay open. No admin = no
#       whitelist.
#     - BF SITL's PORT_PWM is hardcoded to 9002 (sitl.c #define), so
#       redirecting at the BF layer isn't an option — only relay works.
if [ "$USE_RELAY" = "1" ]; then
  BF_SIM_ARG="127.0.0.1"
  echo "[sitl] BF -> 127.0.0.1:9002 (container loopback) -> socat -> ${HOST_IPV4}:${HOST_MOTOR_PORT} (host bridge)"
  # `fork` is required: UDP4-RECVFROM without fork only handles the
  # FIRST datagram (verified — drop fork and rx falls from ~50% to 0.1%).
  # Each datagram spawns a child that exits after sendto. ~250 forks/sec
  # at lockstep rate is fine for session-length runs; if you need to
  # support hour-long flights, replace with a single-process relay
  # (e.g. tiny python socket loop) to stay clear of the cgroup PID cap.
  socat -u UDP4-RECVFROM:9002,bind=127.0.0.1,fork,reuseaddr=1 \
        UDP4-SENDTO:${HOST_IPV4}:${HOST_MOTOR_PORT} &
  RELAY_PID=$!
  echo "[sitl] socat motor-relay started (pid=$RELAY_PID)"
  sleep 0.3   # ensure socat bind completes before BF starts sending
else
  BF_SIM_ARG="${HOST_IPV4}"
  echo "[sitl] BF -> ${HOST_IPV4}:9002 (host bridge, direct, no relay)"
  echo "[sitl]   ensure host bridge listens on 9002 and host firewall allows it"
fi

./betaflight_SITL "${BF_SIM_ARG}" &
SITL_PID=$!
echo "[sitl] BF SITL spawned (pid=$SITL_PID); waiting for MSP on :5761..."
for i in $(seq 1 30); do
  if nc -z localhost 5761; then
    echo "[sitl] MSP up after ${i}s"
    break
  fi
  sleep 1
done
if ! nc -z localhost 5761; then
  echo "[sitl] FATAL: MSP didn't come up within 30s"
  exit 1
fi

# Apply defaults.txt over CLI TCP. Three BF SITL gotchas we hit the hard way:
#   1. CLI mode entry is triggered by `#` as the FIRST byte of a line.
#      Our defaults.txt starts with `# Minimum BF SITL config…` — BF reads
#      the `#`, enters CLI, then tries to execute the rest of that line
#      as a command, which fails. Fix: prepend `\n#\n` for a clean entry.
#   2. Sending all lines in one TCP write blasts BF's CLI parser; only
#      the first 1-2 commands actually get processed before the rest is
#      silently dropped. Fix: drip lines with 50 ms sleep between them.
#   3. NEVER send `save` (see top-of-file comment). Use `exit` to leave
#      CLI mode without resetting; the in-memory `set` values stay live
#      for the rest of the process lifetime.
if [ -f defaults.txt ]; then
  echo "[sitl] Applying defaults.txt (in-memory only, no save)..."
  {
    printf '\n#\n'
    sleep 0.5
    while IFS= read -r line || [ -n "$line" ]; do
      printf '%s\n' "$line"
      sleep 0.05
    done < defaults.txt
    # No `save` — would exit BF. Drop the connection instead; the CLI
    # mode persists until BF reaches its idle loop, which is fine for us.
    sleep 0.5
  } | nc -q 1 localhost 5761 || true
fi

# Brief settle window so any final CLI processing completes before the
# bridge starts hammering UDP 9003.
sleep 1

echo "[sitl] Ready. SITL pid=$SITL_PID"
echo "[sitl] Verify from host:"
echo "[sitl]   python -m integrations.tools.fake_pegasus_loop --duration 10  (UDP bridge smoke)"
echo "[sitl]   python -m integrations.tools.discover_bf_gps                  (MSP CLI dump)"
echo "[sitl] NOTE: bridge must listen on host UDP ${HOST_MOTOR_PORT} (not 9002)"

# Wait forever (or until BF exits / signal). docker compose's
# restart: unless-stopped re-runs this script if BF exits unexpectedly.
wait "$SITL_PID"
