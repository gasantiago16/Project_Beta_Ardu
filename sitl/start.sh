#!/usr/bin/env bash
# Boot Betaflight SITL, then apply our defaults via the CLI TCP port.
#
# After applying defaults we read the config back and assert on the values
# the companion's safety story depends on (msp_override_channels_mask=15,
# mag_hardware=NONE). The original `nc | echo WARNING` path was misleading:
# a partial config apply would still let the smoke test "pass" while
# overrides on AUX channels were unbounded. Hard fail instead.
set -e

echo "[sitl] Starting Betaflight SITL..."
# BF SITL takes the simulator host as argv[1] — that's the destination for
# its motor packets (PORT_PWM = 9002). Default 127.0.0.1 only works when
# the bridge runs inside the container. From the host (Pegasus / fake_pegasus
# _loop), motor packets must be addressed to the host bridge gateway. Docker
# Desktop exposes that as `host.docker.internal`; on plain-Linux Docker we
# rely on the docker-compose `extra_hosts: host-gateway` mapping for the
# same name. SITL_SIM_HOST overrides the default.
SIM_HOST="${SITL_SIM_HOST:-host.docker.internal}"
echo "[sitl] BF will send motor packets to ${SIM_HOST}:9002"
./betaflight_SITL "${SIM_HOST}" &
SITL_PID=$!

cleanup() {
  echo "[sitl] Shutting down (pid=$SITL_PID)"
  kill -TERM "$SITL_PID" 2>/dev/null || true
  wait "$SITL_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# Wait for MSP TCP port to be ready.
echo "[sitl] Waiting for MSP on :5761..."
for i in $(seq 1 30); do
  if nc -z localhost 5761; then
    echo "[sitl] MSP up after ${i}s"
    break
  fi
  sleep 1
done

# Apply defaults.txt over CLI TCP. BF SITL accepts CLI commands on the same
# port that serves MSP; an `#` byte enters CLI mode in some BF versions, but
# the standard path is via Configurator. For automated boot we just send
# the lines verbatim — works on 4.5.x.
if [ -f defaults.txt ]; then
  echo "[sitl] Applying defaults.txt..."
  if ! nc -q 2 localhost 5761 < defaults.txt; then
    echo "[sitl] FATAL: defaults application failed (nc exit non-zero)"
    exit 1
  fi
fi

# Read back the running config and verify the safety-critical lines.
# If the config didn't apply (silent FC, wrong baud, partial parse), MSP
# overrides could engage on AUX channels — pilot can't take the sticks back.
# Hard-fail rather than continue with a misleading "ready" log.
echo "[sitl] Verifying applied defaults..."
DUMP=$(printf 'dump\nexit\n' | nc -q 3 localhost 5761 || true)

assert_setting() {
  local key="$1"
  local expected="$2"
  if ! echo "$DUMP" | grep -qE "^[[:space:]]*${key}[[:space:]]*=[[:space:]]*${expected}[[:space:]]*$"; then
    echo "[sitl] FATAL: expected '${key} = ${expected}' not present in dump"
    echo "[sitl] grep result for ${key}:"
    echo "$DUMP" | grep -E "${key}" || echo "  (no match — setting absent)"
    exit 1
  fi
  echo "[sitl] OK: ${key} = ${expected}"
}

assert_setting "msp_override_channels_mask" "15"
assert_setting "mag_hardware" "NONE"

echo "[sitl] Ready. SITL pid=$SITL_PID"
wait "$SITL_PID"
