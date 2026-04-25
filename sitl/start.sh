#!/usr/bin/env bash
# Boot Betaflight SITL, then apply our defaults via the CLI TCP port.
set -e

echo "[sitl] Starting Betaflight SITL..."
./betaflight_SITL &
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
  nc -q 2 localhost 5761 < defaults.txt || \
    echo "[sitl] WARNING: defaults application failed; apply via Configurator CLI tab manually"
fi

echo "[sitl] Ready. SITL pid=$SITL_PID"
wait "$SITL_PID"
