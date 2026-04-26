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

# Apply defaults.txt over CLI TCP. Two BF SITL gotchas we hit the hard way:
#   1. CLI mode is entered when BF receives `#` as the first byte of a
#      line. Our defaults.txt starts with `# Betaflight SITL...` — BF
#      reads the `#`, enters CLI, then tries to execute the rest of that
#      line as a command, which fails. Fix: prepend `\n#\n` for a clean
#      CLI entry before defaults.txt content.
#   2. Sending all lines in one TCP write (the original `nc -q 2 < file`
#      approach) blasts BF with the entire buffer at once. BF accepts
#      the first line or two and silently drops the rest — only the
#      banner-and-prompt comes back, no per-command echoes. Fix: drip
#      the file one line at a time with a short sleep so BF's CLI parser
#      can keep up.
#
# A tighter implementation would use Python or a CLI tool that waits for
# each prompt; this shell version is a reasonable compromise that uses
# only the netcat already in the container.
if [ -f defaults.txt ]; then
  echo "[sitl] Applying defaults.txt..."
  if ! {
        printf '\n#\n'
        sleep 0.5
        while IFS= read -r line || [ -n "$line" ]; do
          printf '%s\n' "$line"
          sleep 0.05
        done < defaults.txt
        printf 'save\n'
        sleep 1.5
      } | nc -q 3 localhost 5761; then
    echo "[sitl] FATAL: defaults application failed (nc exit non-zero)"
    exit 1
  fi
fi

# Read back the running config and verify the safety-critical lines.
# Same `\n#\n` discipline so the verify connection enters CLI cleanly.
# If the config didn't apply (silent FC, wrong baud, partial parse), MSP
# overrides could engage on AUX channels — pilot can't take the sticks back.
# Hard-fail rather than continue with a misleading "ready" log.
echo "[sitl] Verifying applied defaults..."
DUMP=$(printf '\n#\ndump\nexit\n' | nc -q 3 localhost 5761 || true)

assert_setting() {
  local key="$1"
  local expected="$2"
  # BF's `dump` command emits each line as `set <key> = <value>` (with the
  # `set ` prefix). The previous regex assumed `<key> = <value>` — bare,
  # no prefix — and matched nothing on a real BF SITL dump. Accept either.
  if ! echo "$DUMP" | grep -qE "^[[:space:]]*(set[[:space:]]+)?${key}[[:space:]]*=[[:space:]]*${expected}[[:space:]]*$"; then
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
