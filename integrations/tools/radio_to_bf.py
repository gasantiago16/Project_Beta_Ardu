"""Radiomaster Pocket → pygame → UDP 9004 → BF SITL pilot RC injection.

The companion's MSP_SET_RAW_RC overrides only channels 1-4 (`msp_override
_channels_mask=15`); channels 5-8 (mode, AUX-companion-active, etc.) come
from the pilot's radio. In real hardware the pilot's ELRS RX wires into
BF's UART. In sim, BF SITL listens on UDP 9004 (`PORT_RC` per
betaflight/4.5.1 src/main/target/SITL/sitl.c).

Wire format (verified against `target.h`):
    typedef struct {
        double   timestamp;             // 8 B, host-native double
        uint16_t channels[16];          // 32 B, raw PWM µs, 1000..2000
    } rc_packet;                        // 40 B total
BF does a STRICT `n == sizeof(rc_packet)` check and silently drops any
packet of the wrong size — so the format string MUST be `<d16H`. This
script reads up to 8 axes from the joystick and pads channels 9-16 with
1500 µs (centered) before packing.

Usage:
    cd ~/Project_Beta_Ardu
    python -m integrations.tools.radio_to_bf                 # default Radiomaster Pocket map
    python -m integrations.tools.radio_to_bf --dry-run       # print PWM, don't send (calibration)
    python -m integrations.tools.radio_to_bf --sweep         # synthetic sine sweep — no radio needed
    python -m integrations.tools.radio_to_bf --bf-host 192.168.1.5  # remote SITL

Channel mapping is in `integrations/configs/radiomaster_pocket.json`.
Edit that file if your radio's USB joystick output differs from the
AETR default (axis 0=roll, 1=pitch, 2=throttle, 3=yaw, 4-7=AUX1-4).

Architecture note: this is independent of racer_companion. The companion
talks MSP TCP 5761; this talks UDP 9004. They coexist without conflict
because BF arbitrates per `msp_override_channels_mask`: companion wins
on 1-4 when active, pilot keeps 5-8 either way (and gets 1-4 back when
the companion stops sending MSP).
"""
from __future__ import annotations

import argparse
import json
import math
import socket
import struct
import sys
import time
from dataclasses import dataclass
from pathlib import Path

# BF SITL `rc_packet`: 1 double (timestamp) + 16 uint16 channels, raw PWM µs.
# Layout from betaflight/4.5.1 src/main/target/SITL/target.h. BF does strict
# n==sizeof(rc_packet) match; wrong size = silent drop.
UDP_RC_PACKET = struct.Struct("<d16H")
assert UDP_RC_PACKET.size == 40, f"rc_packet must be 40B, got {UDP_RC_PACKET.size}"

NUM_RC_CHANNELS = 16

# RC PWM band. BF clips outside [1000, 2000] but it's nicer to clamp on our side.
PWM_MIN = 1000
PWM_MAX = 2000
PWM_MID = 1500


@dataclass
class ChannelMap:
    """Mapping from a pygame joystick axis to a BF RC channel (1-8)."""
    name: str
    axis: int
    invert: bool = False
    deadband: float = 0.0


def axis_to_pwm(value: float, invert: bool = False, deadband: float = 0.0) -> int:
    """Map a pygame axis value in [-1, +1] to a PWM µs in [1000, 2000].

    `invert=True` flips the sign before mapping (useful when EdgeTX reports
    stick-up as -1 for pitch). `deadband` zeroes any |value| below the
    threshold so a noisy stick centered between sticks reads exactly 1500
    rather than 1499/1501 jitter.
    """
    if not math.isfinite(value):
        # NaN / inf from a disconnected axis: center, never garbage.
        return PWM_MID
    if invert:
        value = -value
    if abs(value) < deadband:
        value = 0.0
    pwm = int(round(PWM_MID + max(-1.0, min(1.0, value)) * 500))
    return max(PWM_MIN, min(PWM_MAX, pwm))


def load_mapping(path: Path) -> list[ChannelMap]:
    """Load a channel-mapping JSON. Returns the list in declared order;
    callers are responsible for slicing to the first 8."""
    raw = json.loads(path.read_text(encoding="utf-8"))
    return [ChannelMap(
        name=c["name"],
        axis=c["axis"],
        invert=bool(c.get("invert", False)),
        deadband=float(c.get("deadband", 0.0)),
    ) for c in raw["channels"]]


def channels_from_axes(
    axis_values: list[float], mapping: list[ChannelMap],
    aux_overrides: dict[int, int] | None = None,
) -> list[int]:
    """Compute the 16-channel PWM vector for BF's `rc_packet`. Channels
    beyond the mapping (and beyond what the joystick exposes) default to
    1500 µs. `aux_overrides` is a {channel_index_0_based: pwm_us} dict
    that wins over joystick-derived values — useful for `--aux` test
    flags. Pure function — easy to test without a real joystick."""
    channels = [PWM_MID] * NUM_RC_CHANNELS
    for i, c in enumerate(mapping[:NUM_RC_CHANNELS]):
        if 0 <= c.axis < len(axis_values):
            channels[i] = axis_to_pwm(
                axis_values[c.axis], invert=c.invert, deadband=c.deadband,
            )
    if aux_overrides:
        for ch_idx, pwm in aux_overrides.items():
            if 0 <= ch_idx < NUM_RC_CHANNELS:
                channels[ch_idx] = max(PWM_MIN, min(PWM_MAX, int(pwm)))
    return channels


def sweep_axis_value(t: float, channel_idx: int, period_s: float = 4.0) -> float:
    """Synthetic sine sweep for `--sweep` mode. Each channel has a unique
    phase so visualizing the sweep makes it obvious which is which."""
    phase = channel_idx * (math.pi / 4)  # 45° between channels
    return math.sin(2 * math.pi * t / period_s + phase) * 0.5  # ±0.5 → 1250..1750


def _print_packet(channels: list[int], names: list[str], n_show: int = 8) -> None:
    """Print first `n_show` channels (default 8 — channels 9-16 are usually
    pad). Pass n_show=16 to see everything."""
    cells = []
    for i in range(min(n_show, len(channels))):
        name = names[i] if i < len(names) else f"ch{i+1}"
        cells.append(f"{name[:5]:>5}={channels[i]:>4}")
    print("  RC " + " ".join(cells), flush=True)


def _parse_aux_override(spec: str) -> tuple[int, int]:
    """Parse a `--aux` flag like `7=1900` into (channel_index_0_based, pwm).
    Channel numbers in the flag are 1-based to match BF/MSP convention."""
    try:
        ch_str, pwm_str = spec.split("=", 1)
        ch = int(ch_str)
        pwm = int(pwm_str)
    except ValueError as e:
        raise argparse.ArgumentTypeError(
            f"--aux must be CHANNEL=PWM (1-based, e.g. 7=1900); got {spec!r}: {e}"
        )
    if not (1 <= ch <= NUM_RC_CHANNELS):
        raise argparse.ArgumentTypeError(
            f"--aux channel must be 1..{NUM_RC_CHANNELS}; got {ch}"
        )
    if not (PWM_MIN <= pwm <= PWM_MAX):
        raise argparse.ArgumentTypeError(
            f"--aux PWM must be {PWM_MIN}..{PWM_MAX}; got {pwm}"
        )
    return (ch - 1, pwm)


def _msp_preflight(host: str, port: int = 5761, timeout: float = 1.0) -> bool:
    """Best-effort TCP probe of BF MSP. True if reachable, False otherwise.
    Doesn't kill the script — operator may want to start radio_to_bf before
    SITL for setup convenience; we only warn."""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(timeout)
    try:
        s.connect((host, port))
        return True
    except (ConnectionRefusedError, socket.timeout, OSError):
        return False
    finally:
        s.close()


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--bf-host", default="127.0.0.1",
                   help="BF SITL host (default 127.0.0.1).")
    p.add_argument("--rc-port", type=int, default=9004,
                   help="BF SITL PORT_RC (default 9004).")
    p.add_argument("--rate-hz", type=float, default=50.0,
                   help="UDP transmit rate (default 50 Hz, BF default RC frame rate).")
    p.add_argument("--config", type=Path,
                   default=Path(__file__).resolve().parents[1] /
                           "configs" / "radiomaster_pocket.json",
                   help="Channel mapping JSON.")
    p.add_argument("--joystick-index", type=int, default=0,
                   help="pygame joystick index (default 0). List with --list-joysticks.")
    p.add_argument("--list-joysticks", action="store_true",
                   help="Print connected joysticks and exit.")
    p.add_argument("--dry-run", action="store_true",
                   help="Print PWM values to stdout; don't send UDP.")
    p.add_argument("--sweep", action="store_true",
                   help="Synthetic sine sweep on every channel; no radio required.")
    p.add_argument("--print-every", type=int, default=25,
                   help="Print every N packets (default 25 = 0.5 s @ 50 Hz; "
                        "0 disables periodic print).")
    p.add_argument("--aux", action="append", type=_parse_aux_override,
                   default=[], metavar="CH=PWM",
                   help="Override a channel after joystick mapping. Useful "
                        "for testing companion-takeover without a radio: "
                        "`--aux 7=1900` forces AUX3 high (assuming aux3 is "
                        "channel 7, the companion-active toggle).")
    p.add_argument("--no-preflight", action="store_true",
                   help="Skip the BF MSP TCP-5761 reachability warning.")
    args = p.parse_args()
    aux_overrides = dict(args.aux) if args.aux else None

    # Load channel map (always — even sweep mode uses it for pretty-print names).
    if not args.config.exists():
        print(f"[radio_to_bf] FATAL: config not found at {args.config}",
              file=sys.stderr)
        return 2
    try:
        mapping = load_mapping(args.config)
    except (KeyError, ValueError, json.JSONDecodeError) as e:
        print(f"[radio_to_bf] FATAL: failed to load mapping: {e}", file=sys.stderr)
        return 2
    names = [c.name for c in mapping[:8]]
    while len(names) < 8:
        names.append(f"ch{len(names)+1}")

    # pygame imports + joystick setup are skipped in --sweep mode so the
    # script runs without any HID device or display attached.
    js = None
    if not args.sweep:
        try:
            import pygame  # noqa: F401
        except ImportError:
            print("[radio_to_bf] FATAL: pygame not installed. "
                  "`pip install pygame`. Or use --sweep to test without a radio.",
                  file=sys.stderr)
            return 2

        pygame.init()
        pygame.joystick.init()

        if args.list_joysticks:
            n = pygame.joystick.get_count()
            print(f"[radio_to_bf] {n} joystick(s) detected:")
            for i in range(n):
                j = pygame.joystick.Joystick(i)
                j.init()
                print(f"  [{i}] '{j.get_name()}' "
                      f"(axes={j.get_numaxes()}, buttons={j.get_numbuttons()}, "
                      f"hats={j.get_numhats()})")
                j.quit()
            pygame.quit()
            return 0

        if pygame.joystick.get_count() == 0:
            print("[radio_to_bf] FATAL: no joystick detected. "
                  "Plug in the Radiomaster + ensure EdgeTX is in 'USB Joystick (HID)' mode.",
                  file=sys.stderr)
            pygame.quit()
            return 2

        js = pygame.joystick.Joystick(args.joystick_index)
        js.init()
        print(f"[radio_to_bf] Joystick {args.joystick_index}: '{js.get_name()}' "
              f"({js.get_numaxes()} axes, {js.get_numbuttons()} buttons)",
              flush=True)

        max_axis = max((c.axis for c in mapping), default=-1)
        if js.get_numaxes() <= max_axis:
            print(f"[radio_to_bf] FATAL: joystick has {js.get_numaxes()} axes "
                  f"but config references axis {max_axis}.",
                  file=sys.stderr)
            pygame.quit()
            return 2

    if not args.no_preflight and not args.dry_run:
        if _msp_preflight(args.bf_host):
            print(f"[radio_to_bf] Pre-flight: BF MSP reachable at "
                  f"{args.bf_host}:5761.", flush=True)
        else:
            print(f"[radio_to_bf] WARNING: BF MSP not reachable at "
                  f"{args.bf_host}:5761. RC packets will go to a void until "
                  f"`docker compose -f sitl/docker-compose.yml up`.",
                  file=sys.stderr)

    sock = None
    if not args.dry_run:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # `connect()` lets us use send() instead of sendto() and surfaces
        # ICMP port-unreachable as ConnectionRefusedError on Windows
        # (instead of silently dropping). Best-effort.
        try:
            sock.connect((args.bf_host, args.rc_port))
        except OSError:
            pass

    print(f"[radio_to_bf] Sending UDP RC to {args.bf_host}:{args.rc_port} "
          f"@ {args.rate_hz:.0f} Hz "
          f"({'dry-run, no UDP' if args.dry_run else 'live'}, "
          f"{'sweep' if args.sweep else 'joystick'}"
          f"{f', AUX overrides {aux_overrides}' if aux_overrides else ''})",
          flush=True)

    period_s = 1.0 / max(0.5, args.rate_hz)
    n_packets = 0
    t_start = time.monotonic()

    try:
        while True:
            tick_start = time.monotonic()
            if args.sweep:
                t = tick_start - t_start
                axis_values = [sweep_axis_value(t, i) for i in range(8)]
            else:
                import pygame
                pygame.event.pump()
                axis_values = [
                    js.get_axis(i) for i in range(js.get_numaxes())
                ]
            channels = channels_from_axes(axis_values, mapping, aux_overrides)
            packet = UDP_RC_PACKET.pack(tick_start, *channels)

            if not args.dry_run:
                try:
                    sock.send(packet)
                except OSError as e:
                    # Don't kill the loop on a transient send failure.
                    if n_packets % 50 == 0:
                        print(f"[radio_to_bf] send failed: {e}",
                              file=sys.stderr)

            if args.print_every > 0 and n_packets % args.print_every == 0:
                _print_packet(channels, names)
            n_packets += 1

            elapsed = time.monotonic() - tick_start
            if elapsed < period_s:
                time.sleep(period_s - elapsed)
    except KeyboardInterrupt:
        print(f"\n[radio_to_bf] Stopped after {n_packets} packets "
              f"({(time.monotonic() - t_start):.1f}s).",
              flush=True)
    finally:
        if sock is not None:
            sock.close()
        if not args.sweep:
            try:
                import pygame
                pygame.quit()
            except Exception:
                pass

    return 0


if __name__ == "__main__":
    sys.exit(main())
