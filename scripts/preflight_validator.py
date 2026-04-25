"""Pre-flight Betaflight config validator.

Compare a per-drone manifest (the safety-critical settings we expect) against
a live `dump` from a flight controller (or a saved dump file). Exits non-zero
on any divergence, intended to be run as a pre-flight gate.

Why this exists: SITL boot now reads back its own config (`sitl/start.sh`),
but real-drone deploys had no equivalent — a partial CLI apply or a forgotten
`save` command would leave the drone with `msp_override_channels_mask` unset
and the companion would silently have no effect. This catches that on the
ground.

Manifest format: same as a Betaflight `set` command line, one per line:

    set msp_override_channels_mask = 15
    set mag_hardware = NONE

Comments (`# ...`), blank lines, and non-`set` lines are ignored. Whitespace
around `=` is tolerated.

Dump format: standard Betaflight `dump` output. Parser looks for lines
matching `set <key> = <value>` (case-sensitive on the key; BF is too).

Usage:
    # File-based: SCP the dump from the drone, then validate locally.
    python scripts/preflight_validator.py \\
        --manifest bf_config/per_drone/alex_drone1.txt \\
        --dump      /tmp/alex_drone1_dump.txt

    # Live (planned): connect to FC and read dump over CLI.
    # Not implemented yet; CLI-over-MSP varies per BF version. Use the
    # file-based path: paste `dump` output from the Configurator CLI tab.
"""
from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path

# `set <key> = <value>` — value runs to end-of-line, may contain spaces
# (e.g., for serial config like "1 2 0 0 0 0").
SET_LINE_RE = re.compile(
    r"^\s*set\s+([A-Za-z0-9_]+)\s*=\s*(.+?)\s*$"
)


def parse_settings(text: str) -> dict[str, str]:
    """Extract `{key: value}` from BF CLI text. Last occurrence wins (matches
    `dump` behavior — later `set` overrides earlier).

    Comment handling: a `#` is treated as a comment ONLY when it begins the
    line (after optional whitespace). A `#` inside a value (e.g.
    `set craft_name = jacob#1`) is preserved. The previous "split on first #"
    approach silently corrupted manifest values containing `#`.
    """
    out: dict[str, str] = {}
    for raw in text.splitlines():
        stripped = raw.lstrip()
        if stripped.startswith("#"):
            continue  # whole-line comment
        m = SET_LINE_RE.match(raw)
        if m:
            key, value = m.group(1), m.group(2).strip()
            out[key] = value
    return out


@dataclass
class Mismatch:
    key: str
    expected: str
    actual: str | None  # None = key absent from dump

    def render(self) -> str:
        if self.actual is None:
            return f"  MISSING: {self.key} = {self.expected} (not in dump)"
        return f"  MISMATCH: {self.key}: expected {self.expected!r}, got {self.actual!r}"


def compare(manifest: dict[str, str], dump: dict[str, str]) -> list[Mismatch]:
    """Return mismatches. Empty list means manifest is satisfied by dump."""
    mismatches: list[Mismatch] = []
    for key, expected in manifest.items():
        actual = dump.get(key)
        if actual is None:
            mismatches.append(Mismatch(key=key, expected=expected, actual=None))
        elif _normalize(actual) != _normalize(expected):
            mismatches.append(Mismatch(key=key, expected=expected, actual=actual))
    return mismatches


_ENUM_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")


def _normalize(value: str) -> str:
    """Normalize a setting value for comparison.

    Whitespace is always normalized (BF emits single-spaced lists; manifests
    might have extras). Case is normalized ONLY for enum-looking values that
    use UNIFORM case — BF accepts `none` / `NONE` interchangeably for enum
    settings, but a mixed-case value like `Alex1` in a name field must stay
    case-sensitive (a wrong name that differs only in case would silently
    match otherwise).

    Heuristic: single-token alphanumeric value (matches _ENUM_RE) AND all
    letters in the value have the same case (isupper or islower) → treat
    as enum. Anything else → strict.
    """
    collapsed = " ".join(value.split())
    if _ENUM_RE.match(collapsed) and (collapsed.isupper() or collapsed.islower()):
        return collapsed.upper()
    return collapsed


def main() -> int:
    p = argparse.ArgumentParser(description="Validate BF config against per-drone manifest")
    p.add_argument("--manifest", required=True, type=Path,
                   help="Path to manifest with the `set` lines we require.")
    p.add_argument("--dump", required=True, type=Path,
                   help="Path to BF `dump` output (saved from Configurator CLI tab).")
    p.add_argument("--quiet", action="store_true",
                   help="Only print on mismatch / failure (CI mode).")
    args = p.parse_args()

    if not args.manifest.exists():
        print(f"ERROR: manifest not found: {args.manifest}", file=sys.stderr)
        return 2
    if not args.dump.exists():
        print(f"ERROR: dump not found: {args.dump}", file=sys.stderr)
        return 2

    manifest = parse_settings(args.manifest.read_text(encoding="utf-8"))
    dump = parse_settings(args.dump.read_text(encoding="utf-8"))

    if not manifest:
        print(f"ERROR: manifest {args.manifest} has no `set` lines", file=sys.stderr)
        return 2

    mismatches = compare(manifest, dump)
    if mismatches:
        print(f"FAIL: {len(mismatches)} of {len(manifest)} required settings do not match dump.")
        for m in mismatches:
            print(m.render())
        return 1

    if not args.quiet:
        print(f"OK: all {len(manifest)} required settings match.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
