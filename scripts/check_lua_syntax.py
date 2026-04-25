"""CI sanity check: every .lua file under edgetx_scripts/ must parse + execute
its top-level return-table block under embedded Lua. Catches typos like `==`
vs `=`, missing `end`, broken table syntax, etc.

This does NOT verify runtime behavior on a real radio — only that the script
loads. The mocked globals below are the EdgeTX surface our scripts actually
touch; expand if a future script uses more.
"""
from __future__ import annotations

import sys
from pathlib import Path

import lupa

PRELUDE = """
function getTime() return 0 end
function getValue(s) return 0 end
function playTone(f, d, p) end
function playNumber(n, u) end
function playFile(f) end
function playHaptic(d, p, f) end
INVERS = 0
DBLSIZE = 0
SMLSIZE = 0
PLAY_NOW = 0
LCD_W = 480
LCD_H = 272
SOURCE = 0
lcd = {
  clear = function() end,
  drawText = function() end,
  drawNumber = function() end,
  drawFilledRectangle = function() end,
  drawRectangle = function() end,
  RGB = function(r, g, b) return 0 end,
}
"""


def main() -> int:
    repo_root = Path(__file__).resolve().parents[1]
    scripts_dir = repo_root / "edgetx_scripts" / "SCRIPTS"
    if not scripts_dir.exists():
        print(f"ERROR: {scripts_dir} not found", file=sys.stderr)
        return 1

    lua_files = sorted(scripts_dir.rglob("*.lua"))
    if not lua_files:
        print("ERROR: no .lua files found", file=sys.stderr)
        return 1

    failures = 0
    for path in lua_files:
        runtime = lupa.LuaRuntime()
        try:
            runtime.execute(PRELUDE + path.read_text(encoding="utf-8"))
            rel = path.relative_to(repo_root)
            print(f"OK    {rel}")
        except Exception as e:
            rel = path.relative_to(repo_root)
            print(f"FAIL  {rel}: {e}", file=sys.stderr)
            failures += 1

    print(f"\n{len(lua_files) - failures}/{len(lua_files)} passed")
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
