# Lua for Beginners (and EdgeTX gotchas)

For: someone who has never written code before, or someone who knows another
language and is bouncing off Lua's quirks.

This is **not** a comprehensive Lua tutorial. It's the minimum you need to
read, modify, and ship the scripts in this directory.

## Lua in 90 seconds

```lua
-- Comments start with two dashes.
-- A block comment looks like this:
--[[ multi
     line ]]

-- Variables. The `local` keyword is REQUIRED — without it, the variable
-- becomes a GLOBAL, which is almost always a bug.
local count = 0
local name = "Mr. Slippery"
local isReady = true
local nothing = nil   -- nil is "no value", like null/None

-- Strings concatenate with .. (NOT +)
local greeting = "Hello, " .. name

-- Conditionals
if count > 0 then
  print("positive")
elseif count == 0 then       -- equality is `==`, not `=`
  print("zero")
else
  print("negative")
end

-- Logical operators are words, not symbols.
if isReady and count > 0 then
  print("go")
end
if not isReady or count == 0 then
  print("wait")
end

-- Loops
for i = 1, 10 do          -- INCLUSIVE on both ends. NOT 0 to 9. NOT < 10.
  print(i)
end

while count < 5 do
  count = count + 1       -- NO ++ or +=. You must write count = count + 1.
end

-- Functions
local function add(a, b)
  return a + b
end
print(add(3, 4))

-- Tables (Lua's only collection type — both arrays AND maps)
local list = { 10, 20, 30 }
print(list[1])             -- 10. TABLES ARE 1-INDEXED. NOT 0-INDEXED.
print(#list)               -- 3 (length operator)

local map = { name = "Mr. Slippery", weight_g = 320 }
print(map.name)
print(map["name"])         -- same thing

-- Multiple return values
local function divmod(a, b)
  return a // b, a % b
end
local q, r = divmod(17, 5)
```

## Five Lua gotchas that will bite you

1. **Forgetting `local`** makes the variable global. Globals persist across
   script invocations on the radio and can cause weird interactions.
2. **`==` vs `=`** — `=` is assignment, `==` is comparison. `if x = 0 then` is
   a syntax error.
3. **`!=` does not exist.** Use `~=`. So `if count ~= 0 then`.
4. **No `++`, no `+=`.** Write it out: `count = count + 1`.
5. **Tables are 1-indexed.** `list[0]` is `nil` for an array of three.

## EdgeTX Lua API — the parts you'll actually use

```lua
-- Time
local t = getTime()         -- centiseconds since boot. Divide by 100 for seconds.

-- Reading switches and channels
local sa = getValue("sa")   -- value in -1024..+1024
local ch5 = getValue("ch5") -- channel value
local rxbat = getValue("RxBt")  -- telemetry sensor by name

-- Audio
playTone(1500, 200, 0)      -- (freq Hz, duration ms, pause ms)
playNumber(3, 0)            -- speak the number "three"
playFile("warn1.wav")       -- play a file from /SOUNDS/<lang>/

-- Haptic (vibrate)
playHaptic(200, 0, 1)       -- (duration ms, pause ms, flags)

-- LCD (in telemetry/full-screen scripts only)
lcd.clear()
lcd.drawText(10, 20, "Hello", INVERS)
lcd.drawNumber(10, 40, 123, DBLSIZE)
lcd.drawFilledRectangle(0, 0, LCD_W, 30, 0)
local w, h = LCD_W, LCD_H   -- screen size constants
```

## How EdgeTX Lua scripts are structured

There are two flavors used in this repo:

### Mix scripts (run on every mixer cycle, ~30 Hz)

```lua
local function init()
  -- runs once when the script loads
end

local function run(input1, input2, input3)
  -- runs every cycle. Inputs come from the values assigned to this script
  -- on the radio's MODEL > Custom Scripts setup page.
  -- Return as many values as you declared in `output`.
  return 0, 0, 0
end

return {
  init   = init,
  run    = run,
  input  = { "Trig", "Go", "Abort" },     -- names shown on setup page
  output = { "RsArm", "RsMode", "RsComp" }, -- channel names this script feeds
}
```

### Telemetry scripts (run when their telemetry screen is active)

```lua
local function init() end
local function background() end       -- runs even when screen not displayed
local function run(event, touchState) -- runs when screen is displayed
  lcd.clear()
  lcd.drawText(10, 10, "Hello")
end

return { init = init, run = run, background = background }
```

## Debugging

There is no `print` to a serial console on the radio. Two options:

1. **Lua Console** (Radio > Tools > Lua Console). Errors from your script
   appear here. `print()` calls in your script also appear here.
2. **HUD**. Render values on screen via your telemetry script. Crude but
   reliable.

## Suggested first edits (start here)

These are safe, reversible changes that teach you the codebase:

1. **Change a tone frequency.** Open `racestrt.lua`, find a `playTone(...)`
   call, change the first number (frequency in Hz). Reload the script
   (turn radio off/on). Trigger that state. Hear the new tone.
2. **Change the confirm hold time.** `CONFIRM_HOLD_S = 2.0` at the top of
   `racestrt.lua`. Set it to `0.5` for faster engagement, or `4.0` for
   even more confirmation.
3. **Add a state to the HUD.** In `racehud.lua`, add another telemetry value
   (e.g., `getValue("Tmp1")` for an FC temperature sensor) and `lcd.drawText`
   it on screen.
4. **Change the abort behavior.** In `racestrt.lua`, the `STATE_ABORT` block
   sets `outMode = MODE_RESCUE`. What if you want abort to set ANGLE
   instead (let pilot fly down manually)? Change to `MODE_ANGLE`. Test on
   bench (props off) before flying.

## Things to NOT do

- **Don't add network calls or file I/O.** EdgeTX scripts run sandboxed — most
  Lua libraries (io, os, package) are unavailable or limited. Use only the
  EdgeTX API.
- **Don't write infinite loops.** `while true do end` will lock the radio.
  EdgeTX has a per-cycle execution budget; exceed it and the script gets
  killed (silently, sometimes).
- **Don't use globals carelessly.** The race-state global
  (`raceStartStateName`) is intentional and shared with the HUD. Other
  globals will cause script-to-script bleeding.
- **Don't ignore the Lua Console.** If you make a change and the script
  silently stops working, check the console for an error.

## Where to learn more

- **EdgeTX Lua reference (authoritative):** https://luadocs.edgetx.org/
- **Lua language reference:** https://www.lua.org/manual/5.2/
- **Programming in Lua (free first edition):** https://www.lua.org/pil/
  Aimed at programmers, but chapters 1–9 cover everything you'd need for
  EdgeTX scripts.

## When you're stuck

- Compare your edit to the original (in git — `git diff` shows what you
  changed).
- Revert and re-apply one change at a time until you find the breaker.
- Ask in the [EdgeTX Discord](https://discord.com/invite/wF9HuyAdcr) #lua
  channel — friendly, responsive.
