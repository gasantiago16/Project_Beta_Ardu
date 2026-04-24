-- racehud.lua  —  Race telemetry HUD for Project_Beta_Ardu
--
-- Renders on a TELEMETRY screen (assigned via Model > Display).
-- Color radios get a panel layout; monochrome radios get text rows.
--
-- Telemetry sources expected (from BF/CRSF or from a custom companion
-- backchannel — see edgetx_scripts/README.md "Telemetry sources"):
--   RxBt   — receiver/main battery voltage
--   RSSI or 1RSS — link RSSI
--   RQly   — link quality (CRSF)
--   Sats   — GPS satellite count
--   GAlt   — GPS altitude
--   GPS    — GPS position (table {lat, lon})
--   Hdg    — GPS heading
--
-- Reads global `raceStartState` and `raceStartStateName` set by racestrt.lua
-- (so we can show the current sequencer state on the HUD).

-- ── Helpers ──────────────────────────────────────────────────────────────
local function getNum(name, default)
  local v = getValue(name)
  if v == nil or v == 0 then return default end
  return v
end

local function getStr(name, default)
  local v = getValue(name)
  if v == nil then return default end
  return tostring(v)
end

-- Color helpers — only meaningful on color radios.
local function safeColor(name)
  if lcd.RGB then
    if name == "GREEN"  then return lcd.RGB(0,   200, 0)
    elseif name == "RED"    then return lcd.RGB(220, 30,  30)
    elseif name == "AMBER"  then return lcd.RGB(255, 180, 0)
    elseif name == "WHITE"  then return lcd.RGB(255, 255, 255)
    end
  end
  return 0
end

local function stateColor(stateName)
  if stateName == "RACE"   then return safeColor("GREEN")
  elseif stateName == "ABORT"  then return safeColor("RED")
  elseif stateName == "HOLD"   then return safeColor("AMBER")
  elseif stateName == "LAUNCH" then return safeColor("AMBER")
  end
  return safeColor("WHITE")
end

-- ── Init ─────────────────────────────────────────────────────────────────
local function init()
end

-- ── Background (runs even when HUD isn't on screen) ──────────────────────
local function background()
end

-- ── Run ──────────────────────────────────────────────────────────────────
local function run(event, touchState)
  lcd.clear()

  local stateName = raceStartStateName or "?"
  local rxBat = getNum("RxBt", 0)
  local rssi  = getNum("1RSS", getNum("RSSI", 0))
  local lq    = getNum("RQly", 0)
  local sats  = getNum("Sats", 0)
  local galt  = getNum("GAlt", 0)
  local hdg   = getNum("Hdg", 0)

  local isColor = (LCD_W and LCD_W >= 320)

  if isColor then
    -- Color layout (TX16S, Boxer, Pocket, Tandem, X20).
    -- Big state banner top, key telemetry below.
    lcd.drawFilledRectangle(0, 0, LCD_W, 36, stateColor(stateName))
    lcd.drawText(8, 6, "STATE", INVERS)
    lcd.drawText(80, 0, stateName, DBLSIZE + INVERS)

    local y = 50
    lcd.drawText(10, y,      "Bat:");   lcd.drawText(60,  y, string.format("%.2f V", rxBat / 100.0))
    lcd.drawText(10, y + 22, "RSSI:");  lcd.drawNumber(60, y + 22, rssi)
    lcd.drawText(10, y + 44, "LQ:");    lcd.drawNumber(60, y + 44, lq)
    lcd.drawText(10, y + 66, "Sats:");  lcd.drawNumber(60, y + 66, sats)

    lcd.drawText(180, y,      "Alt:");  lcd.drawText(230, y,      string.format("%d m", galt))
    lcd.drawText(180, y + 22, "Hdg:");  lcd.drawText(230, y + 22, string.format("%d deg", hdg))

    lcd.drawText(10, LCD_H - 18, "Project_Beta_Ardu", SMLSIZE)
  else
    -- Monochrome layout (TX12, X9D, T-Lite). Compact text rows.
    lcd.drawText(0,  0, "STATE: " .. stateName, INVERS)
    lcd.drawText(0,  9,  "Bat:  " .. string.format("%.2f V", rxBat / 100.0))
    lcd.drawText(0,  18, "RSSI: " .. tostring(rssi))
    lcd.drawText(0,  27, "LQ:   " .. tostring(lq))
    lcd.drawText(0,  36, "Sats: " .. tostring(sats))
    lcd.drawText(0,  45, "Alt:  " .. string.format("%d m", galt))
    lcd.drawText(0,  54, "Hdg:  " .. string.format("%d", hdg))
  end
end

return {
  init       = init,
  run        = run,
  background = background,
}
