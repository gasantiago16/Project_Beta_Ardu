-- racestrt.lua  —  Race-start sequencer for Project_Beta_Ardu
-- See edgetx_scripts/README.md for setup and switch layout.
--
-- States:
--   IDLE         — pass through manual switches. Wait for TRIG rising edge.
--   CONFIRM      — TRIG must be held for CONFIRM_HOLD_S to commit. Cancels
--                  on early release. Prevents accidental arming.
--   AUTO_LAUNCH  — Arm + Mode=ANGLE + Companion=HIGH. Companion takes over,
--                  climbs and transits to start-line waypoint.
--   HOLDING      — Drone holding at start gate. Waiting for GO trigger.
--                  Auto-aborts after STATE_TIMEOUT_S as a safety net.
--   RACING       — Mode=ACRO + Companion=LOW. Native Betaflight feel,
--                  pilot has the sticks. Sticky until ABORT.
--   ABORT        — Mode=GPS_RESCUE + Companion=LOW. BF GPS Rescue brings
--                  the drone home. Pilot must drop ABORT to reset.
--
-- All output channels go through safelock.lua before reaching the FC.

-- ── Tunables ─────────────────────────────────────────────────────────────
local CONFIRM_HOLD_S    = 2.0    -- TRIG hold time to commit to launch
local AUTO_LAUNCH_HOLD_S = 8.0   -- AUTO_LAUNCH duration before HOLDING
local STATE_TIMEOUT_S   = 60.0   -- HOLDING timeout → auto-abort

-- ── EdgeTX channel value constants (-1024 .. +1024) ──────────────────────
local MODE_ACRO   = -1024
local MODE_ANGLE  = 0
local MODE_RESCUE = 1024

local COMP_LOW   = -1024
local COMP_HIGH  = 1024

local ARM_OFF    = -1024
local ARM_ON     = 1024

-- ── States ───────────────────────────────────────────────────────────────
local STATE_IDLE        = 0
local STATE_CONFIRM     = 1
local STATE_AUTO_LAUNCH = 2
local STATE_HOLDING     = 3
local STATE_RACING      = 4
local STATE_ABORT       = 5

local STATE_NAMES = {
  [0] = "IDLE", [1] = "CONFIRM", [2] = "LAUNCH",
  [3] = "HOLD", [4] = "RACE",    [5] = "ABORT",
}

-- ── Module-local state ───────────────────────────────────────────────────
local state = STATE_IDLE
local stateEntered = 0
local lastTrigVal = -1024
local lastGoVal = -1024

-- Outputs (cached so we always return something even on early returns)
local outArm, outMode, outComp = ARM_OFF, MODE_ANGLE, COMP_LOW

-- Exposed via global so racehud.lua can render current state
raceStartState = STATE_IDLE
raceStartStateName = "IDLE"

-- ── Helpers ──────────────────────────────────────────────────────────────
local function nowSec()
  return getTime() / 100.0   -- getTime() returns centiseconds
end

local function risingEdge(curVal, lastVal)
  return curVal > 0 and lastVal <= 0
end

local function transition(newState)
  if newState == state then return end
  state = newState
  stateEntered = nowSec()
  raceStartState = state
  raceStartStateName = STATE_NAMES[state]

  -- Audio cue per state
  if newState == STATE_CONFIRM then
    playTone(800, 100, 0)
  elseif newState == STATE_AUTO_LAUNCH then
    playTone(1200, 200, 0)
    playNumber(3, 0); playNumber(2, 0); playNumber(1, 0)
  elseif newState == STATE_HOLDING then
    playTone(1500, 200, 0)
    playTone(1500, 200, 100)
  elseif newState == STATE_RACING then
    playTone(2000, 400, 0)
  elseif newState == STATE_ABORT then
    playTone(400, 800, 0)
    playHaptic(150, 0, 1)
  elseif newState == STATE_IDLE then
    playTone(600, 80, 0)
  end
end

-- ── Init ─────────────────────────────────────────────────────────────────
local function init()
  state = STATE_IDLE
  stateEntered = nowSec()
  raceStartState = STATE_IDLE
  raceStartStateName = "IDLE"
end

-- ── Run ──────────────────────────────────────────────────────────────────
-- Inputs:
--   trig  — momentary "engage race start" (e.g., SH)
--   go    — momentary "GO! release to manual" (e.g., SI)
--   abort — toggle "abort sequence" (e.g., SF up)
--   mode  — manual mode pass-through (e.g., SA: -1024/0/+1024)
--   comp  — manual companion AUX pass-through (e.g., SB: -1024/+1024)
local function run(trig, go, abort, mode, comp)
  local now = nowSec()
  local since = now - stateEntered

  -- ABORT always wins, regardless of state.
  if abort > 0 and state ~= STATE_ABORT then
    transition(STATE_ABORT)
  end

  local trigEdge = risingEdge(trig, lastTrigVal)
  local goEdge   = risingEdge(go,   lastGoVal)
  lastTrigVal = trig
  lastGoVal = go

  if state == STATE_IDLE then
    outArm  = ARM_OFF
    outMode = mode
    outComp = comp
    if trigEdge then transition(STATE_CONFIRM) end

  elseif state == STATE_CONFIRM then
    outArm  = ARM_OFF
    outMode = mode
    outComp = comp
    if trig <= 0 then
      transition(STATE_IDLE)              -- pilot let go: cancel
    elseif since >= CONFIRM_HOLD_S then
      transition(STATE_AUTO_LAUNCH)
    end

  elseif state == STATE_AUTO_LAUNCH then
    outArm  = ARM_ON
    outMode = MODE_ANGLE
    outComp = COMP_HIGH
    if since >= AUTO_LAUNCH_HOLD_S then
      transition(STATE_HOLDING)
    end

  elseif state == STATE_HOLDING then
    outArm  = ARM_ON
    outMode = MODE_ANGLE
    outComp = COMP_HIGH
    if goEdge then
      transition(STATE_RACING)
    elseif since >= STATE_TIMEOUT_S then
      transition(STATE_ABORT)
    end

  elseif state == STATE_RACING then
    outArm  = ARM_ON
    outMode = MODE_ACRO
    outComp = COMP_LOW
    -- Sticky. Pilot lands manually + disarms manually + flips ABORT to reset.

  elseif state == STATE_ABORT then
    outArm  = ARM_OFF
    outMode = MODE_RESCUE   -- BF GPS Rescue takes the drone home
    outComp = COMP_LOW
    if abort <= 0 and trig <= 0 then
      transition(STATE_IDLE)
    end
  end

  return outArm, outMode, outComp
end

return {
  init   = init,
  run    = run,
  input  = { "Trig", "Go", "Abrt", "Mode", "Comp" },
  output = { "RsArm", "RsMode", "RsComp" },
}
