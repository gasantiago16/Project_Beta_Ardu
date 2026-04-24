-- safelock.lua  —  Hardware-style safety lockout for Project_Beta_Ardu
-- Implements the safety_runbook.md "ACRO + AUX-companion-HIGH = NEVER" rule.
--
-- Reads the RsMode and RsComp outputs from racestrt.lua (or directly from
-- physical switches if you're not using the sequencer), and outputs SfComp
-- which is forced LOW any time Mode is in the ACRO range.
--
-- This catches:
--   - racestrt.lua bugs that emit the wrong combo
--   - Switches stuck in the bad position
--   - Pilot finger-fumbles between race AUX and companion AUX
--
-- Mix this AFTER racestrt.lua in the script list so it sees the up-to-date
-- outputs from this cycle.

-- ── Tunables ─────────────────────────────────────────────────────────────
-- A mode value below this counts as ACRO. Adjust if your BF mode bands
-- put ACRO somewhere other than the "low" position of the Mode AUX channel.
local ACRO_THRESHOLD = -512

local COMP_LOW = -1024

-- ── Run ──────────────────────────────────────────────────────────────────
-- Inputs:
--   mode — Mode AUX value (typically RsMode from racestrt.lua)
--   comp — Companion AUX value (typically RsComp from racestrt.lua)
local function run(mode, comp)
  if mode < ACRO_THRESHOLD then
    return COMP_LOW
  end
  return comp
end

local function init()
end

return {
  init   = init,
  run    = run,
  input  = { "Mode", "Comp" },
  output = { "SfComp" },
}
