"""Logic-level harness for the EdgeTX Lua scripts.

`check_lua_syntax.py` only validates that scripts parse — it never executes
their state machines. Logic bugs (wrong transition, missed edge case, ABORT
not winning) would slip through. This harness loads each script via lupa,
calls its `run()` function with controlled input sequences, and asserts the
output channel values match what the safety_runbook describes.

Time is controlled by overriding `getTime()` between steps; the script
computes `nowSec()` as `getTime() / 100.0`.
"""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

import lupa

REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = REPO_ROOT / "edgetx_scripts" / "SCRIPTS" / "MIXES"

# Same EdgeTX surface stubs as check_lua_syntax.py, but `getTime()` returns
# whatever `_time_cs` (centiseconds) we set from Python.
PRELUDE = """
_time_cs = 0
function getTime() return _time_cs end
function getValue(s) return 0 end
function playTone(f, d, p) end
function playNumber(n, u) end
function playFile(f) end
function playHaptic(d, p, f) end
INVERS = 0
DBLSIZE = 0
SMLSIZE = 0
PLAY_NOW = 0
"""


def load_script(rel_path: str):
    """Load a Lua script and return (lua_runtime, returned_table)."""
    runtime = lupa.LuaRuntime(unpack_returned_tuples=True)
    src = (REPO_ROOT / rel_path).read_text(encoding="utf-8")
    table = runtime.execute(PRELUDE + src)
    return runtime, table


def set_time_s(runtime, t_s: float) -> None:
    """Advance the script's clock to t_s seconds (getTime returns centiseconds)."""
    runtime.execute(f"_time_cs = {int(t_s * 100)}")


# Constants from racestrt.lua — duplicated here so a drift between Python
# and Lua trips a test rather than passes silently.
MODE_ACRO = -1024
MODE_ANGLE = 0
MODE_RESCUE = 1024
COMP_LOW = -1024
COMP_HIGH = 1024
ARM_OFF = -1024
ARM_ON = 1024
CONFIRM_HOLD_S = 2.0
AUTO_LAUNCH_HOLD_S = 8.0
STATE_TIMEOUT_S = 60.0


class TestRacestrt(unittest.TestCase):
    def setUp(self):
        self.runtime, self.script = load_script("edgetx_scripts/SCRIPTS/MIXES/racestrt.lua")
        self.script.init()
        set_time_s(self.runtime, 0.0)

    def step(self, t, *, trig=-1024, go=-1024, abort=-1024,
             mode=MODE_ANGLE, comp=COMP_LOW):
        set_time_s(self.runtime, t)
        return tuple(self.script.run(trig, go, abort, mode, comp))

    def test_idle_passthrough(self):
        # IDLE: arm off; mode and companion pass through pilot inputs.
        arm, mode, comp = self.step(0.1, mode=MODE_ANGLE, comp=COMP_LOW)
        self.assertEqual(arm, ARM_OFF)
        self.assertEqual(mode, MODE_ANGLE)
        self.assertEqual(comp, COMP_LOW)

    def test_trig_held_for_confirm_then_launch(self):
        # Note: script assigns outputs BEFORE transitions, so the tick that
        # triggers a state change still emits the OLD state's outputs. The
        # AUTO_LAUNCH outputs become visible on the next tick. This is correct
        # behavior (one-tick consistency); the tests just have to take an
        # extra step to observe the new state.
        self.step(0.0, trig=-1024)
        # Rising edge of trig → CONFIRM (this tick still IDLE outputs)
        arm, mode, _ = self.step(0.1, trig=1024)
        self.assertEqual(arm, ARM_OFF)
        # Hold trig past CONFIRM_HOLD_S → tick fires the transition to
        # AUTO_LAUNCH (outputs are still CONFIRM=ARM_OFF this tick).
        self.step(0.1 + CONFIRM_HOLD_S + 0.1, trig=1024)
        # Next tick: now in AUTO_LAUNCH, outputs are ARM_ON / ANGLE / COMP_HIGH.
        arm, mode, comp = self.step(0.1 + CONFIRM_HOLD_S + 0.2, trig=1024)
        self.assertEqual(arm, ARM_ON)
        self.assertEqual(mode, MODE_ANGLE)
        self.assertEqual(comp, COMP_HIGH)

    def test_trig_release_during_confirm_cancels(self):
        self.step(0.0, trig=-1024)
        self.step(0.1, trig=1024)  # → CONFIRM
        # Release before CONFIRM_HOLD_S → transitions back to IDLE.
        arm, _, _ = self.step(0.5, trig=-1024)
        self.assertEqual(arm, ARM_OFF)
        # New rising edge → CONFIRM, but outputs still IDLE this tick.
        arm, _, _ = self.step(0.7, trig=1024)
        self.assertEqual(arm, ARM_OFF)
        # Hold past CONFIRM_HOLD_S; need an extra tick to observe AUTO_LAUNCH outputs.
        self.step(0.7 + CONFIRM_HOLD_S + 0.1, trig=1024)
        arm, _, comp = self.step(0.7 + CONFIRM_HOLD_S + 0.2, trig=1024)
        self.assertEqual(arm, ARM_ON)
        self.assertEqual(comp, COMP_HIGH)

    def test_auto_launch_to_holding(self):
        # Drive to AUTO_LAUNCH
        self.step(0.0, trig=-1024)
        self.step(0.1, trig=1024)
        t_launch = 0.1 + CONFIRM_HOLD_S + 0.1
        self.step(t_launch, trig=1024)  # → AUTO_LAUNCH
        # After AUTO_LAUNCH_HOLD_S → HOLDING (still ARM_ON, ANGLE, COMP_HIGH)
        t_hold = t_launch + AUTO_LAUNCH_HOLD_S + 0.1
        arm, mode, comp = self.step(t_hold, trig=-1024)
        self.assertEqual(arm, ARM_ON)
        self.assertEqual(mode, MODE_ANGLE)
        self.assertEqual(comp, COMP_HIGH)

    def test_holding_to_racing_on_go_edge(self):
        # Drive to HOLDING
        self.step(0.0, trig=-1024)
        self.step(0.1, trig=1024)
        t1 = 0.1 + CONFIRM_HOLD_S + 0.1
        self.step(t1, trig=1024)  # AUTO_LAUNCH
        t2 = t1 + AUTO_LAUNCH_HOLD_S + 0.1
        self.step(t2, trig=-1024)  # HOLDING
        # Rising edge on go fires transition to RACING; outputs still HOLDING this tick.
        self.step(t2 + 0.1, go=-1024)
        self.step(t2 + 0.2, go=1024)
        # Next tick: RACING outputs.
        arm, mode, comp = self.step(t2 + 0.3, go=1024)
        self.assertEqual(arm, ARM_ON)
        self.assertEqual(mode, MODE_ACRO)
        self.assertEqual(comp, COMP_LOW)

    def test_racing_is_sticky(self):
        # Drive to RACING (compressed timeline)
        self.step(0.0, trig=-1024)
        self.step(0.1, trig=1024)
        t1 = 0.1 + CONFIRM_HOLD_S + 0.1
        self.step(t1, trig=1024)
        t2 = t1 + AUTO_LAUNCH_HOLD_S + 0.1
        self.step(t2, trig=-1024)
        self.step(t2 + 0.1, go=-1024)
        self.step(t2 + 0.2, go=1024)  # transition fires
        self.step(t2 + 0.3, go=1024)  # RACING outputs visible
        # Releasing go must NOT exit racing.
        arm, mode, comp = self.step(t2 + 0.5, go=-1024)
        self.assertEqual(mode, MODE_ACRO)
        self.assertEqual(comp, COMP_LOW)
        # Pilot mode input does NOT take effect (sticky).
        _, mode, _ = self.step(t2 + 0.6, mode=MODE_ANGLE)
        self.assertEqual(mode, MODE_ACRO)

    def test_abort_always_wins(self):
        # ABORT is checked at the TOP of run() — transition fires before any
        # state-block runs, so the ABORT branch (RESCUE outputs) is reached
        # in the SAME tick.
        # From IDLE
        _, mode, comp = self.step(0.1, abort=1024)
        self.assertEqual(mode, MODE_RESCUE)
        self.assertEqual(comp, COMP_LOW)
        # From RACING — drive there first
        self.script.init()
        set_time_s(self.runtime, 0.0)
        self.step(0.0, trig=-1024)
        self.step(0.1, trig=1024)
        t1 = 0.1 + CONFIRM_HOLD_S + 0.1
        self.step(t1, trig=1024)
        t2 = t1 + AUTO_LAUNCH_HOLD_S + 0.1
        self.step(t2, trig=-1024)
        self.step(t2 + 0.1, go=-1024)
        self.step(t2 + 0.2, go=1024)  # transition fires
        self.step(t2 + 0.3, go=1024)  # RACING outputs
        _, mode, comp = self.step(t2 + 0.4, abort=1024)
        self.assertEqual(mode, MODE_RESCUE)
        self.assertEqual(comp, COMP_LOW)

    def test_holding_timeout_aborts(self):
        # Drive to HOLDING then never trigger go.
        self.step(0.0, trig=-1024)
        self.step(0.1, trig=1024)
        t1 = 0.1 + CONFIRM_HOLD_S + 0.1
        self.step(t1, trig=1024)
        t2 = t1 + AUTO_LAUNCH_HOLD_S + 0.1
        self.step(t2, trig=-1024)  # HOLDING
        # Wait past timeout → tick fires the transition to ABORT (output for
        # this tick is still HOLDING). Next tick yields RESCUE.
        self.step(t2 + STATE_TIMEOUT_S + 0.5, trig=-1024)
        _, mode, _ = self.step(t2 + STATE_TIMEOUT_S + 0.6, trig=-1024)
        self.assertEqual(mode, MODE_RESCUE)


class TestSafelock(unittest.TestCase):
    def setUp(self):
        self.runtime, self.script = load_script("edgetx_scripts/SCRIPTS/MIXES/safelock.lua")
        self.script.init()

    def test_acro_forces_companion_low(self):
        # ACRO band (mode = -1024): companion forced LOW regardless of input
        out = self.script.run(-1024, COMP_HIGH)
        self.assertEqual(out, COMP_LOW)

    def test_angle_passes_companion_through(self):
        # ANGLE band (mode = 0): companion passes through
        self.assertEqual(self.script.run(0, COMP_HIGH), COMP_HIGH)
        self.assertEqual(self.script.run(0, COMP_LOW), COMP_LOW)

    def test_threshold_boundary(self):
        # ACRO_THRESHOLD = -512. Strictly less → forced LOW.
        self.assertEqual(self.script.run(-513, COMP_HIGH), COMP_LOW)
        # At threshold → passes through
        self.assertEqual(self.script.run(-512, COMP_HIGH), COMP_HIGH)


def main() -> int:
    if not SCRIPTS_DIR.exists():
        print(f"ERROR: {SCRIPTS_DIR} not found", file=sys.stderr)
        return 1
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    suite.addTests(loader.loadTestsFromTestCase(TestRacestrt))
    suite.addTests(loader.loadTestsFromTestCase(TestSafelock))
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    return 0 if result.wasSuccessful() else 1


if __name__ == "__main__":
    sys.exit(main())
