"""Smoke import test for main.py.

A circular import or typo in `main.py` would fail every binary launch on
the Pi while leaving the rest of the test suite green. This catches it.
"""
from __future__ import annotations

import importlib
import unittest


class TestMainImports(unittest.TestCase):
    def test_main_module_imports_cleanly(self):
        m = importlib.import_module("racer_companion.main")
        self.assertTrue(callable(getattr(m, "main", None)))

    def test_fc_betaflight_imports_cleanly(self):
        m = importlib.import_module("racer_companion.fc.betaflight")
        self.assertTrue(hasattr(m, "BetaflightAdapter"))

    def test_recorder_imports_cleanly(self):
        m = importlib.import_module("racer_companion.recorder")
        self.assertTrue(hasattr(m, "RecordingAdapter"))
        self.assertTrue(hasattr(m, "ReplayAdapter"))


if __name__ == "__main__":
    unittest.main()
