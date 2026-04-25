"""Tests for preflight_validator.

Runs as part of the `lua-syntax` job (renamed below in CI to `scripts-tests`)
to keep all repo-tooling tests in one place.
"""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

# Allow `import preflight_validator` when run from repo root.
sys.path.insert(0, str(Path(__file__).resolve().parent))

import preflight_validator as pv  # noqa: E402


class TestParseSettings(unittest.TestCase):
    def test_basic(self):
        text = """
        # comment
        set msp_override_channels_mask = 15
        set mag_hardware = NONE
        """
        self.assertEqual(pv.parse_settings(text), {
            "msp_override_channels_mask": "15",
            "mag_hardware": "NONE",
        })

    def test_ignores_non_set_lines(self):
        text = """
        feature MOTOR_STOP
        save
        set foo = 1
        """
        self.assertEqual(pv.parse_settings(text), {"foo": "1"})

    def test_last_wins(self):
        # BF dump puts `defaults` first then user `set`s. Last must win.
        text = """
        set foo = old
        set foo = new
        """
        self.assertEqual(pv.parse_settings(text)["foo"], "new")

    def test_preserves_hash_in_value(self):
        # `#` inside a value must NOT be stripped — manifest values can contain
        # `#` legitimately (e.g., a name field with #1 suffix). Whole-line
        # comments are still ignored.
        text = "set craft_name = jacob#1\n# whole-line comment"
        self.assertEqual(pv.parse_settings(text), {"craft_name": "jacob#1"})

    def test_whole_line_comments_ignored(self):
        text = "  # leading-comment\nset x = 1\n  # another"
        self.assertEqual(pv.parse_settings(text), {"x": "1"})

    def test_value_with_spaces(self):
        # Serial config and similar carry space-separated tuples.
        text = "set serial_1 = 1 2 0 0 0"
        self.assertEqual(pv.parse_settings(text), {"serial_1": "1 2 0 0 0"})

    def test_whitespace_tolerance(self):
        text = "  set   spacey   =   42  "
        self.assertEqual(pv.parse_settings(text), {"spacey": "42"})

    def test_blank(self):
        self.assertEqual(pv.parse_settings(""), {})

    def test_crlf_line_endings(self):
        # BF dumps from Windows tooling use CRLF. splitlines() handles it,
        # but the regex must not pick up a stray \r. Pin it.
        text = "set foo = 1\r\nset bar = 2\r\n"
        self.assertEqual(pv.parse_settings(text), {"foo": "1", "bar": "2"})

    def test_ignores_non_set_dump_artifacts(self):
        # Real BF dump intersperses `feature`, `serial`, `resource`, save,
        # and inline `# resource MOTOR ...` lines. Only `set ...` is captured.
        text = """
        feature MOTOR_STOP
        # resource MOTOR 1 A02
        resource SERIAL_TX 1 A09
        serial 0 64 115200 57600 0 115200
        beeper -ON_USB
        save
        set foo = 1
        """
        self.assertEqual(pv.parse_settings(text), {"foo": "1"})

    def test_key_case_strict(self):
        # BF emits keys lowercase. `FOO` and `foo` must NOT match — a typo
        # in a manifest key would silently pass otherwise.
        manifest = pv.parse_settings("set FOO = 1")
        dump = pv.parse_settings("set foo = 1")
        self.assertEqual(pv.compare(manifest, dump), [Mismatch_("FOO", "1", None)])

    def test_text_value_case_strict(self):
        # craft_name / pilot name fields preserve case. A wrong-case name
        # must be flagged. (BUG: the previous `value.upper()` would silently
        # match `Alex1` against `alex1`.)
        out = pv.compare({"craft_name": "Alex1"}, {"craft_name": "alex1"})
        self.assertEqual(len(out), 1)


def Mismatch_(key, expected, actual):
    return pv.Mismatch(key=key, expected=expected, actual=actual)


class TestCompare(unittest.TestCase):
    def test_match(self):
        manifest = {"a": "1", "b": "NONE"}
        dump = {"a": "1", "b": "NONE", "c": "extra"}
        self.assertEqual(pv.compare(manifest, dump), [])

    def test_missing(self):
        manifest = {"a": "1", "b": "2"}
        dump = {"a": "1"}
        out = pv.compare(manifest, dump)
        self.assertEqual(len(out), 1)
        self.assertEqual(out[0].key, "b")
        self.assertIsNone(out[0].actual)

    def test_value_differs(self):
        manifest = {"a": "1"}
        dump = {"a": "2"}
        out = pv.compare(manifest, dump)
        self.assertEqual(len(out), 1)
        self.assertEqual(out[0].key, "a")
        self.assertEqual(out[0].actual, "2")

    def test_case_insensitive_enum(self):
        # BF accepts `none` on input but emits `NONE`. Manifest with either
        # must match either dump form.
        self.assertEqual(pv.compare({"x": "NONE"}, {"x": "none"}), [])
        self.assertEqual(pv.compare({"x": "none"}, {"x": "NONE"}), [])

    def test_value_whitespace_normalized(self):
        # `1  2  0` and `1 2 0` should match (BF normalizes on output).
        self.assertEqual(pv.compare({"x": "1 2 0"}, {"x": "1  2  0"}), [])

    def test_numeric_strict(self):
        # Numbers must match exactly — 1 vs 1.0 must NOT match (changes BF
        # semantics on some PID fields).
        out = pv.compare({"x": "1"}, {"x": "1.0"})
        self.assertEqual(len(out), 1)


if __name__ == "__main__":
    unittest.main()
