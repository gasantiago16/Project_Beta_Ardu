"""Tests for the heading-threshold tuning harness."""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import tune_heading_threshold as tune  # noqa: E402


class TestSyntheticTrace(unittest.TestCase):
    def test_clean_then_drift(self):
        # Default: 30s clean + 30s drift
        samples = tune.synthetic_drift_trace(duration_s=60.0, dt=0.1)
        # Early samples (t < 30s) should have small |err| (≤ noise amp).
        early_max = max(abs(e) for t, e, _ in samples if t < 25.0)
        self.assertLess(early_max, 10.0)
        # Late samples (t > 50s) should have large |err|, dominated by drift.
        late_max = max(abs(e) for t, e, _ in samples if t > 50.0)
        self.assertGreater(late_max, 30.0)

    def test_count_trips_clean_doesnt_trip(self):
        # First 25s of a clean trace should not trip a 60°/3s tracker.
        samples = [(t, 0.0, "TRANSIT") for t in [i * 0.1 for i in range(250)]]
        self.assertEqual(tune.count_trips(samples, 60.0, 3.0), 0)

    def test_count_trips_sustained_high_err_trips(self):
        # 10s of 90° error in TRANSIT → trips a 60°/3s tracker once.
        samples = [(t, 90.0, "TRANSIT") for t in [i * 0.1 for i in range(100)]]
        self.assertEqual(tune.count_trips(samples, 60.0, 3.0), 1)

    def test_count_trips_only_TRANSIT_counts(self):
        # 10s of 90° error in HOLD state → tracker never engages, no trips.
        samples = [(t, 90.0, "HOLD") for t in [i * 0.1 for i in range(100)]]
        self.assertEqual(tune.count_trips(samples, 60.0, 3.0), 0)


class TestSweep(unittest.TestCase):
    def test_sweep_shape(self):
        samples = [(t, 0.0, "TRANSIT") for t in [i * 0.1 for i in range(50)]]
        results = tune.sweep(samples, thresholds=(60.0,), windows=(3.0, 5.0))
        self.assertEqual(set(results.keys()), {(60.0, 3.0), (60.0, 5.0)})

    def test_higher_threshold_trips_no_more_than_lower(self):
        # Monotonicity: 80° threshold cannot trip more often than 40°.
        samples = tune.synthetic_drift_trace(duration_s=120.0, dt=0.1)
        results = tune.sweep(samples, thresholds=(40.0, 80.0), windows=(3.0,))
        self.assertGreaterEqual(results[(40.0, 3.0)], results[(80.0, 3.0)])


class TestMeasureCell(unittest.TestCase):
    def test_returns_edges_and_seconds(self):
        # 10s of 90° in TRANSIT → 1 trip edge, several seconds tripped.
        samples = [(t, 90.0, "TRANSIT") for t in [i * 0.1 for i in range(100)]]
        edges, secs = tune.measure_cell(samples, 60.0, 3.0)
        self.assertEqual(edges, 1)
        # Window=3s + must accumulate window before first trip — so tripped
        # for ~7 of the 10 seconds, give or take dt boundary effects.
        self.assertGreater(secs, 5.0)
        self.assertLess(secs, 9.0)

    def test_no_trip_no_seconds(self):
        # Clean trace → 0 edges, 0 seconds tripped.
        samples = [(t, 5.0, "TRANSIT") for t in [i * 0.1 for i in range(100)]]
        edges, secs = tune.measure_cell(samples, 60.0, 3.0)
        self.assertEqual(edges, 0)
        self.assertEqual(secs, 0.0)


class TestLoadLog(unittest.TestCase):
    def test_load_real_log(self):
        import json as _json
        import tempfile
        from pathlib import Path
        with tempfile.TemporaryDirectory() as td:
            log = Path(td) / "log.jsonl"
            log.write_text(
                _json.dumps({"t": 0.0, "heading_err_deg": 5.0, "state": "TRANSIT"}) + "\n" +
                _json.dumps({"t": 0.1, "heading_err_deg": 6.0, "state": "TRANSIT"}) + "\n"
            )
            samples = tune.load_log(log)
        self.assertEqual(len(samples), 2)
        self.assertEqual(samples[0], (0.0, 5.0, "TRANSIT"))

    def test_skips_malformed_lines(self):
        import tempfile
        from pathlib import Path
        with tempfile.TemporaryDirectory() as td:
            log = Path(td) / "log.jsonl"
            log.write_text(
                '{"t": 0.0, "heading_err_deg": 5.0, "state": "TRANSIT"}\n'
                'this is not json\n'
                '{"t": 0.2, "heading_err_deg": 7.0, "state": "TRANSIT"}\n'
                '{"t": 0.3, "heading_err',  # torn last line
            )
            samples = tune.load_log(log)
        self.assertEqual(len(samples), 2)


class TestRenderTable(unittest.TestCase):
    def test_table_contains_all_cells(self):
        results = {
            (40.0, 3.0): (5, 12.5), (40.0, 5.0): (2, 8.0),
            (60.0, 3.0): (3, 9.0), (60.0, 5.0): (1, 4.5),
        }
        out_edges = tune.render_table(results, (40.0, 60.0), (3.0, 5.0), field="edges")
        for v in (5, 2, 3, 1):
            self.assertIn(str(v), out_edges)
        out_secs = tune.render_table(results, (40.0, 60.0), (3.0, 5.0), field="secs")
        # Tripped-second values should appear (one decimal place).
        self.assertIn("12.5", out_secs)
        self.assertIn("4.5", out_secs)

    def test_no_unicode_in_output(self):
        results = {
            (40.0, 3.0): (1, 1.0),
        }
        out = tune.render_table(results, (40.0,), (3.0,), field="edges")
        out.encode("ascii")  # must not raise


if __name__ == "__main__":
    unittest.main()
