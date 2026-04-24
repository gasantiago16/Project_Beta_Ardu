import unittest

from racer_companion import state as sm


class TestStateMachine(unittest.TestCase):
    def setUp(self):
        self.ctx = sm.StateContext()
        self.kw = dict(
            climb_target_alt_m=5.0,
            arrival_radius_m=3.0,
            arrival_dwell_s=2.0,
            current_alt_m=0.0,
            distance_to_target_m=100.0,
        )

    def step(self, *, now, aux, safety_ok, **over):
        merged = {**self.kw, **over}
        return sm.step(
            self.ctx, now=now, aux_companion_active=aux, safety_ok=safety_ok, **merged,
        )

    def test_idle_to_climb(self):
        s = self.step(now=0, aux=True, safety_ok=True)
        self.assertEqual(s, sm.State.CLIMB)

    def test_climb_to_transit(self):
        self.step(now=0, aux=True, safety_ok=True)
        s = self.step(now=1, aux=True, safety_ok=True, current_alt_m=6.0)
        self.assertEqual(s, sm.State.TRANSIT)

    def test_transit_to_hold_after_dwell(self):
        self.step(now=0, aux=True, safety_ok=True)
        self.step(now=1, aux=True, safety_ok=True, current_alt_m=6.0)
        s = self.step(now=2, aux=True, safety_ok=True, current_alt_m=6.0, distance_to_target_m=1.0)
        self.assertEqual(s, sm.State.TRANSIT)
        s = self.step(now=4.5, aux=True, safety_ok=True, current_alt_m=6.0, distance_to_target_m=1.0)
        self.assertEqual(s, sm.State.HOLD)

    def test_aux_low_returns_idle(self):
        self.step(now=0, aux=True, safety_ok=True)
        s = self.step(now=1, aux=False, safety_ok=True)
        self.assertEqual(s, sm.State.IDLE)

    def test_safety_violation_sticky(self):
        self.step(now=0, aux=True, safety_ok=True)
        s = self.step(now=1, aux=True, safety_ok=False)
        self.assertEqual(s, sm.State.RELEASED)
        s = self.step(now=2, aux=True, safety_ok=True)
        self.assertEqual(s, sm.State.RELEASED)
        s = self.step(now=3, aux=False, safety_ok=True)
        self.assertEqual(s, sm.State.IDLE)

    def test_arrival_dwell_resets_on_drift(self):
        # Reach hold-radius briefly, then drift out, then back in. Dwell should reset.
        self.step(now=0, aux=True, safety_ok=True)
        self.step(now=1, aux=True, safety_ok=True, current_alt_m=6.0)
        self.step(now=2, aux=True, safety_ok=True, current_alt_m=6.0, distance_to_target_m=1.0)
        # Drift
        s = self.step(now=3, aux=True, safety_ok=True, current_alt_m=6.0, distance_to_target_m=10.0)
        self.assertEqual(s, sm.State.TRANSIT)
        self.assertEqual(self.ctx.arrival_dwell_start, 0.0)
        # Re-arrive
        s = self.step(now=4, aux=True, safety_ok=True, current_alt_m=6.0, distance_to_target_m=1.0)
        self.assertEqual(s, sm.State.TRANSIT)
        # Wait less than dwell
        s = self.step(now=5, aux=True, safety_ok=True, current_alt_m=6.0, distance_to_target_m=1.0)
        self.assertEqual(s, sm.State.TRANSIT)
        # Wait full dwell from re-arrival
        s = self.step(now=6.5, aux=True, safety_ok=True, current_alt_m=6.0, distance_to_target_m=1.0)
        self.assertEqual(s, sm.State.HOLD)

    def test_is_active(self):
        self.assertFalse(sm.is_active(sm.State.IDLE))
        self.assertFalse(sm.is_active(sm.State.RELEASED))
        self.assertTrue(sm.is_active(sm.State.CLIMB))
        self.assertTrue(sm.is_active(sm.State.TRANSIT))
        self.assertTrue(sm.is_active(sm.State.HOLD))


if __name__ == "__main__":
    unittest.main()
