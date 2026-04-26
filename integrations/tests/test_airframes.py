"""Tests for the per-airframe profile registry."""
from __future__ import annotations

import os
import unittest
from pathlib import Path

from integrations.configs.airframes import (
    IRIS, PROFILES, RACER_5IN, USD_PEGASUS_DEFAULT,
    AirframeProfile, get_profile, resolve_usd_path,
)


class TestProfilesRegistered(unittest.TestCase):
    def test_iris_present(self):
        self.assertIn("iris", PROFILES)
        self.assertIs(PROFILES["iris"], IRIS)

    def test_racer5_present(self):
        self.assertIn("racer5", PROFILES)
        self.assertIs(PROFILES["racer5"], RACER_5IN)

    def test_get_profile_returns_known(self):
        self.assertEqual(get_profile("iris").name, "iris")
        self.assertEqual(get_profile("racer5").name, "racer5")

    def test_get_profile_raises_helpful_on_unknown(self):
        with self.assertRaises(KeyError) as ctx:
            get_profile("not_a_real_quad")
        msg = str(ctx.exception)
        # Helpful message lists known profiles + the misspelled name.
        self.assertIn("not_a_real_quad", msg)
        self.assertIn("iris", msg)
        self.assertIn("racer5", msg)


class TestIrisProfile(unittest.TestCase):
    def test_uses_pegasus_default_sentinel(self):
        # Iris's USD lives in the Pegasus tree, not this repo.
        self.assertEqual(IRIS.usd_path, USD_PEGASUS_DEFAULT)

    def test_rotor_max_omega(self):
        # 880 KV × 11.1 V × 2π/60 ≈ 1023 rad/s. Pinned because the bridge
        # uses this as the PWM→ω scale factor.
        self.assertAlmostEqual(IRIS.rotor_max_omega, 1023.0, places=1)


class TestRacer5InProfile(unittest.TestCase):
    def test_usd_path_in_repo(self):
        # Path points at the repo's assets/ tree, NOT Pegasus's.
        self.assertNotEqual(RACER_5IN.usd_path, USD_PEGASUS_DEFAULT)
        self.assertTrue(RACER_5IN.usd_path.endswith("racer_5in.usd"),
                        f"unexpected path: {RACER_5IN.usd_path}")
        self.assertIn("racer_5in", RACER_5IN.usd_path)

    def test_urdf_source_exists(self):
        # The .usd is gitignored (regenerated locally), but the .urdf
        # source MUST exist — it's the source of truth for the airframe.
        urdf = Path(RACER_5IN.usd_path).with_suffix(".urdf")
        self.assertTrue(
            urdf.exists(),
            f"URDF source missing at {urdf}. Phase 5 broken; check repo.",
        )

    def test_rotor_max_omega(self):
        # Race-quad regime. ~3x Iris is the right ballpark.
        self.assertAlmostEqual(RACER_5IN.rotor_max_omega, 3000.0, places=1)
        self.assertGreater(RACER_5IN.rotor_max_omega, 2 * IRIS.rotor_max_omega)

    def test_mass_is_race_quad_class(self):
        # 0.5 kg ± reasonable. 1 kg+ would be wrong (F450 territory).
        self.assertLess(RACER_5IN.mass_kg, 0.7)
        self.assertGreater(RACER_5IN.mass_kg, 0.3)

    def test_thrust_torque_coeff_signs(self):
        self.assertGreater(RACER_5IN.thrust_coeff, 0)
        self.assertGreater(RACER_5IN.torque_coeff, 0)

    def test_torque_to_thrust_ratio_in_band(self):
        # Empirical c_Q / c_T for a 5" tri-blade prop is ~0.012 (range
        # 0.005..0.05 across diameters). c_Q ≫ c_T·0.05 = nonsense yaw
        # authority (the original 7.4e-6 was 12× the thrust coeff and
        # would have flip-spun on a single yaw stick input).
        ratio = RACER_5IN.torque_coeff / RACER_5IN.thrust_coeff
        self.assertGreater(ratio, 0.001,
                           f"c_Q/c_T={ratio:.2e} too low — yaw won't respond")
        self.assertLess(ratio, 0.05,
                        f"c_Q/c_T={ratio:.2e} too high — yaw will be over-authoritative")

    def test_target_thrust_to_weight_realistic(self):
        # Race-quad sim must produce realistic T:W. Compute end-to-end:
        # T_max_per_rotor = c_T · ω_max² ; T:W = (4·T_max) / (mass·g).
        g = 9.80665
        t_max_per = RACER_5IN.thrust_coeff * RACER_5IN.rotor_max_omega ** 2
        # AUW includes rotor mass (4 × 0.020 = 0.080 kg) on top of the
        # base_link mass — assume URDF total = 0.500 kg.
        auw_kg = 0.500
        t_to_w = (4 * t_max_per) / (auw_kg * g)
        self.assertGreater(t_to_w, 6.0,
                           f"T:W={t_to_w:.1f} too low for race quad (expect >6)")
        self.assertLess(t_to_w, 16.0,
                        f"T:W={t_to_w:.1f} unrealistically high (max real ≈ 14)")


class TestRacer5InUrdf(unittest.TestCase):
    """The URDF is the source of truth for the race-quad rigid body. These
    tests parse it directly and verify the structure Pegasus + the BF
    bridge depend on (rotor link names, joint origins, mass distribution).
    """
    @classmethod
    def setUpClass(cls):
        import xml.etree.ElementTree as ET
        urdf_path = Path(RACER_5IN.usd_path).with_suffix(".urdf")
        cls.tree = ET.parse(urdf_path)
        cls.root = cls.tree.getroot()

    def test_robot_name(self):
        self.assertEqual(self.root.get("name"), "racer_5in")

    def test_rotor_links_present(self):
        links = {l.get("name") for l in self.root.findall("link")}
        for i in range(4):
            self.assertIn(
                f"rotor{i}", links,
                f"rotor{i} link missing — Pegasus's force model would fail",
            )
        self.assertIn("base_link", links)

    def test_rotor_joints_present(self):
        joints = {j.get("name") for j in self.root.findall("joint")}
        for i in range(4):
            self.assertIn(
                f"joint{i}", joints,
                f"joint{i} missing — Pegasus's articulation lookup would fail",
            )

    def test_rotor_x_config_geometry(self):
        # X-config: arm_length × cos(45°) ≈ 0.0884 for arm = 0.125 m.
        # Each rotor at (±0.0884, ±0.0884) with one of each sign combination.
        expected_xy = {(0.0884, -0.0884), (-0.0884, 0.0884),
                       (0.0884, 0.0884), (-0.0884, -0.0884)}
        seen = set()
        for j in self.root.findall("joint"):
            name = j.get("name")
            if not name.startswith("joint"):
                continue
            origin = j.find("origin")
            xyz = origin.get("xyz").split()
            x, y = float(xyz[0]), float(xyz[1])
            seen.add((round(x, 4), round(y, 4)))
        self.assertEqual(
            seen, expected_xy,
            f"Rotor positions not X-config; got {seen}",
        )

    def test_total_mass_targets_500g(self):
        total = 0.0
        for link in self.root.findall("link"):
            m = link.find("inertial/mass")
            if m is not None:
                total += float(m.get("value"))
        # AUW = base + 4 rotors. 0.420 + 4×0.020 = 0.500.
        self.assertAlmostEqual(
            total, 0.500, places=2,
            msg=f"Total URDF mass {total:.3f} ≠ advertised ~500 g AUW",
        )


class TestResolveUsdPath(unittest.TestCase):
    def test_iris_requires_explicit_pegasus_path(self):
        # The orchestrator MUST supply Pegasus's ROBOTS["Iris"] path —
        # we can't import pegasus from inside this repo's tests.
        with self.assertRaises(ValueError):
            resolve_usd_path(IRIS)

    def test_iris_returns_supplied_path(self):
        fake_iris = "/some/pegasus/install/iris.usd"
        self.assertEqual(resolve_usd_path(IRIS, fake_iris), fake_iris)

    def test_racer5_ignores_supplied_iris_path(self):
        # A race-quad profile knows its own path; passing an Iris path
        # alongside doesn't change the result.
        fake_iris = "/some/pegasus/install/iris.usd"
        self.assertEqual(resolve_usd_path(RACER_5IN, fake_iris),
                         RACER_5IN.usd_path)


if __name__ == "__main__":
    unittest.main()
