"""Cross-check the offline power calculation against the actual Rust propeller map.

Run using simulation/.venv/bin/python test_power.py.
"""
import unittest
import numpy as np
from microagu_sim._native import mapped_torque
from power import shaft_power


class PowerTest(unittest.TestCase):
    def test_matches_native_propeller_power(self):
        for throttle in [0., 0.01, 0.1, 0.3, 0.45, 0.7, 1.]:
            rpm = 743.5778222336925 * 22.2 * max(throttle - 0.0005141016498671315, 0.) ** 0.9086807307118114
            for axial in [-1., 0., 2., 4., 8., 30.]:
                expected = mapped_torque(rpm, axial) * rpm * 2 * np.pi / 60
                actual = shaft_power(np.array([throttle]), np.array([axial]))[0]
                self.assertAlmostEqual(actual, expected, delta=1e-9)


if __name__ == "__main__":
    unittest.main()
