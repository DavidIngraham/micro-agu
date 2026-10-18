"""Analytical checks for the retrospective metrics, independent of flight results."""
import unittest
import numpy as np
from holistic import band_rms, distribution, errors, longest, settling


class MetricsTests(unittest.TestCase):
    def test_terminal_and_empty_saturation(self):
        self.assertAlmostEqual(longest([False, True, True, False, True, True, True]), .3)
        self.assertEqual(longest([False]*20), 0)

    def test_recovery_requires_full_hold_and_censors(self):
        t = np.arange(0, 180, .1)
        transient = (t >= 101) & (t < 106)
        good = transient | (t >= 120)
        self.assertAlmostEqual(settling(t, good), 20)
        self.assertIsNone(settling(t, t >= 175))
        self.assertAlmostEqual(settling(t, np.ones(len(t), dtype=bool)), 0)

    def test_band_energy_known_sinusoid_and_constant(self):
        t = np.arange(0, 80, .1)
        self.assertAlmostEqual(band_rms(3*np.sin(2*np.pi*.5*t)), 3/np.sqrt(2), places=2)
        self.assertLess(band_rms(np.full(len(t), 8.)), 1e-10)

    def test_bias_not_removed_from_tracking(self):
        result = errors(np.array([3., 3., 3.]))
        self.assertEqual(result['rms'], 3)
        self.assertEqual(result['bias'], 3)

    def test_worst_quarter_keeps_extreme_case(self):
        result = distribution([1, 1, 1, 1, 3, 9])
        self.assertEqual(result['median'], 1)
        self.assertEqual(result['worst_quartile_mean'], 6)
        self.assertEqual(result['worst'], 9)


if __name__ == '__main__':
    unittest.main()
