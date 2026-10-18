"""A known oscillation checks trim removal, quaternion frames and spectral units."""
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from analyze import metrics


class AnalysisTest(unittest.TestCase):
    def test_known_pitch_oscillation(self):
        rows = []
        for t in np.arange(0, 100, 0.1):
            pitch = 8 + 2 * np.sin(2 * np.pi * t)
            rotation = Rotation.from_euler("xyz", [0, pitch, 30], degrees=True)
            state = np.zeros(26)
            state[3:7] = rotation.as_quat()
            state[10:13] = rotation.apply([0, np.deg2rad(4 * np.pi * np.cos(2 * np.pi * t)), 0])
            rows.append(dict(time=float(t), truth=state.tolist(), command=[0.45, 10, 10],
                             altitude=60, targetAltitude=60, crossTrack=0))
        result = metrics(rows)
        self.assertAlmostEqual(result["pitch_ac_deg"], np.sqrt(2), delta=0.01)
        self.assertAlmostEqual(result["pitch_rate_deg_s"], 4 * np.pi / np.sqrt(2), delta=0.01)
        self.assertAlmostEqual(result["pitch_peak_hz"], 1, delta=0.04)
        self.assertAlmostEqual(result["pitch_band_rms_deg"], np.sqrt(2), delta=0.02)
        self.assertAlmostEqual(result["roll_rate_deg_s"], 0, delta=1e-10)
        self.assertEqual(result["altitude_rms_m"], 0)


if __name__ == "__main__":
    unittest.main()
