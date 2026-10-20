"""Independent boundary/convergence checks for first-contact measurements."""
import unittest
import numpy as np
from native import run
from assess import assess


class HarnessTests(unittest.TestCase):
    def test_first_contact_convergence(self):
        spec = dict(kind="landing", height=30, flare_height=1, flare_brake=36)
        coarse = run(spec, .00125)
        fine = run(spec, .000625)
        a, b = [x["metrics"]["contact"] for x in (coarse, fine)]
        self.assertLess(abs(a["time"]-b["time"]), .01)
        self.assertLess(abs(a["sink"]-b["sink"]), .03)
        self.assertLess(np.linalg.norm(np.array(a["position"])-b["position"]), .05)
        # Incoming velocity must not have been erased by ground impulse.
        self.assertGreater(a["sink"], .5)
        self.assertGreater(-a["position"][2], .1)
        self.assertLess(abs(a["pitch_deg"]), 30)

    def test_constant_wind_galilean_invariance(self):
        spec = dict(kind="landing", height=30, flare_height=1, flare_brake=36)
        a = run(spec)["metrics"]["contact"]
        b = run(dict(spec, wind=[1, 0, 0]))["metrics"]["contact"]
        self.assertAlmostEqual(a["time"], b["time"], places=6)
        self.assertAlmostEqual(a["sink"], b["sink"], places=5)
        # First-contact state is from the previous interval, not endpoint.
        self.assertAlmostEqual(b["position"][0]-a["position"][0], a["time"]-.00125, places=4)

    def test_recovery_metric_requires_sustained_settling(self):
        rows = []
        for t in np.arange(0, 40, .1):
            y = np.zeros(26)
            y[6] = y[19] = 1
            y[2] = -60 if t < 8 or t >= 30 else -55
            rows.append(dict(time=float(t), state=y.tolist(), active=True))
        result = assess(dict(spec=dict(id="synthetic", kind="recovery"), samples=rows,
                             complete=True, failure=None, touchdown=None))
        self.assertAlmostEqual(result["altitude_loss"], 5)
        self.assertAlmostEqual(result["recovery_time"], 35)


if __name__ == "__main__":
    unittest.main()
