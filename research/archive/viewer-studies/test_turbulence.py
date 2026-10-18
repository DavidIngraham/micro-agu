import json
import os
import struct
import sys
import unittest
import numpy as np
from turbulence import Gusts, respond_with_wind


class TurbulenceTest(unittest.TestCase):
    def test_repeatable_ramp_and_nominal_rms(self):
        a, b = Gusts(), Gusts()
        np.testing.assert_array_equal(a.at(0), [0, 0, 0])
        np.testing.assert_array_equal(a.at(12.34), b.at(12.34))
        # All chosen frequencies have an integer number of cycles in 100 s.
        values = np.array([a.at(t) for t in np.arange(100, 200, .01)])
        np.testing.assert_allclose(np.sqrt(np.mean(values**2, axis=0)), [.5, .5, .25], atol=1e-10)

    @unittest.skipUnless(os.environ.get('PG_MODEL_DIR'), 'Set PG_MODEL_DIR for native adapter check')
    def test_native_wind_sensor_consistency_and_retransmission(self):
        sys.path.insert(0, os.environ['PG_MODEL_DIR'])
        from microagu_sim.compat.json_backend import Backend
        b = Backend()
        b.released = True
        packet = lambda i: struct.pack('<HHI16H', 18458, 400, i, *([1450, 1500, 800, 2322] + [1500]*12))
        for i in range(100):
            response = respond_with_wind(b, packet(i), Backend.respond)
        message = json.loads(response)
        self.assertGreater(np.linalg.norm(message['velocity_wind']), 0)
        self.assertAlmostEqual(message['airspeed'], np.linalg.norm(b.state[7:10]-b._viewer_wind))
        state, time = b.state.copy(), b.time
        self.assertEqual(response, respond_with_wind(b, packet(99), Backend.respond))
        np.testing.assert_array_equal(state, b.state)
        self.assertEqual(time, b.time)


    @unittest.skipUnless(os.environ.get('PG_MODEL_DIR'), 'Set PG_MODEL_DIR for native adapter check')
    def test_steady_wind_reaches_physics_and_sensors_without_gusts(self):
        sys.path.insert(0, os.environ['PG_MODEL_DIR'])
        from microagu_sim.compat.json_backend import Backend
        b = Backend()
        b.released = True
        winds = []
        step = b.sim.step_actuated
        def observed_step(*args, **kwargs):
            winds.append(np.array(kwargs['wind']))
            return step(*args, **kwargs)
        b.sim.step_actuated = observed_step
        for i in range(20):
            packet = struct.pack('<HHI16H', 18458, 400, i, *([1450, 1500, 800, 2322] + [1500]*12))
            response = respond_with_wind(b, packet, Backend.respond, (0, 2, 0), False)
        self.assertTrue(winds)
        for wind in winds:
            np.testing.assert_array_equal(wind, [0, 2, 0])
        message = json.loads(response)
        self.assertEqual(message['velocity_wind'], [0, 2, 0])
        self.assertAlmostEqual(message['airspeed'], np.linalg.norm(b.state[7:10]-[0, 2, 0]))

if __name__ == '__main__':
    unittest.main()
