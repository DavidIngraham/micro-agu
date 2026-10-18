"""Bridge backpressure, validation and record handling checks."""
import asyncio
import io
import json
import unittest
from bridge import Receiver, validate


def frame(t=1):
    return dict(schema='microagu.state.v1', time_s=t, state=[0.0]*26, rpm=1000,
                voltage_v=22.2, brake_mm=[1, 2], throttle=.4,
                joint_rotvec_rad=[0, 0, 0], joint_rate_rad_s=[0, 0, 0], released=True)


class BridgeTests(unittest.TestCase):
    def test_latest_only_and_recording(self):
        q = asyncio.Queue(maxsize=1)
        record = io.StringIO()
        r = Receiver({q}, record)
        for i in range(100):
            r.datagram_received(json.dumps(frame(i)).encode(), ('127.0.0.1', 1234))
        self.assertEqual(q.qsize(), 1)
        self.assertEqual(json.loads(q.get_nowait())['time_s'], 99)
        self.assertEqual(len(record.getvalue().splitlines()), 100)
        r.datagram_received(json.dumps(frame(99)).encode(), ('127.0.0.1', 1234))
        self.assertEqual(r.received, 100)
        r.datagram_received(b'broken', ('127.0.0.1', 1234))
        self.assertEqual(r.received, 100)
        r.datagram_received(json.dumps(frame(0)).encode(), ('127.0.0.1', 1234))
        self.assertTrue(record.closed)
        self.assertIsNone(r.record)

    def test_invalid_values(self):
        for changes in [dict(time_s=float('nan')), dict(state=[0]), dict(brake_mm=[1]), dict(released='yes')]:
            with self.assertRaises(ValueError):
                validate(frame() | changes)


if __name__ == '__main__':
    unittest.main()
