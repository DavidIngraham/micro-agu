"""Matched joint releases through settled native ArduPilot AUTO, with a frozen-command shadow.

AI-assisted research harness. Does not modify firmware or controller equations.
Run from the built ArduPilot checkout in its autotest environment.
"""
import argparse
import copy
import json
import os
from pathlib import Path
import runpy
import sys
import threading
import numpy as np
from scipy.spatial.transform import Rotation as R

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--model-dir', type=Path, required=True)
p.add_argument('--output', type=Path, required=True)
p.add_argument('--body', choices=['canopy', 'payload'], default='payload')
p.add_argument('--axis', choices=['roll', 'pitch', 'yaw'], default='pitch')
p.add_argument('--amplitude', type=float, default=9)
p.add_argument('--profile', choices=['mission', 'dynamic_roll'], default='mission')
a, forwarded = p.parse_known_args()
if forwarded[:1] == ['--']: forwarded = forwarded[1:]
sys.path.insert(0, str(a.model_dir.resolve()))
os.environ['MICROAGU_SIM_ROOT']=str(a.model_dir.resolve())
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'research/archive/sitl-studies'))
import compare_autotest as harness
from microagu_sim.compat.json_backend import Backend
os.environ['PG_PROFILE'] = a.profile
os.environ['PG_CONTROLLER_CONFIG'] = str(a.model_dir / 'mission-controller-candidate.json')
os.environ['PG_GAINS'] = 'original'
original_respond = Backend.respond
lock = threading.RLock()
active = False
flying = False
start = None
shadow = None
traces = {'closed': [], 'open': []}
last_sample = -1.

def perturb(b):
    y = b.state.copy()
    rp, rc = R.from_quat(y[3:7]), R.from_quat(y[16:20])
    hp, hc = np.array(b.sim.geometry['hinge']), np.array([0., 0., .85])
    hinge = y[:3] + rp.apply(hp)
    velocity = y[7:10] + np.cross(y[10:13], rp.apply(hp))
    delta = np.zeros(3); delta[['roll', 'pitch', 'yaw'].index(a.axis)] = np.deg2rad(a.amplitude)
    if a.body == 'payload':
        rp = rp * R.from_rotvec(delta)
        y[3:7] = rp.as_quat(); y[:3] = hinge-rp.apply(hp)
        y[7:10] = velocity-np.cross(y[10:13], rp.apply(hp))
    else:
        rc = rp * R.from_rotvec(delta) * rp.inv() * rc
        y[16:20] = rc.as_quat(); y[13:16] = hinge-rc.apply(hc)
        y[20:23] = velocity-np.cross(y[23:26], rc.apply(hc))
    b.state = y

def record(b, elapsed):
    f = b.viewer_state(); f['time_s'] = elapsed
    traces['closed' if b is not shadow else 'open'].append(f)

def observe(b, packet):
    global last_sample
    with lock:
        # Keep moving trim during setup, then let AUTO fly before perturbing.
        if not flying: b.released = False
        old_time = b.time
        response = original_respond(b, packet)
        if active and b.time > old_time and b.time-start <= 8.003:
            dt = b.time-old_time
            shadow.sim.substeps = b.sim.substeps
            shadow.state, shadow.rpm, _ = shadow.sim.step_actuated(
                shadow.state, shadow.rpm, shadow.actuator, shadow.command, voltage=shadow.voltage, dt=dt)
            shadow.time = b.time
            elapsed = b.time-start
            if elapsed >= last_sample + .05 - 1e-8:
                record(b, elapsed); record(shadow, elapsed); last_sample = elapsed
        return response
Backend.respond = observe

def test(self):
    global active, flying, start, shadow, last_sample
    try:
        harness.airborne(self, altitude=100)
        b = self._pg_backend[0]
        waypoint = harness.mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
        route = [(waypoint, 1000, 0, 100), (waypoint, 2000, 0, 100)]
        self.check_mission_upload_download(self.create_simple_relhome_mission(route))
        self.change_mode('AUTO')
        with lock:
            b.released = True; flying = True
        self.delay_sim_time(100, reason='Settle native AUTO on the straight northbound mission leg')
        self.wait_current_waypoint(1, timeout=5)
        self.wait_mode('AUTO')
        parameters = self.get_parameters(list(json.loads((a.model_dir/'mission-controller-candidate.json').read_text())['parameters']))
        with lock:
            perturb(b)
            shadow = copy.deepcopy(b)
            start = b.time
            record(b, 0.); record(shadow, 0.); last_sample = 0.
            b.released = True; active = True
        self.delay_sim_time(8.1, reason='Record matched joint releases')
        with lock:
            active = False
            assert len(traces['closed']) == 161, len(traces['closed'])
            assert not self._pg_backend[3], self._pg_backend[3]
            assert b.clipped_packets == 0, b.clipped_packets
            result = dict(body=a.body, axis=a.axis, amplitude_deg=a.amplitude, profile=a.profile,
                          hinge=b.sim.geometry['hinge'], parameters=parameters, mode='AUTO', settle_s=100, route=route,
                          traces=traces, k=[b.sim.parameters['joint_k_'+x] for x in ['roll','pitch','yaw']],
                          c=[b.sim.parameters['joint_c_'+x] for x in ['roll','pitch','yaw']])
            a.output.write_text(json.dumps(result, allow_nan=False, default=lambda x: x.tolist()))
    finally:
        if hasattr(self, '_pg_backend'):
            b, stop, thread, errors, samples = self._pg_backend
            self.reset_SITL_commandline()
            stop.set(); thread.join(timeout=2)

test.__name__ = 'ParagliderAutoMission'
test.__doc__ = 'Settled native AUTO joint release (not the AUTO mission acceptance test)'
harness.arduplane.AutoTestPlane.ParagliderAutoMission = test
sys.argv = [str(harness.ROOT/'Tools/autotest/autotest.py'), *forwarded]
runpy.run_path(sys.argv[0], run_name='__main__')
