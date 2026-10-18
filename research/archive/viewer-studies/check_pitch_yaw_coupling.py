"""Isolate rotor angular-momentum coupling in the open-loop 9-degree pitch releases."""
import argparse
import json
import sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--model-dir', type=Path, required=True)
p.add_argument('--raw', type=Path, required=True, help='Native closed-suite directory containing paired release states')
p.add_argument('--output', type=Path, required=True)
a = p.parse_args()
sys.path.insert(0, str(a.model_dir))
from microagu_sim.compat.json_backend import Backend
from microagu_sim.compat.actuator import BrakeActuator
rows = []
for body in ['payload', 'canopy']:
    source = a.raw / f'mission-{body}-pitch-9deg/result.json'
    f = json.loads(source.read_text())['traces']['open'][0]
    results = []
    for enabled in [True, False]:
        b = Backend(profile='mission')
        b.state = np.array(f['state']); b.rpm = f['rpm']
        b.command = np.array([f['throttle'], *f['brake_mm']])
        b.actuator = BrakeActuator(position_mm=f['brake_mm'])
        if not enabled:
            b.sim.parameters['rotor_spin_inertia'] = 0
        history = []
        for i in range(801):
            rp, rc = R.from_quat(b.state[3:7]), R.from_quat(b.state[16:20])
            history.append([rp.as_euler('xyz')[2], rc.as_euler('xyz')[2], (rp.inv()*rc).as_rotvec()[2]])
            if i < 800:
                b.state, b.rpm, _ = b.sim.step_actuated(b.state, b.rpm, b.actuator, b.command, voltage=22.2, dt=.01)
        h = np.rad2deg(np.unwrap(np.array(history), axis=0)); h -= h[0]
        results.append(np.max(abs(h), axis=0).tolist())
    rows.append(dict(disturbed_body=body, normal_max_abs_change_deg=results[0], no_rotor_momentum_max_abs_change_deg=results[1]))
result = dict(profile='mission', amplitude_deg=9, duration_s=8, dt_s=.01,
              columns=['payload Euler yaw', 'canopy Euler yaw', 'relative rotation-vector yaw component'],
              intervention='Set rotor_spin_inertia=0; removes gyroscopic and spin-acceleration reaction terms. Thrust, shaft reaction torque, locked-body inertia, initial state, RPM and controls retained.',
              cases=rows)
a.output.parent.mkdir(parents=True, exist_ok=True)
a.output.write_text(json.dumps(result, indent=2)+'\n')
print(json.dumps(result, indent=2))
