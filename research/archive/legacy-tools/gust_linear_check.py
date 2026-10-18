"""Local stability check of filtered pitch damping using saved coupled plants.
Sweeps lateral airspeed scaling 0.9–1.4; this is not a global stability proof.
"""
import argparse
import json
from pathlib import Path
import numpy as np

p = argparse.ArgumentParser()
p.add_argument('plants', type=Path)
p.add_argument('output', type=Path)
p.add_argument('--two-pole', action='store_true')
a = p.parse_args()
rows = []
for file in sorted(a.plants.glob('*/plant.npz')):
    name = file.parent.name
    plant = np.load(file)
    A, B, C = [plant[k] for k in ('A', 'B', 'C')]
    voltage = 20.4 if name == 'low_voltage' else 22.2
    ratio = 22.2 / voltage
    limit = int(min(60 * ratio, 100)) / 100
    kv = limit / 35
    gff = .18 if 'descent' in name else limit - .45
    for scale in (.9, 1.15, 1.4):
        n = len(A)
        coupled = np.zeros((n + 3, n + 3))
        coupled[:n, :n] = A
        wf = 2 * np.pi * 4
        coupled[n:n+2, :n] = wf * C[3:5]
        coupled[n:n+2, n:n+2] = -wf * np.eye(2)
        coupled[n+2, n] = -.002
        coupled[:n, n] = -scale * .005 * B[:, 1]
        coupled[:n, n+1] = -scale * .003 * B[:, 1]
        coupled[:n, n+2] = B[:, 1]
        b = np.r_[B[:, 0] * ratio, 0, 0, 0]
        c = np.pad(C, ((0, 0), (0, 3)))
        n = len(coupled)
        configs = ((0,.75),(.05,.75),(.1,.75),(.15,.75),(.1,1.),(.15,1.),(.2,1.),(.1,1.5)) if a.two_pole else ((0,.5),(.05,.5),(.075,.5),(.1,.5),(.1,.25))
        for gain, cutoff in configs:
            nf = 4 if a.two_pole else 3
            closed = np.zeros((n+nf, n+nf))
            closed[:n, :n] = coupled
            control = np.zeros(n+nf)
            control[:n] = -kv*c[0]
            control[n:n+2] = [(gff+kv)*.1, 1]
            control[-1] = -gain
            closed[:n] += np.outer(b, control)
            closed[n, :n] = -c[0]
            closed[n+1, :n] = -.3*kv*c[0]
            closed[n+1, n] = .3*kv*.1
            closed[n+2, :n] = 2*np.pi*cutoff*c[1]
            closed[n+2, n+2] = -2*np.pi*cutoff
            if a.two_pole:
                closed[n+3,n+2] = 2*np.pi*cutoff
                closed[n+3,n+3] = -2*np.pi*cutoff
            poles = np.linalg.eigvals(closed)
            slow = poles[(poles.imag > 1) & (poles.imag < 4)]
            rows.append(dict(case=name, airspeed_scale=scale, gain=gain, cutoff_hz=cutoff, filter_stages=2 if a.two_pole else 1,
                             max_real=float(poles.real.max()), worst_pole=[float(poles[poles.real.argmax()].real), float(poles[poles.real.argmax()].imag)],
                             slow_poles=[[float(z.real), float(z.imag)] for z in slow]))
a.output.write_text(json.dumps(rows, indent=2)+'\n')
print('Plants:', len(set(r['case'] for r in rows)), 'checks:', len(rows),
      'unstable:', sum(r['max_real'] >= 0 for r in rows))
for r in rows:
    if r['max_real'] >= 0:
        print(r)
