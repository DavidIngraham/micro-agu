"""Reproducible sampled-data controller study; no production parameters are edited.

Run with the ArduPilot Python environment. See README.md for stages and assumptions.
"""
import argparse
import hashlib
import json
import os
import sys
from pathlib import Path

import numpy as np
from scipy.linalg import solve_discrete_are, solve_discrete_lyapunov
from scipy.optimize import differential_evolution
from scipy.spatial.transform import Rotation as R

DT = .02
HERE = Path(__file__).resolve().parent
TRACES = Path(os.environ.get('MICROAGU_STUDY_TRACES', str(HERE/'traces'))).resolve()
NAMES = ['payload roll', 'payload pitch', 'relative roll', 'relative pitch',
         'relative yaw', 'forward velocity', 'side velocity', 'vertical velocity',
         'payload p', 'payload q', 'payload r', 'canopy p', 'canopy q', 'canopy r',
         'rotor / 1000 RPM', 'height']
# Coordinates are normalized by engineering reference excursions, not an energy metric.
XS = np.r_[np.full(5, np.deg2rad(10)), np.full(3, .5),
           np.full(6, np.deg2rad(30)), 1., 1.]
US = np.array([.1, .1])
BASE = dict(kp=.005, ki=.002, rd=.003, lat_hz=1., qgain=.1,
            qhz=1.5, kv=.6/17.5, kh=.1)
KEYS = list(BASE)
CASES = [('level', 'mission', 0., 22.2), ('climb', 'mission', .5, 22.2),
         ('descent', 'mission', -.5, 22.2), ('low_voltage', 'mission', 0., 20.4),
         ('dynamic_roll', 'dynamic_roll', 0., 22.2),
         ('dynamic_climb', 'dynamic_roll', .5, 22.2)]

def save(name, obj):
    (HERE / name).write_text(json.dumps(obj, indent=2, allow_nan=False) + '\n')

def source_hashes():
    """Record the Rust checkout and actual installed Python extension/resources."""
    from microagu_sim import _native, compat
    source = HERE.parents[1] / 'simulation'
    files = [source/'Cargo.toml', source/'Cargo.lock', *sorted((source/'crates').glob('*/src/**/*.rs'))]
    hashes = {str(p.relative_to(source)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    hashes['installed_native'] = hashlib.sha256(Path(_native.__file__).read_bytes()).hexdigest()
    for p in sorted(Path(compat.__file__).parent.iterdir()):
        if p.suffix in {'.py', '.json', '.npz'}:
            hashes['installed_compat/' + p.name] = hashlib.sha256(p.read_bytes()).hexdigest()
    return hashes

def setup(model=None):
    """The model argument is retained for old callers; uv selects the Rust wheel."""
    from microagu_sim.compat.profiles import make_simulator
    return make_simulator

class Chart:
    def __init__(self, sim, state, rpm):
        self.sim, self.base, self.rpm = sim, state.copy(), rpm
        self.base[:3] -= state[:3]
        self.base[13:16] -= state[:3]
        self.rp = R.from_quat(state[3:7]); self.rc = R.from_quat(state[16:20])
        self.euler = self.rp.as_euler('xyz'); self.rel = self.rp.inv()*self.rc
        self.v = self.rp.inv().apply(state[7:10])
        self.p = self.rp.inv().apply(state[10:13]); self.c = self.rc.inv().apply(state[23:26])

    def decode(self, x):
        rp = R.from_euler('xyz', self.euler + np.r_[x[:2], 0.])
        rc = rp*self.rel*R.from_rotvec(x[2:5]); y = self.base.copy()
        y[2] = -x[15]; y[3:7] = rp.as_quat(); y[16:20] = rc.as_quat()
        y[7:10] = rp.apply(self.v+x[5:8]); y[10:13] = rp.apply(self.p+x[8:11])
        y[23:26] = rc.apply(self.c+x[11:14])
        ap, ac = rp.apply(self.sim.geometry['hinge']), rc.apply([0, 0, .85])
        y[13:16] = y[:3]+ap-ac
        y[20:23] = y[7:10]+np.cross(y[10:13], ap)-np.cross(y[23:26], ac)
        return y, self.rpm+1000*x[14]

    def encode(self, y, rpm):
        rp, rc = R.from_quat(y[3:7]), R.from_quat(y[16:20])
        return np.r_[rp.as_euler('xyz')[:2]-self.euler[:2],
                     (self.rel.inv()*rp.inv()*rc).as_rotvec(),
                     rp.inv().apply(y[7:10])-self.v, rp.inv().apply(y[10:13])-self.p,
                     rc.inv().apply(y[23:26])-self.c, (rpm-self.rpm)/1000, -y[2]]

def observations(y):
    rp = R.from_quat(y[3:7]); phi, theta, _ = rp.as_euler('xyz')
    p, q, r = rp.inv().apply(y[10:13])
    return np.array([-y[9], q, rp.inv().apply(y[23:26]-y[10:13])[1],
                     np.rad2deg((q*np.sin(phi)+r*np.cos(phi))/np.cos(theta)),
                     np.rad2deg(p), -y[2]])

def prepare(model, cache):
    make = setup(model)
    from microagu_sim.compat.equilibrium import solve
    records = []
    for name, profile, climb, voltage in CASES:
        sim = make('trout_lake', profile); sim.substeps = 16
        seed = np.load(cache/name/'trim.npz' if cache else HERE/(name+'.npz'))
        trim = solve(sim, seed['state'], seed['command'], climb=climb, voltage=voltage)
        residual = float(np.max(abs(trim['residual'])))
        assert residual < 1e-6, (name, residual)
        chart = Chart(sim, trim['state'], trim['rpm']); command = trim['command']
        assert command[1] == 0 and command[2] > .5
        stroke = np.pi*55/360*.1*1500
        def advance(x, u):
            y, rpm = chart.decode(x); cmd = command+np.array([u[0], 0, stroke*u[1]])
            yn, rn = sim.step(y, rpm, cmd, voltage=voltage, wind=u[2:], dt=DT)
            result = chart.encode(yn, rn); result[15] -= climb*DT
            return result
        def jac(eps):
            A = np.column_stack([(advance(e, np.zeros(5))-advance(-e, np.zeros(5)))/(2*eps) for e in np.eye(16)*eps])
            B = np.column_stack([(advance(np.zeros(16), e)-advance(np.zeros(16), -e))/(2*eps) for e in np.eye(5)*eps])
            C = np.column_stack([(observations(chart.decode(e)[0])-observations(chart.decode(-e)[0]))/(2*eps) for e in np.eye(16)*eps])
            return A, B, C
        A, B, C = jac(1e-5); A2, B2, C2 = jac(5e-6)
        record = dict(name=name, profile=profile, climb=climb, voltage=voltage,
                      trim_residual=residual, zero_drift=float(np.linalg.norm(advance(np.zeros(16), np.zeros(5)))),
                      jacobian_relative_error=float(np.linalg.norm(A-A2)/np.linalg.norm(A)),
                      input_jacobian_relative_error=float(np.linalg.norm(B-B2)/np.linalg.norm(B)),
                      command=command.tolist(), speed=float(np.linalg.norm(chart.base[7:10])))
        np.savez_compressed(HERE/(name+'.npz'), A=A, B=B[:, :2], E=B[:, 2:], C=C,
                            state=chart.base, command=command, rpm=trim['rpm'])
        records.append(record); print(record, flush=True)
    hashes = source_hashes()
    save('plants.json', dict(dt=DT, states=NAMES, state_scales=XS.tolist(), input_scales=US.tolist(),
                            source_hashes=hashes, cases=records))

def plants():
    records = json.loads((HERE/'plants.json').read_text())['cases']
    return [{**r, **dict(np.load(HERE/(r['name']+'.npz')))} for r in records]

def controller(p, cfg):
    """Linear sampled controller: update filters/integrators, then hold command.
    State order: heading LPF, roll LPF, turn I, throttle I, q1, q2, height-reference
    LPF, turn-reference LPF. No actuator limits are active in this local map.
    """
    C = p['C']; ratio = 22.2/p['voltage']; limit = int(min(60*ratio, 100))/100
    ff = .18 if p['climb'] < 0 else limit-.45
    kv = cfg['kv']*limit/.6; ki = .3*kv; scale = 5/p['speed']
    def update(x, r):
        y = C@x[:16]; z = x[16:].copy()
        a = DT/(DT+1/(2*np.pi*cfg['lat_hz']))
        z[:2] += a*(y[3:5]-z[:2])
        z[7] += DT/(DT+.1)*(r[1]-z[7])
        error = z[7]-z[0]; z[2] += cfg['ki']*DT*error
        brake = scale*(.009*z[7]+cfg['kp']*error-cfg['rd']*z[1])+z[2]
        z[6] += DT/(DT+.5)*(r[0]-z[6]); rate = (r[0]-z[6])/.5
        vd = cfg['kh']*(r[0]-y[5])+rate; ev = vd-y[0]
        z[3] += ki*DT*ev
        aq = DT/(DT+1/(2*np.pi*cfg['qhz']))
        z[4] += aq*(y[1]-z[4]); z[5] += aq*(z[4]-z[5])
        throttle = ratio*(ff*vd+kv*ev+z[3]-cfg['qgain']*z[5])
        return z, np.array([throttle, brake])
    z0 = np.zeros(24); r0 = np.zeros(2)
    zx, ux = zip(*(update(e, r0) for e in np.eye(24)))
    zr, ur = zip(*(update(z0, e) for e in np.eye(2)))
    Z, U, Zr, Ur = np.array(zx).T, np.array(ux).T, np.array(zr).T, np.array(ur).T
    F = np.vstack((np.c_[p['A'], np.zeros((16, 8))]+p['B']@U, Z))
    G = np.vstack((p['B']@Ur, Zr)); W = np.vstack((p['E'], np.zeros((8, 3))))
    return F, G, W, U, Ur

def output_matrix(p, U):
    # Cost weights: 1 m altitude, 10 deg/s heading, .5 m/s climb,
    # 30 deg/s pitch and relative pitch, and 10 percentage points control.
    Y = np.zeros((7, 24))
    Y[:5, :16] = p['C'][[5, 3, 0, 1, 2]]/np.array([1, 10, .5, np.deg2rad(30), np.deg2rad(30)])[:, None]
    Y[5:] = U/US[:, None]
    return Y

def performance(p, cfg):
    F, G, W, U, Ur = controller(p, cfg)
    radius = max(abs(np.linalg.eigvals(F)))
    if radius >= 1: return None
    Y = output_matrix(p, U)
    # Finite step-regulation cost about each new equilibrium, not a command-noise H2 norm.
    equilibrium = np.linalg.solve(np.eye(24)-F, G@np.diag([1., 10.]))
    P = solve_discrete_lyapunov(F, equilibrium@equilibrium.T, method='bilinear')
    step_components = DT*np.diag(Y@P@Y.T)
    step = sum(step_components)
    # Stationary sinusoidal wind response, with existing illustrative gust spectrum.
    freq = np.array([.08, .13, .21, .34, .55, .89])
    weights = np.array([1, 1, .8, .6, .4, .25]); weights /= np.linalg.norm(weights)
    amp = np.array([.5, .5, .25])
    gust = 0.
    for f, w in zip(freq, weights):
        T = Y@np.linalg.solve(np.exp(2j*np.pi*f*DT)*np.eye(24)-F, W)
        gust += np.sum(abs(T*amp)**2)*w*w
    return dict(step=float(step), step_components=step_components.tolist(), gust=float(gust), pole_real=float(np.log(radius)/DT))

def optimize(constrained=False):
    ps = plants(); training = [p for p in ps if p['name'] != 'dynamic_climb']
    base = {p['name']: performance(p, BASE) for p in ps}
    assert all(v is not None for v in base.values())
    # Deliberately bounded search; FF and integrator ratio remain unchanged.
    bounds = [(.002, .009), (.0005, .004), (.001, .007), (.5, 2.),
              (.03, .18), (.6, 2.5), (.015, .10), (.06, .18)]
    count = 0; best = 1e10
    def objective(v):
        nonlocal count, best
        cfg = dict(zip(KEYS, v)); ratios = []
        for p in training:
            value = performance(p, cfg)
            if value is None: return 1000.
            b = base[p['name']]
            if constrained:
                violations = [value['gust']/b['gust']-1.,
                              value['step_components'][0]/b['step_components'][0]-1.,
                              value['step_components'][1]/b['step_components'][1]-1.,
                              value['pole_real']- .8*b['pole_real']]
                if max(violations) > 1e-6:
                    return 10.+100*sum(max(v, 0.) for v in violations)
            ratios.append(.5*value['step']/b['step']+.5*value['gust']/b['gust'])
        score = .5*np.mean(ratios)+.5*max(ratios)
        count += 1
        if score < best:
            best = score
            if count % 10 == 0: print('best', count, score, cfg, flush=True)
        return score
    result = differential_evolution(objective, bounds, seed=20261014, maxiter=45,
                                    popsize=7, tol=.002, polish=True, x0=list(BASE.values()))
    cfg = dict(zip(KEYS, result.x)); rows = []
    for p in ps:
        rows.append(dict(case=p['name'], held_out=p['name']=='dynamic_climb',
                         baseline=base[p['name']], candidate=performance(p, cfg)))
    save('optimization.json' if constrained else 'optimization-effort.json', dict(baseline=BASE, candidate=cfg, bounds=dict(zip(KEYS, bounds)),
                                  objective=float(result.fun), evaluations=int(result.nfev),
                                  success=bool(result.success), message=str(result.message), constrained=constrained, rows=rows))
    print(json.dumps(rows, indent=2), flush=True)

if __name__ == '__main__':
    ap = argparse.ArgumentParser(); ap.add_argument('stage', choices=['prepare', 'optimize'])
    ap.add_argument('--model', type=Path); ap.add_argument('--cache', type=Path); ap.add_argument('--constrained', action='store_true')
    args = ap.parse_args()
    if args.stage == 'prepare': prepare(args.model, args.cache)
    else: optimize(args.constrained)
