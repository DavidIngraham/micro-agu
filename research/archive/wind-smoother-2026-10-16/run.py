"""Audit historical flight wind using robust smoothing; never changes model parameters."""
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from smoother import smooth

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
DATA = Path('/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/longitudinal-2026-10-09')
FLIGHTS = {'hood_river': ('2026-01-03', '2026-01-03 15-41-42', [.000421891213389118, -.0001099581589958159, -.03929616736401672], [300,360,480,600,690,720,930,1020]),
           'trout_lake': ('2026-02-15', '2026-02-15 15-38-50', None, [600,840,900,1350,1920,2400,2790,2910])}


def stats(x):
    return dict(median=float(np.median(x)), p90=float(np.quantile(x, .9)), maximum=float(np.max(x)))


def validation():
    rng = np.random.default_rng(72)
    rows = []
    for name in ['turning', 'straight', 'changing-wind', 'biased-attitude']:
        t = np.arange(300.)
        yaw = np.zeros(len(t)) if name == 'straight' else .04*t
        euler = np.column_stack((np.zeros((len(t), 2)), yaw))
        wind = np.tile([1.2, -.6], (len(t), 1))
        if name == 'changing-wind':
            wind[:, 0] += .5*np.sin(t/60)
        velocity = np.column_stack((4*np.cos(yaw), 4*np.sin(yaw), np.zeros(len(t))))
        velocity[:, :2] += wind
        velocity += rng.normal(0, .15, velocity.shape)
        if name == 'biased-attitude':
            euler[:, 2] += np.deg2rad(3)
        out = smooth(t, velocity, euler)
        error = np.linalg.norm(out['wind']-wind, axis=1)
        major = np.sqrt(np.linalg.eigvalsh(out['covariance'])[:, -1])
        row = dict(case=name, vector_error=stats(error), major_sigma=stats(major))
        if name == 'straight':
            assert np.median(major) > 4.9, 'False certainty along unobservable heading'
        elif name != 'biased-attitude':
            assert np.median(error) < .25, row
        rows.append(row)
    # Actual 9-DOF truth trajectories, including physical sideslip and weather.
    folder = ROOT / 'research/archive/low-speed-ekf-2026-10-16/baseline/results'
    for path in sorted(folder.glob('*-lqi.json')):
        d = json.loads(path.read_text())
        samples = d['samples'][::10]
        t = np.array([s['time'] for s in samples])
        state = np.array([s['truth'] for s in samples])
        wind = np.array([s['wind'] for s in samples])
        euler = Rotation.from_quat(state[:, 3:7]).as_euler('xyz')
        velocity = state[:, 7:10]+rng.normal(0, .15, (len(t), 3))
        out = smooth(t, velocity, euler)
        rows.append(dict(case=f'9dof-{d["spec"]["id"]}', mode=d['spec']['mode'],
                         vector_error=stats(np.linalg.norm(out['wind']-wind[:, :2], axis=1)),
                         weak_heading_fraction=float(np.mean(out['observability'] < .1)),
                         supported_error=stats(np.linalg.norm(out['wind']-wind[:, :2], axis=1)[out['observability'] >= .1]) if np.any(out['observability'] >= .1) else None,
                         major_sigma=stats(np.sqrt(np.linalg.eigvalsh(out['covariance'])[:, -1]))))
    (HERE / 'validation.json').write_text(json.dumps(rows, indent=2)+'\n')
    return rows


def main():
    geometry = json.loads((HERE / 'geometry.json').read_text())
    validation_rows = validation()
    report, provenance = {}, {}
    for name, (day, log, _, starts) in FLIGHTS.items():
        path = DATA / 'turn-model-spike' / f'{day}-causal-input.npz'
        aligned_path = DATA / log / 'aligned.npz'
        for p in [path, aligned_path]:
            provenance[str(p)] = hashlib.sha256(p.read_bytes()).hexdigest()
        a, flags = np.load(path), np.load(aligned_path)
        ekf_path = DATA / log / 'XKF2.json'
        provenance[str(ekf_path)] = hashlib.sha256(ekf_path.read_bytes()).hexdigest()
        ekf = [r for r in json.loads(ekf_path.read_text()) if r['C'] == 0]
        ekf_t = np.array([r['TimeUS']/1e6 for r in ekf])
        ekf_w = np.array([[r['VWN'], r['VWE']] for r in ekf])
        t, y = a['t'], a['Y']
        # Valid airborne runs only; never interpolate across ground gaps.
        flying = np.interp(t, flags['t'], flags['airborne'].astype(float), left=0, right=0) > .99
        mask = flying & a['valid'].astype(bool) & np.isfinite(y).all(axis=1)
        edges = np.flatnonzero(np.diff(np.r_[False, mask, False]))
        segments = [(lo, hi) for lo, hi in zip(edges[::2], edges[1::2]) if t[hi-1]-t[lo] >= 30]
        collected, windows = [], []
        fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True, constrained_layout=True)
        for number, (lo, hi) in enumerate(segments):
            ix = np.arange(lo, hi, 25)  # original 25 Hz data -> 1 Hz, limit pseudo-replication
            tt, yy = t[ix], y[ix]
            rot = Rotation.from_euler('xyz', yy[:, :3])
            omega = rot.apply(yy[:, 3:6])
            velocity = yy[:, 6:9] - np.cross(omega, rot.apply(np.tile(geometry[name]['imu'], (len(ix), 1))))
            old = np.column_stack([np.interp(tt, ekf_t, ekf_w[:, j]) for j in range(2)])
            calibration_wind = a['wind'][ix, :2]
            base = smooth(tt, velocity, yy[:, :3])
            alternatives = [smooth(tt, velocity, yy[:, :3], **kw)['wind'] for kw in
                            [dict(diffusion=.02), dict(diffusion=.1), dict(beta_deg=-3), dict(beta_deg=3),
                             dict(yaw_deg=-5), dict(yaw_deg=5), dict(wind_down=-1), dict(wind_down=1),
                             dict(bias_sigma=.001), dict(bias_sigma=1.)]]
            ensemble = np.array([base['wind'], *alternatives])
            spread = np.max(np.linalg.norm(ensemble-base['wind'][None, :, :], axis=2), axis=0)
            error = np.linalg.norm(base['wind']-old, axis=1)
            calibration_error = np.linalg.norm(base['wind']-calibration_wind, axis=1)
            old_air = velocity-np.column_stack((calibration_wind, a['wind'][ix, 2]))
            new_air = velocity-np.column_stack((base['wind'], np.zeros(len(ix))))
            old_speed, new_speed = np.linalg.norm(old_air, axis=1), np.linalg.norm(new_air, axis=1)
            sigma = np.sqrt(np.linalg.eigvalsh(base['covariance'])[:, -1])
            assert np.isfinite(base['wind']).all() and np.linalg.eigvalsh(base['covariance']).min() > 0
            np.savez_compressed(HERE / f'{name}-{number}.npz', t=tt, velocity=velocity, attitude=yy[:, :3],
                                ekf_wind=old, calibration_wind=calibration_wind, **base, alternatives=ensemble, sensitivity=spread)
            collected.append(dict(t=tt, error=error, old_speed=old_speed, new_speed=new_speed,
                                  calibration_error=calibration_error,
                                  sigma=sigma, spread=spread, obs=base['observability']))
            for start in starts:
                if tt[0] <= start and start+10 <= tt[-1]:
                    selected = (tt >= start) & (tt < start+10)
                    windows.append(dict(start=start, discrepancy=float(np.mean(error[selected])),
                                        calibration_speed=float(np.mean(old_speed[selected])),
                                        smoother_speed=float(np.mean(new_speed[selected])),
                                        dynamic_pressure_ratio=float(np.mean((new_speed[selected]/old_speed[selected])**2)),
                                        sensitivity=float(np.max(spread[selected]))))
            for j in range(2):
                axes[j].plot(tt, old[:, j], color='tab:blue', label='Logged EKF' if number == 0 else None)
                axes[j].plot(tt, base['wind'][:, j], color='tab:orange', label='Smoother' if number == 0 else None)
                axes[j].plot(tt, calibration_wind[:, j], color='tab:gray', linestyle='--', label='Calibration assumption' if number == 0 else None)
                axes[j].fill_between(tt, ensemble[:, :, j].min(axis=0), ensemble[:, :, j].max(axis=0), color='tab:orange', alpha=.2)
            axes[2].plot(tt, error, color='tab:purple', label='Wind discrepancy' if number == 0 else None)
            axes[2].plot(tt, sigma, color='tab:green', label='Conditional major-axis sigma' if number == 0 else None)
            axes[3].plot(tt, base['observability'], color='black')
        axes[0].set_title(name+' — shading is assumption sensitivity, not a confidence interval')
        for ax, label in zip(axes, ['North wind (m/s)', 'East wind (m/s)', 'Magnitude (m/s)', 'Local heading information ratio']):
            ax.set_ylabel(label); ax.grid(alpha=.2)
        axes[0].legend(); axes[2].legend(); axes[-1].set_xlabel('Autopilot boot time (s)')
        fig.savefig(HERE / f'{name}.png', dpi=160); plt.close(fig)
        merge = {k: np.concatenate([r[k] for r in collected]) for k in collected[0]}
        report[name] = dict(segments=len(segments), samples=len(merge['t']),
                           discrepancy=stats(merge['error']), sensitivity=stats(merge['spread']),
                           calibration_discrepancy=stats(merge['calibration_error']),
                           supported_discrepancy=stats(merge['error'][merge['obs'] >= .1]),
                           conditional_major_sigma=stats(merge['sigma']),
                           post_first_minute_discrepancy=stats(merge['error'][merge['t'] >= merge['t'][0]+60]),
                           weak_heading_fraction=float(np.mean(merge['obs'] < .1)),
                           speed_change=stats(merge['new_speed']-merge['old_speed']), windows=windows)
    (HERE / 'results.json').write_text(json.dumps(report, indent=2)+'\n')
    (HERE / 'input-hashes.json').write_text(json.dumps(provenance, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    print('VALIDATION', json.dumps(validation_rows[:4]))


if __name__ == '__main__':
    main()
