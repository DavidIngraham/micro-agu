"""Stability, tracking, sensing fidelity and retained-RPM shaft power."""
import importlib.util
import json
from pathlib import Path
import sys
import numpy as np
from scipy.spatial.transform import Rotation as R

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(HERE.parent/'controller-stability-2026-10-16'))
from analyze import metrics
from power import RPM, CP


def shaft_power(rpm, axial):
    valid = (rpm >= RPM[0]) & (rpm < RPM[-1])
    i = np.clip(np.searchsorted(RPM, rpm, side='right')-1, 0, 23)
    f = (rpm-RPM[i])/(RPM[i+1]-RPM[i])
    advance = np.clip(np.maximum(axial, 0)/(np.maximum(rpm, 1e-9)/60*.2286), 0, 2)*200
    k = np.minimum(advance.astype(int), 399); g = advance-k
    cp = (CP[i, k]*(1-g)+CP[i, k+1]*g)*(1-f)+(CP[i+1, k]*(1-g)+CP[i+1, k+1]*g)*f
    return np.where(valid, 1.225*(rpm/60)**3*.2286**5*cp, 0)


def assess(path, design):
    d = json.loads(path.read_text())
    rows = d['samples']
    result = dict(variant=path.parents[1].name, id=d['spec']['id'], mode=d['spec']['mode'],
                  complete=d['complete'], failure=d['failure'], path=str(path.relative_to(HERE)))
    if len(rows) < 20:
        return result
    for name, lo, hi in [('disturbed', 20, 100), ('early', 105, 135), ('late', 150, 180)]:
        subset = [s for s in rows if lo <= s['time'] < hi]
        result[name] = metrics(subset)
    selected = [s for s in rows if 20 <= s['time'] < 180]
    if len(selected) < 20:
        return result
    y = np.array([s['truth'] for s in selected]); wind = np.array([s['wind'] for s in selected])
    t = np.array([s['time'] for s in selected]); rpm = np.array([s['rpm'] for s in selected])
    rotation = R.from_quat(y[:, 3:7])
    axial = rotation.inv().apply(y[:, 7:10]-wind)[:, 0]
    power = shaft_power(rpm, axial)
    result['shaft_W'] = float(np.trapezoid(power, t)/(t[-1]-t[0]))
    result['shaft_Wh'] = float(np.trapezoid(power, t)/3600)
    distance = float(np.trapezoid(np.linalg.norm(y[:, 7:9], axis=1), t))
    result['ground_path_m'] = distance
    result['shaft_Wh_per_ground_km'] = result['shaft_Wh']*1000/distance
    result['mean_true_airspeed_m_s'] = float(np.trapezoid(np.linalg.norm(y[:, 7:10]-wind, axis=1), t)/(t[-1]-t[0]))
    result['delta_height_m'] = float(y[0, 2]-y[-1, 2])
    result['final_mission'] = rows[-1]['mission']
    brakes = np.array([s['brakes'] for s in selected])/(np.array([1300, 1500])*np.pi*55/360*.1)
    result['common_fraction'] = float(np.mean(brakes.min(axis=1)))
    result['upper_brake_fraction'] = float(np.mean(np.any(brakes > .99, axis=1)))
    result['max_abs_pitch_deg'] = float(np.max(abs(rotation.as_euler('xyz', degrees=True)[:, 1])))
    diagnostics = [r for r in d['diagnostics'] if 20 <= r['time'] < 180]
    if diagnostics:
        v = np.array([r['values'] for r in diagnostics])
        assert v.shape[1] == 43
        result['sensor_use_fraction'] = float(np.mean(v[:, 42]))
        nearest = np.abs(t[:, None]-np.array([r['time'] for r in diagnostics])[None, :]).argmin(axis=0)
        truth = y[nearest]; wt = wind[nearest]
        rp, rc = R.from_quat(truth[:, 3:7]), R.from_quat(truth[:, 16:20])
        trim = R.from_quat(design['state'][3:7]).inv()*R.from_quat(design['state'][16:20])
        relative = trim.inv()*rp.inv()*rc
        measured_angles = v[:, 18:21]*design['XS'][2:5]
        angle_error = (relative.inv()*R.from_rotvec(measured_angles)).magnitude()
        result['canopy_angle_error_deg'] = float(np.rad2deg(np.sqrt(np.mean(angle_error**2))))
        base_rate = R.from_quat(design['state'][16:20]).inv().apply(design['state'][23:26])
        rate_error = v[:, 27:30]*design['XS'][11:14]+base_rate-rc.inv().apply(truth[:, 23:26])
        result['canopy_rate_error_deg_s'] = float(np.rad2deg(np.sqrt(np.mean(rate_error**2))))
        result['airspeed_error_m_s'] = float(np.sqrt(np.mean((v[:, 5]-np.linalg.norm(truth[:, 7:10]-wt, axis=1))**2)))
        result['true_speed_tracking_rms_m_s'] = float(np.sqrt(np.mean((v[:, 6]-np.linalg.norm(truth[:, 7:10]-wt, axis=1))**2)))
        p = [line.split(',') for line in d['logs'] if line.startswith('LQIP,')]
        result['proprio_modes'] = sorted(set(int(line[2]) for line in p))
    return result


def main():
    group = HERE/sys.argv[1]
    design = dict(np.load(HERE/(sys.argv[2] if len(sys.argv) > 2 else 'fixed-design')/'design.npz'))
    rows = [assess(p, design) for p in sorted(group.glob('*/results/*.json'))]
    (group/'analysis.json').write_text(json.dumps(rows, indent=2)+'\n')
    lines = ['# Sensing comparison', '', 'Power is modeled propeller shaft power from retained RPM, not battery power.', '',
             '| Mode | Variant | Complete | Pitch RMS ° | Alt RMS m | Xtrack RMS m | Late pitch RMS ° | Shaft W | Common brake % |',
             '|---|---|---:|---:|---:|---:|---:|---:|---:|']
    for mode in ['AUTO', 'CRUISE']:
        for variant in sorted(set(r['variant'] for r in rows)):
            all_rows = [r for r in rows if r['variant'] == variant and r['mode'] == mode]
            good = [r for r in all_rows if r['complete']]
            if not good:
                continue
            med = lambda f: np.median([f(r) for r in good])
            values = [med(lambda r: r['disturbed']['pitch_ac_deg']), med(lambda r: r['disturbed']['altitude_rms_m']),
                      med(lambda r: r['disturbed']['xtrack_rms_m']), med(lambda r: r['late']['pitch_ac_deg']),
                      med(lambda r: r['shaft_W']), 100*med(lambda r: r['common_fraction'])]
            lines.append(f'| {mode} | {variant} | {len(good)}/{len(all_rows)} | '+' | '.join(f'{x:.3f}' for x in values)+' |')
    lines += ['', '## Sensor fidelity', '', '| Variant | Canopy angle error ° | Canopy rate error °/s | Airspeed error m/s | Airspeed use |',
              '|---|---:|---:|---:|---:|']
    for variant in sorted(set(r['variant'] for r in rows)):
        good = [r for r in rows if r['variant'] == variant and r['complete'] and 'sensor_use_fraction' in r]
        if good:
            lines.append('| '+variant+' | '+' | '.join(f'{np.median([r[k] for r in good]):.3f}' for k in
                ['canopy_angle_error_deg', 'canopy_rate_error_deg_s', 'airspeed_error_m_s', 'sensor_use_fraction'])+' |')
    lines += ['', '## Failures', '']+[f'- {r["variant"]} {r["id"]}: {r["failure"]}' for r in rows if not r['complete']]
    (group/'RESULTS.md').write_text('\n'.join(lines)+'\n')
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
