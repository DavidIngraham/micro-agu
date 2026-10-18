"""Summarize paired closed-loop runs; no probability claims for this scenario set."""
import importlib.util
import json
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('sensor_report', HERE.parent / 'airspeed-sensor-2026-10-16/report.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def main():
    groups = {name: module.load(HERE / name) for name in ['baseline', 'adaptive']}
    groups['sensor-reference'] = module.load(HERE.parent / 'airspeed-sensor-2026-10-16/eas-zero')
    lines = ['# Low-speed EKF comparison', '',
             'Fresh sensorless baseline and adaptive estimator, unchanged zero-trim LQI gains. '
             'Sensor reference is the prior campaign, not a fresh paired run.', '',
             'Tracking: 20–100 s. Late: 150–180 s. Power: 20–180 s. '
             'Medians across scenarios except common brake (mean) and pitch oscillation (maximum).', '',
             '| Configuration | Mode | Complete | TAS error RMS m/s | Common brake % | Max late pitch AC RMS deg | Alt RMS m | Xtrack RMS m | Shaft W | Wh/ground-km |',
             '|---|---|---:|---:|---:|---:|---:|---:|---:|---:|']
    for name, rows in groups.items():
        for mode in ['AUTO', 'CRUISE']:
            good = [r for r in rows if r['mode'] == mode and r['complete']]
            values = [np.median([r['airspeed_error_rms'] for r in good]),
                      100*np.mean([r['late']['additional_common_fraction'] for r in good]),
                      max(r['late']['pitch_ac_deg'] for r in good),
                      np.median([r['disturbed']['altitude_rms_m'] for r in good]),
                      np.median([r['disturbed']['xtrack_rms_m'] for r in good]),
                      np.median([r['combined_power']['estimated_shaft_W'] for r in good]),
                      np.median([r['combined_power']['estimated_shaft_Wh_per_ground_km'] for r in good])]
            lines.append(f'| {name} | {mode} | {len(good)}/8 | '+' | '.join(f'{v:.3f}' for v in values)+' |')
    lines += ['', '## Paired changes', '']
    for mode in ['AUTO', 'CRUISE']:
        a = [r for r in groups['adaptive'] if r['mode'] == mode and r['complete']]
        b = {r['id']: r for r in groups['baseline'] if r['complete']}
        for field in ['estimated_shaft_W', 'estimated_shaft_Wh_per_ground_km']:
            changes = [100*(r['combined_power'][field]/b[r['id']]['combined_power'][field]-1) for r in a]
            lines.append(f'- {mode} {field}: median {np.median(changes):+.2f}%; '
                         f'range {min(changes):+.2f} to {max(changes):+.2f}%; improved in {sum(x < 0 for x in changes)}/8.')
    lines += ['', '## Estimator diagnostics', '']
    for name, rows in groups.items():
        errors = [r['airspeed_error_rms'] for r in rows]
        lines.append(f'- {name}: late TAS RMS error median/P90/max '
                     f'{np.median(errors):.3f}/{np.quantile(errors, .9):.3f}/{max(errors):.3f} m/s; '
                     f'{sum(r["speed_integral_at_limit"] > 0 for r in rows)}/16 runs touch the late speed-integral limit.')
    fusion = []
    for r in groups['adaptive']:
        d = json.loads((HERE / 'adaptive/results' / f'{r["id"]:02}-lqi.json').read_text())
        offset = np.median([x['values'][0]/1000-x['time'] for x in d['diagnostics']])
        v = np.array([[float(x) for x in s.split(',')[1:]] for s in d['logs'] if s.startswith('N3LD,')])
        assert len(v), 'Missing EKF diagnostics'
        t = v[:, 0]/1000-offset
        late = v[(t >= 150) & (t < 180) & (v[:, 1] == 0)]
        assert len(late)
        fusion.append(dict(id=r['id'], calls=len(late), fused_fraction=float(late[:, 2].mean()),
                           threshold_mean=float(late[:, 11].mean()),
                           wind_mean=late[:, 6:8].mean(axis=0).tolist(),
                           wind_variance_mean=late[:, 8:10].mean(axis=0).tolist()))
    lines.append('- Per-case late fusion success fractions: '+', '.join(f'{r["id"]}: {r["fused_fraction"]:.0%}' for r in fusion)+'.')
    lines.append('- Diagnostic columns: AP milliseconds, core, fusion applied, body air vx/vy/vz, wind N/E, wind variance N/E, beta innovation, threshold TAS. Body air and wind/covariance are post-update; innovation can be stale when fusion is skipped.')
    # Columns after timestamp: core=1, fused=2, air=3:6, wind=6:8, variance=8:10, innovation=10, threshold=11.
    (HERE / 'comparison.json').write_text(json.dumps(dict(groups=groups, fusion=fusion), indent=2)+'\n')
    fig, axes = plt.subplots(1, 3, figsize=(13, 4), constrained_layout=True)
    for name, rows in groups.items():
        for ax, values, label in [
            (axes[0], [r['airspeed_error_rms'] for r in rows], 'Late TAS error RMS (m/s)'),
            (axes[1], [100*r['late']['additional_common_fraction'] for r in rows], 'Late common brake (%)'),
            (axes[2], [r['combined_power']['estimated_shaft_W'] for r in rows], 'Estimated shaft power (W)')]:
            ax.plot([r['id'] for r in rows], values, 'o-', label=name)
            ax.set(xlabel='Scenario ID', ylabel=label)
            ax.grid(alpha=.2)
    axes[0].legend(fontsize=8)
    fig.savefig(HERE / 'comparison.png', dpi=180)
    (HERE / 'RESULTS.md').write_text('\n'.join(lines)+'\n')
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
