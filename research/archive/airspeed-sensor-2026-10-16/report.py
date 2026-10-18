"""Compare measured airspeed error and control outcomes against sensorless runs."""
import json
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
OLD = HERE.parent / 'filtered-brake-2026-10-16'


def load(folder):
    rows = json.loads((folder / 'analysis.json').read_text())
    for r in rows:
        raw = json.loads((folder / 'results' / f"{r['id']:02}-lqi.json").read_text())
        samples = raw['samples']
        t = np.array([s['time'] for s in samples])
        truth = np.array([np.linalg.norm(np.array(s['truth'][7:10])-s['wind']) for s in samples])
        ds = [d for d in raw['diagnostics'] if 150 <= d['time'] < 180]
        if not ds:
            continue
        dt = np.array([d['time'] for d in ds])
        v = np.array([d['values'] for d in ds])
        error = v[:, 5]-np.interp(dt, t, truth)
        r['airspeed_error_rms'] = float(np.sqrt(np.mean(error**2)))
        r['airspeed_error_mean'] = float(np.mean(error))
        r['speed_integral_at_limit'] = float(np.mean(abs(v[:, 34]) >= 49.99))
    return rows


def main():
    groups = {}
    for name, original in [('eas-zero', 'diagnostic-zero'), ('eas-3000', 'memory-3000')]:
        if not (HERE / name / 'analysis.json').exists():
            continue
        sensor = load(HERE / name)
        ids = {r['id'] for r in sensor}
        groups[original] = [r for r in load(OLD / original) if r['id'] in ids]
        groups[name] = sensor
    lines = ['# Airspeed sensor comparison', '',
             'Mode medians except brake (mean of run means) and pitch (maximum). Late=150–180 s; tracking=20–100 s; power=20–180 s.', '',
             '| Configuration | Mode | Complete | Sensor use % | TAS error RMS m/s | Late common % | Max late pitch RMS deg | Alt RMS m | Xtrack RMS m | Shaft W | Shaft Wh/ground-km |',
             '|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|']
    fig, axes = plt.subplots(1, 3, figsize=(13, 4), constrained_layout=True)
    for name, rows in groups.items():
        for mode in ['AUTO', 'CRUISE']:
            good = [r for r in rows if r['mode'] == mode and r['complete']]
            total = sum(r['mode'] == mode for r in rows)
            if not good:
                continue
            values = [100*np.mean([r.get('airspeed_sensor_use_fraction', 0) for r in good]),
                      np.median([r['airspeed_error_rms'] for r in good]),
                      100*np.mean([r['late']['additional_common_fraction'] for r in good]),
                      max(r['late']['pitch_ac_deg'] for r in good),
                      np.median([r['disturbed']['altitude_rms_m'] for r in good]),
                      np.median([r['disturbed']['xtrack_rms_m'] for r in good]),
                      np.median([r['combined_power']['estimated_shaft_W'] for r in good]),
                      np.median([r['combined_power']['estimated_shaft_Wh_per_ground_km'] for r in good])]
            lines.append(f'| {name} | {mode} | {len(good)}/{total} | '+' | '.join(f'{v:.3f}' for v in values)+' |')
        good = [r for r in rows if r['complete']]
        for ax, values, label in [
            (axes[0], [r['airspeed_error_rms'] for r in good], 'Late TAS error RMS (m/s)'),
            (axes[1], [100*r['late']['additional_common_fraction'] for r in good], 'Late common brake (%)'),
            (axes[2], [r['combined_power']['estimated_shaft_W'] for r in good], 'Estimated shaft power (W)')]:
            ax.plot([r['id'] for r in good], values, 'o-', label=name)
            ax.set(xlabel='Scenario ID', ylabel=label)
            ax.grid(alpha=.2)
            ax.legend(fontsize=8)
    fig.savefig(HERE / 'comparison.png', dpi=180)
    lines += ['', '## Paired scenario differences', '',
              'Median of within-scenario percentage changes, not a ratio of group medians. Startup/estimator states differ; these are descriptive, not causal confidence intervals.', '']
    for before, after in [('diagnostic-zero', 'eas-zero'), ('memory-3000', 'eas-3000'),
                          ('eas-zero', 'eas-3000')]:
        if before not in groups or after not in groups:
            continue
        for mode in ['AUTO', 'CRUISE']:
            pairs = [(a, next((b for b in groups[before] if b['id'] == a['id'] and b['complete']), None))
                     for a in groups[after] if a['mode'] == mode and a['complete']]
            pairs = [(a, b) for a, b in pairs if b is not None]
            for key in ['estimated_shaft_W', 'estimated_shaft_Wh_per_ground_km']:
                changes = [100*(a['combined_power'][key]/b['combined_power'][key]-1) for a, b in pairs]
                lines.append(f'- {before} → {after}, {mode}, {key}: median {np.median(changes):+.2f}%; '
                             f'range {min(changes):+.2f} to {max(changes):+.2f}%; lower in {sum(v < 0 for v in changes)}/{len(changes)} cases.')
    lines += ['', '## Sensor and integral diagnostics', '']
    for name, rows in groups.items():
        good = [r for r in rows if r['complete']]
        lines.append(f'- {name}: {sum(r["speed_integral_at_limit"] > 0 for r in good)}/{len(good)} '
                     'runs touched the speed-integrator limit in the late window; '
                     f'maximum absolute late mean TAS error {max(abs(r["airspeed_error_mean"]) for r in good):.3f} m/s.')
    (HERE / 'comparison.json').write_text(json.dumps(groups, indent=2)+'\n')
    (HERE / 'RESULTS.md').write_text('\n'.join(lines)+'\n')
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
