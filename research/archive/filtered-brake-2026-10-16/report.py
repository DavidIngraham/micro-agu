"""Compare trial medians without treating time samples as independent flights."""
import json
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent


def main():
    trials = {p.parent.name: json.loads(p.read_text()) for p in HERE.glob('*/analysis.json')}
    names = [n for n in ['diagnostic-zero', 'memory-1000', 'memory-3000', 'campaign'] if n in trials]
    lines = ['# Filtered-brake trial results', '',
             'All power numbers are quasi-steady estimated shaft power, not electrical power.', '',
             '| Trial | Mode | Complete | Alt RMS m | Cross-track RMS m | Max late pitch AC RMS deg | Mean late common % | Median shaft W | Median shaft Wh/ground-km |',
             '|---|---|---:|---:|---:|---:|---:|---:|---:|']
    fig, axes = plt.subplots(1, 3, figsize=(13, 4), constrained_layout=True)
    colors = dict(zip(names, ['#326ba8', '#df8b32', '#2e8b62', '#8755a3']))
    for n in names:
        for mode in ['AUTO', 'CRUISE']:
            group = [r for r in trials[n] if r['mode'] == mode]
            ok = [r for r in group if r['complete']]
            if not ok:
                continue
            values = [np.median([r['disturbed'][k] for r in ok]) for k in ['altitude_rms_m', 'xtrack_rms_m']]
            values += [max(r['late']['pitch_ac_deg'] for r in ok),
                       100*np.mean([r['late']['additional_common_fraction'] for r in ok]),
                       np.median([r['combined_power']['estimated_shaft_W'] for r in ok]),
                       np.median([r['combined_power']['estimated_shaft_Wh_per_ground_km'] for r in ok])]
            lines.append(f'| {n} | {mode} | {len(ok)}/{len(group)} | '+' | '.join(f'{v:.3f}' for v in values)+' |')
        for r in trials[n]:
            if not r['complete'] or 'diagnostic_late_means' not in r:
                continue
            d = r['diagnostic_late_means']
            axes[0].scatter(d['truth_airspeed'], d['airspeed_measured'], color=colors[n],
                            label=n if r is trials[n][0] else None, alpha=.7)
        for ax, key, ylabel in [(axes[1], 'additional_common_fraction', 'Late mean common brake (%)'),
                                 (axes[2], 'pitch_ac_deg', 'Late pitch AC RMS (deg)')]:
            good = [r for r in trials[n] if r['complete']]
            scale = 100 if key == 'additional_common_fraction' else 1
            ax.plot([r['id'] for r in good], [scale*r['late'][key] for r in good], 'o-', label=n, color=colors[n])
            ax.set(xlabel='Scenario ID', ylabel=ylabel)
    axes[0].plot([1, 6], [1, 6], '--', color='gray')
    axes[0].set(xlabel='Truth airspeed (m/s)', ylabel='Onboard estimated airspeed (m/s)', title='Late calm window')
    for ax in axes:
        ax.grid(alpha=.2)
        ax.legend(fontsize=8)
    fig.savefig(HERE / 'comparison.png', dpi=180)
    lines += ['', '## Screen against fresh diagnostic-zero runs', '',
              'Tracking gate: each mode median at most 25% higher; every run completes; CRUISE late pitch AC RMS <0.2 deg.',
              'No confidence intervals: startup/estimator states are not identical across runs.', '']
    for n in names:
        if n == 'diagnostic-zero':
            continue
        passed = all(r['complete'] for r in trials[n])
        passed &= all(r['late']['pitch_ac_deg'] < .2 for r in trials[n] if r['complete'] and r['mode'] == 'CRUISE')
        for mode in ['AUTO', 'CRUISE']:
            current = [r for r in trials[n] if r['mode'] == mode and r['complete']]
            ids = {r['id'] for r in current}
            base = [r for r in trials['diagnostic-zero'] if r['id'] in ids]
            current = [r for r in current if r['id'] in {b['id'] for b in base}]
            for key in ['altitude_rms_m', 'xtrack_rms_m']:
                ratio = np.median([r['disturbed'][key] for r in current])/np.median([r['disturbed'][key] for r in base])
                passed &= ratio <= 1.25
                lines.append(f'- {n} {mode} {key}: {(ratio-1)*100:+.1f}%.')
        lines.append(f'- {n}: screen {"PASS" if passed else "FAIL"}.')
    (HERE / 'RESULTS.md').write_text('\n'.join(lines)+'\n')
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
