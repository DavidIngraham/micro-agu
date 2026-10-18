"""Compare every completed trial with the retained baseline and deployed LQI runs."""
import argparse
import json
from pathlib import Path
import sys

import numpy as np
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
OLD = HERE.parent / 'controller-stability-2026-10-16'
sys.path.insert(0, str(OLD))
from analyze import metrics  # noqa: E402
from power import metrics as power_metrics  # noqa: E402


def assess(path):
    d = json.loads(path.read_text())
    out = dict(id=d['spec']['id'], mode=d['spec']['mode'], scheme=d['scheme'],
               complete=d['complete'], failure=d['failure'], source=str(path.relative_to(HERE.parent)))
    if not d['complete']:
        return out
    rows = d['samples']
    out['final_mission'] = rows[-1]['mission']
    for name, lo, hi in [('disturbed', 20, 100), ('late', 150, 180)]:
        subset = [s for s in rows if lo <= s['time'] < hi]
        out[name] = metrics(subset)
        out[name]['power'] = power_metrics(subset)
        brake = np.array([s['brakes'] for s in subset]) / (np.array([1300, 1500]) * np.pi * 55 / 360 * 0.1)
        out[name]['additional_common_fraction'] = float(np.mean(brake.min(axis=1)))
    out['combined_power'] = power_metrics([s for s in rows if 20 <= s['time'] < 180])
    out['minimum_applied_brake_mm'] = min(min(s['brakes']) for s in rows)
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('iteration')
    args = parser.parse_args()
    folder = HERE / args.iteration
    result = []
    lines = ['# ' + args.iteration, '',
             '| Case | Mode | Controller | Complete | Alt RMS m | Xtrack RMS m | Late pitch AC RMS deg | Late extra common % | Combined est. shaft W |',
             '|---|---|---|---|---:|---:|---:|---:|---:|']
    for path in sorted((folder / 'results').glob('*-lqi.json')):
        i = int(path.name.split('-')[0])
        baseline = folder / 'results' / f'{i:02}-baseline.json'
        if not baseline.exists():
            baseline = OLD / 'results' / f'{i:02}-baseline.json'
        for source, label in [(baseline, 'baseline'),
                              (OLD / 'results' / f'{i:02}-lqi.json', 'old LQI'), (path, 'zero LQI')]:
            r = assess(source)
            r['label'] = label
            result.append(r)
            if not r['complete']:
                lines.append(f"| {i} | {r['mode']} | {label} | {r['failure']} | — | — | — | — | — |")
            else:
                a, l, p = r['disturbed'], r['late'], r['combined_power']
                lines.append(f"| {i} | {r['mode']} | {label} | yes | {a['altitude_rms_m']:.3f} | "
                             f"{a['xtrack_rms_m']:.3f} | {l['pitch_ac_deg']:.3f} | "
                             f"{100*l['additional_common_fraction']:.2f} | {p['estimated_shaft_W']:.2f} |")
    lines += ['', '## Median comparisons', '']
    for mode in ['AUTO', 'CRUISE']:
        for label in ['baseline', 'old LQI', 'zero LQI']:
            group = [r for r in result if r['mode'] == mode and r['label'] == label and r['complete']]
            if group:
                alt = np.median([r['disturbed']['altitude_rms_m'] for r in group])
                cross = np.median([r['disturbed']['xtrack_rms_m'] for r in group])
                power = np.median([r['combined_power']['estimated_shaft_W'] for r in group])
                lines.append(f'- {mode} {label}: altitude RMS {alt:.3f} m, cross-track RMS {cross:.3f} m, estimated shaft power {power:.2f} W.')
    lines += ['', '## Recovery and completion', '']
    for label in ['baseline', 'old LQI', 'zero LQI']:
        group = [r for r in result if r['label'] == label]
        cruise = [r for r in group if r['mode'] == 'CRUISE' and r['complete']]
        large = [r['id'] for r in cruise if r['late']['pitch_ac_deg'] > 0.2]
        lines.append(f"- {label}: {sum(r['complete'] for r in group)}/{len(group)} completed; "
                     f'CRUISE late pitch AC RMS >0.2 deg in cases {large}.')
        auto = [r for r in group if r['mode'] == 'AUTO' and r['complete']]
        if auto:
            lines.append(f"  AUTO final mission indices: {[(r['id'], r['final_mission']) for r in auto]}.")
    lines += ['', '## Paired changes against old LQI', '',
              'Median of per-case percentage changes; small denominators can exaggerate tracking percentages.', '']
    for mode in ['AUTO', 'CRUISE']:
        pairs = [(r, next(x for x in result if x['id'] == r['id'] and x['label'] == 'old LQI'))
                 for r in result if r['mode'] == mode and r['label'] == 'zero LQI' and r['complete']]
        for phase, key in [('disturbed', 'altitude_rms_m'), ('disturbed', 'xtrack_rms_m'),
                           ('combined_power', 'estimated_shaft_W'),
                           ('combined_power', 'estimated_shaft_Wh_per_ground_km')]:
            changes = [100 * (a[phase][key] / b[phase][key] - 1)
                       for a, b in pairs if b['complete'] and b[phase][key] > 1e-6]
            if changes:
                lines.append(f'- {mode} {key}: {np.median(changes):+.1f}% '
                             f'(range {min(changes):+.1f}% to {max(changes):+.1f}%, n={len(changes)}).')
    if args.iteration == 'campaign':
        fig, axes = plt.subplots(2, 2, figsize=(11, 7))
        for row, mode in enumerate(['AUTO', 'CRUISE']):
            for col, (phase, key, title) in enumerate([
                    ('disturbed', 'altitude_rms_m', 'Altitude RMS (m)'),
                    ('late', 'pitch_ac_deg', 'Late calm pitch AC RMS (deg)')]):
                ax = axes[row, col]
                for i in sorted({r['id'] for r in result if r['mode'] == mode}):
                    triplet = [next(r for r in result if r['id'] == i and r['label'] == label)
                               for label in ['baseline', 'old LQI', 'zero LQI']]
                    if all(r['complete'] for r in triplet):
                        ax.plot(range(3), [r[phase][key] for r in triplet], 'o-', alpha=0.65, label=str(i))
                ax.set_xticks(range(3), ['PG+TECS', 'Old LQI', 'Zero-trim LQI'])
                ax.set_title(mode + ': ' + title)
                ax.grid(alpha=0.2)
        fig.suptitle('Matched scenarios: baseline rerun, archived LQI, selected zero-trim LQI')
        fig.tight_layout()
        fig.savefig(folder / 'comparison.png', dpi=150)
    (folder / 'comparison.json').write_text(json.dumps(result, indent=2) + '\n')
    (folder / 'RESULTS.md').write_text('\n'.join(lines) + '\n')
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
