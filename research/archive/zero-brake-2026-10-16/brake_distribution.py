"""Time occupancy of additional common brake; samples are not independent trials."""
import json
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
STROKES = np.array([1300, 1500]) * np.pi * 55 / 360 * .1
WINDOWS = {'disturbed': (20, 100), 'late calm': (150, 180)}

def distribution(x, w):
    order = np.argsort(x)
    cdf = np.cumsum(w[order]) / np.sum(w)
    return dict(mean=float(np.average(x, weights=w)),
                **{f'p{p}': float(x[order][min(np.searchsorted(cdf, p / 100), len(x)-1)])
                   for p in [50, 90, 95, 99]},
                below_1=float(np.sum(w[x < 1]) / np.sum(w)),
                above_20=float(np.sum(w[x > 20]) / np.sum(w)))

def main():
    runs = []
    for path in sorted((HERE / 'campaign/results').glob('*-lqi.json')):
        data = json.loads(path.read_text())
        assert data['complete']
        t = np.array([s['time'] for s in data['samples']])
        b = np.array([s['brakes'] for s in data['samples']]) / STROKES
        # Hold each recorded actuator value to the next timestamp, clipping at window edges.
        end = np.r_[t[1:], t[-1]]
        runs.append(dict(id=data['spec']['id'], mode=data['spec']['mode'], t=t,
                         end=end, common=100*b.min(axis=1)))
    report = {'definition': '100 * min(left/full_left_stroke, right/full_right_stroke)',
              'weighting': 'Seconds of sample-and-hold occupancy; no independent-sample inference.',
              'groups': [], 'late_per_run': []}
    fig, axes = plt.subplots(2, 2, figsize=(12, 7), constrained_layout=True)
    for col, mode in enumerate(['AUTO', 'CRUISE']):
        for label, (lo, hi) in WINDOWS.items():
            xs, ws = [], []
            for r in runs:
                if r['mode'] != mode:
                    continue
                w = np.maximum(0, np.minimum(r['end'], hi)-np.maximum(r['t'], lo))
                mask = w > 0
                xs.extend(r['common'][mask]); ws.extend(w[mask])
                if label == 'late calm':
                    stats = distribution(r['common'][mask], w[mask])
                    report['late_per_run'].append(dict(id=r['id'], mode=mode, **stats))
            x, w = np.array(xs), np.array(ws)
            report['groups'].append(dict(mode=mode, window=label, **distribution(x, w)))
            order = np.argsort(x)
            axes[0, col].step(x[order], 100*np.cumsum(w[order])/w.sum(), label=label)
        axes[0, col].set(title=f'{mode}: time-weighted distribution', xlabel='Additional common brake (% travel)', ylabel='Time at or below (%)', xlim=(0, 40), ylim=(0, 101))
        axes[0, col].legend(); axes[0, col].grid(alpha=.2)
        group = [r for r in report['late_per_run'] if r['mode'] == mode]
        axes[1, col].bar([str(r['id']) for r in group], [r['mean'] for r in group], color='#bc5948')
        axes[1, col].set(title=f'{mode}: each run, calm 150–180 s', xlabel='Scenario ID', ylabel='Mean additional common brake (%)', ylim=(0, 30))
        axes[1, col].grid(axis='y', alpha=.2)
    fig.savefig(HERE / 'campaign/brake-distribution.png', dpi=180)
    (HERE / 'campaign/brake-distribution.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))

if __name__ == '__main__':
    main()
