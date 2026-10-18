"""Paired sensing benefits and held-out tuning results; no population inference."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
TRAIN = {0, 1, 6, 9}
rows = json.loads((HERE/'fixed/analysis.json').read_text())
lookup = {(r['variant'], r['id']): r for r in rows}
comparisons = {}
for variant, baseline in [('airspeed', 'sensorless'), ('sensor', 'airspeed'), ('sensor', 'standard')]:
    for mode in ['AUTO', 'CRUISE']:
        pairs = [(r, lookup[baseline, r['id']]) for r in rows if r['variant'] == variant and r['mode'] == mode
                 and r['complete'] and lookup[baseline, r['id']]['complete']]
        result = {}
        for metric, getter in [('shaft_W', lambda r: r['shaft_W']),
                               ('shaft_Wh_per_ground_km', lambda r: r['shaft_Wh_per_ground_km']),
                               ('pitch_RMS', lambda r: r['disturbed']['pitch_ac_deg']),
                               ('altitude_RMS', lambda r: r['disturbed']['altitude_rms_m'])]:
            change = [100*(getter(a)/getter(b)-1) for a, b in pairs]
            result[metric] = dict(n=len(change), median=float(np.median(change)),
                                  p10=float(np.quantile(change, .1)), p90=float(np.quantile(change, .9)),
                                  minimum=min(change), maximum=max(change), paired_changes=change)
        comparisons[f'{variant}_vs_{baseline}_{mode}'] = result
selection = json.loads((HERE/'selection.json').read_text()) if (HERE/'selection.json').exists() else None
held_out = {}
if selection and selection['selected'] != 'fixed':
    chosen = json.loads((HERE/selection['selected']/'analysis.json').read_text())
    for mode in ['AUTO', 'CRUISE']:
        pairs = [(r, lookup['sensor', r['id']]) for r in chosen if r['id'] not in TRAIN and r['mode'] == mode
                 and r['complete'] and lookup['sensor', r['id']]['complete']]
        if pairs:
            held_out[mode] = dict(n=len(pairs), shaft_change_pct=float(np.median([100*(a['shaft_W']/b['shaft_W']-1) for a, b in pairs])),
                                 pitch_change_pct=float(np.median([100*(a['disturbed']['pitch_ac_deg']/b['disturbed']['pitch_ac_deg']-1) for a, b in pairs])),
                                 altitude_change_pct=float(np.median([100*(a['disturbed']['altitude_rms_m']/b['disturbed']['altitude_rms_m']-1) for a, b in pairs])),
                                 xtrack_change_m=float(np.median([a['disturbed']['xtrack_rms_m']-b['disturbed']['xtrack_rms_m'] for a, b in pairs])))
result = dict(paired= comparisons, selection=selection, held_out=held_out)
(HERE/'summary.json').write_text(json.dumps(result, indent=2)+'\n')
lines = ['# Paired results', '', 'Percentages are median **paired** changes, not ratios of group medians. '
         'Ranges are observed P10–P90 across the fixed scenario suite, not confidence intervals.', '',
         '| Comparison | Mode | Shaft power % | P10–P90 % | Energy/ground-distance % | Disturbed pitch RMS % | Altitude RMS % |',
         '|---|---|---:|---|---:|---:|---:|']
for key, r in comparisons.items():
    label, mode = key.rsplit('_', 1)
    p = r['shaft_W']
    lines.append(f'| {label} | {mode} | {p["median"]:+.2f} | {p["p10"]:+.2f} to {p["p90"]:+.2f} | '
                 f'{r["shaft_Wh_per_ground_km"]["median"]:+.2f} | {r["pitch_RMS"]["median"]:+.2f} | {r["altitude_RMS"]["median"]:+.2f} |')
lines += ['', '## Retuning', '', 'Selection uses only cases 0, 1, 6 and 9; see `selection.json` for the rule and rejection reasons.']
if selection:
    lines.append('Selected configuration: **'+selection['selected']+'**.')
    for name, r in selection['candidates'].items():
        lines.append(f'- {name}: score {r["score"]:.4f}; '+('; '.join(r['rejected_reasons']) or 'passes screening guards')+'.')
    for mode, r in held_out.items():
        lines.append(f'- Held-out {mode}, n={r["n"]}: shaft {r["shaft_change_pct"]:+.2f}%, pitch RMS '
                     f'{r["pitch_change_pct"]:+.2f}%, altitude RMS {r["altitude_change_pct"]:+.2f}%, cross-track '
                     f'{r["xtrack_change_m"]:+.3f} m versus the same sensors with fixed gains.')
lines += ['', '## Interpretation limits', '',
          'The realistic angle-sensor comparison includes an augmented observer. It is not merely replacing three '
          'values with truth. The raw angle/rate diagnostic is a different feedback architecture and cannot establish '
          'the attainable performance ceiling of a properly designed sensor-equipped controller.', '',
          'Same missions do not guarantee identical achieved speeds, paths or final energy states. Per-run '
          '`analysis.json` retains achieved airspeed, ground path, energy per ground distance and height change. '
          'Shaft power is not battery power. All results are conditional on the simulator and assumed sensing.', '',
          '![Fixed-gain comparison](comparison.png)']
(HERE/'RESULTS.md').write_text('\n'.join(lines)+'\n')
fig, axes = plt.subplots(2, 3, figsize=(12, 6), constrained_layout=True)
names = ['sensorless', 'airspeed', 'sensor', 'standard']
labels = ['LQI\nno pitot', 'LQI\npitot', 'LQI\npitot + angles', 'Standard\npitot']
getters = [lambda r: r['disturbed']['pitch_ac_deg'], lambda r: r['disturbed']['altitude_rms_m'], lambda r: r['shaft_W']]
for i, mode in enumerate(['AUTO', 'CRUISE']):
    for j, (title, get) in enumerate(zip(['Disturbed pitch RMS (°)', 'Altitude RMSE (m)', 'Modeled shaft power (W)'], getters)):
        groups = [[get(r) for r in rows if r['variant'] == n and r['mode'] == mode and r['complete']] for n in names]
        axes[i, j].bar(range(4), [np.median(g) for g in groups], color=['#999999', '#4b80b5', '#2b9a82', '#c89337'])
        for k, g in enumerate(groups):
            axes[i, j].scatter(k+np.linspace(-.12, .12, len(g)), g, s=14, color='black', alpha=.5)
        axes[i, j].set_xticks(range(4), labels, fontsize=8)
        axes[i, j].set_title(mode+': '+title, fontsize=10)
        axes[i, j].grid(axis='y', alpha=.2)
fig.suptitle('Fixed gains: medians and all eight cases per flight mode')
fig.savefig(HERE/'comparison.png', dpi=160)
fig, axes = plt.subplots(2, 2, figsize=(11, 6), constrained_layout=True)
for column, case in enumerate([0, 1]):
    for variant in ['airspeed', 'angles', 'full', 'sensor']:
        d = json.loads((HERE/f'fixed/{variant}/results/{case:02}-lqi.json').read_text())
        t = np.array([s['time'] for s in d['samples']])
        state = np.array([s['truth'] for s in d['samples']])
        pitch = Rotation.from_quat(state[:, 3:7]).as_euler('xyz', degrees=True)[:, 1]
        brakes = np.array([s['brakes'] for s in d['samples']])/(np.array([1300, 1500])*np.pi*55/360*.1)
        axes[0, column].plot(t, pitch, label=variant, linewidth=.8)
        axes[1, column].plot(t, 100*brakes.min(axis=1), label=variant, linewidth=.8)
    axes[0, column].set_title('Nominal '+('AUTO' if case == 0 else 'CRUISE'))
    axes[0, column].set_ylabel('Truth payload pitch (°)')
    axes[1, column].set_ylabel('Applied common brake (%)')
    for ax in axes[:, column]:
        ax.axvspan(20, 100, alpha=.08, color='black'); ax.grid(alpha=.2); ax.set_xlabel('Simulation seconds')
axes[0, 0].legend(fontsize=8)
fig.suptitle('Raw angle/rate substitution is a diagnostic, not a performance ceiling')
fig.savefig(HERE/'raw-feedback.png', dpi=160)
print('\n'.join(lines[:-2]))
