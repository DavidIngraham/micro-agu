"""Summarize estimates, predictive checks and sensitivity without changing truth."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
FLIGHTS = {'hood_river': [300, 480, 690, 930], 'trout_lake': [600, 900, 1920, 2790]}
VARIANTS = ['nominal', 'start_low', 'start_high', 'slow_wind', 'loose_model']


def read(name):
    return json.loads((HERE/name).read_text())


def main():
    out = dict(fixed={}, joint={}, synthetic={}, predictive={}, initialization={}, diagnostics={})
    for flight, starts in FLIGHTS.items():
        old = np.load(HERE.parent/f'wind-smoother-2026-10-16/{flight}-0.npz')
        for start in starts:
            name = f'{flight}-{start}'
            files = [np.load(HERE/f'augmented-{name}-{v}.npz') for v in VARIANTS]
            z = np.stack([np.column_stack([f['relative_rotvec'], f['relative_angular_velocity'], f['wind_ned'][:, :2]]) for f in files])
            time = files[0]['time']
            old_wind = np.column_stack([np.interp(time, old['t'], old['alternatives'][0, :, k]) for k in [0, 1]])
            orientation_spread = np.stack([(R.from_rotvec(z[0, :, :3]).inv()*R.from_rotvec(v[:, :3])).magnitude() for v in z[1:]])
            info = [read(f'fixed-{name}-{v}.json') for v in VARIANTS]
            out['fixed'][name] = dict(
                all_success=all(i['success'] for i in info),
                mean_wind=z[0, :, 6:8].mean(axis=0).tolist(),
                previous_smoother_difference_rms=float(np.sqrt(np.mean(np.sum((z[0, :, 6:8]-old_wind)**2, axis=1)))),
                multistart_max_cost_fraction=float(max(abs(i['cost']/info[0]['cost']-1) for i in info[1:3])),
                sensitivity_wind_max_m_s=float(np.max(np.linalg.norm(z[1:, :, 6:8]-z[0, :, 6:8], axis=2))),
                sensitivity_angle_max_deg=float(np.rad2deg(orientation_spread.max())),
                max_absolute_relative_angle_deg=float(np.rad2deg(abs(z[0, :, :3])).max()))
            # Empirical assumption envelope, explicitly not a posterior interval.
            np.savez_compressed(HERE/f'augmented-{name}-sensitivity.npz', time=time,
                                nominal=z[0], minimum=z.min(axis=0), maximum=z.max(axis=0),
                                original_observation=files[0]['original_observation'])
    for variant in ['nominal', 'slow_wind', 'loose_model']:
        if (HERE/f'joint-{variant}.json').exists():
            out['joint'][variant] = read(f'joint-{variant}.json')
            cost = sum(read(f'fixed-{f}-{t}-{variant}.json')['cost'] for f, starts in FLIGHTS.items() for t in starts)
            out['joint'][variant]['fixed_damping_cost'] = cost
            out['joint'][variant]['training_cost_reduction_percent'] = 100*(1-out['joint'][variant]['cost']/cost)
            x = np.load(HERE/f'joint-{variant}.npz')['x']
            offset = 0
            for flight, starts in FLIGHTS.items():
                for start in starts:
                    source = np.load(HERE/f'augmented-{flight}-{start}-nominal.npz')
                    size = len(source['time'])*8
                    z = x[offset:offset+size].reshape(-1, 8)
                    offset += size
                    np.savez_compressed(HERE/f'augmented-joint-{flight}-{start}-{variant}.npz',
                        time=source['time'], original_observation=source['original_observation'],
                        relative_rotvec=z[:, :3], relative_angular_velocity=z[:, 3:6],
                        wind_ned=np.column_stack([z[:, 6:8], np.zeros(len(z))]),
                        damping=out['joint'][variant]['damping'])
            assert offset == len(x)-2
    for path in sorted(HERE.glob('diagnostic-*.json')):
        out['diagnostics'][path.stem] = read(path.name)
    for path in sorted(HERE.glob('synthetic-*-truth.json')):
        out['synthetic'][path.stem] = read(path.name)
    for scheme in ['fixed', 'joint']:
        rows = [read(p.name) for p in sorted(HERE.glob(f'check-*-{scheme}.json'))]
        if rows:
            out['predictive'][scheme] = {label: {cut: {metric: float(np.mean([r[label][cut][metric] for r in rows]))
                                                      for metric in rows[0][label][cut]}
                                                 for cut in ['0', '2', '5', '10']}
                                         for label in ['inferred', 'historical']}
            out['initialization'][scheme] = {period: float(np.mean([r['initialization_rate_spread_deg_s'][period] for r in rows]))
                                              for period in rows[0]['initialization_rate_spread_deg_s']}
    (HERE/'summary.json').write_text(json.dumps(out, indent=2)+'\n')
    lines = ['# Results', '', '## Joint damping fits', '',
             '| Assumption | Pitch multiplier | Yaw multiplier | Training objective reduction |',
             '|---|---:|---:|---:|']
    for variant, r in out['joint'].items():
        lines.append(f'| {variant} | {r["damping"][0]:.3f} | {r["damping"][1]:.3f} | {r["training_cost_reduction_percent"]:.1f}% |')
    lines += ['', 'The search bounds are 0.5–12×. Results near 0.5× are boundary solutions, not identified optima. '
              'Objective reductions compare with fixed 3×/3× under the same assumptions; objectives across different '
              'discrepancy scales are not directly comparable.', '', '## Held-out forecasts', '',
              'Arithmetic mean of window RMSE over eight held-out thirty-second windows. Degrees/s for rates. '
              'Initial-state inference uses only the preceding ten seconds; the historical baseline uses alpha=0.35 rad, '
              'not an average over the three initializations used in the earlier study.', '',
              '| Damping | Initialization | Initial seconds excluded | Roll rate | Pitch rate | Yaw rate |',
              '|---|---|---:|---:|---:|---:|']
    for scheme, results in out['predictive'].items():
        for label, cuts in results.items():
            for cut in ['0', '2', '5', '10']:
                r = cuts[cut]
                lines.append(f'| {scheme} | {label} | {cut} | {r["roll_rate"]:.3f} | {r["pitch_rate"]:.3f} | {r["yaw_rate"]:.3f} |')
    lines += ['', '## Initialization disturbance', '',
              'Mean per-window RMS body-rate difference caused by ±0.1 rad initial pitch-component perturbations, '
              'holding wind fixed. Degrees/s, pooled over perturbations and axes within each window.', '',
              '| Damping | 0–2 s | 2–5 s | 5–10 s | 10–20 s | 20–30 s |', '|---|---:|---:|---:|---:|---:|']
    for scheme, r in out['initialization'].items():
        lines.append('| '+scheme+' | '+' | '.join(f'{r[k]:.4f}' for k in ['0-2', '2-5', '5-10', '10-20', '20-30'])+' |')
    lines += ['', '## Synthetic truth recovery', '',
              'Wind RMSE is pooled over north/east components, not horizontal-vector magnitude. '
              'The mismatch case raises the estimator brake-lift coefficient 30%; the varying-wind case adds '
              '0.3/0.2 m/s sinusoidal north/east variations. These are limited validation cases.', '',
              '| Case | Wind RMSE m/s | Relative orientation RMSE ° | Pitch multiplier | Yaw multiplier |',
              '|---|---:|---:|---:|---:|']
    for name, r in out['synthetic'].items():
        lines.append(f'| {name} | {r["wind_rmse"]:.3f} | {r["angle_rmse_deg"]:.3f} | {r["fitted_damping"][0]:.3f} | {r["fitted_damping"][1]:.3f} |')
    lines += ['', '## Optimization checks', '',
              '| Case | Objective | Pitch multiplier | Yaw multiplier | Successful termination |',
              '|---|---:|---:|---:|---|']
    for name, r in out['diagnostics'].items():
        lines.append(f'| {name} | {r["cost"]:.3f} | {r["damping"][0]:.3f} | {r["damping"][1]:.3f} | {r["success"]} |')
    (HERE/'RESULTS.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps(out, indent=2))
    fig, axes = plt.subplots(2, 2, figsize=(11, 6), constrained_layout=True)
    for row, (flight, start) in enumerate([('hood_river', 300), ('trout_lake', 600)]):
        a = np.load(HERE/f'augmented-{flight}-{start}-sensitivity.npz')
        t = a['time']-a['time'][0]
        for k, label in enumerate(['North', 'East']):
            axes[row, 0].plot(t, a['nominal'][:, 6+k], label=label)
            axes[row, 0].fill_between(t, a['minimum'][:, 6+k], a['maximum'][:, 6+k], alpha=.15)
        for k, label in enumerate(['Roll component', 'Pitch component', 'Yaw component']):
            axes[row, 1].plot(t, np.rad2deg(a['nominal'][:, k]), label=label)
            axes[row, 1].fill_between(t, np.rad2deg(a['minimum'][:, k]), np.rad2deg(a['maximum'][:, k]), alpha=.15)
        axes[row, 0].set(title=f'{flight.replace("_", " ").title()}: inferred wind', ylabel='m/s')
        axes[row, 1].set(title='Canopy relative rotation vector', ylabel='degrees')
        for ax in axes[row]:
            ax.set_xlabel('Seconds from window start'); ax.grid(alpha=.2); ax.legend(fontsize=8)
    fig.suptitle('Fixed physics: shaded ranges span assumptions, not confidence intervals')
    fig.savefig(HERE/'hidden-states.png', dpi=160)
    if out['predictive']:
        fig, axes = plt.subplots(1, 3, figsize=(11, 3.6), constrained_layout=True)
        for ax, metric in zip(axes, ['roll_rate', 'pitch_rate', 'yaw_rate']):
            for scheme, rows in out['predictive'].items():
                for label, style in [('historical', '--'), ('inferred', '-')]:
                    ax.plot([0, 2, 5, 10], [rows[label][cut][metric] for cut in ['0', '2', '5', '10']],
                            style+'o', label=f'{scheme} damping / {label} state')
            ax.set(title=metric.replace('_', ' ').title(), xlabel='Initial seconds excluded', ylabel='Mean window RMSE (deg/s)')
            ax.grid(alpha=.2)
        axes[-1].legend(fontsize=7)
        fig.savefig(HERE/'prediction.png', dpi=160)


if __name__ == '__main__':
    main()
