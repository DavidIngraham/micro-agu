"""Summarize parameter sensitivity without promoting an experimental calibration."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
fits = sum([json.loads((HERE / f).read_text()) for f in ['fit.json','fit-extended.json']], [])
checks = sum([json.loads((HERE / f).read_text()) for f in ['checks.json','checks-extended.json']], [])
ranked = json.loads((HERE / 'ranked-extended.json').read_text())
keys = ['roll_rate','pitch_rate','yaw_rate','bank','course','climb']
weights = dict(roll_rate=1, pitch_rate=1, yaw_rate=1, bank=.25, climb=.25)
reference = {(r['flight'],r['start'],r['alpha']): r['metrics'] for r in checks if r['variant']=='original' and r['damping']==3}


def summarize(rows):
    assert rows and all(r['finite'] for r in rows)
    return {k:float(np.mean([r['metrics'][k] for r in rows])) for k in keys}


def score(rows):
    return float(np.mean([sum(w*r['metrics'][k]/reference[r['flight'],r['start'],r['alpha']][k]
                             for k,w in weights.items())/3.5 for r in rows]))


summary = {}
for variant, ranking in ranked.items():
    selected = ranking[0]['damping']
    base = [r for r in checks if r['variant']==variant and r['damping']==3]
    chosen = [r for r in checks if r['variant']==variant and r['damping']==selected]
    fit3 = next(r['score'] for r in ranking if r['damping']==3)
    summary[variant] = dict(selected=selected, upper_bound=selected==8,
                            fit_change_pct=100*(ranking[0]['score']/fit3-1),
                            current=summarize(base), selected_metrics=summarize(chosen),
                            current_check_score=score(base), selected_check_score=score(chosen),
                            check_change_pct=100*(score(chosen)/score(base)-1),
                            by_flight={f:dict(current=summarize([r for r in base if r['flight']==f]),
                                             selected=summarize([r for r in chosen if r['flight']==f]))
                                       for f in ['hood_river','trout_lake']},
                            by_alpha={str(a):dict(current=score([r for r in base if r['alpha']==a]),
                                                 selected=score([r for r in chosen if r['alpha']==a]))
                                      for a in [.25,.35,.45]})
(HERE/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
lines = ['# Wind-sensitive as-built damping calibration', '',
         'AI-assisted research, 16 October 2026. No runtime model, controller gain, or deployment changed.', '',
         '## Finding', '',
         'Changing wind did not make the fit prefer lower damping. All 12 wind assumptions selected 3× on the original 1–3× grid. '
         'An exploratory extension to 4×, 6× and 8× selected 8× in 11 variants and 6× in the constant −1 m/s down-wind variant. '
         'Most optima therefore remain at the search bound. This is not a physical identification of 8× damping.', '',
         '| Wind history | Extended selection | Fit improvement vs 3× | Held-out improvement vs 3×, same wind | Held-out score at 3× | Held-out score at selected |',
         '|---|---:|---:|---:|---:|---:|']
for name,s in summary.items():
    lines.append(f'| {name} | {s["selected"]:.0f}×'+(' (bound)' if s['upper_bound'] else '')+
                 f' | {-s["fit_change_pct"]:.2f}% | {-s["check_change_pct"]:.2f}% | {s["current_check_score"]:.3f} | {s["selected_check_score"]:.3f} |')
lines += ['', 'Positive improvement means lower error. Held-out score is normalized per window/initialization against '
          'the current 3× model under original wind; 1.0 is that reference. It is distinct from the training score, '
          'whose normalization is frozen to the original legacy fit-window errors. Scores combine rates (weight 1 each), '
          'bank and climb (weight 0.25 each); course is reported but not optimized.', '',
          '## Absolute held-out errors', '',
          'Arithmetic means over eight 30-second held-out windows and three initial canopy angles. Rates in degrees/s, '
          'bank/course in degrees, climb in m/s. Windows reuse historical flights and are not independent validation flights.', '',
          '| Wind | Damping | Roll rate | Pitch rate | Yaw rate | Bank | Course | Climb |',
          '|---|---:|---:|---:|---:|---:|---:|---:|']
for name in ['original','smoothed','slow_wind','fast_wind','down_minus1','down_plus1']:
    s=summary[name]
    for label,damping in [('current',3),('selected_metrics',s['selected'])]:
        lines.append(f'| {name} | {damping:.0f}× | '+' | '.join(f'{s[label][k]:.3f}' for k in keys)+' |')
lines += ['', '![Fit and held-out comparison](comparison.png)', '',
          '## Interpretation', '',
          '- The hypothesis that corrected wind alone would remove the need for elevated damping is not supported by this experiment.',
          '- At unchanged 3× damping, nominal smoothed wind changes the aggregate held-out score by only about −0.2%. Rate errors barely change; course error improves from 57.9° to 55.1°, mostly in Hood River. Horizontal sensitivity variants behave similarly. The assumed constant ±1 m/s vertical winds have much larger effects, especially on climb; they are sensitivity cases, not reconstructed thermal truth.',
          '- With nominal smoothed wind, increasing damping from 3× to 8× improves the held-out composite by 3.85%, mainly through pitch-rate error (17.47 to 15.63°/s). Roll-rate error slightly worsens. Large absolute model errors remain.',
          '- That improvement depends strongly on initial canopy angle: about 0.5%, 3.4% and 7.5% for 0.25, 0.35 and 0.45 rad respectively. Unobserved canopy initial state, unresolved aerodynamic/suspension mechanics and the short-horizon objective remain plausible confounders. Separate initialization transients from sustained dynamics before interpreting a fitted damping coefficient physically.',
          '- A search-bound result is a warning about identifiability, not a reason to install an 8× coefficient. No new tune is promoted.',
          '- The smoother is not wind truth: the preceding known-truth tests exposed errors and overconfidence in weakly observed or changing-wind cases. This study measures sensitivity to alternative histories rather than establishing which one is correct.',
          '- Parameters other than the common pitch/yaw damping multiplier were fixed. This does not identify brake authority, drag or independent pitch/yaw damping, and it does not assess closed-loop controller stability or power efficiency.', '',
          '## Frozen method and scope', '',
          'The historical alternating starts were retained: four fit windows per flight at 10 seconds, four held-out '
          'windows per flight at 30 seconds. Fit canopy initialization is 0.35 rad; held-out checks use 0.25/0.35/0.45 rad. '
          'The 132 mm riser spacing, mass/inertia, routing, 60 mm assumed ring height, brake/propulsion parameters and '
          'all other settings were held fixed. Recorded RPM initializes the motor model; recorded voltage at the '
          'window start is held constant, preserving the previous method.', '',
          'Original wind is frozen at each window start exactly as before. Smoothed winds are linearly interpolated '
          'from the saved 1 Hz trajectories and applied at every 40 ms integration interval. Each wind changes both '
          'airflow during replay and the initial canopy-state reconstruction. Vertical variants retain their ±1 m/s '
          'down components. Yaw/beta sensitivity variants change the reconstructed wind only; they do not rewrite '
          'the observed attitude or measured geometry.', '',
          'The 12 histories are the original assumptions plus nominal smoother, slow/fast wind diffusion, '
          '±3° sideslip, ±5° heading bias, ±1 m/s vertical wind, and tight/loose constant lateral-offset prior. '
          'These are one-at-a-time sensitivity choices, not a sampled probability distribution.', '',
          'There are 768 fit replays, 576 held-out checks and eight legacy normalization replays, plus the separately '
          'recorded numerical/provenance audit. The expanded grid was chosen after all original-grid results hit '
          '3×, before examining its held-out results. No wind variant was selected to minimize model error.', '',
          'Smoothing uses the whole flight, including the held-out periods. Those periods are held out from damping '
          'selection, not from wind reconstruction; this is not an independent end-to-end prediction test.', '',
          '## Numerical audit', '',
          'All 768 fit and 576 held-out replays were finite with valid source inputs. Historical as-built held-out '
          'metrics reproduced within 3.9e-12 absolute error; constant-wind bulk and per-step replay matched exactly, '
          'as did the legacy normalization. Doubling integration substeps from 32 to 64 for four high-pitch-error '
          'extended checks changed metrics by at most 0.000123%. Runtime calibration was unchanged. '
          'See `audit.json` for hashes and detailed checks.', '',
          '## Reproduce', '',
          'Run from the repository root:', '', '```bash',
          'simulation/.venv/bin/python research/archive/wind-calibration-2026-10-16/run.py',
          'simulation/.venv/bin/python research/archive/wind-calibration-2026-10-16/run.py --extended',
          'simulation/.venv/bin/python research/archive/wind-calibration-2026-10-16/audit.py',
          '/home/david/envs/venv-ardupilot/bin/python research/archive/wind-calibration-2026-10-16/report.py', '```', '',
          'Completed batch files are reused; use a fresh study directory to repeat runs after changing code or inputs. '
          'See `audit.json` for source/input hashes, original-fit reproduction, constant-wind API equivalence and '
          'integration-refinement results. All large source flight data remain in their existing local directory.']
(HERE/'README.md').write_text('\n'.join(lines)+'\n')
fig,axes=plt.subplots(1,2,figsize=(12,4),constrained_layout=True)
for name in ['original','smoothed','slow_wind','fast_wind','down_minus1','down_plus1']:
    rows=sorted(ranked[name],key=lambda r:r['damping'])
    axes[0].plot([r['damping'] for r in rows],[r['score'] for r in rows], 'o-',label=name)
axes[0].set(xlabel='Damping multiplier',ylabel='Normalized training error'); axes[0].legend(fontsize=8)
names=list(summary); x=np.arange(len(names))
axes[1].bar(x-.18,[summary[n]['current_check_score'] for n in names],.36,label='Current 3×')
axes[1].bar(x+.18,[summary[n]['selected_check_score'] for n in names],.36,label='Extended fit selection')
axes[1].set_xticks(x,names,rotation=65,ha='right');axes[1].set_ylabel('Normalized held-out error');axes[1].legend(fontsize=8)
for ax in axes:ax.grid(axis='y',alpha=.2)
fig.savefig(HERE/'comparison.png',dpi=170)
print('\n'.join(lines[:45]))
