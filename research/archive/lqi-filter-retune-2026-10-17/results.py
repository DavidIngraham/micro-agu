"""Render all tuning/hold-out results with dimensional tradeoffs."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from evaluate import report
HERE=Path(__file__).resolve().parent
def main():
    result=report();chosen=result['selected']
    training=json.loads((HERE/'training/metrics.json').read_text())
    validation=json.loads((HERE/'validation/metrics.json').read_text()) if (HERE/'validation/metrics.json').exists() else []
    missions=json.loads((HERE/'mission-metrics.json').read_text())
    endurance=json.loads((HERE/'endurance-metrics.json').read_text())
    names=list(dict.fromkeys(['published','local',chosen]+sorted({r['variant'] for r in validation})))
    lines=['# Published LQI retuning with a fixed 0.7 s reference filter','',
      'AI-assisted. Original observer and sensing, existing trim and reference limits. All gain selection precedes fresh validation.', '',
      '**Selected:** '+str(chosen)+'. **Release acceptance:** '+('PASS' if result['accepted'] else 'FAIL: '+'; '.join(result['reasons'])), '',
      '## Test correction', '',
      'The earlier crosswind comparison used a fixed earth bearing against aircraft with different headings. In prior case 83, the horizontal wind in payload coordinates was about (−0.35, −1.93) m/s without filtering versus (+1.93, −0.50) m/s with filtering. The disturbance was therefore predominantly lateral in one run and axial in the other. Its 76% crosswind-loss regression was not a controlled estimate of filter effect. This study uses the same lateral incidence at gust onset and aligns CRUISE weather to the aircraft; AUTO weather stays tied to the mission route.', '',
      'The same prior trace also showed common-brake buildup and reduced true airspeed, without throttle saturation. Gain/observer interactions may still matter; correcting incidence does not prove those effects absent. The observer and trim are held fixed here so the gain comparison remains interpretable.', '',
      '## Training sweep', '', '| Candidate | Qualified / runs | Median score | Worst score |', '|---|---:|---:|---:|']
    for name in sorted({r['variant'] for r in training}):
        rr=[r for r in training if r['variant']==name];v=[r['score'] for r in rr if r['score'] is not None]
        lines.append(f'| {name} | {len(v)}/{len(rr)} | '+(f'{np.median(v):.3f} | {max(v):.3f} |' if v else 'n/a | n/a |'))
    lines+=['', 'Selection details, including rejected alternatives: [selection](selection-with-filtered.json).', '',
      '## Fresh validation', '',
      '| Controller | Qualified / runs | Score | Climb relative pitch-rate peak °/s | Descent peak °/s | Climb settling s | Updraft recovery s | Shaft Wh/km |',
      '|---|---:|---:|---:|---:|---:|---:|---:|']
    for name in names:
        rr=[r for r in validation if r['variant']==name];good=[r for r in rr if r['score'] is not None]
        if not good:continue
        vals=[np.median([fn(r) for r in good]) for fn in [lambda r:r['score'],lambda r:r['command_peak_rates_deg_s'][0],lambda r:r['command_peak_rates_deg_s'][1],lambda r:r['steps']['climb']['settling_s'],lambda r:r['gusts']['vertical']['recovery_after_off_s'],lambda r:r['shaft_Wh_per_ground_km']]]
        lines.append(f'| {name} | {len(good)}/{len(rr)} | '+' | '.join(f'{v:.3f}' for v in vals)+' |')
    lines+=['','Median paired changes are calculated per case, not as a ratio of table medians. Negative is lower loss/effort.', '',
      '| Comparator | Metric | Min change % | Median % | Max % | Improved cases |','|---|---|---:|---:|---:|---:|']
    for base,c in result['comparisons'].items():
        for key,v in c['changes'].items():lines.append(f'| {base} | {key} | {v["minimum"]:+.2f} | {v["median"]:+.2f} | {v["maximum"]:+.2f} | {v["improved"]}/12 |')
    lines+=['','## Missions','','| Mode | Controller | Altitude RMS m | Cross-track RMS m | Relative rate RMS °/s | Shaft W | Qualified / runs |','|---|---|---:|---:|---:|---:|---:|']
    for mode in ['AUTO','CRUISE']:
        for name in names:
            rr=[r for r in missions if r['mode']==mode and r['variant']==name];good=[r for r in rr if r['complete'] and not r['screen_failures']]
            if not good:continue
            vals=[np.median([r['disturbed'][k] for r in good]) for k in ['altitude_rms_m','xtrack_rms_m','relative_rate_rms_deg_s']]+[np.median([r['shaft_W'] for r in good])]
            lines.append(f'| {mode} | {name} | '+' | '.join(f'{v:.3f}' for v in vals)+f' | {len(good)}/{len(rr)} |')
    lines+=['','## Ten-minute CRUISE checks','','Final minute shown; three cases per controller.','','| Controller | Shaft W | True airspeed m/s | Common brake fraction | Altitude RMS m | Qualified / runs |','|---|---:|---:|---:|---:|---:|']
    for name in ['published',chosen]:
        rr=[r for r in endurance if r['variant']==name];good=[r for r in rr if r['complete'] and not r['screen_failures']]
        if not good:continue
        vals=[np.median([r[k] for r in good]) for k in ['final_shaft_W','final_true_airspeed_m_s','final_common_brake_fraction']]+[np.median([r['late']['altitude_rms_m'] for r in good])]
        lines.append(f'| {name} | '+' | '.join(f'{v:.3f}' for v in vals)+f' | {len(good)}/{len(rr)} |')
    lines+=['','## Limits','','Finite scenario screening does not prove formal stability or flight readiness. Shaft power is modeled, not measured battery consumption. Physics remains the same as-built model. No enhanced sensors or observer changes are included. The local comparison includes its earlier zero-trim/EKF work and is a comparator, not a gain-only ablation.', '',
      'The corrected training-only decision is in `selection-with-speed-task.json`, frozen before any held-out result. The preliminary symmetric speed-ratio gate also scheduled efficient_brake10k as an additional comparator; its hold-outs are retained but cannot alter the frozen winner.', '', '![Paired validation](validation-pairs.png)']
    (HERE/'RESULTS.md').write_text('\n'.join(lines)+'\n')
    if not chosen or not validation:return
    fig,axes=plt.subplots(1,3,figsize=(12,4),layout='constrained')
    for ax,(label,fn) in zip(axes,[('Performance loss',lambda r:r['score']),('Command relative pitch-rate peak (°/s)',lambda r:r['command_peak_rates_deg_s'][0]),('Shaft energy (Wh/km)',lambda r:r['shaft_Wh_per_ground_km'])]):
        groups=[{r['case']:r for r in validation if r['variant']==name and r['score'] is not None} for name in ['published',chosen]]
        ids=sorted(groups[0].keys() & groups[1].keys())
        for i in ids:ax.plot([0,1],[fn(g[i]) for g in groups],color='#97a8b8',alpha=.7,marker='o',ms=3)
        if ids:ax.plot([0,1],[np.median([fn(g[i]) for i in ids]) for g in groups],color='#cf4c27',lw=3,marker='o')
        ax.set(xticks=[0,1],xticklabels=['Published','Retuned LQI'],title=label,xlim=(-.2,1.2));ax.grid(axis='y',alpha=.2)
    fig.suptitle('Fresh paired validation — original sensing and corrected gust incidence')
    fig.savefig(HERE/'validation-pairs.png',dpi=160)
if __name__=='__main__':main()
