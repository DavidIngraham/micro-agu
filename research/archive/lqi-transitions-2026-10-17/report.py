"""AI-assisted post-selection analysis, with censored outcomes retained."""
import importlib.util
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from score import analyze_phase
HERE=Path(__file__).resolve().parent
OLD=HERE.parent/'lqi-balanced-2026-10-17'
spec=importlib.util.spec_from_file_location('oldreport',OLD/'report.py')
old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)

def main():
    selection=json.loads((HERE/'selection.json').read_text());chosen=selection['selected']
    rows=analyze_phase(HERE/'validation');abl=analyze_phase(HERE/'ablation')
    missions=[old.mission_metrics(p) for p in (HERE/'missions').glob('*/*.json') if p.stem.isdigit()]
    (HERE/'missions/metrics.json').write_text(json.dumps(missions,indent=2))
    names=list(dict.fromkeys(['standard','fixed',chosen]))
    lines=['# Transition and sensing experiment results','',f'Selected on training: **{chosen}**. `fixed` is the previous responsive LQI, not the older fixed-gain baseline.','',
      'All results are simulation-only. The EKF remains active for navigation, reference generation, and speed/turn integrators. Full-state truth replaces the 16 inner feedback states as a diagnostic only.','',
      '## Held-out command and gust cases','',
      '| Variant | Runs / gates passed | Score median | Worst score | Climb rise s | Command to 90% s | Climb settling s | Climb peak °/s | Descent peak °/s | Updraft recovery s | Shaft Wh/km |',
      '|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|']
    if len(abl)<24:lines[2:2]=[f'**Campaign in progress: sensing ablations {len(abl)}/24.**','']
    def median(group,fn):
        vals=[fn(r) for r in group if r['complete']];vals=[x for x in vals if x is not None]
        return f'{np.median(vals):.3f}' if vals else 'n/a'
    for name in names:
        rr=[r for r in rows if r['variant']==name];scores=[r['score'] for r in rr if r['score'] is not None]
        values=[median(rr,lambda r:r['score']),f'{max(scores):.3f}' if scores else 'n/a',
          median(rr,lambda r:r['steps']['climb']['rise_10_90_s']),
          median(rr,lambda r:r['steps']['climb']['command_to_90_s']),
          median(rr,lambda r:r['steps']['climb']['settling_s']),
          median(rr,lambda r:r['command_peak_rates_deg_s'][0]),median(rr,lambda r:r['command_peak_rates_deg_s'][1]),
          median(rr,lambda r:r['gusts']['vertical']['recovery_after_off_s']),median(rr,lambda r:r['shaft_Wh_per_ground_km'])]
        lines.append(f'| {name} | {len(rr)} / {len(scores)} | '+' | '.join(values)+' |')
    comparisons={};lookup={(r['variant'],r['case']):r for r in rows};gates=[]
    for base in ['fixed','standard']:
        pairs=[(r,lookup[base,r['case']]) for r in rows if r['variant']==chosen]
        for a,b in pairs:
            if a['gates']:gates.append(dict(case=a['case'],reasons=a['gates']))
        good=[(a,b) for a,b in pairs if a['score'] is not None and b['score'] is not None]
        comparisons[base]={key:float(np.median([100*(fn(a)/fn(b)-1) for a,b in good])) for key,fn in {
          'score':lambda r:r['score'],'energy':lambda r:r['shaft_Wh_per_ground_km'],
          'climb_peak':lambda r:r['command_peak_rates_deg_s'][0],
          'descent_peak':lambda r:r['command_peak_rates_deg_s'][1],
          'acquisition_time':lambda r:r['steps']['climb']['rise_10_90_s']}.items()} if good else {}
        if base=='fixed' and good:
            for task in ['acquisition','descent','updraft','crosswind']:
                change=float(np.median([a['tasks'][task]/b['tasks'][task] for a,b in good]))
                if change>1.2:gates.append(dict(task=task,median_ratio=change))
            if comparisons[base]['acquisition_time']>10:gates.append('held-out acquisition speed >10% worse')
            if comparisons[base]['energy']>20:gates.append('held-out energy >20% worse')
    for r in missions:
        if r['variant']==chosen and (not r['complete'] or r.get('screen_failures')):gates.append(r)
    for mode in ['AUTO','CRUISE']:
        aa=[r for r in missions if r['variant']==chosen and r['mode']==mode and r['complete']]
        bb={r['case']:r for r in missions if r['variant']=='fixed' and r['mode']==mode and r['complete']}
        if len(aa)!=4 or len(bb)!=4:gates.append(mode+' missing pairs');continue
        for key in ['altitude_rms_m','xtrack_rms_m']:
            ratio=float(np.median([a['disturbed'][key]/max(bb[a['case']]['disturbed'][key],1) for a in aa]))
            if ratio>1.2:gates.append(dict(mode=mode,metric=key,ratio=ratio))
    sensitivity={}
    for emphasis,weights in {'equal':[.25]*4,'acquisition':[.55,.15,.15,.15],
          'descent':[.15,.55,.15,.15],'updraft':[.15,.15,.55,.15],'crosswind':[.15,.15,.15,.55]}.items():
        for motion_weight in [.1,.2,.35]:
            def rescore(r):
                return ((.85-motion_weight)*np.dot(weights,list(r['tasks'].values()))+
                    motion_weight*r['components']['motion']+.1*r['components']['actuator']+.05*r['components']['energy'])
            sensitivity[f'{emphasis}_motion_{motion_weight}']={}
            for base in ['fixed','standard']:
                pairs=[(r,lookup[base,r['case']]) for r in rows if r['variant']==chosen and r['score'] is not None and lookup[base,r['case']]['score'] is not None]
                sensitivity[f'{emphasis}_motion_{motion_weight}'][base]=float(np.median([100*(rescore(a)/rescore(b)-1) for a,b in pairs])) if pairs else None
    lines+=['','## Training selection','','| Candidate | Robust score | Change vs prior LQI | Rejection |','|---|---:|---:|---|']
    for name,c in selection['candidates'].items():
        lines.append(f'| {name} | {c["score"]:.3f} | {-c["improvement_pct"]:.2f}% | '+('; '.join(c['reasons']) or 'none')+' |')
    lines+=['','Median paired changes (negative means less):','','```json',json.dumps(comparisons,indent=2),'```','',
      'Validation rejection reasons: '+json.dumps(gates), '', '## Sensing ablations', '',
      'These reuse training physics and are diagnostic, excluded from selection. `ideal`: noiseless zero-delay angle into the same observer; `nodelay`: 0.5° noise with zero delay; `stress`: 2° and 80 ms; `truth`: correctly mapped full feedback state, no observer.','',
      '| Variant | Complete / gate pass | Score median | Climb peak °/s | Descent peak °/s | Updraft recovery s |','|---|---:|---:|---:|---:|---:|']
    for name in sorted({r['variant'] for r in abl}):
        rr=[r for r in abl if r['variant']==name]
        lines.append(f'| {name} | {sum(r["complete"] for r in rr)} / {sum(r["score"] is not None for r in rr)} | '+' | '.join([
          median(rr,lambda r:r['score']),median(rr,lambda r:r['command_peak_rates_deg_s'][0]),
          median(rr,lambda r:r['command_peak_rates_deg_s'][1]),median(rr,lambda r:r['gusts']['vertical']['recovery_after_off_s'])])+' |')
    lines+=['','## Missions','','| Mode | Variant | Complete | Alt RMS m | Xtrack RMS m | Relative rate RMS °/s | Shaft W |','|---|---|---:|---:|---:|---:|---:|']
    for mode in ['AUTO','CRUISE']:
        for name in names:
            rr=[r for r in missions if r['mode']==mode and r['variant']==name]
            values=[median(rr,lambda r,k=k:r['disturbed'][k]) for k in ['altitude_rms_m','xtrack_rms_m','relative_rate_rms_deg_s']]
            values.append(median(rr,lambda r:r['shaft_W']))
            lines.append(f'| {mode} | {name} | {sum(r["complete"] for r in rr)}/{len(rr)} | '+' | '.join(values)+' |')
    lines+=['','## Preference sensitivity','','Weights changed only after selection; no reselection. Task emphasis gives that task 55% of the task subscore, others 15%. Actuator/energy remain 10%/5%; task weight fills the remainder.','','| Task emphasis / motion weight | Selected vs previous LQI % | Selected vs standard % |','|---|---:|---:|']
    for name,values in sensitivity.items():
        lines.append('| '+name+' | '+' | '.join(f'{values[k]:+.2f}' if values[k] is not None else 'n/a' for k in ['fixed','standard'])+' |')
    lines+=['','![Held-out distributions](validation-distribution.png)','','![First held-out case](response-60.png)','',
      'Finite scenario screens are not stability margins. Shared local linearization, nonlinear unilateral allocation, model uncertainty, and unequal lateral reference caps remain limitations. Shaft energy is not measured battery energy.']
    (HERE/'RESULTS.md').write_text('\n'.join(lines)+'\n')
    (HERE/'summary.json').write_text(json.dumps(dict(selected=chosen,comparisons=comparisons,rejections=gates,sensitivity=sensitivity),indent=2))
    fig,axes=plt.subplots(2,2,figsize=(12,7),constrained_layout=True)
    for name in names:
        d=json.loads((HERE/f'validation/{name}/60.json').read_text());s=d['samples']
        if not s:continue
        t=np.array([v['time'] for v in s]);y=np.array([v['truth'] for v in s]);p=R.from_quat(y[:,3:7])
        rate=p.inv().apply(y[:,23:26]-y[:,10:13])[:,1]*180/np.pi
        for j,(lo,hi) in enumerate([(15,70),(115,170)]):
            m=(t>=lo)&(t<hi);axes[0,j].plot(t[m],-y[m,2],label=name);axes[1,j].plot(t[m],rate[m],label=name)
            axes[0,j].set_ylabel('Altitude (m)');axes[1,j].set_ylabel('Relative pitch rate (°/s)')
            for ax in axes[:,j]:ax.set_xlabel('Time (s)');ax.grid(alpha=.2)
    axes[0,0].legend();fig.suptitle('Held-out seed 60: acquisition and canopy motion')
    fig.savefig(HERE/'response-60.png',dpi=150);plt.close(fig)
    fig,axes=plt.subplots(1,4,figsize=(13,4),constrained_layout=True)
    for ax,(label,getter) in zip(axes,[('Score',lambda r:r['score']),
        ('Climb peak (°/s)',lambda r:r['command_peak_rates_deg_s'][0]),
        ('Updraft recovery (s)',lambda r:r['gusts']['vertical']['recovery_after_off_s']),
        ('Shaft Wh / ground km',lambda r:r['shaft_Wh_per_ground_km'])]):
        for j,name in enumerate(names):
            vals=[getter(r) for r in rows if r['variant']==name and r['complete']];vals=[v for v in vals if v is not None]
            if vals:
                ax.scatter(j+np.linspace(-.12,.12,len(vals)),vals,s=20,alpha=.65)
                ax.plot([j-.2,j+.2],[np.median(vals)]*2,color='black')
        ax.set_title(label);ax.set_xticks(range(len(names)),names,rotation=20);ax.grid(axis='y',alpha=.2)
    fig.suptitle('Eight fresh physics cases per controller; black line = median')
    fig.savefig(HERE/'validation-distribution.png',dpi=150);plt.close(fig)
    print(json.dumps(dict(selected=chosen,comparisons=comparisons,rejections=gates),indent=2))

if __name__=='__main__':main()
