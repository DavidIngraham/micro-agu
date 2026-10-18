"""AI-assisted multi-objective report; no hidden removal of failed cases."""
import importlib.util,json,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from build import HERE
spec=importlib.util.spec_from_file_location('retained_power',HERE.parent/'proprioception-2026-10-17/analyze.py')
power=importlib.util.module_from_spec(spec);spec.loader.exec_module(power)
def main():
    rows=[]
    for p in sorted(HERE.glob('*/*/*/*-metrics.json')):
        m=json.loads(p.read_text());raw=p.with_name(p.name.replace('-metrics',''));d=json.loads(raw.read_text());cohort,variant=p.relative_to(HERE).parts[:2]
        row=dict(cohort=cohort,variant=variant,rate=m['rate'],case=m['id'],passed=m['passed'],complete=m['complete'],reasons=m['reasons'],source=str(raw.relative_to(HERE)))
        if m['complete']:
            ss=d['samples'];t=np.array([s['time'] for s in ss]);x=np.array([s['truth'] for s in ss]);rot=R.from_quat(x[:,3:7]);wind=np.array([s['wind'] for s in ss]);speed=np.linalg.norm(x[:,7:10]-wind,axis=1);watts=power.shaft_power(np.array([s['rpm'] for s in ss]),rot.inv().apply(x[:,7:10]-wind)[:,0]);duration=t[-1]-t[0];distance=np.trapezoid(speed,t);energy=np.trapezoid(watts,t)
            turns=[m['phases'][k] for k in ['turn1','reverse','turn2']];rates=[p['mean_rate'] for p in turns]
            row.update(right=float(np.mean([r for r in rates if r>0])),left=float(np.mean([r for r in rates if r<0])),max_mean_error=max(p['mean_error'] for p in turns),alt_peak=max(p['alt_peak'] for p in turns),pitch_peak=max(p['relative_pitch_rate_peak'] for p in turns),pitch_late=max(p['relative_pitch_rate_late_rms'] for p in turns),turn_variability=max(p['rate_std'] for p in turns),min_speed=min(p['speed_min'] for p in turns),shaft_W=float(energy/duration),shaft_Wh_per_air_km=float(energy/3.6/distance),speed=float(np.trapezoid(speed,t)/duration),recovery_rate=max(m['phases'][k]['rate_rms'] for k in ['recover1','recover2']))
        rows.append(row)
    (HERE/'results.json').write_text(json.dumps(rows,indent=2))
    lines=['# Feasible-turn references and anti-windup results','','AI-assisted. All candidates retain the exact published K and L gains. Power is modeled shaft power, not battery energy. Absolute turbulent motion screens are retained even where baseline fails.','','| Cohort | Variant | Rate | Case | Pass | Right / left deg/s | Alt peak m | Pitch-rate peak deg/s | Shaft W | Reasons |','|---|---|---:|---:|---|---|---:|---:|---:|---|']
    for r in rows:
        values=f'{r["right"]:.2f} / {r["left"]:.2f} | {r["alt_peak"]:.2f} | {r["pitch_peak"]:.2f} | {r["shaft_W"]:.2f}' if r['complete'] else '— | — | — | —'
        lines.append(f'| {r["cohort"]} | {r["variant"]} | {r["rate"]} | {r["case"]} | {r["passed"]} | {values} | '+ '; '.join(r['reasons'])+' |')
    (HERE/'RESULTS.md').write_text('\n'.join(lines)+'\n')
    groups=[]
    for cohort,variant,rate in sorted({(r['cohort'],r['variant'],r['rate']) for r in rows}):
        group=[r for r in rows if (r['cohort'],r['variant'],r['rate'])==(cohort,variant,rate)]
        good=[r for r in group if r['complete']]
        summary=dict(cohort=cohort,variant=variant,rate=rate,runs=len(group),passed=sum(r['passed'] for r in group),completed=len(good))
        for key in ['right','left','max_mean_error','alt_peak','pitch_peak','pitch_late','turn_variability','shaft_W','shaft_Wh_per_air_km','recovery_rate']:
            if good:summary[key]=float(np.median([r[key] for r in good]))
        groups.append(summary)
    (HERE/'summary.json').write_text(json.dumps(groups,indent=2))
    nominal=[r for r in rows if r['cohort']=='ablation' and r['case']==0 and r['complete']]
    if nominal:
        fig,axes=plt.subplots(3,1,figsize=(12,9),sharex=True,layout='constrained')
        for r in nominal:
            d=json.loads((HERE/r['source']).read_text());ss=d['samples'];t=np.array([s['time'] for s in ss]);x=np.array([s['truth'] for s in ss]);diag=np.array([s['diagnostic'] for s in ss]);br=np.array([s['brakes'] for s in ss])/(np.array([1300,1500])*np.pi*55/360*.1)
            axes[0].plot(t,diag[:,2],label=r['variant']);axes[1].plot(t,-x[:,2]-60);axes[2].plot(t,br.min(axis=1)*100)
        axes[0].plot(t,diag[:,1],'k--',lw=1,label='Reference');axes[0].legend(ncol=3)
        for ax,label in zip(axes,['Heading rate (deg/s)','Altitude error (m)','Common brake (%)']):ax.set_ylabel(label);ax.grid(alpha=.25)
        axes[-1].set_xlabel('Time (s)');axes[0].set_title('25 deg/s turn/reversal ablations — unchanged feedback and observer gains');fig.savefig(HERE/'ablation.png',dpi=150)
    print(json.dumps(groups,indent=2))
if __name__=='__main__':main()
