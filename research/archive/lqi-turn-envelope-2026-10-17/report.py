"""AI-assisted turn envelope report; retains failed cases and separate cohorts."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from build import HERE

def main():
    records=[]
    for cohort in ['screen','nominal','diagnostic','comparison','validation']:
        for p in sorted((HERE/cohort).glob('*/*-metrics.json')):
            r=json.loads(p.read_text());r.update(cohort=cohort,file=str(p.relative_to(HERE)));records.append(r)
    (HERE/'results.json').write_text(json.dumps(records,indent=2))
    lines=['# Fixed-gain LQI turn envelope experiment','','AI-assisted. Same published gains, observer, trim and sensing; only turn-reference maximum varies. CRUISE uses full stick, 40 deg/s² reference slew, sustained turns, direct reversals and stick-release recovery.','','The broad-envelope screen and calm-air exploration have separate protocols. A calm-air pass is not a robust-envelope qualification. Turbulent RMS motion includes forced response and does not by itself establish self-excited instability.','','| Cohort | Controller | Limit deg/s | Case | Pass | Achieved steady rates deg/s | Peak altitude error m | Reasons |','|---|---|---:|---:|---|---|---:|---|']
    for r in records:
        phases=r.get('phases',{});turns=[phases[k] for k in ['turn1','reverse','turn2'] if k in phases]
        means=', '.join(f'{p["mean_rate"]:.1f}' for p in turns);alt=max([p['alt_peak'] for p in turns],default=float('nan'))
        lines.append(f'| {r["cohort"]} | {r["scheme"]} | {r["rate"]} | {r["id"]} | {r["passed"]} | {means} | {alt:.2f} | '+ '; '.join(r['reasons'])+' |')
    summary=['# Findings','','The fixed published LQI gains were tested through a 25 deg/s command. The 25 deg/s tracking gate failed, so the conditional extension above 25 was not run. Public firmware remains unchanged.','','| Controller | Requested magnitude deg/s | Right actual deg/s | Left actual deg/s | Nominal cases passing |','|---|---:|---:|---:|---:|']
    aggregate=[]
    for scheme in ['lqi','baseline']:
        for rate in sorted({r['rate'] for r in records if r['scheme']==scheme}):
            rr=[r for r in records if r['scheme']==scheme and r['rate']==rate and r['id']<2 and r['complete']]
            if not rr:continue
            pp=[v['mean_rate'] for r in rr for v in r['phases'].values() if 'mean_rate' in v]
            positive=float(np.mean([v for v in pp if v>0]));negative=float(np.mean([v for v in pp if v<0]));passed=sum(r['passed'] for r in rr)
            summary.append(f'| {scheme} | {rate} | {positive:.2f} | {negative:.2f} | {passed}/{len(rr)} |')
            aggregate.append(dict(scheme=scheme,rate=rate,right=positive,left=negative,nominal_passes=passed,cases=len(rr)))
    summary+=['','The calm-air 12 deg/s level passes the declared absolute tolerance. The 13 deg/s level narrowly crosses the 2 deg/s mean-error threshold; this is not a physical instability boundary. At 15–25 deg/s the right-turn bias grows while altitude, airspeed and recovery remain bounded. No qualified broad-envelope increase is claimed.','','The original eight-case 10 deg/s screen includes six simultaneous model/weather perturbations. Absolute rate/motion limits fail in turbulent cases at the existing limit as well. These failures include forced motion; they do not prove self-excited instability. No higher-rate held-out qualification was started after the screening failures.','','## Allocation mechanism','','At 15 deg/s in a steady right turn, the common-brake request is approximately −4.6% and the applied value is zero. The implemented inverse integral-gain anti-windup maps this residual into the turn integrator. Its correction is approximately −0.28 while the normalized turn error is +0.27: the turn error is nearly cancelled rather than integrated away. The speed integrator sits at its +50 bound. This is a direct arithmetic explanation of why integral action permits nonzero turn error here; it is not a separate intervention proving that changing anti-windup alone fixes the complete system. See `allocation-diagnosis.json`.','','Recommended next work: make turn/speed reference equilibria feasible under nonnegative common brake, and evaluate an allocator-aware anti-windup design that preserves turn tracking when common brake clips. Retest with the same paired protocol before raising the public cap.','','All results are simulation evidence for the as-built model, not a flight-tested envelope.','','---','']
    (HERE/'summary.json').write_text(json.dumps(aggregate,indent=2))
    (HERE/'RESULTS.md').write_text('\n'.join(summary+lines)+'\n')
    traces=[r for r in records if r['id']==0 and r['complete'] and r['scheme']=='lqi']
    if not traces:return
    fig,axes=plt.subplots(4,1,figsize=(12,10),sharex=True,layout='constrained')
    for r in traces:
        p=HERE/r['file'].replace('-metrics.json','.json');d=json.loads(p.read_text());ss=d['samples'];t=np.array([a['time'] for a in ss]);x=np.array([a['truth'] for a in ss]);rot=Rotation.from_quat(x[:,3:7]);e=rot.as_euler('xyz');b=rot.inv().apply(x[:,10:13]);yaw=np.rad2deg((b[:,1]*np.sin(e[:,0])+b[:,2]*np.cos(e[:,0]))/np.cos(e[:,1]));wind=np.array([a['wind'] for a in ss]);br=np.array([a['brakes'] for a in ss])/(np.array([1300,1500])*np.pi*55/360*.1)
        for ax,v in zip(axes,[yaw,-x[:,2]-60,np.linalg.norm(x[:,7:10]-wind,axis=1),np.max(br,axis=1)*100]):ax.plot(t,v,label=f'{r["rate"]}°/s',lw=1)
    for ax,label in zip(axes,['Actual heading rate (°/s)','Altitude error (m)','True airspeed (m/s)','Larger brake command (%)']):
        ax.set_ylabel(label);ax.grid(alpha=.25)
        for t in [20,70,120,160,200]:ax.axvline(t,color='gray',alpha=.25)
    axes[0].legend(ncol=6);axes[0].set_title('Fixed published LQI gains — calm-air sustained turns, reversals and recovery')
    axes[-1].set_xlabel('Time after settling (s)');fig.savefig(HERE/'turn-envelope.png',dpi=150)
if __name__=='__main__':main()
