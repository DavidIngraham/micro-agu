"""Training selection is frozen before this held-out assessment is run."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from score import HERE, power


def summary(x):
    x=np.asarray(x)
    return dict(median=float(np.median(x)),mean=float(np.mean(x)),worst=float(np.max(x)),
                worst_quarter=float(np.mean(np.sort(x)[-max(1,int(np.ceil(len(x)/4))):])))


def mission_metrics(path):
    d=json.loads(path.read_text());r=dict(variant=d['spec']['variant'],case=d['spec']['id'],mode=d['spec']['mode'],
        complete=d['complete'],failure=d['failure'])
    if not r['complete']:return r
    ss=d['samples']; t=np.array([s['time'] for s in ss]);y=np.array([s['truth'] for s in ss])
    alt=-y[:,2]-np.array([s['targetAltitude'] for s in ss]);cross=np.array([s['crossTrack'] for s in ss])
    rp,rc=R.from_quat(y[:,3:7]),R.from_quat(y[:,16:20]);rate=rp.inv().apply(y[:,23:26]-y[:,10:13])*180/np.pi
    angle=(rp.inv()*rc).as_rotvec()*180/np.pi
    for name,m in [('disturbed',(t>=20)&(t<100)),('late',t>=150)]:
        r[name]=dict(altitude_rms_m=float(np.sqrt(np.mean(alt[m]**2))),
            xtrack_rms_m=float(np.sqrt(np.mean(cross[m]**2))),
            relative_rate_rms_deg_s=float(np.sqrt(np.mean(np.sum(rate[m]**2,axis=1)))),
            relative_pitch_rate_rms_deg_s=float(np.sqrt(np.mean(rate[m,1]**2))),
            relative_pitch_std_deg=float(np.std(angle[m,1])))
    m=t>=20;wind=np.array([s['wind'] for s in ss]);axial=rp.inv().apply(y[:,7:10]-wind)[:,0]
    watts=power.shaft_power(np.array([s['rpm'] for s in ss]),axial)
    r['shaft_W']=float(np.trapezoid(watts[m],t[m])/(t[m][-1]-t[m][0]))
    brakes=np.array([s['brakes'] for s in ss])/(np.array([1300,1500])*np.pi*55/360*.1)
    r['upper_brake_fraction']=float(np.mean(np.any(brakes[m]>.99,axis=1)))
    r['min_altitude_m']=float(min(-y[:,2]));att=rp.as_euler('xyz',degrees=True)
    r['max_pitch_deg']=float(max(abs(att[:,1])));r['max_roll_deg']=float(max(abs(att[:,0])))
    r['screen_failures']=[]
    if r['max_roll_deg']>60 or r['max_pitch_deg']>45 or r['min_altitude_m']<3:r['screen_failures'].append('truth envelope')
    if r['upper_brake_fraction']>.05:r['screen_failures'].append('upper brake saturation')
    if r['late']['relative_pitch_std_deg']>1 and r['late']['relative_pitch_rate_rms_deg_s']>2:r['screen_failures'].append('late motion')
    return r


def main():
    selection=json.loads((HERE/'selection.json').read_text());chosen=selection['selected']
    train=json.loads((HERE/'training/metrics.json').read_text());valid=json.loads((HERE/'validation/metrics.json').read_text())
    missions=[mission_metrics(p) for p in sorted((HERE/'missions').glob('*/*.json')) if p.stem.isdigit()]
    (HERE/'missions/metrics.json').write_text(json.dumps(missions,indent=2)+'\n')
    names=list(dict.fromkeys(['standard','fixed',chosen]));paired={};gates=[]
    lookup={(r['variant'],r['case']):r for r in valid}
    for baseline in ['fixed','standard']:
        pairs=[(r,lookup[baseline,r['case']]) for r in valid if r['variant']==chosen]
        failed=[r['case'] for r,b in pairs if r['score'] is None or b['score'] is None]
        if failed:gates.append('transient operational failure: '+str(failed));continue
        changes={k:summary([100*(r['tasks'][k]/b['tasks'][k]-1) for r,b in pairs]) for k in ['acquisition','descent','updraft','crosswind']}
        changes['score']=summary([100*(r['score']/b['score']-1) for r,b in pairs])
        changes['energy']=summary([100*(r['shaft_Wh_per_ground_km']/b['shaft_Wh_per_ground_km']-1) for r,b in pairs])
        paired[baseline]=changes
        if baseline=='fixed':
            for k,v in changes.items():
                if k!='score' and v['median']>20:gates.append('held-out '+k+' median regression >20%')
    for r in missions:
        if r['variant']==chosen and (not r['complete'] or r['screen_failures']):gates.append('mission operational failure '+str(r['case']))
    for mode in ['AUTO','CRUISE']:
        aa=[r for r in missions if r['variant']==chosen and r['mode']==mode and r['complete']]
        bb={r['case']:r for r in missions if r['variant']=='fixed' and r['mode']==mode and r['complete']}
        if len(aa)!=4 or len(bb)!=4:gates.append(mode+' missing complete pairs');continue
        for k in ['altitude_rms_m','xtrack_rms_m']:
            ratio=np.median([r['disturbed'][k]/max(bb[r['case']]['disturbed'][k],1) for r in aa])
            if ratio>1.2:gates.append(mode+' '+k+' median regression >20% (1 m floor)')
    sensitivity={}
    profiles={'equal_tasks':[.25,.25,.25,.25], 'acquisition_first':[.55,.15,.15,.15],
              'descent_first':[.15,.55,.15,.15], 'updraft_first':[.15,.15,.55,.15],
              'crosswind_first':[.15,.15,.15,.55]}
    for label,weights in profiles.items():
        sensitivity[label]={}
        def rescore(r):
            return .65*np.dot(weights,list(r['tasks'].values()))+sum(w*r['components'][k] for w,k in
                [(.2,'motion'),(.1,'actuator'),(.05,'energy')])
        for baseline in ['fixed','standard']:
            pp=[(r,lookup[baseline,r['case']]) for r in valid if r['variant']==chosen and r['score'] is not None
                and lookup[baseline,r['case']]['score'] is not None]
            sensitivity[label][baseline]=float(np.median([100*(rescore(a)/rescore(b)-1) for a,b in pp])) if pp else None
    report=dict(selection=selection,validation_paired=paired,validation_rejection_reasons=gates,sensitivity=sensitivity,
        groups={n:summary([r['score'] for r in valid if r['variant']==n and r['score'] is not None]) for n in names})
    (HERE/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
    lines=['# Matched-governor tuning results','','AI-assisted. Selection used only the three training physics cases. '
        'Eight fresh cases were used afterward for command/gust validation, and the same new physics seeds '
        'with new weather were used for four AUTO and four CRUISE mission checks per configuration.', '',
        f'Selected on training: **{chosen}**. Validation: '+('**passes declared checks**.' if not gates else '**not qualified**: '+ '; '.join(gates)+'.'), '',
        'The selected candidate improves the declared aggregate and mission altitude tracking, but it does not '
        'dominate standard control: updraft recovery remains slower and altitude-command relative pitch-rate '
        'peaks remain substantially larger. The lower crosswind task loss reflects integrated response as well '
        'as peaks; it must not be described as a reduction in peak payload pitch. Passing the finite-horizon '
        'screens is not a proof of stability or a deployment recommendation.', '',
        '## Training', '', '| Candidate | Robust score | Improvement vs fixed % | Rejections |','|---|---:|---:|---|']
    for n,c in selection['candidates'].items():
        score_text=f'{c["score"]:.3f}' if c['score'] is not None else 'failed'
        improvement=f'{c["improvement_pct"]:+.2f}' if c['improvement_pct'] is not None else 'n/a'
        lines.append(f'| {n} | {score_text} | {improvement} | '+('; '.join(c['reasons']) or 'none')+' |')
    lines+=['','## Fresh command/gust cases','','Medians and worst cases are descriptive, not population confidence bounds.', '',
        '| Configuration | Qualified runs | Median score | Worst-quarter score | Worst score | Shaft Wh/ground km |',
        '|---|---:|---:|---:|---:|---:|']
    for n in names:
        rr=[r for r in valid if r['variant']==n];g=report['groups'][n]
        lines.append(f'| {n} | {sum(r["score"] is not None for r in rr)}/{len(rr)} | {g["median"]:.3f} | {g["worst_quarter"]:.3f} | {g["worst"]:.3f} | '
            f'{np.median([r["shaft_Wh_per_ground_km"] for r in rr if r["complete"]]):.3f} |')
    lines+=['','| Selected versus | Acquisition loss % | Descent loss % | Updraft loss % | Crosswind loss % | Total loss % | Energy % |',
        '|---|---:|---:|---:|---:|---:|---:|']
    for n,c in paired.items():lines.append('| '+n+' | '+' | '.join(f'{c[k]["median"]:+.2f}' for k in ['acquisition','descent','updraft','crosswind','score','energy'])+' |')
    lines+=['','| Response (median) | '+' | '.join(names)+' |','|---|'+'---:|'*len(names)]
    getters={'Climb 10–90%, s':lambda r:r['steps']['climb']['rise_10_90_s'],
        'Climb settling, s':lambda r:r['steps']['climb']['settling_s'],
        'Climb peak relative pitch rate, deg/s':lambda r:r['steps']['climb']['max_relative_pitch_rate_deg_s'],
        'Descent settling, s':lambda r:r['steps']['descent']['settling_s'],
        'Descent peak relative pitch rate, deg/s':lambda r:r['steps']['descent']['max_relative_pitch_rate_deg_s'],
        'Descent overshoot, m':lambda r:r['steps']['descent']['overshoot_m'],
        'Updraft height peak, m':lambda r:r['gusts']['vertical']['peak_height_excursion_m'],
        'Updraft recovery, s':lambda r:r['gusts']['vertical']['recovery_after_off_s'],
        'Crosswind pitch peak, deg':lambda r:r['gusts']['crosswind']['peak_pitch_excursion_deg'],
        'Crosswind recovery, s':lambda r:r['gusts']['crosswind']['recovery_after_off_s']}
    for label,func in getters.items():
        vals=[]
        for n in names:
            a=[func(r) for r in valid if r['variant']==n and r['complete']];good=[x for x in a if x is not None]
            vals.append(f'{np.median(good):.2f}'+(f' ({len(a)-len(good)} censored)' if len(good)<len(a) else '') if good else 'unsettled')
        lines.append('| '+label+' | '+' | '.join(vals)+' |')
    lines+=['','## Task preference sensitivity','','Selection remains frozen; these alternative preferences are reported only for interpretation. '
        'Values are median paired percent changes; negative is better. The emphasized task receives 55% '
        'of the task subscore, each other task 15%.', '',
        '| Preference | Selected vs fixed % | Selected vs standard % |','|---|---:|---:|']
    for label,v in sensitivity.items():lines.append(f'| {label} | {v["fixed"]:+.2f} | {v["standard"]:+.2f} |')
    lines+=['','## Fresh mission checks','','Truth altitude error and onboard cross-track during weather (20–100 s). '
        'These tests include acquisition; the new common caps apply from controller engagement.', '',
        '| Mode | Configuration | Complete | Alt RMS m | Xtrack RMS m | Relative rate RMS deg/s | Shaft W |',
        '|---|---|---:|---:|---:|---:|---:|']
    for mode in ['AUTO','CRUISE']:
        for n in names:
            rr=[r for r in missions if r['variant']==n and r['mode']==mode];good=[r for r in rr if r['complete']]
            vals=[np.median([r['disturbed'][k] for r in good]) for k in ['altitude_rms_m','xtrack_rms_m','relative_rate_rms_deg_s']]+[np.median([r['shaft_W'] for r in good])]
            lines.append(f'| {mode} | {n} | {len(good)}/{len(rr)} | '+' | '.join(f'{v:.3f}' for v in vals)+' |')
    lines+=['','## Limits','','Matched altitude rate caps do not make the internal reference trajectories identical. '
        'The LQI heading-rate cap remains 10 deg/s versus the standard PG_TURN_RMAX=25 setting; '
        'mission path results include that lateral implementation difference. '
        'All candidates share one locally identified model and one observer; the physics perturbations cover '
        'a model family, not unknown real canopy mechanics. No hardware qualification, statistical stability '
        'margin, or globally optimal controller is claimed. The reference-governor patch preserves original '
        'defaults; the experiment and generated gains were not deployed.', '',
        '![Validation task losses](validation.png)', '',
        '![First held-out case, fixed in advance](response-40.png)']
    (HERE/'RESULTS.md').write_text('\n'.join(lines)+'\n')
    fig,axes=plt.subplots(1,4,figsize=(13,4),constrained_layout=True)
    for ax,task in zip(axes,['acquisition','descent','updraft','crosswind']):
        for j,n in enumerate(names):
            vals=[r['tasks'][task] for r in valid if r['variant']==n and r['complete']]
            ax.bar(j,np.median(vals),color=['#c89337','#2b9a82','#4b80b5'][j])
            ax.scatter(j+np.linspace(-.12,.12,len(vals)),vals,color='black',alpha=.5,s=14)
        ax.set_xticks(range(len(names)),names,rotation=15);ax.set_title(task);ax.grid(axis='y',alpha=.2)
    fig.suptitle('Fresh validation: task losses, lower is better; all eight cases shown')
    fig.savefig(HERE/'validation.png',dpi=160);plt.close(fig)
    fig,axes=plt.subplots(3,2,figsize=(12,10),constrained_layout=True)
    for n,color in zip(names,['#c89337','#2b9a82','#4b80b5']):
        d=json.loads((HERE/f'validation/{n}/40.json').read_text())
        if not d['complete']:continue
        ss=d['samples'];t=np.array([s['time'] for s in ss]);y=np.array([s['truth'] for s in ss]);alt=-y[:,2]
        rp=R.from_quat(y[:,3:7]);rate=rp.inv().apply(y[:,23:26]-y[:,10:13])[:,1]*180/np.pi
        for i,(lo,hi,title) in enumerate([(10,215,'Altitude commands'),(215,260,'Updraft'),(255,319,'Crosswind')]):
            m=(t>=lo)&(t<hi)
            axes[i,0].plot(t[m],alt[m],label=n,color=color,lw=.9)
            axes[i,1].plot(t[m],rate[m],label=n,color=color,lw=.9)
            if n=='standard':axes[i,0].plot(t[m],[s['requestedAltitude'] for s in np.array(ss,dtype=object)[m]],'k--',lw=.6)
            for ax in axes[i]:ax.grid(alpha=.2);ax.set_xlabel('Time (s)');ax.set_title(title)
            axes[i,0].set_ylabel('Truth altitude (m)');axes[i,1].set_ylabel('Relative pitch rate (deg/s)')
    axes[0,0].legend();fig.suptitle('First held-out physics seed (40): command and gust trajectories')
    fig.savefig(HERE/'response-40.png',dpi=150);plt.close(fig)
    print('\n'.join(lines))


if __name__=='__main__':main()
