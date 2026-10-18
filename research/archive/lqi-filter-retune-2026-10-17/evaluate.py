"""Predeclared selection and held-out release gates; no validation retuning."""
import importlib.util
import json
from pathlib import Path
import numpy as np
from score import analyze_phase
HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('mission_metrics',HERE.parent/'lqi-balanced-2026-10-17/report.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
TASKS=['acquisition','descent','updraft','crosswind']
def endurance_metrics(path):
    r=m.mission_metrics(path)
    if not r['complete']:return r
    d=json.loads(path.read_text());ss=[s for s in d['samples'] if s['time']>=540]
    y=np.array([s['truth'] for s in ss]);wind=np.array([s['wind'] for s in ss])
    rp,rc=m.R.from_quat(y[:,3:7]),m.R.from_quat(y[:,16:20])
    rate=rp.inv().apply(y[:,23:26]-y[:,10:13])*180/np.pi
    angle=(rp.inv()*rc).as_rotvec()*180/np.pi
    r['late']={k:float(np.sqrt(np.mean(v*v))) for k,v in dict(
        altitude_rms_m=-y[:,2]-np.array([s['targetAltitude'] for s in ss]),
        xtrack_rms_m=np.array([s['crossTrack'] for s in ss]),relative_pitch_rate_rms_deg_s=rate[:,1]).items()}
    r['late']['relative_pitch_std_deg']=float(np.std(angle[:,1]))
    if r['late']['relative_pitch_std_deg']>1 and r['late']['relative_pitch_rate_rms_deg_s']>2:r['screen_failures'].append('final-minute persistent motion')
    axial=rp.inv().apply(y[:,7:10]-wind)[:,0]
    r['final_shaft_W']=float(np.mean(m.power.shaft_power(np.array([s['rpm'] for s in ss]),axial)))
    r['final_true_airspeed_m_s']=float(np.mean(np.linalg.norm(y[:,7:10]-wind,axis=1)))
    brakes=np.array([s['brakes'] for s in ss])/(np.array([1300,1500])*np.pi*55/360*.1)
    r['final_common_brake_fraction']=float(np.mean(brakes.min(axis=1)))
    return r
def compare(rows,name,base,count):
    a={r['case']:r for r in rows if r['variant']==name};b={r['case']:r for r in rows if r['variant']==base}
    reasons=[];changes={}
    if len(a)!=count or set(a)!=set(b):return dict(reasons=['missing pairs'],changes={})
    if any(r['score'] is None for r in list(a.values())+list(b.values())):
        return dict(reasons=['operational screen failure'],changes={})
    for key,fn in dict(score=lambda r:r['score'],energy=lambda r:r['shaft_Wh_per_ground_km'],
          acquisition_time=lambda r:r['steps']['climb']['rise_10_90_s'],
          **{t:(lambda r,k=t:r['tasks'][k]) for t in TASKS}).items():
        v=[100*(fn(a[i])/fn(b[i])-1) for i in a]
        changes[key]=dict(median=float(np.median(v)),minimum=min(v),maximum=max(v),improved=sum(x<0 for x in v))
    if changes['score']['median']>-5:reasons.append('score improvement <5%')
    if changes['energy']['median']>5:reasons.append('energy regression >5%')
    if changes['acquisition_time']['median']>10:reasons.append('acquisition time regression >10%')
    for t in TASKS:
        if changes[t]['median']>20:reasons.append(t+' regression >20%')
    if changes['score']['maximum']>20:reasons.append('individual score regression >20%')
    # Prevent apparent efficiency obtained simply by abandoning the speed task.
    for task in ['climb','descent']:
        ratios=[a[i]['steps'][task]['mean_speed_m_s']/b[i]['steps'][task]['mean_speed_m_s'] for i in a]
        if min(ratios)<.9:reasons.append(task+' mean true speed drops >10%')
        if max(a[i]['steps'][task]['mean_speed_m_s'] for i in a)>4.84:reasons.append(task+' mean true speed exceeds 4.4 m/s target by >10%')
    tail=lambda rr:float(np.mean(sorted(r['score'] for r in rr.values())[-max(1,len(rr)//4):]))
    if tail(a)>tail(b):reasons.append('worst-quarter score regression')
    return dict(reasons=reasons,changes=changes)
def select(output='selection.json'):
    rows=analyze_phase(HERE/'training');names=sorted({r['variant'] for r in rows}-{'published'})
    results={n:compare(rows,n,'published',6) for n in names}
    for n in names:
        rr=[r['score'] for r in rows if r['variant']==n]
        results[n]['rank']=float(.5*np.mean(rr)+.5*max(rr)) if all(x is not None for x in rr) else None
    eligible=[n for n in names if not results[n]['reasons']]
    chosen=min(eligible,key=lambda n:results[n]['rank']) if eligible else None
    result=dict(selected=chosen,candidates=results,rule='Training only; operational, task, energy and worst-quarter gates before 50% mean + 50% worst score ranking.')
    (HERE/output).write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
    return chosen
def report():
    chosen=json.loads((HERE/'selection.json').read_text())['selected']
    rows=analyze_phase(HERE/'validation') if (HERE/'validation').exists() else []
    missions=[m.mission_metrics(p) for p in (HERE/'missions').glob('*/*.json') if p.stem.isdigit()]
    endurance=[endurance_metrics(p) for p in (HERE/'endurance').glob('*/*.json') if p.stem.isdigit()]
    result=dict(selected=chosen,accepted=False,reasons=[],comparisons={})
    if not chosen:result['reasons'].append('No candidate qualified on training')
    else:
        for base in ['published','local']:
            c=compare(rows,chosen,base,12);result['comparisons'][base]=c
            if base=='published':result['reasons']+=['validation '+x for x in c['reasons']]
        for mode in ['AUTO','CRUISE']:
            a={r['case']:r for r in missions if r['variant']==chosen and r['mode']==mode}
            b={r['case']:r for r in missions if r['variant']=='published' and r['mode']==mode}
            if len(a)!=6 or set(a)!=set(b):result['reasons'].append(mode+' missing pairs');continue
            if any(not r['complete'] or r.get('screen_failures') for r in list(a.values())+list(b.values())):
                result['reasons'].append(mode+' operational failure');continue
            for k in ['altitude_rms_m','xtrack_rms_m']:
                if np.median([a[i]['disturbed'][k]/max(b[i]['disturbed'][k],1) for i in a])>1.2:
                    result['reasons'].append(mode+' '+k+' regression >20%')
            if np.median([a[i]['shaft_W']/b[i]['shaft_W'] for i in a])>1.05:result['reasons'].append(mode+' power regression >5%')
        a={r['case']:r for r in endurance if r['variant']==chosen};b={r['case']:r for r in endurance if r['variant']=='published'}
        if len(a)!=3 or set(a)!=set(b):result['reasons'].append('endurance missing pairs')
        elif any(not r['complete'] or r.get('screen_failures') for r in list(a.values())+list(b.values())):result['reasons'].append('endurance operational failure')
        else:
            if np.median([a[i]['shaft_W']/b[i]['shaft_W'] for i in a])>1.05:result['reasons'].append('endurance power regression >5%')
            if np.median([a[i]['final_shaft_W']/b[i]['final_shaft_W'] for i in a])>1.05:result['reasons'].append('endurance final-minute power regression >5%')
            if min(a[i]['final_true_airspeed_m_s']/b[i]['final_true_airspeed_m_s'] for i in a)<.9:result['reasons'].append('endurance final-minute speed drop >10%')
            for k in ['altitude_rms_m','xtrack_rms_m']:
                if np.median([a[i]['late'][k]/max(b[i]['late'][k],1) for i in a])>1.2:result['reasons'].append('endurance '+k+' regression >20%')
    result['accepted']=not result['reasons']
    (HERE/'acceptance.json').write_text(json.dumps(result,indent=2))
    (HERE/'mission-metrics.json').write_text(json.dumps(missions,indent=2))
    (HERE/'endurance-metrics.json').write_text(json.dumps(endurance,indent=2))
    print(json.dumps(result),flush=True)
    return result
if __name__=='__main__':report()
