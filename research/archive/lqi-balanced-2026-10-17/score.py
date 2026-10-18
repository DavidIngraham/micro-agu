"""Predeclared dimensional task losses; stability and regression gates are separate."""
import importlib.util
import json
from pathlib import Path
import sys
import numpy as np
from scipy.signal import detrend, welch
from scipy.spatial.transform import Rotation as R

HERE=Path(__file__).resolve().parent
PREV=HERE.parent/'proprioception-2026-10-17'
sys.path.insert(0,str(PREV))
import transient_analyze as transient
transient.HERE=HERE
spec=importlib.util.spec_from_file_location('prior_power_assessment',PREV/'analyze.py')
power=importlib.util.module_from_spec(spec);spec.loader.exec_module(power)


def assess(path):
    r,_=transient.assess(path)
    if not r['complete']:
        r.update(score=None,gates=['incomplete flight']);return r
    d=json.loads(path.read_text());ss=d['samples']
    t=np.array([s['time'] for s in ss]);y=np.array([s['truth'] for s in ss])
    rp,rc=R.from_quat(y[:,3:7]),R.from_quat(y[:,16:20])
    angle=(rp.inv()*rc).as_rotvec()*180/np.pi
    rate=rp.inv().apply(y[:,23:26]-y[:,10:13])*180/np.pi
    m=t>300
    band_rms=float(np.sqrt(np.mean(detrend(angle[m,1])**2)))
    late_rate=float(np.sqrt(np.mean(rate[m,1]**2)))
    controls=np.array([s['command'] for s in ss])/np.r_[1,np.array([1300,1500])*np.pi*55/360*.1]
    brakes=np.array([s['brakes'] for s in ss])/(np.array([1300,1500])*np.pi*55/360*.1)
    travel=np.abs(np.diff(controls,axis=0)).sum(axis=0)/(t[-1]-t[0])
    wind=np.array([s['wind'] for s in ss]);axial=rp.inv().apply(y[:,7:10]-wind)[:,0]
    watts=power.shaft_power(np.array([s['rpm'] for s in ss]),axial)
    distance=np.trapezoid(np.linalg.norm(y[:,7:9],axis=1),t)
    wh_km=float(np.trapezoid(watts,t)/3600/distance*1000)
    r.update(late_relative_pitch_rms_deg=band_rms,late_relative_pitch_rate_rms_deg_s=late_rate,
        mean_shaft_W=float(np.trapezoid(watts,t)/(t[-1]-t[0])),shaft_Wh_per_ground_km=wh_km,
        travel_per_s=travel.tolist(),upper_brake_fraction=float(np.mean(np.any(brakes>.99,axis=1))),
        common_brake_fraction=float(np.mean(brakes.min(axis=1))))
    gates=[]
    if r['full_brake_longest_s']>5 or r['upper_brake_fraction']>.05:gates.append('upper brake saturation')
    if band_rms>1 and late_rate>2:gates.append('persistent late longitudinal motion')
    if np.max(abs(rp.as_euler('xyz',degrees=True)[:,0]))>60 or r['max_pitch_deg']>45 or min(-y[:,2])<3:gates.append('truth envelope')
    for s in r['steps'].values():
        if s['settling_s'] is None or s['rise_10_90_s'] is None:gates.append('unsettled altitude command')
    for s in r['gusts'].values():
        if s['recovery_after_off_s'] is None:gates.append('unrecovered gust')
    if d['scheme']=='lqi':
        diag=np.array([x['values'] for x in d['diagnostics']])
        climb=diag[:,4]
        assert np.max(abs(climb))<=1.00001
        assert np.max(climb)>.99 and np.min(climb)<-.99, 'Matched governor parameters not active'
        r['observed_reference_rate_bounds_m_s']=[float(min(climb)),float(max(climb))]
    def finite(v,limit):return limit if v is None else v
    c,e=r['steps']['climb'],r['steps']['descent'];u,w=r['gusts']['vertical'],r['gusts']['crosswind']
    tasks=dict(acquisition=float(np.mean([(finite(c['rise_10_90_s'],100)/12)**2,
        (finite(c['settling_s'],100)/25)**2,(c['iae_m_s']/120)**2])),
        descent=float(np.mean([(finite(e['settling_s'],100)/25)**2,(e['overshoot_m']/.5)**2,(e['iae_m_s']/120)**2])),
        updraft=float(np.mean([(u['peak_height_excursion_m']/3)**2,(finite(u['recovery_after_off_s'],35)/15)**2,(u['altitude_iae_m_s']/35)**2])),
        crosswind=float(np.mean([(w['peak_pitch_excursion_deg']/15)**2,(finite(w['recovery_after_off_s'],55)/8)**2,(w['altitude_iae_m_s']/15)**2])))
    motion=float(np.mean([(s['relative_pitch_rate_rms_deg_s']/15)**2 for s in r['gusts'].values()]))
    actuator=float(np.mean([(travel[0]/.2)**2,(np.mean(travel[1:])/.2)**2,(r['upper_brake_fraction']/.05)**2]))
    r.update(tasks=tasks,components=dict(task=float(np.mean(list(tasks.values()))),motion=motion,actuator=actuator,energy=(wh_km/5)**2),gates=gates)
    r['score']=float(np.dot([.65,.20,.10,.05],list(r['components'].values()))) if not gates else None
    return r


def analyze_phase(phase):
    rows=[assess(p) for p in sorted(phase.glob('*/*.json')) if p.stem.isdigit()]
    (phase/'metrics.json').write_text(json.dumps(rows,indent=2,allow_nan=False)+'\n')
    return rows


def select(phase):
    rows=analyze_phase(phase)
    base={r['case']:r for r in rows if r['variant']=='fixed'}
    assert len(base)==3 and all(r['score'] is not None for r in base.values())
    def robust(rr):
        scores=[r['score'] for r in rr]
        return float(.5*np.mean(scores)+.5*np.max(scores))
    baseline=robust(list(base.values()));candidates={}
    for name in sorted({r['variant'] for r in rows}-{'standard','fixed'}):
        group=[r for r in rows if r['variant']==name];reasons=[]
        if len(group)!=3:reasons.append('missing cases')
        for r in group:
            reasons.extend(f'case {r["case"]}: {x}' for x in r['gates'])
        if not reasons:
            for task in ['acquisition','descent','updraft','crosswind']:
                ratios=[r['tasks'][task]/base[r['case']]['tasks'][task] for r in group]
                if np.median(ratios)>1.2:reasons.append(task+' median regression >20%')
            if np.median([r['shaft_Wh_per_ground_km']/base[r['case']]['shaft_Wh_per_ground_km'] for r in group])>1.2:
                reasons.append('energy regression >20%')
        score=robust(group) if all(r['score'] is not None for r in group) else None
        candidates[name]=dict(score=score,reasons=reasons,improvement_pct=100*(1-score/baseline) if score else None)
    eligible=[n for n,c in candidates.items() if not c['reasons'] and c['score']<.95*baseline]
    chosen=min(eligible,key=lambda n:candidates[n]['score']) if eligible else 'fixed'
    result=dict(selected=chosen,baseline_score=baseline,candidates=candidates,
        rule='Training only: complete/settled, operational gates, no >20% median task or energy regression vs matched fixed. Rank 50% mean + 50% worst case of dimensional loss, require >=5% improvement.')
    (HERE/'selection.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result),flush=True);return result
