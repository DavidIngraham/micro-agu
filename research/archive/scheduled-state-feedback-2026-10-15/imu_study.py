"""Restore accelerometer information for every compared controller; no native-EKF claim."""
import argparse
import itertools
import json
from concurrent.futures import ProcessPoolExecutor
import numpy as np
import experiment as ex
import observer as ob
from imu_observer import ImuObserver
from iterate import Shaped,stress

def job(args):
    case,config,family,variant,tag=args
    ImuObserver.config=config;ob.Observer=ImuObserver
    original=ex.Conventional
    if family=='direct':
        Shaped.variant=variant;ex.Scheduled=Shaped
        name=f'scheduled_imu-{tag}-q{config["state_q"]:g}-w{config["wind_q"]:g}-{variant["id"]}_observer'
    else:
        cfg=ex.BASE if family=='baseline' else json.loads((ex.OLD/'optimization.json').read_text())['candidate']
        ex.Conventional=lambda p,unused:original(p,cfg)
        name=f'{family}_imu-{tag}-q{config["state_q"]:g}-w{config["wind_q"]:g}_observer'
    row=ex.run_case(case,name)
    ex.Conventional=original
    row.update(config=config,family=family,variant=variant,seed=case['seed'],gust_multiplier=case['gust'],sensing='Noisy payload accelerometer/attitude/gyro 50 Hz; navigation 5 Hz; no canopy/rotor/wind measurements')
    e=np.array(ImuObserver.latest.errors);start=5 if case['name']=='maneuver' else 15
    row['state_error_rms']=np.sqrt(np.mean(e[np.arange(len(e))*ex.DT>=start]**2,axis=0)).tolist()
    ex.save(f'{case["name"]}-{name}.json',row)
    return row


def run():
    configs=[dict(state_q=q,wind_q=w) for q,w in itertools.product([0.,1e-7,1e-6],[.001,.01])]
    cases=[c for c in ex.scenarios() if c['name'] in ['maneuver','gust','voltage_ramp']]
    variant=dict(id='h1-d0.05',hidden=1.,damper=.05)
    jobs=[(case,config,family,variant,'development') for config in configs for case in cases for family in ['baseline','direct']]
    with ProcessPoolExecutor(max_workers=4) as pool:rows=list(pool.map(job,jobs))
    ex.save('iteration2/imu-development.json',rows);rank=[]
    for cfg in configs:
        score=[];constraints=[]
        for c in cases:
            b=next(r for r in rows if r['case']==c['name'] and r['config']==cfg and r['family']=='baseline')
            r=next(r for r in rows if r['case']==c['name'] and r['config']==cfg and r['family']=='direct')
            ratio={k:r[k]/b[k] for k in ['height_rms','pitch_rms','heading_rms']}
            score.append(.4*ratio['height_rms']+.35*ratio['pitch_rms']+.25*ratio['heading_rms'])
            constraints.extend([ratio['height_rms']/1.05,ratio['pitch_rms']/1.05,ratio['heading_rms']/1.1])
        rank.append(dict(config=cfg,score=.5*np.mean(score)+.5*max(score),worst_constraint=max(constraints),passes=bool(max(constraints)<=1)))
    passed=[r for r in rank if r['passes']]
    best=min(passed,key=lambda r:r['score']) if passed else min(rank,key=lambda r:r['worst_constraint'])
    ex.save('iteration2/imu-selection.json',dict(selected=best,variant=variant,stability=stress(variant),ranking=rank))
    print('IMU SELECTION',best,flush=True)


def validate():
    selected=json.loads((ex.HERE/'iteration2/imu-selection.json').read_text())
    cfg=selected['selected']['config'];variant=selected['variant'];cases=[]
    for case in ex.scenarios():
        case=dict(case);case['seed']+=2000;case['gust']*=1.2;cases.append(case)
    jobs=[(case,cfg,family,variant,'validation') for case in cases for family in ['baseline','candidate','direct']]
    with ProcessPoolExecutor(max_workers=4) as pool:rows=list(pool.map(job,jobs))
    ex.save('iteration2/imu-validation.json',rows)


def final_job(args):
    case,config,family,variant,obs=args
    if obs:
        row=job((case,config,family,variant,'final-validation'))
        row['observer']=True
    else:
        from iterate import job as truth_job
        if family=='direct':
            Shaped.variant=variant;ex.Scheduled=Shaped
            row=ex.run_case(case,'scheduled_final_validation')
        else:
            original=ex.Conventional
            cfg=ex.BASE if family=='baseline' else json.loads((ex.OLD/'optimization.json').read_text())['candidate']
            ex.Conventional=lambda p,unused:original(p,cfg)
            row=ex.run_case(case,family+'_final_validation')
            ex.Conventional=original
        row.update(observer=False,family=family,config=config,variant=variant)
    row.update(seed=case['seed'],gust_multiplier=case['gust'],maneuver_height=case.get('maneuver_height'),maneuver_turn=case.get('maneuver_turn'))
    return row


def final_validate():
    selected=json.loads((ex.HERE/'iteration2/refined-codesign-selection.json').read_text())
    cfg=selected['config'];variant=selected['selected']['variant'];cases=[]
    assert selected['selected']['passes']
    # Freeze before looking: new gust/noise seeds, amplitudes and maneuver amplitude.
    for case in ex.scenarios():
        case=dict(case);case['seed']+=3000;case['gust']*=1.2
        if case['name']=='maneuver':case.update(maneuver_height=1.5,maneuver_turn=8.)
        cases.append(case)
    jobs=[(case,cfg,family,variant,obs) for case in cases for obs in [False,True] for family in ['baseline','candidate','direct']]
    with ProcessPoolExecutor(max_workers=4) as pool:rows=list(pool.map(final_job,jobs))
    ex.save('iteration2/final-validation.json',rows)


def codesign_job(args):
    case,config,variant,obs=args
    if obs:return job((case,config,'direct',variant,'codesign'))
    from iterate import job as truth_job
    row=truth_job((variant,case,False));row.update(config=config,family='direct',observer=False)
    return row


def codesign(refine=False):
    config=json.loads((ex.HERE/'iteration2/imu-selection.json').read_text())['selected']['config']
    variants=[dict(id=f'v{v:g}-h{h:g}',hidden=1.,damper=.05,velocity_feedback=v,height_feedback=h)
              for v,h in itertools.product([.3,.5,.7],[.25,.5,1.])]
    prefix='refined-' if refine else ''
    if refine:
        variants=[dict(id=f'v0.7-h0.25-s{s:g}',hidden=1.,damper=.05,velocity_feedback=.7,height_feedback=.25,steering_feedback=s)
                  for s in [1.15,1.3,1.5]]
    stable=[]
    for v in variants:
        v.update(stress(v))
        if v['unstable']==0:stable.append(v)
    ex.save('iteration2/'+prefix+'codesign-stability.json',variants)
    cases=[c for c in ex.scenarios() if c['name'] in ['maneuver','gust','voltage_ramp']]
    jobs=[(case,config,v,obs) for v in stable for case in cases for obs in [False,True]]
    with ProcessPoolExecutor(max_workers=4) as pool:rows=list(pool.map(codesign_job,jobs))
    ex.save('iteration2/'+prefix+'codesign-development.json',rows)
    truth=json.loads((ex.HERE/'results.json').read_text());obsbase=json.loads((ex.HERE/'iteration2/imu-development.json').read_text())
    ranking=[]
    for variant in stable:
        ratios=[];constraints=[]
        for r in [r for r in rows if r['variant']['id']==variant['id']]:
            observed=r['controller'].endswith('_observer')
            b=next(b for b in obsbase if b['case']==r['case'] and b['config']==config and b['family']=='baseline') if observed else next(b for b in truth if b['case']==r['case'] and b['controller']=='baseline')
            rr={k:r[k]/b[k] for k in ['height_rms','pitch_rms','heading_rms']}
            ratios.append(.4*rr['height_rms']+.35*rr['pitch_rms']+.25*rr['heading_rms'])
            constraints.extend([rr['height_rms']/1.05,rr['pitch_rms']/1.05,rr['heading_rms']/1.1])
        ranking.append(dict(variant=variant,score=float(.5*np.mean(ratios)+.5*max(ratios)),worst_constraint=max(constraints),passes=bool(max(constraints)<=1)))
    passed=[r for r in ranking if r['passes']]
    best=min(passed,key=lambda r:r['score']) if passed else min(ranking,key=lambda r:r['worst_constraint'])
    ex.save('iteration2/'+prefix+'codesign-selection.json',dict(selected=best,config=config,ranking=ranking))
    print('CODESIGN SELECTION',best,flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('stage',choices=['tune','validate','codesign','refine','final'],default='tune',nargs='?');a=p.parse_args()
    {'tune':run,'validate':validate,'codesign':codesign,'refine':lambda:codesign(True),'final':final_validate}[a.stage]()
