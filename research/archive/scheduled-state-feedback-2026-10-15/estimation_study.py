"""Diagnose and tune estimator process assumptions without changing LQI feedback."""
import argparse
import itertools
import json
from concurrent.futures import ProcessPoolExecutor
import numpy as np
import experiment as ex
import observer as ob

ORIGINAL=ob.Observer

class DiagnosticObserver(ORIGINAL):
    config=None
    latest=None
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.Q=np.diag(np.r_[np.full(16,self.config['state_q']),np.full(3,self.config['wind_q'])])
        self.errors=[];DiagnosticObserver.latest=self
    def correct(self,truth_x):
        estimate=super().correct(truth_x)
        self.errors.append(estimate-truth_x)
        return estimate

def job(args):
    config,case=args
    DiagnosticObserver.config=config;ob.Observer=DiagnosticObserver
    name=f'scheduled_estq{config["state_q"]:g}-w{config["wind_q"]:g}_observer'
    row=ex.run_case(case,name)
    err=np.array(DiagnosticObserver.latest.errors)
    start=5 if case['name']=='maneuver' else 15
    e=err[np.arange(len(err))*ex.DT>=start]
    row['config']=config;row['state_error_rms']=np.sqrt(np.mean(e*e,axis=0)).tolist()
    # Nominal-gain diagnostic: which estimated state contributes throttle-command error?
    K=np.load(ex.HERE/'gains.npz')['K'][13]
    contribution=e/ex.XS[None,:]*(ex.US[0]*K[0,:16])[None,:]
    row['nominal_throttle_error_contributions']=np.sqrt(np.mean(contribution**2,axis=0)).tolist()
    return row

def run():
    configs=[dict(state_q=q,wind_q=w) for q,w in itertools.product([0.,1e-7,1e-6,1e-5],[.0001,.001,.01,.1])]
    cases=[c for c in ex.scenarios() if c['name'] in ['maneuver','gust','voltage_ramp']]
    with ProcessPoolExecutor(max_workers=4) as pool:rows=list(pool.map(job,[(c,s) for c in configs for s in cases]))
    ex.save('iteration2/estimator-development.json',rows)
    baseline=json.loads((ex.HERE/'observer-results.json').read_text());ranking=[]
    for config in configs:
        selected=[r for r in rows if r['config']==config];ratios=[];constraints=[]
        for r in selected:
            b=next(r0 for r0 in baseline if r0['case']==r['case'] and r0['controller']=='baseline_observer')
            rr={k:r[k]/b[k] for k in ['height_rms','pitch_rms','heading_rms']}
            ratios.append(.4*rr['height_rms']+.35*rr['pitch_rms']+.25*rr['heading_rms'])
            constraints.extend([rr['height_rms']/1.05,rr['pitch_rms']/1.05,rr['heading_rms']/1.1])
        ranking.append(dict(config=config,score=float(.5*np.mean(ratios)+.5*max(ratios)),worst_constraint=max(constraints),passes=max(constraints)<=1))
    passed=[r for r in ranking if r['passes']]
    best=min(passed,key=lambda r:r['score']) if passed else min(ranking,key=lambda r:r['worst_constraint'])
    ex.save('iteration2/estimator-selection.json',dict(selected=best,ranking=ranking))
    print('ESTIMATOR SELECTION',best,flush=True)

if __name__=='__main__':run()
