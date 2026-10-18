"""Second pass: limit reliance on hidden-state estimates and restore shaped pitch damping.

Uses the unchanged first-pass harness. Variant parameters and validation split are explicit.
"""
import argparse
import itertools
import json
from concurrent.futures import ProcessPoolExecutor
import numpy as np
import experiment as ex

ITER=ex.HERE/'iteration2'
ITER.mkdir(exist_ok=True)
ORIGINAL_SCHEDULED=ex.Scheduled
HIDDEN=[2,3,4,11,12,13]

def variants():
    return [dict(id=f'h{hidden:g}-d{damper:g}',hidden=hidden,damper=damper)
            for hidden,damper in itertools.product([.25,.5,1.],[0.,.05,.1])]

def hybrid_variants():
    return [dict(id=f'blend{blend:g}-hz{hz:g}',hidden=1.,damper=0.,blend=blend,hz=hz)
            for blend,hz in itertools.product([.1,.2,.3,.5],[.2,.5,1.])] + [
            dict(id=f'blend0.3-hz0.5-extra{d:g}',hidden=1.,damper=0.,blend=.3,hz=.5,inner_damper=d)
            for d in [.02,.04,.08]]


class Shaped(ORIGINAL_SCHEDULED):
    variant=None
    baseplant=None
    def __init__(self,grid,K,**kwargs):
        K=K.copy();K[:,:,HIDDEN]*=self.variant['hidden']
        K[:,:,7]*=self.variant.get('velocity_feedback',1.)
        K[:,:,15]*=self.variant.get('height_feedback',1.)
        K[:,1,:]*=self.variant.get('steering_feedback',1.)
        super().__init__(grid,K,**kwargs);self.qfilters=np.zeros(2)
        if 'blend' in self.variant:
            self.inner=ex.Conventional(self.baseplant,ex.BASE);self.correction=np.zeros(2)
    def request(self,x,obs,height,climb,turn,voltage,speed):
        raw=super().request(x,obs,height,climb,turn,voltage,speed)
        xr=ex.weights(voltage,climb,turn)@self.grid['x']
        # The nominal level trim has zero pitch rate to numerical precision.
        qerror=obs[1]-xr[9]
        a=ex.DT/(ex.DT+1/(2*np.pi*1.5))
        self.qfilters[0]+=a*(qerror-self.qfilters[0])
        self.qfilters[1]+=a*(self.qfilters[0]-self.qfilters[1])
        raw[0]-=22.2/voltage*self.variant['damper']*self.qfilters[1]
        if 'blend' in self.variant:
            base=self.inner.request(obs,height,climb,turn,voltage,speed)
            base[0]-=22.2/voltage*self.variant.get('inner_damper',0.)*self.qfilters[1]
            self.inner.raw=base
            a=ex.DT/(ex.DT+1/(2*np.pi*self.variant['hz']))
            self.lqi_raw=raw.copy()
            self.correction+=a*(np.clip(self.variant['blend']*(raw-base),-.05,.05)-self.correction)
            raw=base+self.correction
        self.raw=raw
        return raw
    def applied(self,value):
        if 'blend' in self.variant:
            # Share actual actuator discrepancy without treating the intended correction
            # as a saturation error in the conventional inner loop.
            discrepancy=value-self.raw
            self.inner.applied(self.inner.raw+discrepancy)
            # LQI output is only partially realized: track final applied effort.
            saved=self.raw;self.raw=self.lqi_raw
            super().applied(value);self.raw=saved
        else:super().applied(value)


def closure(grid,K,index,variant,scale_multiplier=1.):
    A,B,C,scale=(grid[k][index].copy() for k in ['A','B','C','scale'])
    scale[1]*=scale_multiplier
    AA,_,_=ex.augmented(A,B,C,scale)
    F=np.zeros((20,20));F[:18,:18]=AA
    a=ex.DT/(ex.DT+1/(2*np.pi*1.5))
    F[18,9]=a*ex.XS[9];F[18,18]=1-a
    F[19]=a*F[18];F[19,19]=1-a
    gain=K[index].copy();gain[:,HIDDEN]*=variant['hidden']
    gain[:,7]*=variant.get('velocity_feedback',1.)
    gain[:,15]*=variant.get('height_feedback',1.)
    gain[1]*=variant.get('steering_feedback',1.)
    U=np.zeros((2,20));U[:,:18]=-ex.US[:,None]*scale[:,None]*gain
    U[0]-=scale[0]*variant['damper']*F[19]
    BP=np.zeros((20,2));BP[:16]=B/ex.XS[:,None]
    return F,BP,U


def stress(variant):
    grid=dict(np.load(ex.HERE/'grid.npz'));K=np.load(ex.HERE/'gains.npz')['K'];poles=[]
    for i in range(18):
        A,B,U=closure(grid,K,i,variant)
        for gt,gb,dt,db in itertools.product([.75,1.25],[.75,1.25],[0,2],[0,2]):
            F=np.zeros((20+dt+db,20+dt+db));F[:20,:20]=A;pos=20
            for j,delay,gain in [(0,dt,gt),(1,db,gb)]:
                if delay:
                    F[:20,pos+delay-1]=B[:,j]*gain;F[pos,:20]=U[j]
                    for k in range(1,delay):F[pos+k,pos+k-1]=1
                    pos+=delay
                else:F[:20,:20]+=np.outer(B[:,j]*gain,U[j])
            poles.append(float(np.log(max(abs(np.linalg.eigvals(F))))/ex.DT))
    return dict(worst_pole=max(poles),unstable=sum(p>=0 for p in poles))


def job(args):
    variant,case,observer=args
    Shaped.variant=variant;ex.Scheduled=Shaped
    Shaped.baseplant=next(p for p in ex.plants() if p['profile']==case['profile'] and p['voltage']==case['voltage'] and p['climb']==0)
    suffix='_observer' if observer else ''
    row=ex.run_case(case,'scheduled_'+variant['id']+suffix)
    row['variant']=variant
    return row


def tune(hybrid=False):
    candidates=[{**v,**(hybrid_stress(v) if hybrid else stress(v))} for v in (hybrid_variants() if hybrid else variants())]
    feasible=[v for v in candidates if v['unstable']==0]
    prefix='hybrid-' if hybrid else ''
    ex.save('iteration2/'+prefix+'stability.json',candidates)
    # Existing nominal maneuver, gust and voltage-ramp realizations are development data now.
    cases=[c for c in ex.scenarios() if c['name'] in ['maneuver','gust','voltage_ramp']]
    jobs=[(v,c,obs) for v in feasible for c in cases for obs in [False,True]]
    with ProcessPoolExecutor(max_workers=4) as pool:rows=list(pool.map(job,jobs))
    ex.save('iteration2/'+prefix+'development-results.json',rows)
    baseline=json.loads((ex.HERE/'results.json').read_text())+json.loads((ex.HERE/'observer-results.json').read_text())
    ranking=[]
    for v in feasible:
        vr=[r for r in rows if r['variant']['id']==v['id']];ratios=[];constraints=[]
        for r in vr:
            name='baseline_observer' if r['controller'].endswith('_observer') else 'baseline'
            b=next(x for x in baseline if x['case']==r['case'] and x['controller']==name)
            ratio={k:r[k]/b[k] for k in ['height_rms','pitch_rms','heading_rms']}
            ratios.append(.4*ratio['height_rms']+.35*ratio['pitch_rms']+.25*ratio['heading_rms'])
            constraints.extend([ratio['height_rms']/1.05,ratio['pitch_rms']/1.05,ratio['heading_rms']/1.1])
        ranking.append(dict(variant=v,score=float(.5*np.mean(ratios)+.5*max(ratios)),
                            worst_constraint=float(max(constraints)),passes=bool(max(constraints)<=1)))
    # Never silently loosen constraints if none pass; retain the least-violating diagnostic.
    passed=[r for r in ranking if r['passes']]
    chosen=min(passed,key=lambda r:r['score']) if passed else min(ranking,key=lambda r:(r['worst_constraint'],r['score']))
    ex.save('iteration2/'+prefix+'selection.json',dict(selected=chosen,all_constraints_met=bool(passed),ranking=ranking,
        constraints='Every development case: height/pitch RMS <=105% and heading RMS <=110% of matched conventional baseline.'))
    print('SELECTION',chosen,flush=True)


def validate():
    selected=json.loads((ITER/'hybrid-selection.json').read_text())['selected']['variant']
    cases=[]
    for c in ex.scenarios():
        c=dict(c);c['seed']+=1000
        # All new random phases; changed amplitudes for a genuine disturbance holdout.
        c['gust']*=1.2
        c['name_original']=c['name'];cases.append(c)
    # Distinct names must retain scenario logic, so use isolated output directory by moving
    # only the output path in each worker, while preserving dependency file access.
    jobs=[(selected,c,obs,name) for c in cases for obs in [False,True]
          if (not obs or c['name'] in ['maneuver','gust','voltage_ramp','soft_roll_gust'])
          for name in ['baseline','candidate','initial','scheduled']]
    with ProcessPoolExecutor(max_workers=4) as pool:rows=list(pool.map(validation_job,jobs))
    ex.save('iteration2/validation-results.json',rows)


def validation_job(args):
    variant,case,observer,name=args
    # run_case needs grid/gain files at HERE; separate results and traces using unique controller IDs.
    if name in ['scheduled','initial']:
        if name=='scheduled':
            Shaped.variant=variant;ex.Scheduled=Shaped
            Shaped.baseplant=next(p for p in ex.plants() if p['profile']==case['profile'] and p['voltage']==case['voltage'] and p['climb']==0)
        else:ex.Scheduled=ORIGINAL_SCHEDULED
        controller=('scheduled_validation' if name=='scheduled' else 'scheduled_initial_validation')+('_observer' if observer else '')
    else:
        # Conventional() selection recognizes baseline only: intercept the constructor to
        # keep the desired gains when adding the unique recording identifier.
        original=ex.Conventional
        cfg=ex.BASE if name=='baseline' else json.loads((ex.OLD/'optimization.json').read_text())['candidate']
        ex.Conventional=lambda p,unused:original(p,cfg)
        controller=name+'_validation'+('_observer' if observer else '')
    row=ex.run_case(case,controller);row['family']=name;row['observer']=observer;row['seed']=case['seed'];row['gust_multiplier']=case['gust']
    if name=='scheduled':row['variant']=variant
    # ProcessPool workers are reused: remove constructor wrapper after each job.
    if name not in ['scheduled','initial']:ex.Conventional=original
    return row


def hybrid_closure(grid,K,index,variant):
    """Differentiate the actual composite controller's memories at a matched equilibrium."""
    rec=json.loads((ex.HERE/'grid.json').read_text())['nodes'][index]
    plant=next(p for p in ex.plants() if p['name']==('low_voltage' if rec['voltage']==20.4 else 'level'))
    sim=ex.setup(ex.MODEL)('trout_lake','mission')
    nominal=ex.plants()[0];chart=ex.Chart(sim,nominal['state'],float(nominal['rpm']))
    Shaped.variant=variant;Shaped.baseplant=plant
    x0=grid['x'][index];u0=grid['u'][index];obs0=ex.observations(chart.decode(x0)[0])
    ctrl=Shaped(grid,K);ctrl.inner.z=np.r_[obs0[[3,4]],obs0[1],obs0[1]]
    ctrl.inner.request(obs0,0,rec['climb'],rec['turn'],rec['voltage'],rec['speed'])
    ctrl.inner.integral=(u0-ctrl.inner.raw)/np.array([22.2/rec['voltage'],1.])
    base_memory=np.r_[np.zeros(4),ctrl.inner.z,ctrl.inner.integral,np.zeros(2)]
    def controller_step(z):
        ctrl=Shaped(grid,K);m=base_memory+z[16:]
        ctrl.integral=m[:2].copy();ctrl.qfilters=m[2:4].copy();ctrl.inner.z=m[4:8].copy();ctrl.inner.integral=m[8:10].copy();ctrl.correction=m[10:12].copy()
        state,_=chart.decode(x0+ex.XS*z[:16]);obs=ex.observations(state)
        raw=ctrl.request(x0+ex.XS*z[:16],obs,0,rec['climb'],rec['turn'],rec['voltage'],rec['speed'])
        ctrl.applied(raw)
        mem=np.r_[ctrl.integral,ctrl.qfilters,ctrl.inner.z,ctrl.inner.integral,ctrl.correction]-base_memory
        return mem,raw-u0
    drift,u=controller_step(np.zeros(28))
    assert max(np.max(abs(drift)),np.max(abs(u)))<1e-6,(index,drift,u)
    eps=1e-6;M=[];U=[]
    for e in np.eye(28)*eps:
        mp,up=controller_step(e);mm,um=controller_step(-e)
        M.append((mp-mm)/(2*eps));U.append((up-um)/(2*eps))
    A=np.zeros((28,28));A[:16,:16]=grid['A'][index]*ex.XS[None,:]/ex.XS[:,None];A[16:]=np.array(M).T
    B=np.zeros((28,2));B[:16]=grid['B'][index]/ex.XS[:,None]
    return A,B,np.array(U).T


def hybrid_stress(variant):
    grid=dict(np.load(ex.HERE/'grid.npz'));K=np.load(ex.HERE/'gains.npz')['K'];poles=[]
    for i in range(18):
        A,B,U=hybrid_closure(grid,K,i,variant)
        for gt,gb,dt,db in itertools.product([.75,1.25],[.75,1.25],[0,2],[0,2]):
            F=np.zeros((28+dt+db,28+dt+db));F[:28,:28]=A;pos=28
            for j,delay,gain in [(0,dt,gt),(1,db,gb)]:
                if delay:
                    F[:28,pos+delay-1]=B[:,j]*gain;F[pos,:28]=U[j]
                    for k in range(1,delay):F[pos+k,pos+k-1]=1
                    pos+=delay
                else:F[:28,:28]+=np.outer(B[:,j]*gain,U[j])
            poles.append(float(np.log(max(abs(np.linalg.eigvals(F))))/ex.DT))
    return dict(worst_pole=max(poles),unstable=sum(p>=0 for p in poles))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('stage',choices=['tune','hybrid','validate']);a=p.parse_args()
    validate() if a.stage=='validate' else tune(a.stage=='hybrid')
