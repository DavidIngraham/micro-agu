"""Independent checks of controller closure, scheduling, limits, and model identity."""
import itertools
import json
import hashlib
import numpy as np
from experiment import (HERE, OLD, MODEL, DT, XS, US, Chart, plants, setup, weights,
                        scaling, limits, observations, augmented, Scheduled, Actuation, save)

def main():
    grid=dict(np.load(HERE/'grid.npz'));gains=np.load(HERE/'gains.npz')['K']
    records=json.loads((HERE/'grid.json').read_text())['nodes']
    original=json.loads((OLD/'plants.json').read_text())['source_hashes']
    hashes={f:hashlib.sha256((MODEL/f).read_bytes()).hexdigest() for f in original}
    assert all(hashes[f]==original[f] for f in hashes),'Model changed from original study'
    assert max(r['drift'] for r in records)<1e-7
    make=setup(MODEL);sim=make('trout_lake','mission');sim.substeps=16
    p=plants()[0];chart=Chart(sim,p['state'],float(p['rpm']))
    closure=[]
    for idx in [4,9,13,17]:
        r=records[idx];x0=grid['x'][idx];u0=grid['u'][idx]
        A,B,C=grid['A'][idx],grid['B'][idx],grid['C'][idx]
        AA,BB,_=augmented(A,B,C,grid['scale'][idx]);expected=AA-BB@gains[idx]
        def step(z):
            x=x0+XS*z[:16];state,rpm=chart.decode(x)
            ctrl=Scheduled(grid,gains);ctrl.integral=z[16:].copy()
            raw=ctrl.request(x,observations(state),0,r['climb'],r['turn'],r['voltage'],r['speed'])
            command=np.r_[u0[0],max(-u0[1],0)*1300*np.pi*55/360*.1,max(u0[1],0)*1500*np.pi*55/360*.1]
            act=Actuation(sim,command);cmd,applied=act.apply(raw,r['voltage']);ctrl.applied(applied)
            state,rpm=sim.step(state,rpm,cmd,voltage=r['voltage'],dt=DT)
            out=chart.encode(state,rpm)-x0;out[15]-=r['climb']*DT
            return np.r_[out/XS,ctrl.integral]
        eps=1e-6
        actual=np.column_stack([(step(e)-step(-e))/(2*eps) for e in np.eye(18)*eps])
        error=float(np.linalg.norm(actual-expected)/np.linalg.norm(expected))
        closure.append(dict(node=idx,relative_error=error));assert error<1e-5,closure[-1]
    frozen=[]
    for v,c,t in itertools.product(np.linspace(20.4,22.2,5),np.linspace(-.5,.5,5),np.linspace(-10,10,5)):
        w=weights(v,c,t);A=np.tensordot(w,grid['A'],axes=1);B=np.tensordot(w,grid['B'],axes=1);C=np.tensordot(w,grid['C'],axes=1);K=np.tensordot(w,gains,axes=1)
        scale=w@grid['scale']
        for multiplier in [.75,1,1.25]:
            s=scale.copy();s[1]*=multiplier
            AA,BB,_=augmented(A,B,C,s)
            pole=float(np.log(max(abs(np.linalg.eigvals(AA-BB@K))))/DT)
            frozen.append(pole)
    # Node uncertainty sweep with independently changed authority and 0/40 ms input delay.
    robust=[]
    for i in range(18):
        AA,BB,_=augmented(grid['A'][i],grid['B'][i],grid['C'][i],grid['scale'][i])
        for gt,gb,dt,db in itertools.product([.75,1.25],[.75,1.25],[0,2],[0,2]):
            F=np.zeros((18+dt+db,18+dt+db));F[:18,:18]=AA;pos=18
            for j,delay,gain in [(0,dt,gt),(1,db,gb)]:
                if delay:
                    F[:18,pos+delay-1]=BB[:,j]*gain;F[pos,:18]=-gains[i,j]
                    for k in range(1,delay):F[pos+k,pos+k-1]=1
                    pos+=delay
                else:F[:18,:18]-=np.outer(BB[:,j]*gain,gains[i,j])
            robust.append(float(np.log(max(abs(np.linalg.eigvals(F))))/DT))
    # Saturation/slew and partition-of-unity checks, including out-of-grid requests.
    for v in [19,20.4,21.3,22.2,24]:
        w=weights(v,2,30);assert np.all(w>=0) and np.isclose(sum(w),1)
        act=Actuation(sim,p['command'])
        for raw in [np.array([2.,2.]),np.array([-1.,-2.])]*20:
            old=act.throttle;_,applied=act.apply(raw,v)
            assert 0<=applied[0]<=limits(v)
            assert abs(applied[0]-old)<=DT*limits(v)+1e-10
            assert abs(applied[1])<=1+1e-10
    result=dict(model_hashes_match=True,closure=closure,interpolated_frozen_cases=len(frozen),
                interpolated_unstable=sum(p>=0 for p in frozen),interpolated_worst_pole=max(frozen),
                uncertainty_cases=len(robust),uncertainty_unstable=sum(p>=0 for p in robust),uncertainty_worst_pole=max(robust),
                note='Frozen interpolated plant/gain checks are not a nonlinear scheduling stability proof.')
    save('verification.json',result);print(json.dumps(result,indent=2))

if __name__=='__main__':main()
