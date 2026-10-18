"""A declared LQI weight screen; a useful benchmark, not a global controller bound."""
import json
import itertools
import numpy as np
from scipy.linalg import solve_discrete_lyapunov
from analyze import lqi
from study import HERE, DT, XS, US, plants, BASE, performance, save

def evaluate(p, K):
    A=p['A']*XS[None,:]/XS[:,None];B=p['B']*US[None,:]/XS[:,None];C=p['C']*XS
    F=np.zeros((20,20));F[:16,:16]=A;F[:16,16:18]=B;F[16:18,16:18]=np.eye(2)
    F[18:,:16]=DT*C[[5,3]]/np.array([1.,10.])[:,None];F[18:,18:]=np.eye(2)
    Bu=np.zeros((20,2));Bu[:16]=B;Bu[16:18]=np.eye(2);F-=Bu@K
    pole=float(np.log(max(abs(np.linalg.eigvals(F))))/DT)
    if pole>=0:return None
    G=np.zeros((20,2));G[18:]=-DT*np.diag([1.,.1]);W=np.zeros((20,3));W[:16]=p['E']/XS[:,None]
    U=-K.copy();U[:,16:18]+=np.eye(2)
    Y=np.zeros((7,20));Y[:5,:16]=C[[5,3,0,1,2]]/np.array([1,10,.5,np.deg2rad(30),np.deg2rad(30)])[:,None];Y[5:]=U
    eq=np.linalg.solve(np.eye(20)-F,G@np.diag([1.,10.]));P=solve_discrete_lyapunov(F,eq@eq.T,method='bilinear')
    step=DT*np.diag(Y@P@Y.T);gust=np.zeros(7);weights=np.array([1,1,.8,.6,.4,.25]);weights/=np.linalg.norm(weights)
    for f,w in zip([.08,.13,.21,.34,.55,.89],weights):
        H=Y@np.linalg.solve(np.exp(2j*np.pi*f*DT)*np.eye(20)-F,W)
        gust+=np.sum(abs(H*np.array([.5,.5,.25]))**2,axis=1)*w*w
    return dict(step=float(sum(step)),gust=float(sum(gust)),step_components=step.tolist(),gust_components=gust.tolist(),pole_real=pole)

if __name__=='__main__':
    ps=plants();base={p['name']:performance(p,BASE) for p in ps};rows=[]
    for move,integ,rate,state in itertools.product([1.,10.,100.,1000.],[.01,.08,.5],[.3,3.,30.],[.002,.1]):
        cfg=dict(move_penalty=move,integral_weight=integ,rate_weight=rate,state_weight=state)
        *_,K=lqi(ps[0],**cfg)
        values={p['name']:evaluate(p,K) for p in ps}
        training=[p for p in ps if p['name']!='dynamic_climb']
        if any(values[p['name']] is None for p in training):continue
        ratios=[.5*values[p['name']]['step']/base[p['name']]['step']+.5*values[p['name']]['gust']/base[p['name']]['gust'] for p in training]
        score=.5*np.mean(ratios)+.5*max(ratios)
        rows.append(dict(config=cfg,score=float(score),cases=values))
    best=min(rows,key=lambda r:r['score']);save('oracle-screen.json',dict(evaluated=72,stable=len(rows),selected=best,rows=rows))
    save('oracle-selected.json',best['config']);print(json.dumps(best,indent=2))
