"""Initial-state transient amplification and equal-objective linear oracle comparisons."""
import json
import numpy as np
from scipy.linalg import solve_discrete_lyapunov
from study import HERE, DT, XS, US, BASE, plants, controller, output_matrix, save
from analyze import lqi, COLORS
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def main():
    p=plants()[0];cfg=json.loads((HERE/'optimization.json').read_text())['candidate']
    rows=[];fig,ax=plt.subplots(1,2,figsize=(12,5),layout='constrained')
    for name,color in zip(['baseline','candidate','oracle'],COLORS):
        if name=='oracle':
            oracle_cfg=json.loads((HERE/'oracle-selected.json').read_text())
            F,G,W,U,K=lqi(p,**oracle_cfg);n=len(F);initial=np.zeros((n,16));initial[:16]=np.eye(16)
            physical=np.zeros((16,n));physical[:,:16]=np.eye(16)
            Y=np.zeros((7,n));Y[:5,:16]=p['C'][[5,3,0,1,2]]*XS/np.array([1,10,.5,np.deg2rad(30),np.deg2rad(30)])[:,None]
            Y[5:]=U/US[:,None]
        else:
            F,G,W,U,Ur=controller(p,BASE if name=='baseline' else cfg);n=len(F)
            initial=np.zeros((n,16));initial[:16]=np.diag(XS)
            physical=np.zeros((16,n));physical[:,:16]=np.diag(1/XS)
            Y=output_matrix(p,U)
        # Same initial physical-state unit ball, controller memories initially zero.
        propagation=initial.copy();times=np.arange(0,10+DT/2,DT);amp=[];peakv=None;peak=-1
        for t in times:
            _,sv,vh=np.linalg.svd(physical@propagation,full_matrices=False)
            amp.append(sv[0])
            if sv[0]>peak:peak=sv[0];peakv=vh[0].copy();peakt=t
            propagation=F@propagation
        eq=np.linalg.solve(np.eye(n)-F,G@np.diag([1.,10.]))
        gram=solve_discrete_lyapunov(F,eq@eq.T,method='bilinear')
        cost=DT*np.diag(Y@gram@Y.T)
        column_costs=[]
        for j in range(2):
            gram_j=solve_discrete_lyapunov(F,np.outer(eq[:,j],eq[:,j]),method='bilinear')
            column_costs.append((DT*np.diag(Y@gram_j@Y.T)).tolist())
        gust=np.zeros(7)
        weights=np.array([1,1,.8,.6,.4,.25]);weights/=np.linalg.norm(weights)
        for f,w in zip([.08,.13,.21,.34,.55,.89],weights):
            H=Y@np.linalg.solve(np.exp(2j*np.pi*f*DT)*np.eye(n)-F,W)
            gust+=np.sum(abs(H*np.array([.5,.5,.25]))**2,axis=1)*w*w
        rows.append(dict(controller=name,peak_normalized_state_amplification=float(peak),time_s=float(peakt),
                         initial_direction=peakv.tolist(),step_components=cost.tolist(),gust_components=gust.tolist(),
                         step_components_by_reference=dict(altitude=column_costs[0],heading_rate=column_costs[1]),
                         note='Dimensionless engineering-scaled norm; not mechanical energy. Zero controller memories.'))
        label='Full-state LQI' if name=='oracle' else name
        ax[0].plot(times,amp,label=label,color=color)
        ax[1].bar(np.arange(7)+(len(rows)-2)*.25,gust,width=.25,label=label,color=color)
    ax[0].set(title='Stable poles do not describe the whole transient',xlabel='Time after initial disturbance (s)',ylabel='Largest normalized state amplification')
    ax[1].set(title='Linear gust cost, separated by output',ylabel='Squared normalized response')
    ax[1].set_xticks(range(7),['Height','Turn\nrate','Climb','Pitch\nrate','Relative\nrate','Throttle','Brake'])
    for a in ax:a.legend();a.grid(axis='y',alpha=.2)
    fig.suptitle('Full-state benchmark: improvement depends on what is measured and penalized')
    fig.savefig(HERE/'benchmark.png',dpi=160);plt.close(fig);save('benchmark.json',rows)
    print(json.dumps(rows,indent=2))

if __name__=='__main__':main()
