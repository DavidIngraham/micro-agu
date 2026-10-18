from up_away_model import *
from scipy.optimize._numdiff import approx_derivative
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
D=dict(np.load(O/'up-away-data.npz'));rows=json.loads((O/'up-away-records.json').read_text());fit=json.loads((O/'up-away-A1-fit.json').read_text());p=np.array([fit['parameters'][k] for k in NAMES]);dev=np.array([r['partition']=='development' for r in rows]);DD=subset(D,dev)
J=approx_derivative(lambda p:residual(p,DD),p,method='3-point');e=residual(p,DD);g=J.T@(e/np.sqrt(1+e*e));sv=np.linalg.svd(J*(HIGH-LOW),compute_uv=False)
metric=lambda x:np.c_[np.trapz(x[:,:,0],dx=.04,axis=1)/10,np.trapz(x[:,:,1],dx=.04,axis=1)/10,np.rad2deg(x[:,-1,2])/10]
y=metric(D['truth']);cases={}
for label,delta,hist in [('nominal',(0,0),51),('north_plus',(.5,0),51),('north_minus',(-.5,0),51),('east_plus',(0,.5),51),('east_minus',(0,-.5),51),('rate_last1s',(0,0),26)]:
 d={k:v.copy() for k,v in D.items()};d['wind']+=np.array(delta);ground=np.c_[D['initial'][:,0]*np.cos(D['initial'][:,2]),D['initial'][:,0]*np.sin(D['initial'][:,2])]+D['wind'];av=ground-d['wind'];d['initial'][:,0]=np.linalg.norm(av,axis=1);d['initial'][:,2]=np.arctan2(av[:,1],av[:,0]);h=D['history'][:,-hist:]-d['wind'][:,None,:];ang=np.unwrap(np.arctan2(h[:,:,1],h[:,:,0]),axis=1);time=np.arange(hist)*.04;time-=time.mean();d['initial'][:,3]=(ang@time)/(time@time)
 pr=metric(predict(p,d));cases[label]={part:{str(day):np.sqrt(np.mean((pr[(dev if part=='development' else ~dev)&(D['day']==day)]-y[(dev if part=='development' else ~dev)&(D['day']==day)])**2,axis=0)).tolist() for day in [0,1]} for part in ['development','check']}
# Approximate parameter uncertainty from clustered episode score, not extra model fits.
# One linearization, classical sandwich. Bounds and weak nonlinear identification limit interpretation.
n=dev.sum();r=e.reshape(n,-1);jj=J.reshape(n,-1,len(p));scores=np.einsum('nki,nk->ni',jj,r);bread=np.linalg.pinv(J.T@J);cov=bread@(scores.T@scores)@bread;sd=np.sqrt(np.maximum(np.diag(cov),0));corr=cov/np.maximum(sd[:,None]*sd[None,:],1e-15)
result=dict(sensitivity=cases,analyst_wind_perturbation_m_s=.5,warning='Wind shifts are sensitivity scenarios, not measured confidence. Linear clustered uncertainty is conditional on fixed wind/model and unreliable at active bounds; do not treat it as calibrated coverage.',scaled_jacobian_condition=float(sv[0]/sv[-1]),parameter_sd=dict(zip(NAMES,sd.tolist())),development_robust_gradient=dict(zip(NAMES,g.tolist())),trimHR_throttle_parameter_correlation=float(corr[12,14]))
(O/'up-away-A1-sensitivity.json').write_text(json.dumps(result,indent=2));np.savez_compressed(O/'up-away-A1-local-uncertainty.npz',covariance=cov,jacobian=J)
fig,ax=plt.subplots(1,2,figsize=(11,4));labels=list(cases)
for day in [0,1]:
 vals=[cases[k]['check'][str(day)][2] for k in labels];ax[day].bar(np.arange(len(labels)),vals);ax[day].axhline(2,color='r',ls='--',label='declared per-regime budget (context only)');ax[day].set_xticks(range(len(labels)),labels,rotation=30,ha='right');ax[day].set_ylabel('Aggregate course-rate RMSE deg/s');ax[day].set_title(['Hood River','Trout Lake'][day]);ax[day].grid(axis='y',alpha=.2)
fig.tight_layout();fig.savefig(O/'up-away-A1-sensitivity.png',dpi=160)
print(json.dumps(result,indent=2))
