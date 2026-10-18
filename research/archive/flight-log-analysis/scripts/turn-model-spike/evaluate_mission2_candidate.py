import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mission2_model import O,PLAN,predict
D=dict(np.load(O/'mission2-evaluation-data.npz'));T=dict(np.load(O/'mission2-development-data.npz'));rr=json.loads((O/'mission2-evaluation-records.json').read_text());tr=json.loads((O/'mission2-development-records.json').read_text());p=json.loads((O/'mission2-fit.json').read_text())['parameters'];pred,series,bad=predict(p,D,series=True);fine=predict(p,D,substeps=4)
constant=np.array([T['truth'][[x['day']==r['day'] for x in tr]].mean(axis=0) for r in rr]);sensitivity=[]
for offset in [(0,0),(.5,0),(-.5,0),(0,.5),(0,-.5)]:
 result=predict(p,D,wind_delta=offset);sensitivity.append(dict(wind_delta_N_E_m_s=offset,predictions=result.tolist(),all_finite=bool(np.isfinite(result).all())))
report=dict(scope='Candidate and simple-reference evaluation; detailed baseline comparison pending',metrics=PLAN['metrics'],candidate=pred.tolist(),constant_reference=constant.tolist(),observed=D['truth'].tolist(),invalid=bad.tolist(),step_max_difference=np.nanmax(abs(pred-fine),axis=0).tolist(),wind_sensitivity=sensitivity,records=rr)
(O/'mission2-candidate-evaluation.json').write_text(json.dumps(report,indent=2));np.savez_compressed(O/'mission2-candidate-series.npz',series=series)
fig,axes=plt.subplots(2,3,figsize=(12,7))
for j,day in enumerate(['2026-01-03','2026-02-15']):
 ii=np.array([r['day']==day for r in rr])
 for k,label in enumerate(['Path length (m)','Altitude change (m)','Course change (deg)']):
  ax=axes[j,k];ax.scatter(D['truth'][ii,k],pred[ii,k],label='Coupled');ax.scatter(D['truth'][ii,k],constant[ii,k],marker='x',label='Constant');lim=[np.nanmin(np.r_[D['truth'][ii,k],pred[ii,k],constant[ii,k]]),np.nanmax(np.r_[D['truth'][ii,k],pred[ii,k],constant[ii,k]])];ax.plot(lim,lim,'k--',lw=.8);ax.set(title=day+' '+label,xlabel='Observed',ylabel='Predicted');ax.legend();ax.grid(alpha=.2)
fig.tight_layout();fig.savefig(O/'mission2-candidate-evaluation.png',dpi=150)
print(json.dumps(dict(all_finite=bool(np.isfinite(pred).all()),step_max_difference=report['step_max_difference'],candidate_rmse=np.sqrt(np.mean((pred-D['truth'])**2,axis=0)).tolist(),constant_rmse=np.sqrt(np.mean((constant-D['truth'])**2,axis=0)).tolist()),indent=2))
