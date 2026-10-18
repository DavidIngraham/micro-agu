from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from fit_mission_family1 import dataset,predict,names
O=Path(__file__).parent
protocol=json.loads((O/'mission-performance-frozen-protocol.json').read_text())
rows=[r for r in protocol['blocks'] if r['partition']=='evaluation' and r['category']!='other']
fit=json.loads((O/'mission-family1-development-fit.json').read_text());p=[fit['parameters'][n] for n in names]
unsupported=[]; eligible=[]
raw={day:np.load(O/f'{day}-causal-input.npz') for day in {r['day'] for r in rows}}
for row in rows:
 a=raw[row['day']];t=a['t'];i=np.searchsorted(t,row['start_s']);prior=(t<t[i])&(t>=t[i]-120)&a['valid']&(abs(a['Y'][:,0])<np.deg2rad(25))
 if len(np.flatnonzero(prior)[::5])<100: unsupported.append(dict(row=row,reason='Insufficient prior data for prescribed wind estimator; full acceptance remains unavailable.'))
 else:eligible.append(row)
rows=eligible
D=dataset(rows);pred=predict(p,D);truth=np.array([d['truth'] for d in D]);report=[]
fig,axes=plt.subplots(2,3,figsize=(12,7))
for j,day in enumerate(['2026-01-03','2026-02-15']):
 idx=np.array([r['day']==day for r in rows]);cats=sorted({r['category'] for r in rows if r['day']==day});category=[]
 for cat in cats:
  ii=np.array([r['day']==day and r['category']==cat for r in rows]);rmse=np.sqrt(np.mean((pred[ii]-truth[ii])**2,axis=0));category.append(rmse**2);report.append(dict(day=day,category=cat,n=int(ii.sum()),rmse=rmse.tolist()))
 for k,label in enumerate(['Mean ground speed (m/s)','Mean climb (m/s)','Net ground turn rate (deg/s)']):
  ax=axes[j,k];ax.scatter(truth[idx,k],pred[idx,k],s=16,alpha=.65);low=min(truth[idx,k].min(),pred[idx,k].min());high=max(truth[idx,k].max(),pred[idx,k].max());ax.plot([low,high],[low,high],'k--',lw=1);ax.set_title(day+'\n'+label);ax.set_xlabel('Observed');ax.set_ylabel('Predicted');ax.grid(alpha=.2)
report_obj=dict(scope='Retrospective mission evaluation; absolute errors only. No paired detailed-baseline comparison yet, therefore no net improvement claim.',unsupported_blocks=unsupported,metric_order=['groundspeed m/s','climb m/s','ground turn rate deg/s'],by_category=report,blocks=[dict(row=r,predicted=pp.tolist(),observed=tt.tolist(),wind=d['wind'].tolist()) for r,pp,tt,d in zip(rows,pred,truth,D)])
(O/'mission-family1-evaluation.json').write_text(json.dumps(report_obj,indent=2));fig.tight_layout();fig.savefig(O/'mission-family1-evaluation.png',dpi=150);print(json.dumps(report,indent=2))

