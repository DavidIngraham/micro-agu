from pathlib import Path
import json,copy,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from fit_mission_family1 import dataset,predict,names,rows
O=Path(__file__).parent
plan=dict(scope='Development-only initialization sensitivity, not a new coefficient fit',alternative='OLS slope of unwrapped air-relative course over preceding 5 seconds, including origin; same prior wind estimate',selection='Consider only if turn RMSE improves in both flights without >10% degradation of other development metrics. Otherwise reject; no window-length sweep.',coefficients='Frozen mission-family1-development-fit.json')
(O/'mission-init-sensitivity-plan.json').write_text(json.dumps(plan,indent=2))
D=dataset(rows);E=copy.deepcopy(D);cache={day:np.load(O/f'{day}-causal-input.npz') for day in {r['day'] for r in rows}}
for d in E:
 a=cache[d['row']['day']];t=a['t'];i=np.searchsorted(t,d['row']['start_s']);jj=np.flatnonzero((t>=t[i]-5)&(t<=t[i])&a['valid']);v=a['Y'][jj,6:8]-d['wind'];h=np.unwrap(np.arctan2(v[:,1],v[:,0]));d['init'][3]=float(np.polyfit(t[jj]-t[i],h,1)[0])
fit=json.loads((O/'mission-family1-development-fit.json').read_text());p=[fit['parameters'][n] for n in names];base=predict(p,D);alt=predict(p,E);truth=np.array([d['truth'] for d in D]);result=[]
fig,axes=plt.subplots(1,2,figsize=(10,4))
for ax,day in zip(axes,cache):
 ii=np.array([d['row']['day']==day for d in D]);b=np.sqrt(np.mean((base[ii]-truth[ii])**2,axis=0));a=np.sqrt(np.mean((alt[ii]-truth[ii])**2,axis=0));result.append(dict(day=day,baseline_rmse=b.tolist(),alternative_rmse=a.tolist(),ratio=(a/np.maximum(b,[.25,.1,1])).tolist(),passes=bool(a[2]<b[2] and np.all(a[:2]<=1.1*np.maximum(b[:2],[.25,.1])))))
 ax.scatter(truth[ii,2],base[ii,2],label='1s endpoint',alpha=.6);ax.scatter(truth[ii,2],alt[ii,2],label='5s OLS',alpha=.6);ax.set(title=day,xlabel='Observed mean turn deg/s',ylabel='Predicted mean turn deg/s');ax.grid(alpha=.2);ax.legend()
report=dict(plan=plan,metric_order=['speed m/s','climb m/s','turn deg/s'],results=result,decision='retain alternative for further evaluation' if all(r['passes'] for r in result) else 'reject alternative; no further initialization-window sweep')
(O/'mission-init-sensitivity.json').write_text(json.dumps(report,indent=2));fig.tight_layout();fig.savefig(O/'mission-init-sensitivity.png',dpi=150);print(json.dumps(report,indent=2))
