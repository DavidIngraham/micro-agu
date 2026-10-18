import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mission2_model import O,predict
D=dict(np.load(O/'mission2-evaluation-data.npz'));rr=json.loads((O/'mission2-evaluation-records.json').read_text());p=json.loads((O/'mission2-fit.json').read_text())['parameters'];raw={day:np.load(O/f'{day}-causal-input.npz') for day in {r['day'] for r in rr}};result=[]
offsets=[(0,0),(.5,0),(-.5,0),(0,.5),(0,-.5)]
for offset in offsets:
 pred,series,bad=predict(p,D,wind_delta=offset,series=True)
 for j,r in enumerate(rr):
  y=raw[r['day']]['Y'][r['index']:r['index']+251];va=y[:,6:8]-D['wind'][j]-offset;speed=np.linalg.norm(va,axis=1);heading=np.unwrap(np.arctan2(va[:,1],va[:,0]));q=(heading[-1]-heading[0])/10;qm=np.trapz(series[j,:,4],dx=.04)/10;vmean=np.trapz(speed,dx=.04)/10;vm=np.trapz(series[j,:,3],dx=.04)/10;dh=D['truth'][j,1]
  result.append(dict(day=r['day'],start_s=r['start_s'],category=r['category'],wind_delta_N_E=offset,observed_inferred_air_speed=vmean,model_air_speed=vm,observed_inferred_turn_deg_s=float(np.rad2deg(q)),model_turn_deg_s=float(np.rad2deg(qm)),observed_effective_radius_m=float(vmean/abs(q)) if abs(q)>1e-6 else None,model_effective_radius_m=float(vm/abs(qm)) if abs(qm)>1e-6 else None,observed_altitude_change_m=float(dh),model_altitude_change_m=float(pred[j,1]),observed_inferred_glide_ratio=float(vmean*10/-dh) if r['category']=='glide' and dh<0 else None,model_glide_ratio=float(vm*10/-pred[j,1]) if r['category']=='glide' and pred[j,1]<0 else None,mean_throttle=float(D['U'][j,125:,0].mean()),mean_brake_mm=(D['U'][j,125:,1:].mean(axis=0)*72).tolist()))
(O/'mission2-inferred-performance.json').write_text(json.dumps(dict(scope='Wind-dependent effective segment summaries, not steady trim, maximum sustained performance or independently measured airdata. Effective radius uses mean speed divided by net turn rate; oscillatory paths need not trace a circle.',rows=result),indent=2))
fig,axes=plt.subplots(2,2,figsize=(12,7));nom=[x for x in result if x['wind_delta_N_E']==(0,0)]
for j,day in enumerate(['2026-01-03','2026-02-15']):
 x=[r for r in nom if r['day']==day];labels=[str(round(r['start_s'])) for r in x];idx=np.arange(len(x))
 for k,(obs,mod,title) in enumerate([('observed_altitude_change_m','model_altitude_change_m','Altitude change (m)'),('observed_inferred_turn_deg_s','model_turn_deg_s','Inferred air-relative turn (deg/s)')]):
  ax=axes[j,k];ax.plot(idx,[r[obs] for r in x],'o-',label='Observed / inferred');ax.plot(idx,[r[mod] for r in x],'s--',label='Model')
  if k==1:
   for i,r in enumerate(x):
    z=[v[obs] for v in result if v['day']==day and v['start_s']==r['start_s']];ax.vlines(i,min(z),max(z),color='gray',lw=3,alpha=.5)
  ax.set_xticks(idx,labels);ax.set(title=day+' '+title,xlabel='Window start (s)');ax.legend();ax.grid(alpha=.2)
fig.suptitle('Fixed windows; gray ranges show prescribed wind sensitivity, not confidence intervals');fig.tight_layout();fig.savefig(O/'mission2-inferred-performance.png',dpi=150)
print(json.dumps([x for x in nom if x['category']=='glide'],indent=2))
