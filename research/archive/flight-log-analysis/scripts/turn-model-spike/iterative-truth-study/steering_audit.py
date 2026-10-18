from pathlib import Path
import json,sys,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).parent;O=R.parent;sys.path.insert(0,str(O));from mission2_model import thrust
rows=json.loads((R/'frozen-atlas.json').read_text())['rows'];p=json.loads((R/'cycle2-result.json').read_text())['parameters'];out=[];traces={}
for day in ['2026-01-03','2026-02-15']:
 a=np.load(O/f'{day}-causal-input.npz');prop=np.load(O/f'{day}-propulsion-9x45.npz');mass=1.385 if day=='2026-01-03' else 1.758
 for k,r in enumerate(rows):
  if r['day']!=day or r['tier']!='ordinary':continue
  i=np.searchsorted(a['t'],r['start_s']);sl=slice(i,i+751);u=a['U'][sl];vxy=a['Y'][sl,6:8]-a['wind'][i,:2];v=np.linalg.norm(vxy,axis=1);chi=np.unwrap(np.arctan2(vxy[:,1],vxy[:,0]));q=np.gradient(chi,.04);q=np.convolve(q,np.ones(51)/51,'same');T=thrust(prop['rpm'][sl],v)/mass;eq=(p['right_gain']*u[:,2]-p['left_gain']*u[:,1]+p['trim_HR' if day=='2026-01-03' else 'trim_TL']+p['trim_thrust']*T)/np.maximum(v,1)
  pred=np.zeros(len(v));pred[0]=q[0]
  for j in range(len(v)-1):pred[j+1]=eq[j]+(pred[j]-eq[j])*np.exp(-.04/p['turn_tau'])
  valid=slice(50,-50);qobs=float(np.rad2deg((chi[-51]-chi[50])/(.04*(len(chi)-101))));features=[float(u[valid,1].mean()),float(u[valid,2].mean()),float(T[valid].mean()),float(v[valid].mean())]
  out.append(dict(index=k,day=day,start_s=r['start_s'],features=features,observed_rate=qobs,equilibrium_rate=float(np.rad2deg(eq[valid]).mean()),forced_rate=float(np.rad2deg(pred[valid]).mean()),residual=float(np.rad2deg(pred[valid]).mean()-qobs)))
  traces[k]=(np.arange(len(v))*.04,u,T,v,np.rad2deg(q),np.rad2deg(eq),np.rad2deg(pred))
# Match episodes by scaled measured controls/thrust/speed, independent of response residual.
tl=[r for r in out if r['day']=='2026-02-15'];scale=np.array([.1,.1,1.,1.]);pairs=[]
for r in tl:
 candidates=[s for s in tl if abs(s['start_s']-r['start_s'])>=600]
 if not candidates:continue
 s=min(candidates,key=lambda s:np.linalg.norm((np.array(s['features'])-r['features'])/scale));dist=float(np.linalg.norm((np.array(s['features'])-r['features'])/scale));pairs.append(dict(first=r['index'],second=s['index'],distance=dist,observed_rate_difference=r['observed_rate']-s['observed_rate'],residual_difference=r['residual']-s['residual']))
(R/'steering-forcing-audit.json').write_text(json.dumps(dict(note='Diagnostic uses measured V/RPM and historical wind; cannot establish coupled acceptance. Smoothed derivative boundaries excluded from means.',features=['left fraction','right fraction','thrust N/kg','airspeed m/s'],rows=out,matched_pairs=sorted(pairs,key=lambda r:r['distance'])),indent=2))
fig,axs=plt.subplots(3,2,figsize=(14,11))
for ax,start in zip(axs.flat,[780,990,810,1920,2670,2790]):
 day='2026-01-03' if start in [780,990] else '2026-02-15';r=next(s for s in out if s['start_s']==start and s['day']==day);t,u,T,v,q,eq,pr=traces[r['index']];ax.plot(t,q,'k',label='inferred air-course rate');ax.plot(t,eq,alpha=.5,label='equilibrium from measured V/RPM');ax.plot(t,pr,label='forced steering replay');ax.set_title(f'{day} {start}s');ax.set(xlabel='Seconds',ylabel='deg/s');ax.grid(alpha=.2);ax.legend(fontsize=8)
fig.tight_layout();fig.savefig(R/'steering-forcing-audit.png',dpi=140);print(json.dumps(dict(selected=[r for r in out if r['start_s'] in [780,990,810,1920,2670,2790]],closest_pairs=sorted(pairs,key=lambda r:r['distance'])[:5]),indent=2))
