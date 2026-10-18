from pathlib import Path
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.stats import spearmanr
R=Path(__file__).parent;O=R.parent;day='2026-02-15';a=np.load(O/f'{day}-causal-input.npz');p=np.load(O/f'{day}-propulsion-9x45.npz');rows=[]
for path in sorted(R.glob(f'candidate_v2-{day}-*.npz')):
 start=int(path.stem.split('-')[-1]);d=np.load(path);e=np.load(R/f'preload_2.0-{day}-{start}.npz');t=d['t'];truth=d['truth'];rates=[]
 for v in [truth[:,6:9],d['state'][:,7:10],e['state'][:,7:10]]:
  course=np.unwrap(np.arctan2(v[:,1],v[:,0]));rates.append(float(np.polyfit(t,np.rad2deg(course-course[0]),1)[0]))
 i=np.searchsorted(a['t'],start);sl=slice(i,i+len(t));sens=(rates[2]-rates[1])/2;inferred=(rates[0]-rates[1])/sens if abs(sens)>.1 else np.nan;rows.append(dict(start=start,flight_rate=rates[0],model_rate=rates[1],sensitivity_deg_s_mm=sens,inferred_preload_mm=inferred,throttle=float(a['U'][sl,0].mean()),rpm=float(p['rpm'][sl].mean()),shaft_power=float(p['shaft_power_apc'][sl].mean()),electric_power=float(np.mean(p['voltage'][sl]*p['current'][sl])),voltage=float(p['voltage'][sl].mean()),right_mm=float(a['brake_mm'][sl,1].mean()),left_mm=float(a['brake_mm'][sl,0].mean()),airspeed=float(np.linalg.norm(truth[:,6:9]-d['wind'],axis=1).mean())))
(R/'trim-bias-diagnostic.json').write_text(json.dumps(rows,indent=2));fig,axs=plt.subplots(2,3,figsize=(14,8))
for ax,key in zip(axs.flat,['start','throttle','rpm','right_mm','left_mm','airspeed']):
 valid=[r for r in rows if np.isfinite(r['inferred_preload_mm'])];x=[r[key] for r in valid];y=[r['inferred_preload_mm'] for r in valid];sc=ax.scatter(x,y,c=[r['start'] for r in valid],cmap='viridis',s=22);ax.axhline(0,color='k',lw=.5);ax.set(xlabel=key,ylabel='Equivalent right preload (mm)',ylim=(-12,12));ax.grid(alpha=.2);print(key,'spearman',spearmanr(x,y).statistic)
fig.suptitle('Trout Lake residual-turn diagnostic: local equivalent preload, not measured rigging');fig.tight_layout();fig.savefig(R/'trim-bias-diagnostic.png',dpi=150)
for lo,hi in [(810,990),(2670,2850)]:
 rr=[r for r in rows if lo<=r['start']<hi];print(lo,{k:float(np.mean([r[k] for r in rr])) for k in ['throttle','rpm','shaft_power','electric_power','right_mm','left_mm','airspeed','inferred_preload_mm']})
