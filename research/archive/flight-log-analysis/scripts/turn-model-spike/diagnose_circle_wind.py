from pathlib import Path
import numpy as np,json
from scipy.optimize import least_squares
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
O=Path(__file__).parent;rows=[];samples=[]
for day in ['2026-01-03','2026-02-15']:
 a=np.load(O/(day+'-physical-input.npz'));meta=np.load(O/(day+'.npz'));t=a['t'];Y=a['Y']
 for start in np.arange(330 if day.endswith('03') else 220,t[-1]-20,20):
  ix=np.where((t>=start)&(t<start+20))[0][::5]
  if len(ix)<90 or not np.all(meta['airborne'][ix]) or np.ptp(Y[ix,2])<2.1 or np.ptp(a['U'][ix,0])>.18:continue
  v=Y[ix,6:8];psi=Y[ix,2];A=np.column_stack([-np.sin(psi),np.cos(psi)]);target=np.sum(v*A,axis=1)
  hw=least_squares(lambda w:A@w-target,[0,0],loss='soft_l1',f_scale=.3).x
  f=least_squares(lambda z:np.linalg.norm(v-z[:2],axis=1)-z[2],np.r_[hw,5.],bounds=([-5,-5,1],[5,5,12]),loss='soft_l1',f_scale=.3)
  base=np.array([1.04,.59]) if day.endswith('03') else np.array([.1,.086] if start<2500 else [.217,-1.139]);vb=np.linalg.norm(v-base,axis=1)
  rows.append(dict(day=day,start=float(start),circle_wind_NE=f.x[:2].tolist(),heading_wind_NE=hw.tolist(),circle_airspeed=float(f.x[2]),circle_speed_rmse=float(np.sqrt(np.mean(f.fun**2))),baseline_speed_std=float(vb.std()),heading_crossflow_rmse=float(np.sqrt(np.mean((A@hw-target)**2))),condition=float(np.linalg.cond(A)),median_abs_bank=float(np.median(np.abs(np.rad2deg(Y[ix,0]))))))
  samples.append((v,f.x,day,start))
(O/'turn-circle-wind-diagnostic.json').write_text(json.dumps(rows,indent=2))
fig,axes=plt.subplots(2,2,figsize=(12,9))
for day,color,label in [('2026-01-03','#d17728','Hood River'),('2026-02-15','#235b78','Trout Lake')]:
 r=[r for r in rows if r['day']==day];x=[r['start'] for r in r]
 axes[0,0].scatter(x,[r['circle_airspeed'] for r in r],s=22,c=color,label=label)
 axes[0,1].scatter([r['baseline_speed_std'] for r in r],[r['circle_speed_rmse'] for r in r],s=22,c=color)
 axes[1,0].scatter([r['heading_wind_NE'][0] for r in r],[r['circle_wind_NE'][0] for r in r],s=22,c=color)
 axes[1,1].scatter([r['heading_wind_NE'][1] for r in r],[r['circle_wind_NE'][1] for r in r],s=22,c=color)
axes[0,0].set(xlabel='Boot time (s)',ylabel='Circle-fitted airspeed (m/s)');axes[0,0].legend();axes[0,1].set(xlabel='Existing-wind speed standard deviation (m/s)',ylabel='Circle-fit speed RMS residual (m/s)')
for ax,label in zip(axes[1],['North','East']):ax.plot([-3,3],[-3,3],'k--',alpha=.4);ax.set(xlabel=f'{label} wind from heading assumption (m/s)',ylabel=f'{label} wind from circle assumption (m/s)',xlim=(-3,3),ylim=(-3,3))
for ax in axes.flat:ax.grid(alpha=.2)
fig.suptitle('Twenty-second turning segments: two different wind assumptions',fontsize=15);fig.tight_layout(rect=(0,0,1,.95));fig.savefig(O/'iteration-circle-wind.png',dpi=150)
print('segments',len(rows));print(json.dumps(rows[:5],indent=2));print('median circle residual',np.median([r['circle_speed_rmse'] for r in rows]))
