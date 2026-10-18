from pathlib import Path
import sys,json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation as Rot
R=Path(__file__).parent;sys.path.insert(0,str(R/'candidate-v3'));from simulator import Simulator
rows=[];names=['reference','retained','dynamic_roll']
for day in ['2026-01-03','2026-02-15']:
 for path in sorted(R.glob(f'atlas-{day}-*-combined_small.npz')):
  start=int(path.name.split('-')[-2]);d=np.load(path);truth=d['truth'];wind=d['wind'];v=truth[:,6:9]-wind;course=np.unwrap(np.arctan2(v[:,1],v[:,0]));rate=np.rad2deg(savgol_filter(course,101,3,deriv=1,delta=.04));mask=np.arange(len(rate));mask=(mask>=125)&(mask<700);mean=float(rate[mask].mean());consistency=abs(mean)/max(np.mean(abs(rate[mask])),1e-6);variation=float(rate[mask].std()/max(abs(mean),1e-6))
  if not (3<=abs(mean)<=20 and consistency>=.8 and variation<=.5):continue
  flight='hood_river' if day=='2026-01-03' else 'trout_lake';files={'reference':R/f'atlas-{day}-{start}-reference.npz','retained':R/f'candidate_v3-{day}-{start}.npz','dynamic_roll':R/f'soft_v3_retrim-{day}-{start}.npz'};row=dict(day=day,start=start,observed_turn_deg_s=mean,observed_radius_m=float(np.linalg.norm(v[mask,:2],axis=1).mean()/abs(np.deg2rad(mean))),observed_climb=float(-truth[mask,8].mean()),observed_bank=float(np.rad2deg(truth[mask,0]).mean()),direction_consistency=consistency,relative_rate_std=variation)
  for name,file in files.items():
   y=np.load(file)['state'];g=Simulator(flight,model='aero' if name=='reference' else 'modal').geometry;velocity=y[:,7:10]+np.cross(y[:,10:13],Rot.from_quat(y[:,3:7]).apply(np.broadcast_to(g['imu'],(len(y),3))));va=velocity-wind;c=np.unwrap(np.arctan2(va[:,1],va[:,0]));r=np.rad2deg(savgol_filter(c,101,3,deriv=1,delta=.04));m=float(r[mask].mean());row[name]=dict(turn_deg_s=m,radius_m=float(np.linalg.norm(va[mask,:2],axis=1).mean()/max(abs(np.deg2rad(m)),1e-8)),direction_correct=bool(mean*m>0),climb=float(-velocity[mask,2].mean()))
  rows.append(row)
good=[r for r in rows if all(r[q]['direction_correct'] and abs(r[q]['turn_deg_s'])>1 for q in names)];summary=[];fig,axs=plt.subplots(1,3,figsize=(15,5))
for name,color,label in [('reference','#999999','Starting model'),('retained','#228866','Mission'),('dynamic_roll','#d88735','Dynamic roll')]:
 directions=sum(r[name]['direction_correct'] for r in rows);axs[0].scatter([r['observed_turn_deg_s'] for r in rows],[r[name]['turn_deg_s'] for r in rows],label=f'{label} ({directions}/{len(rows)} signs)',c=color,alpha=.8,s=25);axs[1].scatter([r['observed_radius_m'] for r in good],[r[name]['radius_m'] for r in good],label=label,c=color,alpha=.8,s=25);axs[2].scatter([r['observed_climb'] for r in rows],[r[name]['climb'] for r in rows],label=label,c=color,alpha=.8,s=25);q=dict(name=name,windows=len(rows),correct_directions=directions,wrong_direction_starts=[r['start'] for r in rows if not r[name]['direction_correct']],rate_mae_deg_s=float(np.mean([abs(r[name]['turn_deg_s']-r['observed_turn_deg_s']) for r in rows])),radius_common_windows=len(good),median_abs_log_radius_error=float(np.median([abs(np.log(r[name]['radius_m']/r['observed_radius_m'])) for r in good])));summary.append(q);print(q)
for ax,label in zip(axs,['Turn rate (deg/s)','Air-relative radius (m)','Climb during turn (m/s)']):ax.set(xlabel='Observed '+label,ylabel='Predicted '+label);ax.grid(alpha=.2);ax.legend(fontsize=8)
for ax in [axs[0],axs[2]]:
 lim=[min(ax.get_xlim()[0],ax.get_ylim()[0]),max(ax.get_xlim()[1],ax.get_ylim()[1])];ax.plot(lim,lim,'k--',lw=.8)
axs[1].set_xscale('log');axs[1].set_yscale('log');axs[1].plot([3,1000],[3,1000],'k--',lw=.8);fig.suptitle(f'{len(rows)} sustained TL right turns; radius comparison uses {len(good)} windows with correct direction and >1°/s in all models');fig.tight_layout();fig.savefig(R/'sustained-turn-crosscheck.png',dpi=150)
(R/'sustained-turn-crosscheck.json').write_text(json.dumps(dict(selection='Previously selected ordinary windows; analyzed5..28s, wind-estimated air-course rate3..20deg/s, direction consistency>=.8, relative rate standard deviation<=.5. All selected turns are right turns in TL. Observed sustained turns, not a maximum capability envelope.',summary=summary,windows=rows),indent=2))
