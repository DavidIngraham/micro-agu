from pathlib import Path
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as Rot
R=Path(__file__).parent
import sys
sys.path.insert(0,str(R/'candidate-v3'))
from simulator import Simulator
rows=json.loads((R/'atlas-results.json').read_text());v2=json.loads((R/'candidate-v2-atlas-results.json').read_text());rows+=v2+json.loads((R/'candidate-v3-atlas-results.json').read_text())
names=['reference','combined_small','candidate_v2','retained'];labels=['Starting model','Aero correction','Modal mechanics','Modal + rigging'];summary=[];fig,axs=plt.subplots(2,4,figsize=(15,8))
for i,day in enumerate(['2026-01-03','2026-02-15']):
 for name in names:
  rr=[r for r in rows if r['day']==day and r['variant']==name];summary.append(dict(day=day,variant=name,n=len(rr),**{k:float(np.mean([r[k] for r in rr])) for k in ['course_E_deg_s','bank_rmse_deg','speed_rmse','climb_rmse']}))
 for j,(key,label) in enumerate([('course_E_deg_s','Ground-course E (deg/s)'),('bank_rmse_deg','Bank RMSE (deg)'),('speed_rmse','Ground-speed RMSE (m/s)'),('climb_rmse','Climb RMSE (m/s)')]):
  values=[next(r[key] for r in summary if r['day']==day and r['variant']==name) for name in names];axs[i,j].bar(range(4),values,color=['#999999','#4477aa','#8877aa','#228866']);axs[i,j].set_xticks(range(4),labels,rotation=30,ha='right',fontsize=8);axs[i,j].set(title='Hood River' if i==0 else 'Trout Lake',ylabel=label);axs[i,j].grid(axis='y',alpha=.2)
fig.suptitle('Iterative improvement across 98 ordinary flight windows; lower is better');fig.tight_layout();fig.savefig(R/'retained-comparison.png',dpi=150);(R/'retained-summary.json').write_text(json.dumps(summary,indent=2))
fig,axs=plt.subplots(3,3,figsize=(14,11))
for i,(day,start) in enumerate([('2026-01-03',300),('2026-02-15',810),('2026-02-15',2670)]):
 file=R/(f'preload-context-{day}-{start}-preload_6.0.npz' if i==0 else f'v2-context-{day}-{start}-modal.npz');d=np.load(file);base=np.load(R/f'long-{day}-{start}-reference.npz');t=d['t'];truth=d['truth']
 for name,y,color in [('Flight',None,'black'),('Starting model',base['state'],'#999999'),('Retained candidate',d['state'],'#228866')]:
  v=truth[:,6:9] if y is None else y[:,7:10]+np.cross(y[:,10:13],Rot.from_quat(y[:,3:7]).apply(np.broadcast_to(Simulator('hood_river' if day=='2026-01-03' else 'trout_lake',model='aero' if name=='Starting model' else 'modal').geometry['imu'],(len(y),3))));c=np.unwrap(np.arctan2(v[:,1],v[:,0]));c=np.rad2deg(c-c[0]);bank=np.rad2deg(truth[:,0]) if y is None else Rot.from_quat(y[:,3:7]).as_euler('xyz',degrees=True)[:,0]
  for j,z in enumerate([c,bank,-v[:,2]]):axs[i,j].plot(t,z,label=name,color=color,lw=.9)
 for j,label in enumerate(['Relative course (deg)','Payload bank (deg)','Climb (m/s)']):axs[i,j].set(title=f'{day} at {start}s',xlabel='Elapsed time (s)',ylabel=label);axs[i,j].grid(alpha=.2)
axs[0,0].legend(fontsize=8);fig.suptitle('Three-minute continuous replays: improvement and unresolved early Trout Lake bias');fig.tight_layout();fig.savefig(R/'retained-long-contexts.png',dpi=150)
fig,axs=plt.subplots(2,3,figsize=(14,8))
for i,flight in enumerate(['hood_river','trout_lake']):
 for name in ['level_trim','power_trim']:
  d=np.load(R/f'v3-controller-{flight}-{name}.npz');t=d['t'];a=d['data']
  for j,z in enumerate([a[:,3],a[:,7],a[:,4]]):axs[i,j].plot(t,z,label=name,lw=1)
 axs[i,0].plot(t,a[:,0],'k--',label='request');axs[i,1].plot(t,a[:,1],'k--')
 for j,label in enumerate(['Air-course rate (deg/s)','Climb (m/s)','Normalized brake command']):axs[i,j].set(title=flight.replace('_',' ').title(),xlabel='Elapsed time (s)',ylabel=label);axs[i,j].grid(alpha=.2)
axs[0,0].legend();fig.suptitle('Same conservative feedback: adding a power-dependent turn-trim correction');fig.tight_layout();fig.savefig(R/'retained-controller-comparison.png',dpi=150)

