from pathlib import Path
import sys,json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as Rot
R=Path(__file__).parent;O=R.parent;sys.path.insert(0,str(O));sys.path.insert(0,str(R/'candidate-v2'))
from simulator import Simulator
source=(R/'ordinary_replay.py').read_text().split('rows=[]')[0];ns={'__file__':str(R/'ordinary_replay.py')};exec(source,ns);initialize=ns['initialize']
rows=[];cases=[('2026-01-03',571,60),('2026-02-15',3343,60),('2026-01-03',300,180),('2026-02-15',810,180),('2026-02-15',2670,180)];fig,axs=plt.subplots(5,3,figsize=(14,16))
for ci,(day,start,duration) in enumerate(cases):
 a=np.load(O/f'{day}-causal-input.npz');prop=np.load(O/f'{day}-propulsion-9x45.npz');i=np.searchsorted(a['t'],start);sl=slice(i,i+round(duration/.04)+1);truth=a['Y'][sl];t=np.arange(len(truth))*.04;wind=a['wind'][i];rpm=float(prop['rpm'][i]);voltage=float(prop['voltage'][i]);u=np.column_stack([a['U'][sl,0],a['brake_mm'][sl]]);ct=np.unwrap(np.arctan2(truth[:,7],truth[:,6]));ct-=ct[0]
 for j,arr in enumerate([np.rad2deg(ct),np.rad2deg(truth[:,0]),-truth[:,8]]):axs[ci,j].plot(t,arr,'k',label='Flight',lw=1)
 for preload in [0.,2.,4.]:
  model=f'preload_{preload}'
  sim=Simulator('hood_river' if day=='2026-01-03' else 'trout_lake',model="modal");sim.parameters["right_offset"]=-preload/1000;g=sim.geometry;z=initialize(a,i,g,wind,rpm);y,r,applied=sim.replay_actuated(z,u,wind=wind,voltage=voltage,rpm=rpm);rot=Rot.from_quat(y[:,3:7]);v=y[:,7:10]+np.cross(y[:,10:13],rot.apply(np.broadcast_to(g['imu'],(len(y),3))));c=np.unwrap(np.arctan2(v[:,1],v[:,0]));c-=c[0];bank=rot.as_euler('xyz',degrees=True)[:,0];ce=np.rad2deg(c-ct);row=dict(day=day,start=start,duration=duration,model=model,course_E=float(2/duration**2*np.trapz(abs(ce),t)),bank_rmse=float(np.sqrt(np.mean((bank-np.rad2deg(truth[:,0]))**2))),climb_rmse=float(np.sqrt(np.mean((v[:,2]-truth[:,8])**2))))
  if duration==60:
   cut=591.24385 if day=='2026-01-03' else 3363.333506;mask=(t>=cut-start)&(t<=cut-start+10);row.update(observed_sink=float(np.mean(truth[mask,8])),model_sink=float(np.mean(v[mask,2])))
  rows.append(row);np.savez_compressed(R/f'preload-context-{day}-{start}-{model}.npz',state=y,rpm=r,truth=truth,t=t,controls=u,wind=wind)
  for j,arr in enumerate([np.rad2deg(c),bank,-v[:,2]]):axs[ci,j].plot(t,arr,label=model,lw=.8)
  print(row,flush=True)
 for j,label in enumerate(['Relative course (deg)','Bank (deg)','Climb (m/s)']):axs[ci,j].set(title=f'{day} at {start}s: {label}',xlabel='Elapsed time (s)');axs[ci,j].grid(alpha=.2)
axs[0,0].legend();fig.suptitle('Longer continuous contexts: constant right-brake preload hypotheses');fig.tight_layout();fig.savefig(R/'preload-long-contexts.png',dpi=140);(R/'preload-context-results.json').write_text(json.dumps(rows,indent=2))

