"""Sustained idle-throttle spiral authority screen; no controller or aero edits."""
import json,sys,hashlib,subprocess
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from microagu_sim import compat
MODEL=Path(compat.__file__).parent
from microagu_sim.compat.profiles import make_simulator
from microagu_sim.compat.trim import find_trim
out=Path(__file__).parent
raw=Path('/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/spiral-swoop-2026-10-13');raw.mkdir(exist_ok=True)
rows=[];fig,axs=plt.subplots(2,3,figsize=(14,8))
for pi,profile in enumerate(['mission','dynamic_roll']):
 sim=make_simulator('trout_lake',profile);trim=find_trim(sim)
 for pull in [0,10,20,30,40,55,65]:
  dt=.02;t=np.arange(0,32+dt/2,dt);u=np.tile(trim['command'],(len(t),1));u[:,0]=0
  ramp=np.clip(t/1.5,0,1)*np.clip((24-t)/1.5,0,1)
  u[:,2]=np.minimum(72,u[:,2]+pull*ramp)
  y,r,a=sim.replay_actuated(trim['state'],u,rpm=trim['rpm'],dt=dt)
  e=Rotation.from_quat(y[:,3:7]).as_euler('xyz',degrees=True);yaw=np.rad2deg(np.unwrap(np.deg2rad(e[:,2])));v=np.linalg.norm(y[:,7:10],axis=1);hs=np.linalg.norm(y[:,7:9],axis=1)
  steady=(t>=14)&(t<=22);ix=np.argmin(abs(t-23));after=(t>=24)&(t<=28)
  row=dict(profile=profile,pull_mm=pull,trim_speed=float(trim['horizontal_speed']),spiral_airspeed=float(np.mean(v[steady])),spiral_horizontal=float(np.mean(hs[steady])),spiral_sink=float(np.mean(y[steady,9])),spiral_bank=float(np.mean(e[steady,0])),heading_change=float(yaw[ix]-yaw[0]),height_loss=float(y[ix,2]-y[0,2]),rollout_peak_airspeed=float(max(v[after])),rollout_mean_horizontal=float(np.mean(hs[after])))
  rows.append(row);np.savez_compressed(raw/f'screen-{profile}-{pull}.npz',t=t,state=y,rpm=r,applied=a,command=u)
  axs[pi,0].plot(t,v,label=f'{pull} mm');axs[pi,1].plot(t,y[:,9]);axs[pi,2].plot(t,e[:,0])
  print(row,flush=True)
 for j,label in enumerate(['Airspeed (m/s)','Downward speed (m/s)','Payload bank (deg)']):
  axs[pi,j].set(title=profile,ylabel=label,xlabel='Time (s)');axs[pi,j].axvline(24,color='gray',ls='--');axs[pi,j].grid(alpha=.2)
 axs[pi,0].legend(ncol=2)
fig.suptitle('Sustained asymmetric brake at idle; release over 22.5-24 s. Same physical limits and model.');fig.tight_layout();fig.savefig(out/'authority.png',dpi=150)
(out/'authority.json').write_text(json.dumps(rows,indent=2))
(out/'provenance.json').write_text(json.dumps(dict(simulator_package=str(MODEL),model_parameters={profile:make_simulator('trout_lake',profile).parameters for profile in ['mission','dynamic_roll']}),indent=2,default=lambda x:np.asarray(x).tolist()))
