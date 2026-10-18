"""Open-loop authority screening only; not native SITL or a landing controller."""
import sys,json,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from microagu_sim.compat.profiles import make_simulator
from microagu_sim.compat.trim import find_trim
raw=Path('/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/swoop-2026-10-13')
out=Path(__file__).parent
sim=make_simulator('trout_lake','mission')
trim=find_trim(sim)
np.savez(raw/'trim.npz',**{k:trim[k] for k in ['state','rpm','command']})
dt=.02;t=np.arange(0,20+dt/2,dt);rows=[];fig,axs=plt.subplots(2,2,figsize=(12,8))
for turn in [0,10,20,30]:
 for brake in [.2,.4,.6,.8,1.]:
  u=np.tile(trim['command'],(len(t),1));u[:,0]=0
  u[(t>=0)&(t<4),2]+=turn
  ramp=np.clip((t-7)/2,0,1)*brake
  u[:,1]=np.minimum(62.39,u[:,1]+62.39*ramp)
  u[:,2]=np.minimum(72.0,u[:,2]+72*ramp)
  y,r,applied=sim.replay_actuated(trim['state'],u,rpm=trim['rpm'],dt=dt)
  angles=Rotation.from_quat(y[:,3:7]).as_euler('xyz',degrees=True)
  speed=np.linalg.norm(y[:,7:9],axis=1);sink=y[:,9]
  mask=(t>=7)&(t<14)
  ix=np.where(mask)[0][np.argmin(speed[mask]**2+4*sink[mask]**2+(angles[mask,0]/10)**2)]
  row=dict(turn_mm=turn,brake=brake,best_time=float(t[ix]),horizontal=float(speed[ix]),sink=float(sink[ix]),roll=float(angles[ix,0]),height_loss=float(y[ix,2]),heading_change=float(np.rad2deg(np.unwrap(np.deg2rad(angles[:,2])))[ix]-angles[0,2]),min_sink=float(sink[mask].min()),min_speed=float(speed[mask].min()))
  rows.append(row)
  np.savez_compressed(raw/f'probe-{turn}-{brake:.1f}.npz',t=t,state=y,rpm=r,command=u,applied=applied)
  if turn in [0,20]:
   ax=axs[0,turn//20];ax.plot(t,speed,label=f'{brake:.0%} common')
   ax=axs[1,turn//20];ax.plot(t,sink)
for j,turn in enumerate([0,20]):
 axs[0,j].set(title=f'{turn} mm right pull for 4 s, idle throttle',ylabel='Horizontal speed (m/s)');axs[0,j].legend()
 axs[1,j].set(ylabel='Downward velocity (m/s)',xlabel='Time (s)')
 for ax in axs[:,j]:ax.axvline(7,color='k',ls='--',alpha=.4);ax.grid(alpha=.2)
fig.suptitle('Authority screening: release turn at 4 s; ramp common brake at 7 s over 2 s')
fig.tight_layout();fig.savefig(out/'authority.png',dpi=140)
(out/'authority.json').write_text(json.dumps(rows,indent=2))
print('Trim',trim['command'],trim['horizontal_speed'])
for row in sorted(rows,key=lambda x:x['horizontal']**2+4*x['sink']**2)[:8]:print(row)