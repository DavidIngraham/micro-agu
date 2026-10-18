"""Bounded fast-flare sensitivity; open-loop physics diagnostic, not controller proof."""
import sys,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from microagu_sim.compat.profiles import make_simulator
raw=Path('/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/swoop-2026-10-13');out=Path(__file__).parent
trim=np.load(raw/'trim.npz');sim=make_simulator('trout_lake','mission');dt=.02;t=np.arange(0,14+dt/2,dt);rows=[]
fig,axes=plt.subplots(2,1,figsize=(10,8),sharex=True)
for delay in [0.,1.,2.]:
 for ramp_s in [.15,.4,.8,1.5]:
  u=np.tile(trim['command'],(len(t),1));u[:,0]=0
  u[t<3.0,2]+=20
  flare=3+delay;ramp=np.clip((t-flare)/ramp_s,0,1)*.95
  u[:,1]=np.minimum(62.39,u[:,1]+62.39*ramp);u[:,2]=np.minimum(72.,u[:,2]+72*ramp)
  y,r,applied=sim.replay_actuated(trim['state'],u,rpm=float(trim['rpm']),dt=dt)
  speed=np.linalg.norm(y[:,7:9],axis=1);sink=y[:,9];angles=Rotation.from_quat(y[:,3:7]).as_euler('xyz',degrees=True)
  mask=(t>flare)&(t<flare+4);ix=np.where(mask)[0][np.argmin(speed[mask]**2+4*sink[mask]**2+(angles[mask,0]/10)**2)]
  row=dict(delay=delay,ramp_s=ramp_s,time=float(t[ix]),horizontal=float(speed[ix]),sink=float(sink[ix]),min_sink=float(sink[mask].min()),min_speed=float(speed[mask].min()),bank=float(angles[ix,0]),heading=float(angles[ix,2]),height_loss=float(y[ix,2]))
  rows.append(row);np.savez_compressed(raw/f'fast-{delay}-{ramp_s}.npz',t=t,state=y,rpm=r,command=u,applied=applied)
  if delay==1:
   axes[0].plot(t-flare,speed,label=f'{ramp_s}s commanded ramp');axes[1].plot(t-flare,sink)
axes[0].legend();axes[0].set(ylabel='Horizontal speed (m/s)');axes[1].set(xlabel='Time from flare command (s)',ylabel='Downward speed (m/s)')
for ax in axes:ax.grid(alpha=.2);ax.axvline(0,color='gray',ls='--')
fig.suptitle('95% brake flare after a 3 s turning pull and 1 s release; actuator rate retained');fig.tight_layout();fig.savefig(out/'fast-flare.png',dpi=140)
(out/'fast-flare.json').write_text(json.dumps(rows,indent=2))
for row in sorted(rows,key=lambda x:x['horizontal']**2+4*x['sink']**2):print(row)