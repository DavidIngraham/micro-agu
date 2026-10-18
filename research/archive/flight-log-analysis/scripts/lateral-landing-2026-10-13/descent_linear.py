"""Local straight-descent lateral plant and inner-loop poles; no guidance or flare transient."""
import sys,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from microagu_sim import compat
model=Path(compat.__file__).parent
from microagu_sim.compat.profiles import make_simulator
from microagu_sim.compat.equilibrium import solve,perturb,difference
out=Path('/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/lateral-tuning-2026-10-13')
sim=make_simulator('trout_lake','mission');sim.substeps=2
a=np.load(model/'equilibrium-trim-trout_lake-0.0-0.0.npz');seed=a['state'];command=a['command'];dt=.0025
f=np.geomspace(.025,8,450);results=[];fig,axes=plt.subplots(2,2,figsize=(12,8),sharex=True)
for sink in [0,.5,1.,1.5,2.]:
 eq=solve(sim,seed,command,climb=-sink);res=float(np.max(np.abs(eq['residual'])))
 if res>1e-6:
  results.append(dict(sink=sink,residual=res,accepted=False));continue
 base=eq['state'];u=eq['command'];rpm=eq['rpm'];seed=base;command=u
 # Positive differential trims pull right. A zero-trim mixer cusp needs one-sided models.
 if u[2]<1e-3 or u[1]>1e-3:raise RuntimeError('Need different mixer branch')
 stroke=1500*np.pi*55/360*.1
 def obs(y):
  rot=R.from_quat(y[3:7]);roll,pitch,_=rot.as_euler('xyz');p,q,r=rot.inv().apply(y[10:13])
  return np.rad2deg([(q*np.sin(roll)+r*np.cos(roll))/np.cos(pitch),p])
 def advance(x,du=0):
  y=perturb(base,x[:15],sim.geometry);c=u.copy();c[2]+=du*stroke
  yn,rn=sim.step(y,rpm+x[15],c,dt=dt)
  return np.r_[difference(base,yn),rn-rpm]
 A=np.empty((16,16));C=np.zeros((2,16))
 for j in range(16):
  e=np.zeros(16);e[j]=1 if j==15 else 1e-5
  A[:,j]=(advance(e)-advance(-e))/(2*e[j])
  C[:,j]=(obs(perturb(base,e[:15],sim.geometry))-obs(perturb(base,-e[:15],sim.geometry)))/(2*e[j])
 B=(advance(np.zeros(16),1e-5)-advance(np.zeros(16),-1e-5))/2e-5
 H=np.array([C@np.linalg.solve(np.exp(2j*np.pi*ff*dt)*np.eye(16)-A,B) for ff in f])
 tas=float(np.linalg.norm(base[7:10]));scale=np.clip(5/max(tas,2),.25,2.5)
 modes={}
 for name,kp,ki,kroll,cutoff in [('landing',.04,.002,.03,4),('mission',.005,.002,.003,4),('mission_1hz',.005,.002,.003,1)]:
  alpha=dt/(dt+1/(2*np.pi*cutoff))
  def closed(x):
   filtered=x[16:18]+alpha*(C@x[:16]-x[16:18]);integ=x[18]-ki*dt*filtered[0]
   control=-kp*scale*filtered[0]-kroll*scale*filtered[1]+integ
   return np.r_[A@x[:16]+B*control,filtered,integ]
  F=np.column_stack([closed(e) for e in np.eye(19)]);z=np.log(np.linalg.eigvals(F).astype(complex))/dt
  modes[name]=[dict(real=float(v.real),hz=float(v.imag/(2*np.pi))) for v in sorted(z,key=lambda v:-v.real)]
 for j in range(2):
  axes[0,j].semilogx(f,20*np.log10(np.maximum(abs(H[:,j]),1e-12)),label=f'{sink:g} m/s sink')
  axes[1,j].semilogx(f,np.unwrap(np.angle(H[:,j]))*180/np.pi)
 np.savez(out/f'descent-linear-{sink:g}.npz',A=A,B=B,C=C,state=base,command=u,rpm=rpm,dt=dt)
 row=dict(sink=sink,accepted=True,residual=res,tas=tas,command=u.tolist(),modes=modes)
 results.append(row);print(sink,u,tas,{k:next((v for v in m if abs(v['hz'])>.01),m[0]) for k,m in modes.items()},flush=True)
axes[0,0].set_title('Heading rate / differential brake fraction');axes[0,1].set_title('Body roll rate / differential brake fraction')
for ax in axes[0]:ax.set_ylabel('Gain (dB, deg/s per fraction)');ax.legend()
for ax in axes[1]:ax.set_ylabel('Phase (deg)');ax.set_xlabel('Frequency (Hz)')
for ax in axes.flat:ax.grid(True,which='both',alpha=.25)
fig.suptitle('Straight descending equilibria: 9-DOF mission plant, fixed throttle')
fig.tight_layout();fig.savefig(out/'descent-bode.png',dpi=150)
(out/'descent-linear.json').write_text(json.dumps(results,indent=2))
