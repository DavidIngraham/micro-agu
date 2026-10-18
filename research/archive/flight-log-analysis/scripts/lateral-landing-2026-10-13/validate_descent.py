import sys,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
from microagu_sim.compat.profiles import make_simulator
r=Path('/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/lateral-tuning-2026-10-13');sim=make_simulator('trout_lake','mission');sim.substeps=2;results=[]
for sink in [0,.5,1]:
 a=np.load(r/f'descent-linear-{sink:g}.npz');dt=float(a['dt']);base=a['state'];u=a['command'];A=a['A'];B=a['B'];C=a['C']
 for freq in [.35,2.5]:
  amp=1e-5;t=np.arange(0,80,dt);commands=np.tile(u,(len(t),1));commands[:,2]+=amp*(1500*np.pi*55/360*.1)*np.sin(2*np.pi*freq*t)
  y,_=sim.replay(base,commands,rpm=float(a['rpm']),dt=dt)
  rot=R.from_quat(y[:,3:7]);angles=rot.as_euler('xyz');rates=rot.inv().apply(y[:,10:13]);roll=angles[:,0];pitch=angles[:,1]
  obs=np.rad2deg(np.c_[(rates[:,1]*np.sin(roll)+rates[:,2]*np.cos(roll))/np.cos(pitch),rates[:,0]])
  mask=t>40;tt=t[mask];X=np.c_[np.sin(2*np.pi*freq*tt),np.cos(2*np.pi*freq*tt),np.ones(len(tt)),tt-tt.mean()]
  fit=np.linalg.lstsq(X,obs[mask],rcond=None)[0];actual=(fit[0]+1j*fit[1])/amp
  H=C@np.linalg.solve(np.exp(2j*np.pi*freq*dt)*np.eye(16)-A,B)
  row=dict(sink=sink,hz=freq,relative_complex_error=(abs(actual-H)/np.maximum(abs(H),1e-9)).tolist())
  results.append(row);print(row,flush=True)
(r/'descent-validation.json').write_text(json.dumps(results,indent=2))
