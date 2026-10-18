"""Exact physical travel, both directions, three entry and release times."""
from common import *
import numpy as np,json
from scipy.spatial.transform import Rotation
from profiles import make_simulator
from trim import find_trim
LIMITS=np.array([1300,1500])*np.pi*55/360*.1
rows=[];dt=.04;t=np.arange(0,40+dt/2,dt)
for profile in ['mission','dynamic_roll']:
 sim=make_simulator('trout_lake',profile);trim=find_trim(sim)
 tests=[('baseline',0,1.5,1.5)]+[(side,fraction,entry,release) for side in ['left','right'] for fraction,entry,release in [(f,1.5,1.5) for f in [.5,.75]]+[(1.,e,x) for e in [.15,1.5,4.] for x in [.15,1.5,4.]]]
 for side,fraction,entry,release in tests:
  name=f'{profile}-{side}-{fraction:g}-{entry:g}-{release:g}'
  u=np.tile(trim['command'],(len(t),1));u[t>=4,0]=0
  target=np.zeros(2)
  if side!='baseline':target[0 if side=='left' else 1]=LIMITS[0 if side=='left' else 1]*fraction
  else:target=trim['command'][1:].copy()
  ramp=np.clip((t-6)/entry,0,1)*np.clip(1-(t-26)/release,0,1)
  u[:,1:]=trim['command'][1:]+ramp[:,None]*(target-trim['command'][1:])
  y,rpm,applied=sim.replay_actuated(trim['state'],u,rpm=trim['rpm'],dt=dt)
  np.savez_compressed(RAW/f'{name}.npz',t=t,state=y,rpm=rpm,applied=applied,command=u)
  angles=Rotation.from_quat(y[:,3:7]).as_euler('xyz',degrees=True);canopy=Rotation.from_quat(y[:,16:20]).as_euler('xyz',degrees=True)
  speed=np.linalg.norm(y[:,7:10],axis=1);horizontal=np.linalg.norm(y[:,7:9],axis=1)
  heading=np.unwrap(np.arctan2(y[:,8],y[:,7]));rate=np.gradient(heading,dt)
  steady=(t>=18)&(t<=25);exit=(t>=26)&(t<=36);on=(t>=6)&(t<26)
  row=dict(name=name,profile=profile,side=side,fraction=fraction,entry_s=entry,release_s=release,mean_airspeed=float(speed[steady].mean()),mean_horizontal=float(horizontal[steady].mean()),mean_sink=float(y[steady,9].mean()),mean_payload_bank=float(angles[steady,0].mean()),mean_canopy_bank=float(canopy[steady,0].mean()),peak_entry_bank=float(max(abs(canopy[on,0]))),steady_speed_std=float(speed[steady].std()),steady_bank_std=float(canopy[steady,0].std()),mean_course_rate_dps=float(np.rad2deg(rate[steady].mean())),mean_applied_brakes_mm=applied[steady,1:].mean(axis=0).tolist(),exit_peak_airspeed=float(max(speed[exit])),exit_peak_horizontal=float(max(horizontal[exit])),exit_min_sink=float(min(y[exit,9])))
  rows.append(row)
  if len(rows)%5==0:print('Completed',len(rows),'of 46',flush=True)
(OUT/'sweep.json').write_text(json.dumps(rows,indent=2));print('Completed all',len(rows),flush=True)
