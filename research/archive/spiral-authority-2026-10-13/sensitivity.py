"""Diagnostic ablations, not fitted or recommended aircraft configurations."""
from common import *
import numpy as np,json
from scipy.spatial.transform import Rotation
from profiles import make_simulator
from trim import find_trim
trim=find_trim(make_simulator('trout_lake','mission'))
rows=[];dt=.04;t=np.arange(0,40+dt/2,dt)
for label,parameter,scale in [('reference','CDbrake',1),('no_explicit_brake_drag','CDbrake',0),('half_induced_drag','k_drag',.5),('half_brake_lift','CLbrake',.5),('double_brake_lift','CLbrake',2),('tenth_roll_stiffness','joint_k_roll',.1)]:
 sim=make_simulator('trout_lake','mission');sim.parameters[parameter]*=scale
 u=np.tile(trim['command'],(len(t),1));u[t>=4,0]=0
 ramp=np.clip((t-6)/1.5,0,1)*np.clip(1-(t-26)/1.5,0,1)
 u[:,1:]=trim['command'][1:]+ramp[:,None]*(np.array([0,1500*np.pi*55/360*.1])-trim['command'][1:])
 y,r,applied=sim.replay_actuated(trim['state'],u,rpm=trim['rpm'],dt=dt)
 np.savez_compressed(RAW/f'ablation-{label}.npz',t=t,state=y,rpm=r,applied=applied)
 m=(t>=18)&(t<=25);bank=Rotation.from_quat(y[:,16:20]).as_euler('xyz',degrees=True)[:,0];speed=np.linalg.norm(y[:,7:10],axis=1)
 rows.append(dict(label=label,parameter=parameter,scale=scale,airspeed=float(speed[m].mean()),speed_std=float(speed[m].std()),bank_std=float(bank[m].std()),sink=float(y[m,9].mean()),canopy_bank=float(bank[m].mean()),exit_peak_airspeed=float(max(speed[(t>=26)&(t<=36)]))))
 print(rows[-1],flush=True)
(OUT/'sensitivity.json').write_text(json.dumps(rows,indent=2))
