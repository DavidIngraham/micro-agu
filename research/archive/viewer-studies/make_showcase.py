#!/usr/bin/env python3
"""Generate a physics-driven reversing-turn showcase; no native autopilot involved."""
import argparse,json,sys,hashlib
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--model-dir',type=Path,required=True)
p.add_argument('--output',type=Path,required=True)
p.add_argument('--turbulence',action='store_true')
a=p.parse_args();sys.path.insert(0,str(a.model_dir))
from microagu_sim.compat.json_backend import Backend
from turbulence import Gusts, SETTINGS
gusts=Gusts()
b=Backend(flight='trout_lake',profile='mission',altitude=100,voltage=22.2)
b.released=True
base=b.trim['command'].copy();dt=.02;records=[]
# Smooth right entry, reversal to left, then release. Throttle supports the turns.
knots=[0,2,5,11,15,21,25,30]
left=[0,0,0,0,20,20,0,0];right=[0,0,22,22,0,0,0,0]
for i in range(1501):
 t=i*dt
 command=base.copy();command[1:]+=np.array([np.interp(t,knots,left),np.interp(t,knots,right)])
 command[0]+= .06*max(np.interp(t,knots,left)/20,np.interp(t,knots,right)/22)
 b.time=t;b.command=command
 wind=gusts.at(t) if a.turbulence else np.zeros(3)
 frame=b.viewer_state()
 if a.turbulence:frame.update(wind_ned_m_s=wind.tolist(),turbulence='moderate synthetic gusts')
 records.append(frame)
 b.state,b.rpm,_=b.sim.step_actuated(b.state,b.rpm,b.actuator,command,voltage=b.voltage,wind=wind,dt=dt)
a.output.parent.mkdir(parents=True,exist_ok=True)
with a.output.open('x') as f:
 for row in records:f.write(json.dumps(row,allow_nan=False)+'\n')
y=np.array([r['state'] for r in records]);angles=Rotation.from_quat(y[:,3:7]).as_euler('xyz',degrees=True)
info=dict(kind='Scripted open-loop actuator demonstration; not a flight log or native AUTO test',
 duration_s=30,flight='trout_lake',profile='mission',mass_kg=b.sim.geometry['mass'],voltage_v=22.2,
 maneuver='Right turn, left reversal, release',command_knots_s=knots,left_increment_mm=left,right_increment_mm=right,
 throttle_increment_max=.06,max_abs_payload_bank_deg=float(abs(angles[:,0]).max()),
 heading_change_range_deg=float(np.ptp(np.unwrap(angles[:,2]*np.pi/180))*180/np.pi),
 altitude_range_m=[float(-y[:,2].max()),float(-y[:,2].min())],
 joint_peak_abs_deg=np.max(abs(np.array([r['joint_rotvec_rad'] for r in records]))*180/np.pi,axis=0).tolist(),
 trace_sha256=hashlib.sha256(a.output.read_bytes()).hexdigest(),turbulence=SETTINGS if a.turbulence else None)
a.output.with_suffix('.json').write_text(json.dumps(info,indent=2)+'\n');print(json.dumps(info,indent=2))
