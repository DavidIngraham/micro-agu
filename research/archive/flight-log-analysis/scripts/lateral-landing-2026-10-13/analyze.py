import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pymavlink import mavutil
from scipy.spatial.transform import Rotation
r=Path('/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/lateral-tuning-2026-10-13')
old=r.parent/'landing-iteration-2026-10-13/heldout-candidate'
rows=[]
fig,ax=plt.subplots(3,1,figsize=(11,9),sharex=True)
for trial in range(4):
 for name,d in [('landing gains',old/f'moderate-{trial:02d}'),('mission lateral gains',r/('landing-mission-lateral' if trial==1 else f'landing-mission-lateral-{trial:02d}'))]+([('mission lateral 1 Hz',r/'landing-1hz-02')] if trial==2 else []):
  if not (d/'flight.BIN').exists():continue
  landing=json.loads((d/'landing.json').read_text())
  c=mavutil.mavlink_connection(str(d/'flight.BIN'));pg=[];flare=None;params={}
  while True:
   m=c.recv_match(type=['PGYR','PGFL','PARM'])
   if m is None:break
   if m.get_type()=='PARM':params[m.Name]=m.Value
   elif m.get_type()=='PGFL' and m.Active and flare is None:flare=m.TimeUS*1e-6
   elif m.get_type()=='PGYR':pg.append([m.TimeUS*1e-6,m.Tgt,m.Act,m.Out,m.FF,m.P,m.I,m.RD,m.DFF])
  a=np.array(pg);a=a[(a[:,0]>=flare-20)&(a[:,0]<flare)]
  row=dict(trial=trial,controller=name,miss_m=landing['radial_error_m'],sink_m_s=landing['contact']['velocity'][2],rate_rmse_deg_s=float(np.sqrt(np.mean((a[:,1]-a[:,2])**2))),saturation_fraction=float(np.mean(abs(a[:,3])>=.98)),command_travel_per_s=float(np.abs(np.diff(a[:,3])).sum()/(a[-1,0]-a[0,0])),parameters={k:v for k,v in params.items() if k.startswith(('PG_TURN','TECS_'))})
  frames=json.loads((d/'viewer.json').read_text())
  vf=[f for f in frames if flare-20<=f['time_s']<flare]
  states=np.array([f['state'] for f in vf]);times=np.array([f['time_s'] for f in vf])
  rot=Rotation.from_quat(states[:,3:7]);angles=rot.as_euler('xyz');rates=rot.inv().apply(states[:,10:13])
  actual=np.rad2deg((rates[:,1]*np.sin(angles[:,0])+rates[:,2]*np.cos(angles[:,0]))/np.cos(angles[:,1]))
  row['truth_rate_rmse_deg_s']=float(np.sqrt(np.mean((np.interp(times,a[:,0],a[:,1])-actual)**2)))
  rows.append(row)
  if trial==1:
   t=a[:,0]-flare
   ax[0].plot(t,a[:,2],label=name)
   ax[1].plot(t,a[:,3],label=name)
   ax[2].plot(t,a[:,7],label=name)
   ax[0].plot(t,a[:,1],alpha=.5,ls='--',label=name+' target')
for a,label in zip(ax,['Heading rate (deg/s)','Differential brake command','Roll damping contribution']):
 a.set_ylabel(label);a.grid(alpha=.25);a.legend(fontsize=8)
ax[-1].set_xlabel('Seconds before native flare entry')
fig.suptitle('Matched landing trial 1: only lateral configuration changed')
fig.tight_layout();fig.savefig(r/'landing-lateral-comparison.png',dpi=150)
(r/'screen-results.json').write_text(json.dumps(rows,indent=2))
for row in rows:print({k:round(v,3) if isinstance(v,float) else v for k,v in row.items() if k!='parameters'})
