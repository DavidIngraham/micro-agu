import os

import json,numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation
from pymavlink import mavutil
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
r=Path(os.environ.get('PG_RESEARCH_ROOT', '/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/landing-guidance-2026-10-13'))
dirs=[r.parent/'landing-refinement-2026-10-13'/name for name in ['preview-calm','preview-1hz','landing-1hz-06','landing-1hz-02']]+sorted(r.glob('calm-*'))+sorted(r.glob('check-*'))
fig,axs=plt.subplots(4,1,figsize=(11,11),sharex=True);results={}
for d in dirs:
 if not (d/'completed.json').exists():continue
 if json.loads((d/'completed.json').read_text())['returncode']:continue
 log=mavutil.mavlink_connection(str(d/'flight.BIN'));pg=[];flare=None;params={}
 while True:
  m=log.recv_match(type=['PGYR','PGFL','PARM'])
  if m is None:break
  if m.get_type()=='PARM':params[m.Name]=m.Value
  elif m.get_type()=='PGYR':pg.append([m.TimeUS/1e6,m.Tgt,m.Out])
  elif m.Active and flare is None:flare=m.TimeUS/1e6
 if d.parent==r:
  expected=json.loads((r/f"controller-{int(params['NAVL1_PERIOD'])}.json").read_text())['parameters'] if params['NAVL1_PERIOD']!=8 else json.loads((r.parent/'landing-refinement-2026-10-13/controller-1hz.json').read_text())['parameters']
  assert all(abs(params[k]-v)<1e-5 for k,v in expected.items()),d
  assert 'PASSED STEP: test.Plane.ParagliderLandingStudy' in (d/'run.log').read_text(),d
 frames=[f for f in json.loads((d/'viewer.json').read_text()) if flare-20<f['time_s']<flare]
 t=np.array([f['time_s'] for f in frames]);s=np.array([f['state'] for f in frames]);rot=Rotation.from_quat(s[:,3:7]);angles=rot.as_euler('xyz');v=rot.inv().apply(s[:,10:13]);rate=np.rad2deg((v[:,1]*np.sin(angles[:,0])+v[:,2]*np.cos(angles[:,0]))/np.cos(angles[:,1]))
 pg=np.array(pg);target=np.interp(t,pg[:,0],pg[:,1]);out=np.interp(t,pg[:,0],pg[:,2]);roll=np.rad2deg(angles[:,0]);pitch=np.rad2deg(angles[:,1])
 land=json.loads((d/'landing.json').read_text())
 results[d.name]=dict(period=params['NAVL1_PERIOD'],filter=params['PG_TURN_FILT'],roll_90pct_span=float(np.diff(np.percentile(roll,[5,95]))[0]),pitch_90pct_span=float(np.diff(np.percentile(pitch,[5,95]))[0]),target_rms=float(np.sqrt(np.mean(target**2))),rate_error_rms=float(np.sqrt(np.mean((target-rate)**2))),command_travel_per_s=float(abs(np.diff(out)).sum()/(t[-1]-t[0])),miss_m=land['radial_error_m'],sink_m_s=land['contact']['velocity'][2])
 if d.name in ['preview-calm','calm-12']:
  label='8 s guidance period' if d.name=='preview-calm' else '12 s guidance period'
  for ax,z in zip(axs,[roll,pitch,target,out]):ax.plot(t-flare,z,label=label)
for ax,label in zip(axs,['Payload roll (deg)','Payload pitch (deg)','Target heading rate (deg/s)','Brake command']):
 ax.set_ylabel(label);ax.grid(alpha=.2);ax.legend(fontsize=7)
axs[-1].set_xlabel('Seconds before flare');fig.tight_layout();fig.savefig(r/'guidance-comparison.png',dpi=140)
(r/'results.json').write_text(json.dumps(results,indent=2));print(json.dumps(results,indent=2))
