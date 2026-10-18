import os

import json,numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation
from pymavlink import mavutil
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
r=Path(os.environ.get('PG_RESEARCH_ROOT', '/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/l1-regression-2026-10-13'))
dirs=[r.parent/'landing-refinement-2026-10-13'/f'validation-{i:02d}' for i in range(12)]+sorted(r.glob('landing-*'))+sorted(r.glob('case08-*'))+[r/'calm10']+sorted(r.glob('landing10-*'))
results={}
cached=json.loads((r/'landing-results.json').read_text()) if (r/'landing-results.json').exists() else {}
for d in dirs:
 if d.name in cached:
  results[d.name]=cached[d.name];continue
 if not (d/'completed.json').exists():continue
 if json.loads((d/'completed.json').read_text())['returncode']:continue
 log=mavutil.mavlink_connection(str(d/'flight.BIN'));pg=[];flare=None;params={}
 while True:
  m=log.recv_match(type=['PGYR','PGFL','PARM'])
  if m is None:break
  if m.get_type()=='PARM':params[m.Name]=m.Value
  elif m.get_type()=='PGYR':pg.append([m.TimeUS/1e6,m.Tgt,m.Out])
  elif m.Active and flare is None:flare=m.TimeUS/1e6
 expected=json.loads((r/'standard-calm-12.json').read_text())['parameters']
 expected['NAVL1_PERIOD']=int(d.name.split('-')[1]) if d.name.startswith('case08-') else (10 if d.name=='calm10' or d.name.startswith('landing10-') else (12 if d.parent==r else 8))
 assert all(abs(params[k]-v)<1e-5 for k,v in expected.items()),d
 assert 'PASSED STEP: test.Plane.ParagliderLandingStudy' in (d/'run.log').read_text(),d
 frames=[f for f in json.loads((d/'viewer.json').read_text()) if flare-20<f['time_s']<flare]
 t=np.array([f['time_s'] for f in frames]);s=np.array([f['state'] for f in frames]);rot=Rotation.from_quat(s[:,3:7]);angles=rot.as_euler('xyz');v=rot.inv().apply(s[:,10:13]);rate=np.rad2deg((v[:,1]*np.sin(angles[:,0])+v[:,2]*np.cos(angles[:,0]))/np.cos(angles[:,1]))
 pg=np.array(pg);target=np.interp(t,pg[:,0],pg[:,1]);out=np.interp(t,pg[:,0],pg[:,2]);roll=np.rad2deg(angles[:,0]);pitch=np.rad2deg(angles[:,1])
 land=json.loads((d/'landing.json').read_text())
 results[d.name]=dict(period=params['NAVL1_PERIOD'],filter=params['PG_TURN_FILT'],roll_90pct_span=float(np.diff(np.percentile(roll,[5,95]))[0]),pitch_90pct_span=float(np.diff(np.percentile(pitch,[5,95]))[0]),target_rms=float(np.sqrt(np.mean(target**2))),rate_error_rms=float(np.sqrt(np.mean((target-rate)**2))),command_travel_per_s=float(abs(np.diff(out)).sum()/(t[-1]-t[0])),miss_m=land['radial_error_m'],sink_m_s=land['contact']['velocity'][2])
(r/'landing-results.json').write_text(json.dumps(results,indent=2));print('Landing records:',len(results))
