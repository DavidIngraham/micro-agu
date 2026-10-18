import os

import json,hashlib,numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation
from pymavlink import mavutil
r=Path(os.environ.get('PG_RESEARCH_ROOT', '/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/l1-regression-2026-10-13'))
old=r.parent/'lateral-tuning-2026-10-13'
runs={'standard-calm-8':old/'mission-1hz','standard-crosswind-8':old/'mission-1hz-crosswind','standard-gust-8':old/'mission-1hz-gust'}
runs.update({c['name']:r/c['name'] for c in json.loads((r/'plan.json').read_text())['mission_cases']})
expected=json.loads((r/'standard-calm-12.json').read_text())['parameters'];out={}
cached=json.loads((r/'mission-results.json').read_text()) if (r/'mission-results.json').exists() else {}
for name,d in runs.items():
 if name in cached:
  out[name]=cached[name];continue
 if not (d/'ParagliderAutoMission.csv.json').exists():continue
 if d.parent==r and not (d/'completed.json').exists():continue
 report=json.loads((d/'ParagliderAutoMission.csv.json').read_text());csv=np.genfromtxt(d/'ParagliderAutoMission.csv',delimiter=',',names=True)
 log=mavutil.mavlink_connection(str(d/'flight.BIN'));params={};pg=[]
 while True:
  m=log.recv_match(type=['PARM','PGYR'])
  if m is None:break
  if m.get_type()=='PARM':params[m.Name]=m.Value
  else:pg.append([m.TimeUS/1e6,m.Tgt,m.Out])
 expected_run=dict(expected,NAVL1_PERIOD=int(name.split('-')[-1]))
 if name.startswith('tight'):expected_run['WP_LOITER_RAD']=20
 if name.startswith('wide'):expected_run['WP_LOITER_RAD']=45
 assert all(abs(params[k]-v)<1e-5 for k,v in expected_run.items()),name
 if (d/'truth.jsonl').exists():
  data=[json.loads(line) for line in (d/'truth.jsonl').open()]
  t=np.array([f['time_s'] for f in data]);s=np.array([f['state'] for f in data]);del data
 else:
  data=np.load(d/'high-dof-trace.npz')['samples'];t=data[:,0];s=data[:,2:]
 rot=Rotation.from_quat(s[:,3:7]);angles=rot.as_euler('xyz');v=rot.inv().apply(s[:,10:13]);rate=np.rad2deg((v[:,1]*np.sin(angles[:,0])+v[:,2]*np.cos(angles[:,0]))/np.cos(angles[:,1]));pg=np.array(pg)
 res={'passed':'PASSED STEP: test.Plane.ParagliderAutoMission' in (d/'run.log').read_text(),'parameters':expected_run,'source':str(d),'loiters':{},'legs':{},'log_sha256':hashlib.sha256((d/'flight.BIN').read_bytes()).hexdigest()}
 res['max_corner_overrun_m']=max((v['next_track_overshoot_m'] or 0) for v in report['metrics'].values())
 res['max_waypoint_height_error_m']=max(abs(v['altitude_at_closest_m']-report['route'][int(k)-1][3]) for k,v in report['metrics'].items() if v['altitude_at_closest_m'] is not None and int(k) not in [3,8,12])
 for seq in range(1,12):
  rows=csv[csv['seq']==seq]
  if len(rows)<2:continue
  if seq in [3,8]:
   radius=abs(report['loiter_radius_m'][str(seq)]);near=rows[rows['target_distance_m']<2*radius]
   if not len(near):continue
   start=near['time_s'][0]+20
  else:start=rows['time_s'][0]+20
  z=pg[(pg[:,0]>=start)&(pg[:,0]<=rows['time_s'][-1])];sel=rows[rows['time_s']>=start]
  if len(z)<10 or len(sel)<2:continue
  truth=np.interp(z[:,0],t,rate)
  m={'truth_rate_rmse_deg_s':float(np.sqrt(np.mean((truth-z[:,1])**2))),'command_travel_per_s':float(abs(np.diff(z[:,2])).sum()/(z[-1,0]-z[0,0])),'saturation_pct':float(np.mean(abs(z[:,2])>=.99)*100)}
  if seq in [3,8]:
   m['radial_rmse_m']=float(np.sqrt(np.mean((sel['target_distance_m']-radius)**2)));res['loiters'][seq]=m
  else:
   m['xtrack_rmse_m']=float(np.sqrt(np.mean(sel['nav_cross_track_error_m']**2)));res['legs'][seq]=m
 out[name]=res
 print(name,res['passed'],'corner',round(res['max_corner_overrun_m'],2),'height',round(res['max_waypoint_height_error_m'],2),'loiters',[(k,round(v['radial_rmse_m'],2),round(v['truth_rate_rmse_deg_s'],2)) for k,v in res['loiters'].items()])
(r/'mission-results.json').write_text(json.dumps(out,indent=2))
