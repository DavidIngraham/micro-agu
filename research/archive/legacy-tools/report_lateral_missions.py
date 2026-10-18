"""Summarize complete native missions without rewarding filtered feedback."""
import argparse,json,hashlib
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.signal import butter,sosfiltfilt
from pymavlink import mavutil
p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--crosswind-baseline',type=Path);a=p.parse_args()
base=a.root.parent/'longitudinal-2026-10-09/turn-model-spike/longitudinal-final-design/native-gust'
runs={'calm_4hz':a.root/'mission-regression','calm_1hz':a.root/'mission-1hz','gust_4hz':base,'gust_1hz':a.root/'mission-1hz-gust','crosswind_1hz':a.root/'mission-1hz-crosswind'}
if a.crosswind_baseline:runs['crosswind_4hz']=a.crosswind_baseline
results={}
for name,d in runs.items():
 if not (d/'ParagliderAutoMission.csv.json').exists():continue
 if 'PASSED STEP: test.Plane.ParagliderAutoMission' not in (d/'run.log').read_text():continue
 mission=json.loads((d/'ParagliderAutoMission.csv.json').read_text());csv=np.genfromtxt(d/'ParagliderAutoMission.csv',delimiter=',',names=True)
 if (d/'truth.jsonl').exists():
  data=[json.loads(line) for line in (d/'truth.jsonl').open()];t=np.array([x['time_s'] for x in data]);y=np.array([x['state'] for x in data]);del data
 else:
  data=np.load(d/'high-dof-trace.npz')['samples'];t=data[:,0];y=data[:,2:]
 rot=Rotation.from_quat(y[:,3:7]);angles=rot.as_euler('xyz');rates=rot.inv().apply(y[:,10:13]);rate=np.rad2deg((rates[:,1]*np.sin(angles[:,0])+rates[:,2]*np.cos(angles[:,0]))/np.cos(angles[:,1]))
 c=mavutil.mavlink_connection(str(d/'flight.BIN'));params={};pg=[]
 while True:
  m=c.recv_match(type=['PARM','PGYR'])
  if m is None:break
  if m.get_type()=='PARM':params[m.Name]=m.Value
  else:pg.append([m.TimeUS*1e-6,m.Tgt,m.Out])
 pg=np.array(pg);expected=json.loads((a.root/'mission-full-1hz.json').read_text())['parameters'];expected['PG_TURN_FILT']=4 if name.endswith('4hz') else 1
 for k,v in expected.items():assert abs(params[k]-v)<1e-5,(name,k,params[k],v)
 result={'passed':True,'parameters':{k:params[k] for k in expected},'source':str(d),'log_sha256':hashlib.sha256((d/'flight.BIN').read_bytes()).hexdigest(),'loiters':{}}
 result['max_corner_overrun_m']=max(v['next_track_overshoot_m'] or 0 for v in mission['metrics'].values())
 result['max_waypoint_height_error_m']=max(abs(v['altitude_at_closest_m']-mission['route'][int(k)-1][3]) for k,v in mission['metrics'].items() if v['altitude_at_closest_m'] is not None)
 for seq in [3,8]:
  rows=csv[csv['seq']==seq];near=rows[rows['target_distance_m']<60];start=float(near['time_s'][0])+20;end=float(rows['time_s'][-1]);mask=(pg[:,0]>=start)&(pg[:,0]<=end);z=pg[mask];sel=rows[rows['time_s']>=start]
  truth=np.interp(z[:,0],t,rate);dt=np.median(np.diff(z[:,0]));fast=sosfiltfilt(butter(3,1,fs=1/dt,btype='highpass',output='sos'),z[:,2]);radius=abs(mission['loiter_radius_m'][str(seq)])
  result['loiters'][str(seq)]={'truth_rate_rmse_deg_s':float(np.sqrt(np.mean((truth-z[:,1])**2))),'brake_above_1hz_rms':float(np.sqrt(np.mean(fast**2))),'saturation_pct':float(np.mean(abs(z[:,2])>=.99)*100),'radial_rmse_m':float(np.sqrt(np.mean((sel['target_distance_m']-radius)**2)))}
 results[name]=result;print(name,{k:round(v,3) for k,v in result.items() if isinstance(v,float)},result['loiters'])
a.output.write_text(json.dumps(results,indent=2))
