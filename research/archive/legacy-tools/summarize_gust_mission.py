"""Summarize complete native missions and settled portions of both loiters."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from pymavlink import mavutil
p=argparse.ArgumentParser();p.add_argument('run',type=Path);a=p.parse_args();run=a.run
mission=json.loads((run/'ParagliderAutoMission.csv.json').read_text())
frames=[json.loads(s) for s in (run/'truth.jsonl').read_text().splitlines()]
t=np.array([f['time_s'] for f in frames]);state=np.array([f['state'] for f in frames]);angles=Rotation.from_quat(state[:,3:7]).as_euler('xyz',degrees=True);throttle=np.array([f['throttle'] for f in frames])
q=Rotation.from_quat(state[:,3:7]).inv().apply(state[:,10:13])[:,1]*180/np.pi
csv=np.genfromtxt(run/'ParagliderAutoMission.csv',delimiter=',',names=True)
params={};log=mavutil.mavlink_connection(str(run/'flight.BIN'))
while True:
    msg=log.recv_match(type='PARM')
    if msg is None:break
    if msg.Name.startswith(('TECS_','PG_TURN_','FWD_BAT_')):params[msg.Name]=float(msg.Value)
log.close()
result=dict(truth_sha256=hashlib.sha256((run/'truth.jsonl').read_bytes()).hexdigest(),parameters=params,loiters={})
for seq in (3,8):
    rows=csv[csv['seq']==seq];start=rows[rows['target_distance_m']<60]['time_s'][0]+20;end=rows['time_s'][-1];sel=(t>=start)&(t<=end);target=mission['route'][seq-1][3]
    result['loiters'][str(seq)]=dict(start_s=float(start),end_s=float(end),duration_s=float(end-start),pitch_std_deg=float(angles[sel,1].std()),pitch_rate_rms_deg_s=float(np.sqrt(np.mean(q[sel]**2))),climb_rms_m_s=float(np.sqrt(np.mean(state[sel,9]**2))),height_rms_m=float(np.sqrt(np.mean((-state[sel,2]-target)**2))),throttle_std_pct=float(throttle[sel].std()*100),throttle_min_pct=float(throttle[sel].min()*100),throttle_max_pct=float(throttle[sel].max()*100))
result['max_waypoint_height_error_m']=max(abs(v['altitude_at_closest_m']-mission['route'][int(k)-1][3]) for k,v in mission['metrics'].items())
result['max_corner_overshoot_m']=max(v['next_track_overshoot_m'] or 0 for v in mission['metrics'].values())
result['backend']=json.loads((run/'high-dof-backend.json').read_text())
result['backend'].pop('state',None)
(run/'gust-metrics.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k!='parameters'},indent=2))
