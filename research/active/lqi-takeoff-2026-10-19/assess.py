import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
P=Path(__file__).resolve().parent
def assess(path):
 d=json.loads(path.read_text());s=d['samples'];out={k:d[k] for k in ['spec','complete','failure']}
 if not s:return out
 y=np.array([x['state'] for x in s]);t=np.array([x['time'] for x in s]);elapsed=np.array([x['elapsed'] for x in s]);phase=np.array([x['phase'] for x in s]);mode=np.array([x['mode'] for x in s]);active=np.array([x['active'] for x in s]);stage=np.array([x['takeoffStage'] for x in s]);alt=-y[:,2]
 free=phase==2;climb=free&(mode==13)&(alt<13);att=Rotation.from_quat(y[:,3:7]).as_euler('xyz',degrees=True);rates=Rotation.from_quat(y[:,3:7]).inv().apply(y[:,10:13])*180/np.pi;cmd=np.array([x['command'] for x in s])
 first=lambda mask:float(elapsed[np.flatnonzero(mask)[0]]) if mask.any() else None
 out.update(first_free_s=first(free),first_active_s=first(active),first_active_alt_m=float(alt[np.flatnonzero(active)[0]]) if active.any() else None,height_15_s=first(alt>=15),min_free_height_m=float(min(alt[free])) if free.any() else None,climb_active_fraction=float(np.mean(active[climb])) if climb.any() else None,active_during_takeoff_stage=bool((active&stage).any()),active_before_power_ready=any(x['active'] and not x['powerReady'] for x in s),climb_attitude_peak_deg=np.max(abs(att[climb,:2]),axis=0).tolist() if climb.any() else None,climb_rate_peak_deg_s=np.max(abs(rates[climb]),axis=0).tolist() if climb.any() else None,climb_throttle_mean=float(np.mean(cmd[climb,0])) if climb.any() else None,climb_throttle_max=float(max(cmd[climb,0])) if climb.any() else None,late_active_fraction=float(np.mean(active[t>t[-1]-10])),final_alt_m=float(alt[-1]),gain_file_loaded=any('LQI gain file loaded' in x['text'] for x in d['logs']),ekf_active=any('EKF3 active' in x['text'] for x in d['logs']),suspensions=[x for x in d['logs'] if 'suspended' in x['text']])
 return out
if __name__=='__main__':
 rows=[assess(f) for f in sorted((P/'runs').glob('*.json'))];(P/'metrics.json').write_text(json.dumps(rows,indent=2))
 for r in rows:print(json.dumps(r))
