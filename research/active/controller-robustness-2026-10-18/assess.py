import json,gzip
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from microagu_control.pipeline.assess import power_table,shaft_power
P=Path(__file__).resolve().parent;ROOT=P.parents[2]
def rms(a):return float(np.sqrt(np.mean(np.asarray(a)**2)))
def assess(path):
 d=json.loads(path.read_text());spec=d['spec'];ss=d['samples'];r=dict(id=spec['id'],scheme=spec['scheme'],case=spec['id'].rsplit('-',1)[0],complete=d['complete'],failure=d['failure'],spec=spec)
 if len(ss)<30:return r
 t=np.array([s['time'] for s in ss]);y=np.array([s['state'] for s in ss]);br=np.array([s['brakes'] for s in ss]);cmd=np.array([s['command'] for s in ss]);rp=Rotation.from_quat(y[:,3:7]);rate=rp.inv().apply(y[:,10:13])*180/np.pi;relative=(rp.inv()*Rotation.from_quat(y[:,16:20])).as_rotvec()*180/np.pi;att=rp.as_euler('xyz',degrees=True);mask=(t>=20)&(t<=45);late=t>=65
 if not mask.any():return r
 hz=spec['experiment']['hz'];X=np.column_stack([np.sin(2*np.pi*hz*t[mask]),np.cos(2*np.pi*hz*t[mask]),np.ones(sum(mask))]);c=np.linalg.lstsq(X,rate[mask],rcond=None)[0];harm=np.abs(c[0]+1j*c[1])/np.sqrt(2)
 target=np.array([s['targetAlt'] for s in ss]);alt=-y[:,2];dt=np.diff(t);bspeed=np.diff(br,axis=0)/dt[:,None]
 watts=None;rpm=np.array([s['rpm'] for s in ss]);axial=rp.inv().apply(y[:,7:10])[:,0]
 table=power_table(ROOT/'simulation/crates/sim-core/src/propeller_data.rs');valid=(rpm[mask]>=table[0][0])&(rpm[mask]<table[0][-1]);watts=np.zeros(sum(mask));watts[valid]=shaft_power(rpm[mask][valid],axial[mask][valid],*table)
 desired=np.zeros(len(t))
 if spec.get('maneuver'):desired[(t>=20)&(t<35)]=20;desired[(t>=35)&(t<50)]=-20
 r.update(duration=float(t[-1]),harmonic_rms=harm.tolist(),rate_rms=[rms(rate[mask,k]) for k in range(3)],rate_peak=np.max(abs(rate),axis=0).tolist(),altitude_error_rms=rms(alt[mask]-target[mask]),yaw_tracking_rms=rms(rate[mask,2]-desired[mask]),late_altitude_error_rms=rms(alt[late]-target[late]) if late.any() else None,late_rate_rms=[rms(rate[late,k]-np.mean(rate[late,k])) for k in range(3)] if late.any() else None,late_relative_pitch_rms=rms(relative[late,1]-np.mean(relative[late,1])) if late.any() else None,attitude_peak=np.max(abs(att[:,:2]),axis=0).tolist(),brake_upper_fraction=float(np.mean(np.any(br[mask]>np.array([62.4,72.])*0.99,axis=1))),brake_rate_limit_fraction=float(np.mean(np.any(abs(bspeed)>220,axis=1))),brake_travel_mm_per_s=float(np.sum(abs(np.diff(br[mask],axis=0)))/(t[mask][-1]-t[mask][0])),common_brake_mm=float(np.mean(np.min(br[mask],axis=1))),shaft_W=float(np.mean(watts)) if watts is not None and valid.all() else None,power_map_outside_fraction=float(np.mean(~valid)),modal_peak=float(max(abs(np.array([s['modal'][1] for s in ss])))),estimated_attitude_peak_deg=np.max(abs(np.array([[s['roll'],s['pitch']] for s in ss])*180/np.pi),axis=0).tolist(),attitude_estimation_error_peak_deg=np.max(abs((np.array([[s['roll'],s['pitch']] for s in ss])*180/np.pi-att[:,:2]+180)%360-180),axis=0).tolist(),gain_file_loaded=any('LQI gain file loaded'in x for x in d['logs']),ekf_active=any('EKF3 active'in x for x in d['logs']))
 return r
if __name__=='__main__':
 rows=[assess(f) for f in sorted((P/'runs').glob('*.json'))];(P/'metrics.json').write_text(json.dumps(rows,indent=2));print('runs',len(rows),'complete',sum(r['complete'] for r in rows));print('failures',[(r['id'],r['failure']) for r in rows if not r['complete']])
 for r in rows:
  if r.get('harmonic_rms'):print(r['id'],'harmonic',np.round(r['harmonic_rms'][1:],1),'alt',round(r['altitude_error_rms'],2),'late',np.round(r['late_rate_rms'],1) if r['late_rate_rms'] else None,'power',round(r['shaft_W'],1) if r['shaft_W'] else None)
