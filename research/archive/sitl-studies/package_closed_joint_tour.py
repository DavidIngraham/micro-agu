"""Validate and package native releases as bounded 20 Hz viewer samples."""
import argparse, hashlib, json, subprocess
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
p=argparse.ArgumentParser();p.add_argument('--raw',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--repo',type=Path,required=True);a=p.parse_args()
out=json.loads(a.output.read_text())
out['cases']=[c for c in out['cases'] if c.get('loop','open')=='open']
files=sorted(a.raw.glob('*/result.json'));assert len(files)==12,len(files)
for path in files:
 d=json.loads(path.read_text())
 expected=json.loads((Path(__file__).resolve().parents[3]/'simulation/mission-controller-candidate.json').read_text())['parameters']
 for key,value in expected.items():assert np.isclose(d['parameters'][key],value,rtol=1e-6,atol=1e-8),(key,d['parameters'][key],value)
 paired=d['traces'];assert paired['open'][0]['state']==paired['closed'][0]['state']
 assert d['mode']=='AUTO' and d['settle_s']==100
 for loop,frames in paired.items():
  if loop=='open': continue  # Keep the frozen-command diagnostic in raw research only.
  assert len(frames)==161 and abs(frames[-1]['time_s']-8)<1e-6
  assert all(np.isfinite(f['state']).all() for f in frames)
  k,c=np.array(d['k']),np.array(d['c']);torques=[];performance=[]
  base=frames[0]['state'];course0=np.arctan2(base[8],base[7])
  for f in frames:
   y=np.array(f['state']);gap=y[:3]+R.from_quat(y[3:7]).apply(d['hinge'])-y[13:16]-R.from_quat(y[16:20]).apply([0,0,.85]);assert np.linalg.norm(gap)<1e-4
   e=np.array(f['joint_rotvec_rad']);w=np.array(f['joint_rate_rad_s']);angle=np.linalg.norm(e)
   E=np.array([[0,-e[2],e[1]],[e[2],0,-e[0]],[-e[1],e[0],0]])
   coef=1/12+angle**2/720 if angle<1e-4 else 1/angle**2-(1+np.cos(angle))/(2*angle*np.sin(angle))
   spring=-(np.eye(3)-.5*E+coef*E@E).T@(k*e);damper=-c*w
   assert np.dot(damper,w)<=1e-12
   for start in [3,16]:assert abs(np.linalg.norm(f['state'][start:start+4])-1)<1e-6
   torques.append(dict(spring=spring.tolist(),damper=damper.tolist()))
   course=np.arctan2(f['state'][8],f['state'][7])-course0
   performance.append([base[2]-f['state'][2],float(np.rad2deg(np.arctan2(np.sin(course),np.cos(course))))])
  out['cases'].append(dict(body=d['body'],axis=d['axis'],amplitude_deg=d['amplitude_deg'],profile=d['profile'],loop='held' if loop=='open' else 'closed',hinge=d['hinge'],k=d['k'],c=d['c'],frames=frames,torques=torques,performance=performance,mode=d['mode'] if loop=='closed' else 'frozen AUTO commands',parameters=d['parameters']))
out['auto_comparison_summary'] = []
for path in files:
 d=json.loads(path.read_text());rows={}
 for loop,frames in d['traces'].items():
  states=np.array([f['state'] for f in frames]);course=np.rad2deg(np.unwrap(np.arctan2(states[:,8],states[:,7])))
  rows[loop]=dict(initial_course_deg=float(course[0]),final_course_deg=float(course[-1]),max_course_change_deg=float(max(abs(course-course[0]))),lateral_displacement_m=float(states[-1,1]-states[0,1]),max_altitude_change_m=float(max(abs(states[:,2]-states[0,2]))))
 out['auto_comparison_summary'].append(dict(profile=d['profile'],body=d['body'],axis=d['axis'],metrics=rows))
out['closed_loop_provenance']=dict(mode='AUTO',settle_s=100,amplitude_deg=9,model_commit=subprocess.check_output(['git','-C',str(a.repo),'rev-parse','HEAD'],text=True).strip(),binary_sha256=hashlib.sha256((a.repo/'build/sitl/bin/arduplane').read_bytes()).hexdigest(),description='Native AUTO flies a northbound waypoint leg for 100 seconds before release; paired releases share state, RPM and actuators. Matched open loop freezes the pre-disturbance AUTO commands; closed loop remains in AUTO. Original equilibrium-trim open-loop cases remain separate. Eight-second diagnostic, not a stability or mission acceptance test.')
a.output.write_text(json.dumps(out,separators=(',',':'),allow_nan=False));print('Packaged',len(out['cases']),'cases')
