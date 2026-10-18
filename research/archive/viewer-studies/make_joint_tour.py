"""Generate bounded, physics-driven joint-release demonstrations for the viewer."""
import argparse,json,sys,hashlib,subprocess
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
p=argparse.ArgumentParser();p.add_argument('--model-dir',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--raw',type=Path,required=True);p.add_argument('--body',choices=['canopy','payload','both'],default='both');a=p.parse_args()
sys.path.insert(0,str(a.model_dir))
from microagu_sim.compat.json_backend import Backend
out={'kind':'Open-loop relative deflection releases; fixed trim actuators; no flight controller','amplitudes_deg':[3,6,9,12,15],'cases':[]}
if a.body!='both' and a.output.exists():
 out=json.loads(a.output.read_text());out['cases']=[c for c in out['cases'] if c.get('body','canopy')!=a.body]
for body in (['canopy','payload'] if a.body=='both' else [a.body]):
 for amplitude in out['amplitudes_deg']:
  for profile in ['mission','dynamic_roll']:
   for axis,name in enumerate(['roll','pitch','yaw']):
    b=Backend(flight='trout_lake',profile=profile,altitude=100,voltage=22.2);b.released=True
    y=b.state.copy();rp=R.from_quat(y[3:7]);rc=R.from_quat(y[16:20]);hp=np.array(b.sim.geometry['hinge']);hc=np.array([0,0,.85])
    joint=y[:3]+rp.apply(hp);jointv=y[7:10]+np.cross(y[10:13],rp.apply(hp))
    delta=np.zeros(3);delta[axis]=np.deg2rad(amplitude)
    if body=='canopy':
     rc=rp*R.from_rotvec(delta)*(rp.inv()*rc)
     y[16:20]=rc.as_quat();y[13:16]=joint-rc.apply(hc);y[20:23]=jointv-np.cross(y[23:26],rc.apply(hc))
    else:
     rp=rp*R.from_rotvec(delta)
     y[3:7]=rp.as_quat();y[:3]=joint-rp.apply(hp);y[7:10]=jointv-np.cross(y[10:13],rp.apply(hp))
    b.state=y
    frames=[];torques=[];max_constraint=0
    par=b.sim.parameters;k=np.array([par['joint_k_'+z] for z in ['roll','pitch','yaw']]);c=np.array([par['joint_c_'+z] for z in ['roll','pitch','yaw']])
    for i in range(801):
     b.time=i*.01;f=b.viewer_state();e=np.array(f['joint_rotvec_rad']);w=np.array(f['joint_rate_rad_s']);angle=np.linalg.norm(e)
     E=np.array([[0,-e[2],e[1]],[e[2],0,-e[0]],[-e[1],e[0],0]])
     coef=1/12+angle**2/720 if angle<1e-4 else 1/angle**2-(1+np.cos(angle))/(2*angle*np.sin(angle))
     spring=-(np.eye(3)-.5*E+coef*E@E).T@(k*e);damper=-c*w
     assert np.dot(damper,w)<=1e-12
     gap=b.state[:3]+R.from_quat(b.state[3:7]).apply(hp)-b.state[13:16]-R.from_quat(b.state[16:20]).apply(hc)
     max_constraint=max(max_constraint,float(np.linalg.norm(gap)))
     frames.append(f);torques.append({'spring':spring.tolist(),'damper':damper.tolist()})
     if i<800:b.state,b.rpm,_=b.sim.step_actuated(b.state,b.rpm,b.actuator,b.command,voltage=22.2,dt=.01)
    assert max_constraint<1e-4,max_constraint
    item={'body':body,'amplitude_deg':amplitude,'profile':profile,'axis':name,'hinge':hp.tolist(),'k':k.tolist(),'c':c.tolist(),'max_constraint_m':max_constraint,'frames':frames,'torques':torques}
    a.raw.mkdir(parents=True,exist_ok=True);(a.raw/f'{body}-{profile}-{name}-{amplitude}deg.json').write_text(json.dumps(item))
    item['frames']=frames[::5];item['torques']=torques[::5];out['cases'].append(item)
    print(body,profile,name,amplitude,'constraint',max_constraint)
out['model_commit']=subprocess.check_output(['git','-C',str(a.model_dir),'rev-parse','HEAD'],text=True).strip()
a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(out,separators=(',',':'),allow_nan=False))
