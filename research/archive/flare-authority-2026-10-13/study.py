"""Bounded flare authority screen. AI-assisted research, no production changes.

Conditions are specified at flare onset, rather than claiming a navigation policy
can deliver every combination. Native SITL separately tests touchdown accuracy.
"""
import sys,json,itertools
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from scipy.spatial.transform import Rotation as R
MODEL=Path('/home/david/repos/ardupilot-high-dof/Tools/simulation/paraglider')
sys.path.insert(0,str(MODEL))
from profiles import make_simulator
from actuator import BrakeActuator
OUT=Path(__file__).parent
RAW=Path('/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/flare-authority-2026-10-13')
SOURCE=RAW.parent/'spiral-authority-2026-10-13'
LIMIT=np.array([1300,1500])*np.pi*55/360*.1

def entry(name):
 path='mission-baseline-0-1.5-1.5.npz' if name=='glide' else 'mission-right-1-1.5-0.15.npz'
 a=np.load(SOURCE/path);i=np.argmin(abs(a['t']-(25 if name=='glide' else 27)))
 y=a['state'][i].copy();yaw=np.arctan2(y[8],y[7]);rot=R.from_euler('z',-yaw)
 origin=y[:3].copy()
 for k in [0,13]:
  y[k:k+3]=rot.apply(y[k:k+3]-origin)
  y[k+3:k+7]=(rot*R.from_quat(y[k+3:k+7])).as_quat()
  y[k+7:k+10]=rot.apply(y[k+7:k+10]);y[k+10:k+13]=rot.apply(y[k+10:k+13])
 return y,float(a['rpm'][i]),a['applied'][i,1:].copy()

def run(cfg):
 name=cfg['name'];dt=cfg.get('dt',.02);sim=make_simulator('trout_lake','mission')
 sim.parameters['CLbrake']*=cfg.get('lift',1);sim.parameters['CDbrake']*=cfg.get('drag',1)
 y,rpm,brake=entry(cfg['entry']);y[[2,15]]-=cfg['height'];act=BrakeActuator(position_mm=brake)
 initial=y.copy();states=[y.copy()];times=[0.];brakes=[brake.copy()];winds=[np.zeros(3)]
 contact=None
 for i in range(int(15/dt)):
  time=i*dt;pct=min(cfg['pull'],cfg['rate']*time)
  # Same normalized common-brake floor as firmware, with initial trim held.
  command=np.r_[0,np.minimum(LIMIT,brake+LIMIT*pct/100)]
  phase=cfg.get('phase',0);amp=cfg.get('gust',0);omega=2*np.pi/12
  wind=amp*np.array([np.sin(omega*time+phase),np.sin(.73*omega*time+phase+1),.25*np.sin(1.31*omega*time+phase+2)])
  yn,rpm,applied=sim.step_actuated(y,rpm,act,command,wind=wind,dt=dt)
  if y[2]<0<=yn[2]:
   fraction=-y[2]/(yn[2]-y[2]);contact=y+fraction*(yn-y);states.append(contact);times.append(time+fraction*dt);brakes.append(act.position.copy());winds.append(wind);break
  y=yn;states.append(y.copy());times.append(time+dt);brakes.append(act.position.copy());winds.append(wind)
 a=np.array(states);t=np.array(times);angles=R.from_quat(a[:,3:7]).as_euler('xyz',degrees=True)
 result=cfg|dict(contact=contact is not None,initial_airspeed=float(np.linalg.norm(initial[7:10])),initial_bank=float(R.from_quat(initial[3:7]).as_euler('xyz',degrees=True)[0]),duration=float(t[-1]),rise_above_trigger=float(max(0,-a[:,2].min()-cfg['height'])),peak_upward_speed=float(max(0,-a[:,9].min())),max_payload_pitch=float(angles[:,1].max()))
 if contact is not None:result.update(horizontal=float(np.linalg.norm(contact[7:9])),sink=float(contact[9]),travel_north=float(contact[0]),travel_east=float(contact[1]),brakes_mm=brakes[-1].tolist(),touchdown_pitch=float(angles[-1,1]))
 np.savez_compressed(RAW/f'{name}.npz',t=t,state=a,brake=np.array(brakes),wind=np.array(winds))
 return result

if __name__=='__main__':
 RAW.mkdir(exist_ok=True)
 if len(sys.argv)>1:configs=json.loads(Path(sys.argv[1]).read_text());label=Path(sys.argv[1]).stem
 else:
  configs=[dict(name=f'{e}-h{h:g}-p{p}-r{r}',entry=e,height=h,pull=p,rate=r) for e,h,p,r in itertools.product(['glide','release'],[.5,.8,1.1,1.5,2.,3.],[60,80,95,100],[30,100,300])];label='screen'
 results=[]
 with ProcessPoolExecutor(max_workers=4) as pool:
  for result in pool.map(run,configs):
   results.append(result)
   if len(results)%24==0:print('Completed',len(results),'/',len(configs),flush=True)
 (OUT/f'{label}-results.json').write_text(json.dumps(results,indent=2))
 print('Finished',len(results),flush=True)
