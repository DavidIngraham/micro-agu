# AP_FLAKE8_CLEAN
import os
os.environ['OPENBLAS_NUM_THREADS']='1'
from pathlib import Path
import sys,json,itertools,concurrent.futures
import numpy as np
from scipy.spatial.transform import Rotation as R
HERE=Path(__file__).resolve().parent
from microagu_sim.compat.profiles import make_simulator
from microagu_sim.compat import dynamics,routing
DATA=Path(os.environ.get('MICROAGU_REPLAY_DATA','/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/longitudinal-2026-10-09/turn-model-spike'))
FLIGHTS={'hood_river':('2026-01-03',[300,360,480,600,690,720,930,1020]),'trout_lake':('2026-02-15',[600,840,900,1350,1920,2400,2790,2910])}

def make(flight,variant='legacy',tuning=(1.,1.,1.),height=.06,substeps=32):
 s=make_simulator(flight,substeps=substeps)
 trim=dict(np.load(Path(dynamics.__file__).parent/f'equilibrium-trim-{flight}-0.0-0.0.npz'))
 if variant in ('geometry','routed'):s.parameters['joint_k_yaw']*=.066/.0743
 if variant in ('routing_only','routed'):routing.configure(s,trim['state'],ring_height=height)
 damping,brake,yaw=tuning
 for k in ['joint_c_pitch','joint_c_yaw']:s.parameters[k]*=damping
 for k in ['CLbrake','CDbrake','Cldelta','Cndelta','brake_tilt_rad_per_m']:s.parameters[k]*=brake
 s.parameters['joint_k_yaw']*=yaw
 return s,trim

def init(s,a,b,i,alpha):
 obs=a['Y'][i];rp=R.from_euler('xyz',obs[:3]);wp=rp.apply(obs[3:6]);g=s.geometry
 v=obs[6:9]-np.cross(wp,rp.apply(g['imu']));wind=a['wind'][i];e=v-wind;e/=np.linalg.norm(e)
 axial=rp.inv().apply(v-wind)[0]
 thrust=dynamics.lib.mapped_thrust(float(b['rpm'][i]),float(axial))
 force=g['mass']*a['force'][max(0,i-10):i+1].mean(axis=0)-rp.apply([thrust,0,0])
 down=-(force-e*(force@e));down/=np.linalg.norm(down);span=np.cross(down,e);span/=np.linalg.norm(span)
 rc=R.from_matrix(np.column_stack([np.cos(alpha)*e-np.sin(alpha)*down,span,np.sin(alpha)*e+np.cos(alpha)*down]))
 pr=rp.apply(g['hinge']);cr=rc.apply([0,0,.85])
 return np.r_[np.zeros(3),rp.as_quat(),v,wp,pr-cr,rc.as_quat(),v+np.cross(wp,pr)-np.cross(wp,cr),wp]


def replay(job):
 flight,start,variant,tuning,seconds,alpha,height,substeps=job
 day=FLIGHTS[flight][0];a=np.load(DATA/f'{day}-causal-input.npz');b=np.load(DATA/f'{day}-propulsion-9x45.npz')
 assert np.max(abs(a['t']-b['t']))<1e-6
 i=int(np.argmin(abs(a['t']-start)));sl=slice(i,i+round(seconds/.04)+1)
 s,_=make(flight,variant,tuning,height,substeps);y0=init(s,a,b,i,alpha)
 u=np.column_stack([a['U'][sl,0],a['brake_mm'][sl]])
 row=dict(flight=flight,start=start,variant=variant,tuning=list(tuning),seconds=seconds,alpha=alpha,height=height,substeps=substeps,valid_fraction=float(a['valid'][sl].mean()))
 try:y,r=s.replay(y0,u,voltage=float(b['voltage'][i]),rpm=float(b['rpm'][i]),wind=a['wind'][i])
 except FloatingPointError:return row|dict(finite=False)
 rp=R.from_quat(y[:,3:7]);rates=rp.inv().apply(y[:,10:13]);nav=y[:,7:10]+np.cross(y[:,10:13],rp.apply(s.geometry['imu']));truth=a['Y'][sl]
 rmse=lambda x:float(np.sqrt(np.mean(x*x)))
 wrap=lambda x:np.arctan2(np.sin(x),np.cos(x))
 metrics={k:rmse(np.rad2deg(rates[:,j]-truth[:,j+3])) for j,k in enumerate(['roll_rate','pitch_rate','yaw_rate'])}
 metrics.update(bank=rmse(np.rad2deg(wrap(rp.as_euler('xyz')[:,0]-truth[:,0]))),course=rmse(np.rad2deg(wrap(np.arctan2(nav[:,1],nav[:,0])-np.arctan2(truth[:,7],truth[:,6])))),climb=rmse(nav[:,2]-truth[:,8]))
 return row|dict(finite=bool(np.isfinite(y).all()),metrics=metrics)

def batch(jobs,path):
 rows=[]
 with concurrent.futures.ProcessPoolExecutor(max_workers=4) as pool:
  for row in pool.map(replay,jobs):
   rows.append(row)
   if len(rows)%16==0:print(path,len(rows),'/',len(jobs),flush=True)
 (HERE/path).write_text(json.dumps(rows,indent=2)+'\n')
 return rows

if __name__=='__main__':
 fit=[(f,t) for f,(_,starts) in FLIGHTS.items() for t in starts[::2]]
 candidates=list(itertools.product([.7,1.,1.4],[.85,1.,1.15],[1.,.0743/.066]))
 jobs=[(f,t,v,c,10,.35,.06,32) for v in ['legacy','routed'] for c in candidates for f,t in fit]
 rows=batch(jobs,'fit.json')
 baseline={(r['flight'],r['start']):r['metrics'] for r in rows if r['variant']=='legacy' and r['tuning']==[1.,1.,1.]}
 def score(group):
  if not all(r['finite'] for r in group):return 1e6
  return float(np.mean([sum(w*r['metrics'][k]/max(baseline[r['flight'],r['start']][k],1e-6) for k,w in [('roll_rate',1),('pitch_rate',1),('yaw_rate',1),('bank',.25),('climb',.25)])/3.5 for r in group]))
 ranked={v:sorted([dict(tuning=list(c),score=score([r for r in rows if r['variant']==v and r['tuning']==list(c)])) for c in candidates],key=lambda r:r['score']) for v in ['legacy','routed']}
 (HERE/'selected.json').write_text(json.dumps(ranked,indent=2))
 configs=[('legacy',(1,1,1)),('geometry',(1,1,1)),('routing_only',(1,1,1)),('routed',(1,1,1))]+[(v,tuple(ranked[v][0]['tuning'])) for v in ranked]
 jobs=[(f,t,v,c,30,a,.06,32) for f,(_,starts) in FLIGHTS.items() for t in starts for a in [.25,.35,.45] for v,c in configs]
 batch(jobs,'evaluation.json')
 jobs=[(f,t,'routed',tuple(ranked['routed'][0]['tuning']),30,.35,h,32) for f,(_,starts) in FLIGHTS.items() for t in starts[1::2] for h in [.02,.04,.08]]
 batch(jobs,'height-sensitivity.json')
