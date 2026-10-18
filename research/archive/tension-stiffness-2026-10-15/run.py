"""Matched replay experiment. Run prepare.py first; see PLAN.md."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1'
from pathlib import Path
import sys,json,hashlib,concurrent.futures
import numpy as np
from scipy.spatial.transform import Rotation as R
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE/'model-rust/python'))
from microagu_sim.compat.profiles import make_simulator
from microagu_sim.compat import dynamics
if not hasattr(dynamics, 'tension_diagnostic') or not Path(dynamics.__file__).is_relative_to(HERE/'model-rust'):
 raise RuntimeError('Build the isolated Rust tension study with prepare.py first')
VARIANTS={'fixed':(0,0),'yaw':(0,1),'roll':(1,0),'both':(1,1),'half':(.5,.5)}
DATA=Path('/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/longitudinal-2026-10-09/turn-model-spike')
FLIGHTS={'hood_river':('2026-01-03',[300,360,480,600,690,720,930,1020]),'trout_lake':('2026-02-15',[600,840,900,1350,1920,2400,2790,2910])}
def make(flight,variant,profile='mission',substeps=32):
 s=make_simulator(flight,profile,substeps=substeps)
 a=dict(np.load(Path(dynamics.__file__).parent/f'equilibrium-trim-{flight}-0.0-0.0.npz'))
 if profile!='mission':
  from microagu_sim.compat.equilibrium import solve
  a=solve(s,a['state'],a['command'])
 ref=dynamics.tension_diagnostic(s,a['state'],a['rpm'],a['command'])[0]
 s.parameters.update(tension_reference=float(ref),tension_roll_fraction=VARIANTS[variant][0],tension_yaw_fraction=VARIANTS[variant][1])
 return s,a

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
 flight,start,alpha,variant,substeps=job;day=FLIGHTS[flight][0]
 a=np.load(DATA/f'{day}-causal-input.npz');b=np.load(DATA/f'{day}-propulsion-9x45.npz')
 assert np.max(abs(a['t']-b['t']))<1e-6
 i=int(np.argmin(abs(a['t']-start)));sl=slice(i,i+751)
 s,_=make(flight,variant,substeps=substeps);y0=init(s,a,b,i,alpha)
 u=np.column_stack([a['U'][sl,0],a['brake_mm'][sl]]);v=float(b['voltage'][i]);wind=a['wind'][i]
 y,r=s.replay(y0,u,voltage=v,rpm=float(b['rpm'][i]),wind=wind)
 row=dict(flight=flight,start=start,alpha=alpha,variant=variant,substeps=substeps,valid_fraction=float(a['valid'][sl].mean()),finite=bool(np.isfinite(y).all()))
 if not row['finite']:return row
 rp=R.from_quat(y[:,3:7]);rates=rp.inv().apply(y[:,10:13]);nav=y[:,7:10]+np.cross(y[:,10:13],rp.apply(s.geometry['imu']))
 euler=rp.as_euler('xyz');truth=a['Y'][sl];err=rates-truth[:,3:6]
 rmse=lambda x:float(np.sqrt(np.mean(x*x)))
 wrap=lambda x:np.arctan2(np.sin(x),np.cos(x))
 metrics={k:rmse(np.rad2deg(err[:,j])) for j,k in enumerate(['roll_rate','pitch_rate','yaw_rate'])}
 metrics.update(bank=rmse(np.rad2deg(wrap(euler[:,0]-truth[:,0]))),course=rmse(np.rad2deg(wrap(np.arctan2(nav[:,1],nav[:,0])-np.arctan2(truth[:,7],truth[:,6])))),climb=rmse(nav[:,2]-truth[:,8]))
 diag=np.array([dynamics.tension_diagnostic(s,y[j],r[j],u[j],v,wind) for j in range(0,len(y),5)])
 row.update(metrics=metrics,load_quantiles=np.quantile(diag[:,0]/diag[:,1],[0,.05,.5,.95,1]).tolist(),scale_quantiles=np.quantile(diag[:,2],[0,.05,.5,.95,1]).tolist(),clipped_fraction=float(np.mean((diag[:,2]<=.25)|(diag[:,2]>=2))),closure_max=float(max(abs(diag[:,6]))),constraint_max=float(max(diag[:,7])),denominator_min=float(min(diag[:,4])))
 (HERE/'traces').mkdir(exist_ok=True)
 np.savez_compressed(HERE/'traces'/f'{flight}-{start}-{alpha}-{variant}-{substeps}.npz',state=y,rpm=r,diag=diag,rates=rates,nav=nav,truth=truth,euler=euler)
 return row

def main():
 jobs=[(f,t,a,v,32) for f,(_,starts) in FLIGHTS.items() for t in starts for a in [.35,.25,.45] for v in VARIANTS]
 rows=[]
 with concurrent.futures.ProcessPoolExecutor(max_workers=4) as pool:
  for row in pool.map(replay,jobs):
   rows.append(row);print(len(rows),row['flight'],row['start'],row['variant'],row.get('metrics'),flush=True)
   (HERE/'flight-results.json').write_text(json.dumps(rows,indent=2)+'\n')
if __name__=='__main__':main()
