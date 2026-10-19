import json,sys
import numpy as np
from pathlib import Path
from microagu_sim import compat
from microagu_sim.compat.profiles import make_simulator
from microagu_sim.compat.equilibrium import perturb,difference
from study import OUT
sys.path.insert(0,str(OUT.parent/'power-off-modes-2026-10-18'))
from trim import solve
fits=json.loads((OUT/'fits.json').read_text());fit=min([r for r in fits if not r['dynamic']],key=lambda r:r['cost']);rows=[]
for flight in ['trout_lake','hood_river']:
 for th in [0.,.4]:
  sim=make_simulator(flight,profile='as_built',substeps=32)
  for key,v in zip(['rotor_spin_inertia','joint_k_yaw','joint_c_yaw','joint_c_pitch'],np.exp(fit['x'][:4])):sim.parameters[key]*=v
  seed=np.load(Path(compat.__file__).parent/f'equilibrium-trim-{flight}-0.0-0.0.npz')['state'];r=solve(sim,seed,th);err=float(max(abs(r['residual'])));row=dict(flight=flight,throttle=th,residual=err)
  if err<1e-4:
   y,u,rpm=r['state'],r['command'],r['rpm'];h=.004;base,_=sim.step(y,rpm,u,dt=h);cols=[]
   for k in range(15):
    x=np.eye(15)[k]*1e-5;a,_=sim.step(perturb(y,x,sim.geometry),rpm,u,dt=h);b,_=sim.step(perturb(y,-x,sim.geometry),rpm,u,dt=h);cols.append((difference(base,a)-difference(base,b))/2e-5)
   lam=np.log(np.linalg.eigvals(np.column_stack(cols)).astype(complex))/h;row['max_real']=float(max(lam.real));row['modes']=[dict(hz=float(v.imag/2/np.pi),decay=float(-v.real)) for v in lam if v.imag>.1]
  rows.append(row)
(OUT/'stability.json').write_text(json.dumps(rows,indent=2));print(rows)
