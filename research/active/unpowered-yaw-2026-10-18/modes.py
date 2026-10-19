import json,sys
import numpy as np
from pathlib import Path
from microagu_sim import compat
from microagu_sim.compat.profiles import make_simulator
from microagu_sim.compat.equilibrium import perturb,difference
from run import OUT,variants
sys.path.insert(0,str(OUT.parent/'power-off-modes-2026-10-18'))
from trim import solve
rows=[]
for flight in ['hood_river','trout_lake']:
 for label in ['k1.0-c1.0-j1.0','k1.0-c0.05-j1.3','k0.5-c0.05-j0.7']:
  o=variants()[label]
  for th in [0.,.4]:
   sim=make_simulator(flight,profile='as_built',substeps=32)
   sim.parameters['joint_k_yaw']*=o['k'];sim.parameters['joint_c_yaw']*=o['c'];sim.parameters['Izz']*=o['j']
   for key in ['Ixz','Iyz']:sim.parameters[key]*=np.sqrt(o['j'])
   sim.geometry['inertia'][2,:]*=np.sqrt(o['j']);sim.geometry['inertia'][:,2]*=np.sqrt(o['j'])
   seed=np.load(Path(compat.__file__).parent/f'equilibrium-trim-{flight}-0.0-0.0.npz')['state'];fit=solve(sim,seed,th)
   err=float(max(abs(fit['residual'])));row=dict(flight=flight,variant=label,throttle=th,residual=err)
   if err<1e-4:
    y,u,rpm=fit['state'],fit['command'],fit['rpm'];h=.004;base,_=sim.step(y,rpm,u,dt=h);cols=[]
    for k in range(15):
     x=np.eye(15)[k]*1e-5;a,_=sim.step(perturb(y,x,sim.geometry),rpm,u,dt=h);b,_=sim.step(perturb(y,-x,sim.geometry),rpm,u,dt=h)
     cols.append((difference(base,a)-difference(base,b))/2e-5)
    vals,vec=np.linalg.eig(np.column_stack(cols));lam=np.log(vals.astype(complex))/h;row['max_real']=float(max(lam.real));row['modes']=[]
    for j,z in enumerate(lam):
     if z.imag<=.1:continue
     v=vec[9:12,j];mag=abs(v);mag/=np.linalg.norm(mag)
     row['modes'].append(dict(hz=float(z.imag/2/np.pi),decay=float(-z.real),rates=mag.tolist(),phase=float(np.angle(v[1]*v[2].conjugate(),deg=True))))
   rows.append(row)
(OUT/'modes.json').write_text(json.dumps(rows,indent=2))
