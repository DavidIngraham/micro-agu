import sys,json
from pathlib import Path
import numpy as np
from profile import OUT,PREV,Backend,load,replay,metrics
from microagu_sim import compat
from microagu_sim.compat.profiles import make_simulator
from microagu_sim.compat.equilibrium import perturb,difference
sys.path.insert(0,str(OUT.parent/'power-off-modes-2026-10-18'))
from trim import solve
chosen=json.loads((OUT/'selection.json').read_text());x=np.array(chosen['selected']['x']);payload=chosen['payload'];b=Backend();checks=[]
for j,s in enumerate(load()):
 xx=x.copy()
 if j:xx[4:8]=0
 a=replay(b,s,xx,payload=payload);fine=replay(b,s,xx,payload=payload,steps=64);e=float(np.max(abs(a-fine)));assert e<.03;checks.append(dict(name=s.name,max_rate_difference=e))
b.proc.terminate();modes=[]
for flight in ['trout_lake','hood_river']:
 for th in [0.,.4]:
  sim=make_simulator(flight,profile='as_built',substeps=32)
  for key,v in zip(['rotor_spin_inertia','joint_k_yaw','joint_c_yaw','joint_c_pitch'],np.exp(x[:4])):sim.parameters[key]*=v
  for key in ['Ixz','Iyz']:sim.parameters[key]*=np.sqrt(payload)
  sim.parameters['Izz']*=payload;sim.geometry['inertia'][2,:]*=np.sqrt(payload);sim.geometry['inertia'][:,2]*=np.sqrt(payload)
  seed=np.load(Path(compat.__file__).parent/f'equilibrium-trim-{flight}-0.0-0.0.npz')['state'];r=solve(sim,seed,th);err=float(max(abs(r['residual'])));row=dict(flight=flight,throttle=th,residual=err)
  assert err<1e-4
  y,u,rpm=r['state'],r['command'],r['rpm'];h=.004;base,_=sim.step(y,rpm,u,dt=h);cols=[]
  for k in range(15):
   dx=np.eye(15)[k]*1e-5;a,_=sim.step(perturb(y,dx,sim.geometry),rpm,u,dt=h);c,_=sim.step(perturb(y,-dx,sim.geometry),rpm,u,dt=h);cols.append((difference(base,a)-difference(base,c))/2e-5)
  val,vec=np.linalg.eig(np.column_stack(cols));lam=np.log(val.astype(complex))/h;row['max_real']=float(max(lam.real));row['modes']=[]
  for k,v in enumerate(lam):
   if v.imag<=.1:continue
   z=vec[9:12,k];row['modes'].append(dict(hz=float(v.imag/2/np.pi),decay=float(-v.real),pitch_yaw_ratio=float(abs(z[1])/max(abs(z[2]),1e-12)),phase=float(np.angle(z[1]*z[2].conjugate(),deg=True))))
  modes.append(row)
(OUT/'checks.json').write_text(json.dumps(dict(resolution=checks,equilibria=modes),indent=2));print('Resolution max',max(x['max_rate_difference'] for x in checks),'maximum growth rate',max(x['max_real'] for x in modes))
