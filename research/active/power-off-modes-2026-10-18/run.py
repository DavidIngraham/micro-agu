import json,os
from pathlib import Path
import numpy as np
from microagu_sim import compat
from microagu_sim.compat.profiles import make_simulator
from microagu_sim.compat.equilibrium import perturb,difference
from trim import solve
OUT=Path(__file__).resolve().parent
rows=[]
common=float(os.environ.get("COMMON","12"))
for flight in ['hood_river','trout_lake']:
 for gyro in [True,False]:
  seed=np.load(Path(compat.__file__).parent/f'equilibrium-trim-{flight}-0.0-0.0.npz')['state']
  for throttle in [0.4,0.3,0.2,0.1,0.0]:
   sim=make_simulator(flight,profile='as_built',substeps=32)
   if not gyro:sim.parameters['rotor_spin_inertia']=0.
   fit=solve(sim,seed,throttle,common)
   y,u,rpm=fit['state'],fit['command'],fit['rpm']
   err=float(max(abs(fit['residual'])))
   if err<1e-4:seed=y
   for h in [.004,.002]:
    row=dict(flight=flight,common_mm=common,gyro=gyro,throttle=throttle,dt=h,trim_residual=err,rpm=float(rpm),velocity=y[7:10].tolist(),command=u.tolist())
    if err<1e-4:
     base,_=sim.step(y,rpm,u,dt=h);cols=[]
     for k in range(15):
      x=np.eye(15)[k]*1e-5
      a,_=sim.step(perturb(y,x,sim.geometry),rpm,u,dt=h)
      b,_=sim.step(perturb(y,-x,sim.geometry),rpm,u,dt=h)
      cols.append((difference(base,a)-difference(base,b))/2e-5)
     jac=np.column_stack(cols)
     val,vec=np.linalg.eig(jac);lam=np.log(val.astype(complex))/h
     row['max_real']=float(max(lam.real));row['modes']=[]
     for j,v in enumerate(lam):
      if v.imag<.1:continue
      z=vec[9:12,j];mag=abs(z);mag/=max(np.linalg.norm(mag),1e-12)
      row['modes'].append(dict(hz=float(v.imag/2/np.pi),decay=float(-v.real),rates=mag.tolist(),phase=float(np.angle(z[1]*z[2].conjugate(),deg=True))))
     up=u.copy();um=u.copy();up[2]+=.005;up[1]-=.005;um[2]-=.005;um[1]+=.005
     ap,_=sim.step(y,rpm,up,dt=h);am,_=sim.step(y,rpm,um,dt=h)
     b=(difference(base,ap)-difference(base,am))/.02
     freq=np.linspace(.5,5,451)
     resp=np.array([np.linalg.solve(np.exp(2j*np.pi*f*h)*np.eye(15)-jac,b)[9:12] for f in freq])
     row['brake_response']={'hz':freq.tolist(),'magnitude':(abs(resp)*180/np.pi).tolist(),'peak_hz':freq[np.argmax(abs(resp),axis=0)].tolist()}
    rows.append(row)
   print(flight,gyro,throttle,err,flush=True)
(OUT/('results.json' if common==12 else f'results-{common:g}mm.json')).write_text(json.dumps(rows,indent=2))
