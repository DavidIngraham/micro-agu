"""Nonlinear flight-path equilibria with deployed as-built geometry and actuator bounds."""
import importlib.util,json
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
s=importlib.util.spec_from_file_location('design',ROOT/'simulation/tools/design_mimo.py')
d=importlib.util.module_from_spec(s);s.loader.exec_module(d)
cfg=json.loads((HERE/'physics-config.json').read_text())
sim=d.make_simulator('trout_lake','as_built');sim.substeps=16
assert np.allclose(d.compat.dynamics.params(sim.parameters), cfg['parameters'], rtol=0, atol=1e-14), 'Trim must match deployed physics'
state=np.array(cfg['state']);rpm=cfg['rpm'];chart=d.Chart(sim,state,rpm)
records=[]
for voltage in [22.2,20.4]:
 solutions={}
 for rate,speed in [(r,s) for r in [0,25,35] for s in [3.8,4.0,4.2,4.3,4.4]]:
  last=None
  for climb in [0,.5,.75,1.,-.5,-.75,-1.]:
   def advance(x,u):
    y,r=chart.decode(x*d.XS);y,r=sim.step(y,r,np.clip(d.command(u),0,np.r_[1,d.STROKES]),voltage=voltage,dt=d.DT)
    return chart.encode(y,r)/d.XS
   def outputs(x):
    y,_=chart.decode(x*d.XS);roll,pitch=Rotation.from_quat(y[3:7]).as_euler('xyz')[:2]
    rates=chart.payload_rates+x[8:11]*d.XS[8:11]
    return (rates[1]*np.sin(roll)+rates[2]*np.cos(roll))/np.cos(pitch),np.linalg.norm(y[7:10])
   def residual(v):
    x=np.r_[v[:15],0];u=v[15:];r,speed_actual=outputs(x)
    dx=(advance(x,u)-x)/d.DT;dx[15]-=climb
    return np.r_[dx,(r-np.deg2rad(rate))/np.deg2rad(10),(speed_actual-speed)/.5,max(0,u[1]+abs(u[2])-1)*100]
   initial=last if last is not None else np.r_[np.zeros(15),.45,0,.05]
   # FWD voltage compensation raises actual actuator duty below reference voltage.
   ceiling=min(1,.6*22.2/voltage)
   fit=least_squares(residual,initial,bounds=(np.r_[np.full(14,-10),-rpm/1000,0,0,-1],np.r_[np.full(14,10),10,ceiling,1,1]),max_nfev=180,ftol=1e-9,xtol=1e-9,gtol=1e-9,x_scale='jac')
   error=float(max(abs(residual(fit.x))))
   if error>=1e-3:
    candidates=[]
    for path in (HERE/'runs').glob('nominal-lqi-*.json'):
     if path.name.endswith(('-spec.json','-metrics.json')):continue
     flight=json.loads(path.read_text())
     for sample in flight.get('samples',[]):
      if sample['phase']!=('climb' if climb>=0 else 'descend'):continue
      y=np.array(sample['truth']);cost=abs(-y[9]-climb)+abs(np.linalg.norm(y[7:10])-speed)+abs(flight['spec']['turn']-rate)/10
      candidates.append((cost,sample))
    if candidates:
     sample=min(candidates,key=lambda z:z[0])[1]
     x0=chart.encode(np.array(sample['truth']),sample['rpm'])/d.XS
     brakes=np.array(sample['brakes'])/d.STROKES
     u0=np.r_[min(ceiling-1e-6,sample['command'][0]*22.2/voltage),min(brakes),brakes[1]-brakes[0]]
     alternative=least_squares(residual,np.r_[x0[:15],u0],bounds=(np.r_[np.full(14,-10),-rpm/1000,0,0,-1],np.r_[np.full(14,10),10,ceiling,1,1]),max_nfev=400,ftol=1e-10,xtol=1e-10,gtol=1e-10,x_scale='jac')
     if max(abs(residual(alternative.x)))<error:fit=alternative;error=float(max(abs(residual(fit.x))))
   feasible=bool(fit.success and error<1e-3)
   if feasible:last=fit.x
   row=dict(voltage=voltage,turn=rate,climb=climb,speed=speed,feasible=feasible,residual=error,u=fit.x[15:].tolist(),x=fit.x[:15].tolist(),throttle_headroom=float(ceiling-fit.x[15]))
   records.append(row);print(json.dumps(row),flush=True);(HERE/'trims.json').write_text(json.dumps(records,indent=2))
