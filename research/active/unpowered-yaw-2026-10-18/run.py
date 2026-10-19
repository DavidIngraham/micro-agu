import json,itertools,concurrent.futures as cf
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.signal import welch,csd
from microagu_sim.compat.equilibrium import perturb
from microagu_control.sysid.model import configured_simulator,initialize
from microagu_control.sysid.data import load_session
OUT=Path(__file__).resolve().parent
BASE=Path('/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/longitudinal-2026-10-09/turn-model-spike')
def segments():
 out=[]
 for flight,date,start,duration in [('trout_lake','2026-02-15',3366,14),('trout_lake','2026-02-15',3459,6),('hood_river','2026-01-03',594,6),('hood_river','2026-01-03',1060,6)]:
  a=np.load(BASE/(date+'-causal-input.npz'));b=np.load(BASE/(date+'-propulsion-9x45.npz'));i=np.argmin(abs(a['t']-start));n=round(duration/.04)+1;sl=slice(i,i+n)
  assert a['valid'][sl].all() and len(a['t'][sl])==n
  out.append(SimpleNamespace(name=flight+'-'+str(start),flight=flight,time=a['t'][sl],observation=a['Y'][sl],commands=np.column_stack([a['U'][sl,0],a['brake_mm'][sl]]),force_initial=a['force'][i-10:i+1].mean(0),wind=a['wind'][i],voltage=float(b['voltage'][i]),rpm=float(b['rpm'][i]),initial_state=None,aircraft_profile=None))
 return out
def variants():
 return {f'k{k}-c{c}-j{j}':dict(k=k,c=c,j=j) for k,c,j in itertools.product([.5,1.,2.,4.],[.05,.2,1.],[.7,1.,1.3])}
def one(args):
 seg,label,o,*extra=args
 nuisance=extra[0] if extra else [0,0,0]
 try:
  sim=configured_simulator(seg,substeps=o.get("substeps",32));sim.parameters['joint_k_yaw']*=o['k'];sim.parameters['joint_c_yaw']*=o['c']
  for key in ['Ixz','Iyz']:sim.parameters[key]*=np.sqrt(o['j'])
  sim.parameters['Izz']*=o['j'];sim.geometry['inertia'][2,:]*=np.sqrt(o['j']);sim.geometry['inertia'][:,2]*=np.sqrt(o['j'])
  wind=seg.wind+np.r_[nuisance[1:3],0.];state=initialize(sim,seg,.35+nuisance[0],wind)
  if len(nuisance)>3:
   x=np.zeros(15);x[5]=nuisance[3];state=perturb(state,x,sim.geometry)
  y,rpm=sim.replay(state,seg.commands,seg.voltage,seg.rpm,wind)
  rates=np.rad2deg(Rotation.from_quat(y[:,3:7]).inv().apply(y[:,10:13]));obs=np.rad2deg(seg.observation[:,3:6]);sl=slice(50,-1)
  f,P=welch(rates[sl],fs=25,nperseg=min(300,len(rates[sl])),axis=0);_,Q=welch(obs[sl],fs=25,nperseg=min(300,len(rates[sl])),axis=0)
  band=(f>=1.8)&(f<=2.4);wide=(f>=.5)&(f<=5);df=f[1]-f[0]
  oldband=(f>=1.2)&(f<=1.9)
  def coupling(a):
   ff,pp=welch(a[sl],fs=25,nperseg=min(300,len(a[sl])),axis=0)
   _,cross=csd(a[sl,2],a[sl,1],fs=25,nperseg=min(300,len(a[sl])))
   ii=np.flatnonzero(oldband)[np.argmax(pp[oldband,2])]
   return dict(band_rms=np.sqrt(pp[oldband].sum(0)*df).tolist(),phase=float(np.angle(cross[ii],deg=True)))
  def spec(a):return dict(band_rms=np.sqrt(a[band].sum(0)*df).tolist(),peak_hz=f[wide][np.argmax(a[wide],axis=0)].tolist())
  return dict(name=seg.name,variant=label,options=o,nuisance=nuisance,rmse=np.sqrt(np.mean((rates[sl]-obs[sl])**2,axis=0)).tolist(),model=spec(P),observed=spec(Q),rpm_mean=float(np.mean(rpm)),powered_band_model=coupling(rates),powered_band_observed=coupling(obs),trace=rates.tolist() if extra and len(extra)>1 else None)
 except Exception as e:return dict(name=seg.name,variant=label,error=str(e))
if __name__=='__main__':
 v=variants();ss=segments();rows=[]
 with cf.ProcessPoolExecutor(max_workers=4) as pool:
  for r in pool.map(one,[(s,k,o) for k,o in v.items() for s in ss]):
   rows.append(r)
   if len(rows)%24==0:print(len(rows),flush=True)
 (OUT/'sweep.json').write_text(json.dumps(rows,indent=2))
