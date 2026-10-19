import json,subprocess,hashlib
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from scipy.signal import welch,csd
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from microagu_control.sysid.model import configured_simulator,initialize
from microagu_sim.compat import dynamics
from microagu_sim.compat.equilibrium import perturb
OUT=Path(__file__).resolve().parent
BASE=Path('/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/longitudinal-2026-10-09')
class Backend:
 def __init__(self):self.proc=subprocess.Popen([str(OUT/'target/release/power-transition-study')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
 def call(self,r):
  self.proc.stdin.write(json.dumps(r)+chr(10));self.proc.stdin.flush();out=json.loads(self.proc.stdout.readline())
  if 'Err'in out:raise ValueError(out['Err'])
  return np.array(out['Ok'])
def load():
 ss=[];hashes={}
 for flight,date,folder,start,end,cut in [('trout_lake','2026-02-15','2026-02-15 15-38-50',3350,3380,3363.13),('trout_lake','2026-02-15','2026-02-15 15-38-50',3443,3465,3456.45),('hood_river','2026-01-03','2026-01-03 15-41-42',580,601,591.54),('hood_river','2026-01-03','2026-01-03 15-41-42',1040,1067,1057.86)]:
  paths=[BASE/'turn-model-spike'/(date+'-causal-input.npz'),BASE/'turn-model-spike'/(date+'-propulsion-9x45.npz'),BASE/folder/'ESC.json']
  for p in paths:hashes[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
  a=np.load(paths[0]);b=np.load(paths[1]);esc=json.loads(paths[2].read_text());et=np.array([x['TimeUS']/1e6 for x in esc]);er=np.array([x.get('RawRPM',x['RPM']) for x in esc]);i=np.argmin(abs(a['t']-start));n=round((end-start)/.04)+1;sl=slice(i,i+n);t=a['t'][sl]
  assert a['valid'][sl].all() and len(t)==n
  inside=(et>=t[0]-.2)&(et<=t[-1]+.2);assert np.max(np.diff(et[inside]))<.5
  rpm=np.interp(t,et,er)
  ss.append(SimpleNamespace(name=flight+'-'+str(start),flight=flight,time=t,cut=cut,observation=a['Y'][sl],commands=np.column_stack([a['U'][sl,0],a['brake_mm'][sl]]),force_initial=a['force'][i-10:i+1].mean(0),wind=a['wind'][i],voltage=float(b['voltage'][i]),rpm=float(rpm[0]),rpm_history=rpm,initial_state=None,aircraft_profile=None))
 (OUT/'input-hashes.json').write_text(json.dumps(hashes,indent=2));return ss
# log multipliers: rotor inertia, yaw stiffness, yaw damping, pitch damping;
# canopy pitch/yaw, north/east wind corrections, then optional RPM damping coefficient.
def replay(b,s,x,payload=1.,steps=32):
 sim=configured_simulator(s,steps)
 for key,v in zip(['rotor_spin_inertia','joint_k_yaw','joint_c_yaw','joint_c_pitch'],np.exp(x[:4])):sim.parameters[key]*=v
 for key in ['Ixz','Iyz']:sim.parameters[key]*=np.sqrt(payload)
 sim.parameters['Izz']*=payload;sim.geometry['inertia'][2,:]*=np.sqrt(payload);sim.geometry['inertia'][:,2]*=np.sqrt(payload)
 wind=s.wind+np.r_[x[6:8],0.];y=initialize(sim,s,.35+x[4],wind);dx=np.zeros(15);dx[5]=x[5];y=perturb(y,dx,sim.geometry)
 r=dict(parameters=dynamics.params(sim.parameters).tolist(),mass=sim.geometry['mass'],state=y.tolist(),commands=[sim.controls(*u).tolist() for u in s.commands],rpm=s.rpm_history.tolist(),wind=wind.tolist(),substeps=steps,power_damping=float(x[8]) if len(x)>8 else 0.)
 y=b.call(r);return np.rad2deg(Rotation.from_quat(y[:,3:7]).inv().apply(y[:,10:13]))
def features(a,mask):
 z=a[mask];n=min(len(z),200);f,P=welch(z,fs=25,nperseg=n,axis=0);_,C=csd(z[:,2],z[:,1],fs=25,nperseg=n);df=f[1]-f[0]
 bands=[(1.2,1.9),(1.8,2.4),(2.4,3.)];amp=np.array([np.sqrt(P[(f>=lo)&(f<=hi),1:].sum(0)*df) for lo,hi in bands])
 w=(f>=1.2)&(f<=2.4);cross=C[w]/np.sqrt(np.maximum(P[w,1]*P[w,2],1e-12))
 return amp,cross,f,P
def residual(s,a,x):
 obs=np.rad2deg(s.observation[:,3:6]);masks=[(s.time>s.time[0]+2)&(s.time<s.cut-1),(s.time>s.cut+2)]
 parts=[]
 for mask in masks:
  aa,ca,f,P=features(a,mask);bb,cb,_,Q=features(obs,mask)
  scale=np.maximum(np.std(obs[mask],0),3.)
  parts.append(.35*((a[mask][::4]-obs[mask][::4])/scale).ravel()/np.sqrt(len(a[mask][::4])))
  parts.append((np.log(aa+.5)-np.log(bb+.5)).ravel())
  parts.append(.3*np.r_[(ca-cb).real,(ca-cb).imag]/np.sqrt(len(ca)))
  # Envelope shape in three chronological blocks constrains decay/persistence.
  for ix in np.array_split(np.flatnonzero(mask),3):
   parts.append(.3*np.log((np.std(a[ix],0)+1)/(np.std(obs[ix],0)+1)))
 parts.append(.15*x[4:8]/np.array([.1,.1,1.,1.]))
 return np.concatenate(parts)
def metrics(s,a):
 obs=np.rad2deg(s.observation[:,3:6]);out={}
 for label,mask in [('powered',(s.time>s.time[0]+2)&(s.time<s.cut-1)),('off',s.time>s.cut+2)]:
  A,_,f,P=features(a,mask);B,_,_,Q=features(obs,mask);w=(f>=.5)&(f<=4)
  out[label]=dict(rmse=np.sqrt(np.mean((a[mask]-obs[mask])**2,0)).tolist(),model_band=A.tolist(),observed_band=B.tolist(),model_peak=f[w][np.argmax(P[w],0)].tolist(),observed_peak=f[w][np.argmax(Q[w],0)].tolist())
 return out
if __name__=='__main__':
 b=Backend();ss=load();fits=[]
 bounds=(np.r_[np.log([.5,.25,.02,.1]),[-.175,-.175,-2,-2]],np.r_[np.log([1.5,4,2,3]),[.175,.175,2,2]])
 for dynamic in [False,True]:
  for seed in [0,1]:
   x=np.zeros(8);x[2]=np.log(.05) if seed else 0.
   lo,hi=bounds
   if dynamic:x=np.r_[x,1.];lo=np.r_[lo,0.];hi=np.r_[hi,20.]
   calls=[0]
   def fun(x):
    calls[0]+=1
    a=replay(b,ss[0],x)
    return residual(ss[0],a,x)
   result=least_squares(fun,x,bounds=(lo,hi),max_nfev=30,diff_step=1e-3,ftol=2e-4,xtol=2e-4,gtol=2e-4)
   fits.append(dict(dynamic=dynamic,seed=seed,x=result.x.tolist(),cost=float(result.cost),nfev=result.nfev,calls=calls[0],success=bool(result.success),message=result.message))
   (OUT/'fits.json').write_text(json.dumps(fits,indent=2));print(fits[-1],flush=True)
 selected=[min([x for x in fits if x['dynamic']==d],key=lambda x:x['cost']) for d in [False,True]]
 rows=[];traces={}
 for name,x in [('baseline',np.zeros(8))]+[('rpm_damping' if r['dynamic'] else 'constant',np.array(r['x'])) for r in selected]:
  for j,s in enumerate(ss):
   xx=x.copy()
   if j:xx[4:8]=0 # No validation-data fitting or training wind/angle transfer.
   a=replay(b,s,xx);rows.append(dict(name=s.name,variant=name,metrics=metrics(s,a),score=float(np.sum(residual(s,a,xx)**2))))
   traces[s.name+'-'+name]=a
 (OUT/'results.json').write_text(json.dumps(rows,indent=2));np.savez_compressed(OUT/'traces.npz',**traces)
 b.proc.terminate()
