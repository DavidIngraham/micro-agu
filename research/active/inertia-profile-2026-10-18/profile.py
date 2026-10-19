import sys,json,itertools,concurrent.futures as cf
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
OUT=Path(__file__).resolve().parent
PREV=OUT.parent/'power-transition-fit-2026-10-18'
sys.path.insert(0,str(PREV))
from study import Backend,load,replay,residual,metrics
PAYLOAD=[.85,1.,1.15]
ROTOR=[.75,1.,1.25,1.5]
def cell(pair):
 payload,rotor=pair;b=Backend();s=load()[0];fits=[]
 lo=np.r_[np.log([.25,.02,.1]),[-.175,-.175,-2,-2]];hi=np.r_[np.log([4.,2.,3.]),[.175,.175,2,2]]
 for seed in [0,1]:
  z=np.zeros(7)
  if seed:z[:3]=np.log([.83,.05,.23])
  calls=[0]
  def fun(z):
   calls[0]+=1;x=np.r_[np.log(rotor),z];return residual(s,replay(b,s,x,payload=payload),x)
  fit=least_squares(fun,z,bounds=(lo,hi),max_nfev=35,diff_step=1e-3,ftol=2e-4,xtol=2e-4,gtol=2e-4)
  x=np.r_[np.log(rotor),fit.x];a=replay(b,s,x,payload=payload)
  fits.append(dict(seed=seed,x=x.tolist(),cost=float(fit.cost),calls=calls[0],nfev=fit.nfev,success=bool(fit.success),message=fit.message,metrics=metrics(s,a)))
 b.proc.terminate();return dict(payload=payload,rotor=rotor,fits=fits,selected=min(fits,key=lambda f:f['cost']))
if __name__=='__main__':
 (OUT/'protocol.json').write_text(json.dumps(dict(payload_yaw_inertia_multipliers=PAYLOAD,rotor_spin_inertia_multipliers=ROTOR,selection='Lowest first-transition objective; identical previous objective. Second transition and Hood River excluded from all fitting/selection.',limits='Engineering sensitivity grid, not measured uncertainty bounds; rotor and payload inertia are fixed within each cell; no claim of global optimum.'),indent=2))
 rows=[]
 with cf.ProcessPoolExecutor(max_workers=4) as pool:
  futures=[pool.submit(cell,pair) for pair in itertools.product(PAYLOAD,ROTOR)]
  for f in cf.as_completed(futures):
   row=f.result();rows.append(row);(OUT/'profile.json').write_text(json.dumps(rows,indent=2));print(row['payload'],row['rotor'],row['selected']['cost'],flush=True)
 # Freeze choice before any transfer replay.
 chosen=min(rows,key=lambda r:r['selected']['cost']);(OUT/'selection.json').write_text(json.dumps(chosen,indent=2))
 b=Backend();ss=load();validation=[];traces={}
 for row in rows:
  for j,s in enumerate(ss):
   x=np.array(row['selected']['x']);
   if j:x[4:8]=0.
   a=replay(b,s,x,payload=row['payload']);validation.append(dict(payload=row['payload'],rotor=row['rotor'],name=s.name,metrics=metrics(s,a),score=float(np.sum(residual(s,a,x)**2))))
   if row['payload']==chosen['payload'] and row['rotor']==chosen['rotor']:traces[s.name]=a
 (OUT/'transfer.json').write_text(json.dumps(validation,indent=2));np.savez_compressed(OUT/'selected-traces.npz',**traces);b.proc.terminate()
