import json,concurrent.futures as cf
import numpy as np
from run import OUT,segments,variants,one,load_session
if __name__=='__main__':
 labels=['k1.0-c1.0-j1.0','k1.0-c0.05-j1.3','k0.5-c0.05-j0.7','k0.5-c1.0-j0.7'];v=variants()
 _,powered,_=load_session(OUT.parent/'pitch-inertia-damping-2026-10-18/session.json')
 jobs=[(s,k,v[k]) for k in labels for s in powered]
 nuisances=[[0,0,0,0],[.0873,0,0,0],[-.0873,0,0,0],[0,1,0,0],[0,-1,0,0],[0,0,1,0],[0,0,-1,0],[0,0,0,.0873],[0,0,0,-.0873]]
 jobs += [(s,k,v[k],n,True) for k in labels[:3] for s in segments() for n in nuisances]
 with cf.ProcessPoolExecutor(max_workers=4) as pool:rows=list(pool.map(one,jobs))
 (OUT/'validation.json').write_text(json.dumps(rows,separators=(',',':')))
 print('Completed',len(rows),'failures',sum('error'in r for r in rows))
 for k in labels:
  a=[r for r in rows[:40] if r['variant']==k and 'error'not in r]
  print(k,'powered mean rate RMSE',np.mean([r['rmse'] for r in a],axis=0))
