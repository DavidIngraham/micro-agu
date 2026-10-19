import copy,json
import numpy as np
from study import Backend,load,replay,metrics,residual,OUT
if __name__=='__main__':
 b=Backend();ss=load();fits=json.loads((OUT/'fits.json').read_text());chosen=[min([f for f in fits if f['dynamic']==d],key=lambda f:f['cost']) for d in [False,True]];rows=[]
 for fit in chosen:
  label='rpm_damping' if fit['dynamic'] else 'constant'
  for j,s in enumerate(ss):
   x=np.array(fit['x']);x[4:8]=x[4:8] if j==0 else 0.;base=replay(b,s,x)
   fine=replay(b,s,x,steps=64);err=float(np.max(abs(base-fine)));assert err<.03
   for kind,value in [('yaw_inertia',.85),('yaw_inertia',1.15),('pitch_angle',-.0873),('pitch_angle',.0873),('yaw_angle',-.0873),('yaw_angle',.0873),('north_wind',-1.),('north_wind',1.),('east_wind',-1.),('east_wind',1.),('rpm_shift',-.12),('rpm_shift',.12)]:
    xx=x.copy();payload=1.;seg=copy.copy(s)
    if kind=='yaw_inertia':payload=value
    elif kind=='rpm_shift':seg.rpm_history=np.interp(s.time+value,s.time,s.rpm_history);seg.rpm=float(seg.rpm_history[0])
    else:xx[{'pitch_angle':4,'yaw_angle':5,'north_wind':6,'east_wind':7}[kind]]+=value
    a=replay(b,seg,xx,payload=payload);rows.append(dict(variant=label,name=s.name,kind=kind,value=value,metrics=metrics(s,a),score=float(np.sum(residual(s,a,xx)**2)),resolution_max_rate_difference=err))
 (OUT/'sensitivity.json').write_text(json.dumps(rows,indent=2));b.proc.terminate();print('Completed',len(rows),'sensitivity cases and 8 resolution checks')
