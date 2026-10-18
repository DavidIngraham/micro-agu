"""Research flight-path plant A. SI units; body attitude is not modeled."""
from pathlib import Path
import json,hashlib,numpy as np
from scipy.optimize import least_squares
from scipy.linalg import expm
from mission2_model import thrust,rpm_target,TAU
O=Path(__file__).parent
NAMES=['speed0','speed_thrust','speed_brake','speed_turn','speed_tau','sink0','climb_thrust','sink_brake','sink_turn','climb_tau','left_gain','right_gain','trim_HR','trim_TL','trim_thrust','turn_tau','turn_damping']
START=np.array([3.8,.08,1,.1,2,1.2,.45,.3,.1,2,2.4,3.4,.1,0,-.03,3.,.7])
LOW=np.array([2,-.3,0,0,.3,.5,.05,0,0,.3,.05,.05,-2,-2,-1,.3,.15])
HIGH=np.array([7,.5,5,2,10,2.5,2,3,2,10,15,15,2,2,1,12,2.])

def prepare():
 raw=(O/'up-away-frozen-episodes.json').read_bytes();assert hashlib.sha256(raw).hexdigest()==(O/'up-away-frozen-episodes.sha256').read_text().strip();plan=json.loads(raw);cache={};records=[];out={k:[] for k in ['U','initial','truth','wind','mass','day','voltage','rpm','history']}
 for r in plan['rows']:
  day=r['day']
  if day not in cache:
   a=dict(np.load(O/f'{day}-causal-input.npz'));b=np.load(O/f'{day}.npz');a['air']=np.interp(a['t'],b['t'],b['airborne'].astype(float))>.99;cache[day]=(a,np.load(O/f'{day}-propulsion-9x45.npz'))
  a,prop=cache[day];t=a['t'];y=a['Y'];i=np.searchsorted(t,r['start_s']);jj=np.flatnonzero((t<t[i]-5)&(t>=t[i]-125)&a['valid']&a['air']&(abs(y[:,0])<np.deg2rad(25)))[::5]
  # Early episodes use the previously declared wind prior; do not discard frozen rows.
  h=y[jj,2];A=np.c_[-np.sin(h),np.cos(h)];b=np.sum(A*y[jj,6:8],axis=1);w=least_squares(lambda w:A@w-b,[0.,0.],bounds=(-5,5),loss='soft_l1',f_scale=.3).x
  if len(jj)<100:w=np.array([1.04,.59]) if day=='2026-01-03' else np.array([.1,.086])
  obs=y[i:i+251];av=obs[:,6:8]-w;hist=y[i-50:i+1,6:8];ang=np.unwrap(np.arctan2((hist-w)[:,1],(hist-w)[:,0]));q=np.polyfit(np.arange(51)*.04,ang,1)[0];chi=np.unwrap(np.arctan2(obs[:,7],obs[:,6]));truth=np.c_[np.linalg.norm(obs[:,6:8],axis=1),-obs[:,8],chi-chi[0]]
  vals=dict(U=a['U'][i:i+251],initial=[np.linalg.norm(av[0]),-obs[0,8],np.arctan2(av[0,1],av[0,0]),q],truth=truth,wind=w,mass=1.385 if day=='2026-01-03' else 1.758,day=int(day!='2026-01-03'),voltage=prop['voltage'][i],rpm=prop['rpm'][i],history=hist)
  for k,v in vals.items():out[k].append(v)
  records.append(r|dict(wind=w.tolist(),wind_source='causal120s' if len(jj)>=100 else 'historical_prior_sparse_history',wind_samples=len(jj),wind_condition=float(np.linalg.cond(A)),wind_crossflow_rmse=float(np.sqrt(np.mean((A@w-b)**2)))))
 np.savez_compressed(O/'up-away-data.npz',**{k:np.array(v) for k,v in out.items()});(O/'up-away-records.json').write_text(json.dumps(records,indent=2));return {k:np.array(v) for k,v in out.items()},records

def subset(D,ix):return {k:v[ix] for k,v in D.items()}

def predict(p,D,substeps=1):
 v0,vt,vb,vq,tv,s0,ct,cb,cq,tc,kl,kr,bh,bt,bT,tq,zeta=p
 m=D['mass'];mu=np.sqrt(m/1.385);w=D['wind'];state=np.c_[D['initial'].copy(),np.zeros(len(m))];rpm=D['rpm'].copy();out=[];h=.04/substeps;U=D['U'];E=expm(np.array([[0.,1.],[-1/tq**2,-2*zeta/tq]])*h)
 for k in range(U.shape[1]):
  v,c,psi,q,qd=state.T;gv=np.c_[v*np.cos(psi),v*np.sin(psi)]+w;out.append(np.c_[np.linalg.norm(gv,axis=1),c,np.arctan2(gv[:,1],gv[:,0]),v,q])
  if k==U.shape[1]-1:break
  u=U[:,k];mean=(u[:,1]+u[:,2])/2
  for _ in range(substeps):
   tar=rpm_target(u[:,0],D['voltage']);rmid=tar+(rpm-tar)*np.exp(-h/2/TAU);T=thrust(rmid,state[:,0])/m
   v,c,psi,q,qd=state.T;loading=(v*q)**2
   veq=np.clip(mu*v0+vt*T-vb*mean-vq*loading,1,12)
   ceq=-s0*mu+ct*T-cb*mean-cq*loading
   qeq=(kr*u[:,2]-kl*u[:,1]+np.where(D['day']==0,bh,bt)+bT*T)/np.maximum(v,1)
   vn=veq+(v-veq)*np.exp(-h/tv);cn=ceq+(c-ceq)*np.exp(-h/tc);qn=qeq+E[0,0]*(q-qeq)+E[0,1]*qd;qdn=E[1,0]*(q-qeq)+E[1,1]*qd
   state=np.c_[vn,cn,psi+h*(q+qn)/2,qn,qdn];rpm=tar+(rpm-tar)*np.exp(-h/TAU)
 arr=np.array(out).transpose(1,0,2);arr[:,:,2]=np.unwrap(arr[:,:,2],axis=1);arr[:,:,2]-=arr[:,0:1,2];return arr

def residual(p,D,target=None):
 out=predict(p,D)[:,:,0:3];truth=D['truth'] if target is None else target
 # One-second trace samples, not 25Hz pseudo-independent observations. Equal flight weight.
 e=(out[:,25::25]-truth[:,25::25])/np.array([.6,.3,np.deg2rad(20)])
 weights=np.array([1/np.sqrt(np.sum(D['day']==d)) for d in D['day']]);return (e*weights[:,None,None]).ravel()
if __name__=='__main__':
 D,r=prepare();print('Prepared',len(r),'episodes')

