from pathlib import Path
import json,hashlib,re,numpy as np
from scipy.optimize import least_squares
O=Path(__file__).parent
PLAN=json.loads((O/'mission2-plan.json').read_text())
assert hashlib.sha256((O/'mission2-plan.json').read_bytes()).hexdigest()==(O/'mission2-plan.sha256').read_text().strip()
SOURCE=(O/'fast_napkin_causal.cpp').read_text()
def grid(name):return np.array([float(v) for v in re.search(r'\b'+name+r'\[\]\s*=\s*\{([^}]+)\}',SOURCE).group(1).split(',') if v.strip()])
RG=grid('rpm_grid');CT=grid('ct_grid').reshape(len(RG),401)
def thrust(rpm,axial):
 rpm=np.asarray(rpm);n=rpm/60;j=np.clip(np.maximum(axial,0)/np.maximum(n*.2286,1e-9),0,2)*200;k=np.minimum(399,j.astype(int));g=j-k;i=np.clip(np.searchsorted(RG,rpm,side='right')-1,0,len(RG)-2);f=(rpm-RG[i])/(RG[i+1]-RG[i]);a=CT[i,k]*(1-g)+CT[i,k+1]*g;b=CT[i+1,k]*(1-g)+CT[i+1,k+1]*g
 return np.where((rpm>=RG[0])&(rpm<RG[-1]),1.225*n*n*.2286**4*(a*(1-f)+b*f),0)
def rpm_target(u,voltage):return 743.5778222336925*voltage*np.maximum(u-.0005141016498671315,0)**.9086807307118114
TAU=.01756125311487475

def prepare(partition):
 rows=PLAN[partition];cache={};records=[];fields={k:[] for k in ['U','wind','mass','voltage','rpm_lead','rpm_initial','initial','truth','truth_series','velocity_history']}
 for r in rows:
  day=r['day']
  if day not in cache:
   a=dict(np.load(O/f'{day}-causal-input.npz'));b=np.load(O/f'{day}.npz');a['air']=np.interp(a['t'],b['t'],b['airborne'].astype(float))>.99;cache[day]=(a,dict(np.load(O/f'{day}-propulsion-9x45.npz')))
  a,prop=cache[day];t=a['t'];y=a['Y'];i=np.searchsorted(t,r['start_s']);il=i-125;prior=(t<t[il])&(t>=t[il]-120)&a['valid']&a['air']&(abs(y[:,0])<np.deg2rad(25));jj=np.flatnonzero(prior)[::5];assert len(jj)>=100
  ps=y[jj,2];A=np.c_[-np.sin(ps),np.cos(ps)];target=np.sum(A*y[jj,6:8],axis=1);fit=least_squares(lambda w:A@w-target,[0,0],bounds=(-5,5),loss='soft_l1',f_scale=.3);wind=fit.x
  U=a['U'][il:i+251];voltage=float(prop['voltage'][il]);rpm=float(prop['rpm'][il]);r0=rpm
  for u in U[:125,0]:tar=rpm_target(u,voltage);rpm=tar+(rpm-tar)*np.exp(-.04/TAU)
  obs=y[i:i+251];airvel=obs[:,6:8]-wind;psi=np.arctan2(airvel[0,1],airvel[0,0]);hist=y[i-25:i+1,6:8]-wind;ph=np.unwrap(np.arctan2(hist[:,1],hist[:,0]));q=(ph[-1]-ph[0])/(t[i]-t[i-25]);mass=1.385 if day=='2026-01-03' else 1.758
  course=np.unwrap(np.arctan2(obs[:,7],obs[:,6]));speed=np.linalg.norm(obs[:,6:8],axis=1);climb=-obs[:,8];truth=[np.trapz(speed,dx=.04),np.trapz(climb,dx=.04),np.rad2deg(course[-1]-course[0])]
  vals=dict(velocity_history=y[i-25:i+1,6:8],U=U,wind=wind,mass=mass,voltage=voltage,rpm_lead=r0,rpm_initial=rpm,initial=[np.linalg.norm(airvel[0]),climb[0],psi,q],truth=truth,truth_series=np.c_[speed,climb,course])
  for k,v in vals.items():fields[k].append(v)
  records.append(r|dict(index=int(i),lead_index=int(il),wind=wind.tolist(),wind_samples=len(jj),wind_crossflow_rmse=float(np.sqrt(np.mean((A@wind-target)**2))),wind_design_condition=float(np.linalg.cond(A))))
 np.savez_compressed(O/f'mission2-{partition}-data.npz',**{k:np.array(v) for k,v in fields.items()});(O/f'mission2-{partition}-records.json').write_text(json.dumps(records,indent=2));return {k:np.array(v) for k,v in fields.items()}

def predict(p,D,substeps=2,wind_delta=(0,0),series=False):
 A,B,C,vtrim,kbr,kc,tc,kl,kr,tq=p;m=D['mass'];mu=np.sqrt(m/1.385);wind=D['wind']+np.array(wind_delta);state=D['initial'].copy()
 # Reconstruct observed ground velocity at origin, then initialize for perturbed wind.
 vg=np.c_[state[:,0]*np.cos(state[:,2]),state[:,0]*np.sin(state[:,2])]+D['wind'];va=vg-wind;state[:,0]=np.linalg.norm(va,axis=1);state[:,2]=np.arctan2(va[:,1],va[:,0])
 hist=D['velocity_history']-wind[:,None,:];ph=np.unwrap(np.arctan2(hist[:,:,1],hist[:,:,0]),axis=1);state[:,3]=ph[:,-1]-ph[:,0]
 rpm=D['rpm_initial'].copy();U=D['U'][:,125:];out=[];invalid=np.zeros(len(m),bool);dt=.04/substeps
 def rhs(s,u,r):
  v,c,psi,q=s.T;vv=np.maximum(v,.3);mean=(u[:,1]+u[:,2])/2;load=1+(vv*q/9.81)**2;drag=A*vv*vv+B*(m*9.81)**2*load/(vv*vv)+C*mean*vv*vv;T=thrust(r,vv)
  return np.c_[(T-drag)/m-9.81*c/vv,(kc*(v-vtrim*mu/(1+kbr*mean))-c)/tc,q,((kr*u[:,2]-kl*u[:,1])/vv-q)/tq]
 for k in range(251):
  v,c,psi,q=state.T;gv=np.c_[v*np.cos(psi),v*np.sin(psi)]+wind;out.append(np.c_[np.linalg.norm(gv,axis=1),c,np.arctan2(gv[:,1],gv[:,0]),v,q])
  if k==250:break
  u=U[:,k];tar=rpm_target(u[:,0],D['voltage'])
  for _ in range(substeps):
   rmid=tar+(rpm-tar)*np.exp(-dt/2/TAU);rend=tar+(rpm-tar)*np.exp(-dt/TAU)
   with np.errstate(over='ignore',invalid='ignore'):
    k1=rhs(state,u,rpm);k2=rhs(state+dt/2*k1,u,rmid);k3=rhs(state+dt/2*k2,u,rmid);k4=rhs(state+dt*k3,u,rend);state+=dt/6*(k1+2*k2+2*k3+k4)
   bad=(~np.isfinite(state).all(axis=1))|(state[:,0]<.3)|(np.max(abs(state),axis=1)>1e4);invalid|=bad;state[bad]=[3,0,0,0];rpm=rend
 out=np.array(out).transpose(1,0,2);course=np.unwrap(out[:,:,2],axis=1);score=np.c_[np.trapz(out[:,:,0],dx=.04,axis=1),np.trapz(out[:,:,1],dx=.04,axis=1),np.rad2deg(course[:,-1]-course[:,0])];score[invalid]=np.nan
 return (score,out,invalid) if series else score
if __name__=='__main__':
 for part in ['development','evaluation']:
  d=prepare(part);print(part,len(d['mass']),flush=True)


