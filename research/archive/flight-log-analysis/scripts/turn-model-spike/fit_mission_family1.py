from pathlib import Path
import json, numpy as np
from scipy.optimize import least_squares
O=Path(__file__).parent
protocol=json.loads((O/'mission-performance-frozen-protocol.json').read_text())
rows=[r for r in protocol['blocks'] if r['partition']=='development']
# Bounds and start fixed before optimization; ten shared coefficients only.
names=['v0','kB','tauV','kR','kL','tauQ','aT','s0','kTurn','tauC']
x0=[4.,.5,2.,1.,1.,2.,8.,1.2,.1,2.]
lo=[1.,0.,.2,0.,0.,.2,0.,.2,0.,.2]
hi=[10.,4.,15.,10.,10.,15.,30.,4.,2.,15.]
settings=dict(names=names,start=x0,lower=lo,upper=hi,max_nfev=150,loss='soft_l1',fit_metrics=['block mean groundspeed','block mean climb','block net ground-course rate'],scales=[.75,.3,3.],selection='development only; no evaluation results inspected')
(O/'mission-family1-fit-settings.json').write_text(json.dumps(settings,indent=2))
def dataset(rows):
 data={d:np.load(O/f'{d}-causal-input.npz') for d in {r['day'] for r in rows}}
 out=[]
 for row in rows:
  a=data[row['day']]; t=a['t']; y=a['Y'];i=np.searchsorted(t,row['start_s']); ids=np.arange(i,i+251)
  prior=(t<t[i])&(t>=t[i]-120)&a['valid']&(abs(y[:,0])<np.deg2rad(25))
  jj=np.flatnonzero(prior)[::5]
  if len(jj)<100:raise ValueError('Insufficient prior wind data '+str(row))
  psi=y[jj,2];A=np.c_[-np.sin(psi),np.cos(psi)];target=np.sum(A*y[jj,6:8],axis=1)
  wind=least_squares(lambda w:A@w-target,[0.,0.],bounds=(-5,5),loss='soft_l1',f_scale=.3).x
  av=y[ids,6:8]-wind; h=np.unwrap(np.arctan2(av[:,1],av[:,0]));initids=np.arange(max(0,i-25),i+1);vprior=y[initids,6:8]-wind
  hp=np.unwrap(np.arctan2(vprior[:,1],vprior[:,0])); q=(hp[-1]-hp[0])/(t[i]-t[initids[0]])
  gc=np.unwrap(np.arctan2(y[ids,7],y[ids,6]));truth=[np.linalg.norm(y[ids,6:8],axis=1).mean(),(-y[ids,8]).mean(),np.rad2deg(gc[-1]-gc[0])/10]
  out.append(dict(row=row,wind=wind,init=[np.linalg.norm(av[0]),-y[i,8],h[0],q],u=a['U'][ids],truth=truth,mu=np.sqrt((1.385 if row['day']=='2026-01-03' else 1.758)/1.385)))
 return out
def predict(p,D):
 v0,kB,tv,kr,kl,tq,at,s0,kt,tc=p
 state=np.array([d['init'] for d in D]);wind=np.array([d['wind'] for d in D]);U=np.array([d['u'] for d in D]);mu=np.array([d['mu'] for d in D]); sums=np.zeros((len(D),2));total=np.zeros(len(D));previous=None
 for k in range(251):
  v,c,psi,q=state.T;gv=np.c_[v*np.cos(psi),v*np.sin(psi)]+wind;course=np.arctan2(gv[:,1],gv[:,0]);sums+=np.c_[np.linalg.norm(gv,axis=1),c]
  if previous is not None:total+=np.arctan2(np.sin(course-previous),np.cos(course-previous))
  previous=course
  if k==250:break
  u,l,r=U[:,k,:].T;veq=v0*mu/(1+kB*(l+r)/2);qeq=(kr*r-kl*l)/np.maximum(v,.5);ceq=at*u*u/mu-s0*mu-kt*np.abs(q)*v
  state=np.c_[veq+(v-veq)*np.exp(-.04/tv),ceq+(c-ceq)*np.exp(-.04/tc),psi+.04*q,qeq+(q-qeq)*np.exp(-.04/tq)]
 return np.c_[sums/251,np.rad2deg(total)/10]
if __name__=='__main__':
 D=dataset(rows);truth=np.array([d['truth'] for d in D]);weights=[]
 for d in D:
  row=d['row'];same=[r for r in rows if r['day']==row['day']];cats={r['category'] for r in same};n=sum(r['category']==row['category'] for r in same)
  weights.append(1/np.sqrt(2*len(cats)*n))
 weights=np.array(weights)
 def residual(p):return ((predict(p,D)-truth)/np.array([.75,.3,3.])*weights[:,None]).ravel()
 fit=least_squares(residual,x0,bounds=(lo,hi),max_nfev=150,loss='soft_l1')
 result=dict(parameters=dict(zip(names,fit.x.tolist())),nfev=fit.nfev,cost=fit.cost,success=bool(fit.success),message=fit.message,scope='Development fit only; no acceptance claim',development_rmse=np.sqrt(np.mean((predict(fit.x,D)-truth)**2,axis=0)).tolist())
 (O/'mission-family1-development-fit.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
