from pathlib import Path
import json,subprocess
import numpy as np
from scipy.optimize import least_squares
from scipy.ndimage import gaussian_filter1d
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
O=Path('/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/longitudinal-2026-10-09/turn-model-spike');R=O.parent
proc=subprocess.Popen([str(O/'rigid_evaluator')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,bufsize=1)
def evalstate(y,inp,wind=(0,0),params=(1.55,10,.0001,-.05)):
 proc.stdin.write(' '.join(map(str,[*y,*wind,*inp,*params]))+'\n');proc.stdin.flush();return np.fromstring(proc.stdout.readline(),sep=' ')
res={};fig,ax=plt.subplots(3,2,figsize=(12,10),layout='constrained')
for col,(day,name,starts) in enumerate([('2026-01-03','2026-01-03 15-41-42',[443,447,450]),('2026-02-15','2026-02-15 15-38-50',[326,402,3387])]):
 z=dict(np.load(O/(day+'.npz')));t=z['t'];imu=json.loads((R/name/'IMU.json').read_text());imu=[x for x in imu if x.get('I')==0];it=[x['TimeUS']/1e6 for x in imu]
 omega=np.array([np.interp(t,it,[x[k] for x in imu]) for k in ['GyrX','GyrY','GyrZ']]).T
 Y=np.column_stack([np.deg2rad(z['roll']),np.deg2rad(z['pitch']),np.deg2rad(z['yaw']),omega,z['VN'],z['VE'],-z['climb']]);U=np.column_stack([z['throttle_pwm'],z['left'],z['right']]);np.savez_compressed(O/(day+'-replay-input.npz'),t=t,Y=Y,U=U)
 wind=(1.04,.59) if col==0 else (.32,-.13)
 # Conditional acceleration fit: measured state supplied at each sample, not a free replay.
 vel=gaussian_filter1d(Y[:,6:9],12.5,axis=0);acc=np.gradient(vel,t,axis=0)
 ids=np.arange(0,len(t),13);ids=ids[z['airborne'][ids]&(z['mode'][ids]==0)]
 ids=ids[(t[ids]>330)&(t[ids]<1000)] if col==0 else ids[(t[ids]>220)&(t[ids]<570)]
 features=[];target=[];times=[]
 for i in ids:
  if col==0 and (435<t[i]<465 or 500<t[i]<525):continue
  a=evalstate(Y[i],U[i],wind,(1,0,0,-.05))[:3]-[0,0,9.80665]
  b=evalstate(Y[i],U[i],wind,(1,1,0,-.05))[:3]-[0,0,9.80665]-a
  c=evalstate(Y[i],U[i],wind,(1,0,1,-.05))[:3]-[0,0,9.80665]-a
  features.append(np.array([a,b,c]).T);target.append(acc[i]-[0,0,9.80665]);times.append(t[i])
 X=np.array(features);target=np.array(target);fit=least_squares(lambda b:(X@b-target).ravel(),[1/1.55,10/1.55,.0001/1.55],bounds=([1/6,0,0],[1,40,3]),loss='soft_l1')
 beta=fit.x;params=(1/beta[0],beta[1]/beta[0],beta[2]/beta[0],-.05)
 baseline=np.array([1/1.55,10/1.55,.0001/1.55]);rows=[]
 for lo in starts:
  # 3-second open-loop state replay from measured initial state, same recorded actuator commands.
  i=np.searchsorted(t,lo);y0=Y[i].copy();truth=Y[(t>=lo)&(t<lo+3)];tt=np.arange(lo,lo+3,.01)
  trajectories={}
  for label,pp in [('Default', (1.55,10,.0001,-.05)),('Conditional fit',params)]:
   y=y0.copy();out=[]
   def der(y,u):
    e=evalstate(y,u,wind,pp);ph,th=y[:2];p,q,r=y[3:6];edot=np.array([p+np.tan(th)*(q*np.sin(ph)+r*np.cos(ph)),q*np.cos(ph)-r*np.sin(ph),(q*np.sin(ph)+r*np.cos(ph))/np.cos(th)])
    return np.r_[edot,e[3:6],e[:3]]
   for ti in tt:
    u=np.array([np.interp(ti,t,U[:,k]) for k in range(3)]);k1=der(y,u);y+=.01*der(y+.005*k1,u);out.append(y.copy())
   trajectories[label]=np.array(out)
  for label,y in trajectories.items():
   actual=np.column_stack([np.interp(tt,t,Y[:,k]) for k in range(9)])
   rows.append(dict(start=lo,model=label,bank_rmse_deg=float(np.sqrt(np.mean(np.rad2deg(y[:,0]-actual[:,0])**2))),heading_change_error_deg=float(np.rad2deg(y[-1,2]-actual[-1,2])),climb_rmse=float(np.sqrt(np.mean((y[:,8]-actual[:,8])**2)))))
  if lo==starts[0]:
   for label,y in trajectories.items():
    ax[0,col].plot(tt-lo,np.rad2deg(y[:,0]),label=label);ax[1,col].plot(tt-lo,np.rad2deg(y[:,2]-y0[2]));ax[2,col].plot(tt-lo,-y[:,8])
   m=(t>=lo)&(t<=lo+3);ax[0,col].plot(t[m]-lo,z['roll'][m],color='black',label='Flight');ax[1,col].plot(t[m]-lo,np.rad2deg(Y[m,2]-y0[2]),color='black');ax[2,col].plot(t[m]-lo,z['climb'][m],color='black')
   ax[0,col].set_title(day+' · start '+str(lo)+' s');ax[0,col].legend()
 res[day]=dict(assumed_wind_NE=wind,fit_samples=len(times),effective_parameters=dict(mass_kg=params[0],thrust_max_N=params[1],CD_da=params[2]),conditional_training_accel_rmse_default=float(np.sqrt(np.mean((X@baseline-target)**2))),conditional_training_accel_rmse_fit=float(np.sqrt(np.mean((X@beta-target)**2))),replays=rows)
 print(day,json.dumps(res[day]),flush=True)
for a in ax.flat:a.grid(alpha=.2);a.set_xlabel('Seconds into open-loop replay')
for i,label in enumerate(['Bank (deg)','Heading change (deg)','Climb (m/s)']):ax[i,0].set_ylabel(label)
fig.suptitle('Rigid published force model: 3 s actuator replay\nMeasured initial state; assumed wind; fitted values are effective, not measured hardware properties')
fig.savefig(O/'native-rigid-replay.png',dpi=150);(O/'native-replay-results.json').write_text(json.dumps(res,indent=2));proc.stdin.close();proc.wait()
