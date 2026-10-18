"""Relative equilibria and fixed-throttle lateral feedback, including mixer branches.
Common brake is normalized servo travel, so unequal left/right strokes are retained.
Controller update and command hold are 50 Hz, confirmed from SCHED_LOOP_RATE and PGYR timestamps.
The physics retains 1.25 ms integration substeps.
No outer guidance, estimator, longitudinal loop or moving flare is linearized.
"""
import sys,json
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from microagu_sim import compat
model=Path(compat.__file__).parent
from microagu_sim.compat.profiles import make_simulator
from microagu_sim.compat.equilibrium import perturb,difference
out=Path(__file__).parent
sim=make_simulator('trout_lake','mission');sim.substeps=16
initial=np.load(model/'equilibrium-trim-trout_lake-0.0-0.0.npz')
strokes=np.array([1300,1500])*np.pi*55/360*.1
dt=.02;voltage=22.2

def unturn(y,turn):
 y=y.copy();rot=R.from_euler('z',-np.deg2rad(turn)*dt)
 for sl in [slice(3,7),slice(16,20)]:y[sl]=(rot*R.from_quat(y[sl])).as_quat()
 for sl in [slice(7,10),slice(10,13),slice(20,23),slice(23,26)]:y[sl]=rot.apply(y[sl])
 return y

def observation(y):
 rot=R.from_quat(y[3:7]);roll,pitch,_=rot.as_euler('xyz');p,q,r=rot.inv().apply(y[10:13])
 return np.rad2deg([(q*np.sin(roll)+r*np.cos(roll))/np.cos(pitch),p])

rows=[]
conditions=[(s,0,0) for s in [0,.5,1]]+[(s,t,0) for s in [.5,1] for t in [-10,10]]+[(1,0,b) for b in [.15,.3]]
for index,(sink,turn,common) in enumerate(conditions):
 base=initial['state'].copy()
 def commands(x):return np.r_[x[-2],strokes*(common+np.maximum([-x[-1],x[-1]],0))]
 def unpack(x):
  y=perturb(base,x[:15],sim.geometry);u=commands(x);rpm=743.5778222336925*voltage*max(u[0]-.0005141016498671315,0)**.9086807307118114
  return y,u,rpm
 def residual(x):
  y,u,rpm=unpack(x);n,_=sim.step(y,rpm,u,voltage=voltage,dt=dt);n=unturn(n,turn)
  return np.r_[difference(y,n)/dt,y[9]-sink,R.from_quat(y[3:7]).as_euler('xyz')[2]]
 x0=np.r_[np.zeros(15),initial['command'][0],initial['command'][2]/strokes[1]]
 lo=np.r_[np.full(6,-1.),np.full(3,-3.),np.full(6,-3.),.001,-(1-common)]
 hi=np.r_[np.full(6,1.),np.full(3,3.),np.full(6,3.),.65,1-common]
 fit=least_squares(residual,x0,bounds=(lo,hi),x_scale=np.r_[np.ones(15),.1,.1],diff_step=1e-4,max_nfev=180,ftol=1e-11,xtol=1e-11,gtol=1e-9)
 err=float(max(abs(residual(fit.x))));row=dict(index=index,sink=sink,turn=turn,common=common,residual=err,accepted=err<1e-6)
 if not row['accepted']:
  rows.append(row);print(row,flush=True);continue
 state,u,rpm=unpack(fit.x);branch=1 if fit.x[-1]>0 else -1
 def advance(x,du=0):
  y=perturb(state,x[:15],sim.geometry);c=u.copy();c[2 if branch==1 else 1]+=du*strokes[1 if branch==1 else 0]*branch
  yn,rn=sim.step(y,rpm+x[15],c,voltage=voltage,dt=dt)
  return np.r_[difference(state,unturn(yn,turn)),rn-rpm]
 A=np.empty((16,16));C=np.zeros((2,16));speed_gradient=np.zeros(16)
 for j in range(16):
  e=np.zeros(16);e[j]=1 if j==15 else 1e-5
  A[:,j]=(advance(e)-advance(-e))/(2*e[j])
  C[:,j]=(observation(perturb(state,e[:15],sim.geometry))-observation(perturb(state,-e[:15],sim.geometry)))/(2*e[j])
  speed_gradient[j]=(np.linalg.norm(perturb(state,e[:15],sim.geometry)[7:10])-np.linalg.norm(perturb(state,-e[:15],sim.geometry)[7:10]))/(2*e[j])
 B=(advance(np.zeros(16),1e-5)-advance(np.zeros(16),-1e-5))/2e-5
 # Independent step-size check, away from the unilateral mixer cusp.
 v=np.arange(1,17,dtype=float);v/=np.linalg.norm(v);v*=1e-6
 difference_check=advance(v,1e-6)-advance(-v,-1e-6)
 jacobian_error=float(np.linalg.norm(difference_check-2*(A@v+B*1e-6))/np.linalg.norm(difference_check))
 tas=np.linalg.norm(state[7:10]);scale=np.clip(5/tas,.25,2.5);modes=[]
 scale_gradient=-5/tas**2*speed_gradient
 scale_coefficient=.009*turn-.003*observation(state)[1]
 integral_trim=float(fit.x[-1]-scale*scale_coefficient)
 assert abs(integral_trim)<.15, 'Integrator trim exceeds configured bound'
 for cutoff in [4,1]:
  for delay in [0,.04,.08,.12]:
   n=round(delay/dt);alpha=dt/(dt+1/(2*np.pi*cutoff))
   def closed(x):
    filt=x[16:18]+alpha*(C@x[:16]-x[16:18]);integ=x[18]-.002*dt*filt[0]
    control=-.005*scale*filt[0]-.003*scale*filt[1]+integ+scale_coefficient*(scale_gradient@x[:16])
    return np.r_[A@x[:16]+B*(x[-1] if n else control),filt,integ,([control,*x[19:-1]] if n else [])]
   F=np.column_stack([closed(e) for e in np.eye(19+n)])
   if delay==.12:
    # Nonlinear fixed-target controller response at a small roll perturbation.
    xn=np.zeros(19+n);xn[0]=1e-5;xl=xn.copy();truth_history=[];linear_history=[]
    trim_obs=observation(state);offset_map=advance(np.zeros(16))
    for _ in range(round(4/dt)):
     y=perturb(state,xn[:15],sim.geometry)
     filtered=xn[16:18]+alpha*(observation(y)-trim_obs-xn[16:18])
     integral=xn[18]-.002*dt*filtered[0]
     live_scale=np.clip(5/np.linalg.norm(y[7:10]),.25,2.5)
     control=live_scale*(scale_coefficient-.005*filtered[0]-.003*filtered[1])-scale*scale_coefficient+integral
     xn=np.r_[advance(xn[:16],xn[-1])-offset_map,filtered,integral,control,xn[19:-1]]
     xl=F@xl;truth_history.append(C@xn[:16]);linear_history.append(C@xl[:16])
    response_error=float(np.linalg.norm(np.array(truth_history)-linear_history)/np.linalg.norm(linear_history))
   else:response_error=None
   z=np.linalg.eigvals(F);z=np.log(z[abs(z)>1e-12].astype(complex))/dt;z=z[abs(z)>1e-5]
   dominant=z[np.argmax(z.real)]
   modes.append(dict(nonlinear_response_relative_error=response_error,cutoff=cutoff,delay=delay,real=float(dominant.real),hz=float(abs(dominant.imag)/2/np.pi)))
 row.update(command=u.tolist(),differential=float(fit.x[-1]),tas=float(np.linalg.norm(state[7:10])),branch=branch,integral_trim=integral_trim,jacobian_error=jacobian_error,modes=modes)
 np.savez(out/f'envelope-{index}.npz',A=A,B=B,C=C,state=state,command=u,rpm=rpm,dt=dt)
 rows.append(row);print({k:v for k,v in row.items() if k!='modes'},flush=True)
(out/'envelope.json').write_text(json.dumps(rows,indent=2))
fig,axes=plt.subplots(1,2,figsize=(13,5),sharey=True)
valid=[r for r in rows if r['accepted']]
for ax,delay in zip(axes,[0,.12]):
 for cutoff,color in [(4,'#cb7648'),(1,'#188c99')]:
  ax.plot(range(len(valid)),[next(m['real'] for m in r['modes'] if m['cutoff']==cutoff and m['delay']==delay) for r in valid],'o-',label=f'{cutoff} Hz',color=color)
 ax.axhline(0,color='black',lw=1);ax.set_xticks(range(len(valid)),[f"{r['sink']:g} / {r['turn']:g} / {100*r['common']:g}%" for r in valid],rotation=50,ha='right');ax.set_title(f'{1000*delay:g} ms additional command delay');ax.grid(alpha=.2);ax.legend();ax.set_xlabel('Sink (m/s) / turn (deg/s) / common brake')
mode=next(m for m in valid[4]['modes'] if m['cutoff']==4 and m['delay']==.12)
axes[1].annotate(f"{mode['hz']:.2f} Hz oscillatory mode",xy=(4,mode['real']),xytext=(4.7,.045),arrowprops=dict(arrowstyle='->',color='#cb7648'),fontsize=9,ha='center')
axes[0].set_ylim(-.18,.065)
axes[0].set_ylabel('Largest non-neutral pole real part (1/s)')
fig.suptitle('Local landing-envelope feedback: negative means decaying perturbations')
fig.tight_layout();fig.savefig(out/'landing-envelope.png',dpi=160)
