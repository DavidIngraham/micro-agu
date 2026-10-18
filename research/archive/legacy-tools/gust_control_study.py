"""Bounded gust-control screen with native-algebra diagnostic loops and truth sensing.
Raw trajectories stay in the supplied research directory. Native SITL follows screening.
"""
import argparse
import json
import sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

p=argparse.ArgumentParser();p.add_argument('--model',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--seed',type=int,default=20261011);p.add_argument('--scale',type=float,default=1.0);p.add_argument('--validation-only',action='store_true');p.add_argument('--two-pole',action='store_true');p.add_argument('--two-pole-hz',type=float,default=.75);p.add_argument('--gain-check',action='store_true');p.add_argument('--cutoff-check',action='store_true');p.add_argument('--candidates',type=Path);p.add_argument('--height-ramp',type=float,default=0);a=p.parse_args()
sys.path.insert(0,str(a.model));sys.path.insert(0,str(Path(__file__).parents[1]/'research/archive/viewer-studies'))
from json_backend import Backend
from controllers import Turn
from turbulence import Gusts,SETTINGS

a.output.mkdir(parents=True,exist_ok=True)
cases={
 'baseline':{}, 'fixed_throttle':{'fixed':True}, 'without_034':{'remove034':True},
 'slow_pitch_010':{'q_gain':.10,'q_hz':.5},
 'slow_pitch_020':{'q_gain':.20,'q_hz':.5},
 'climb_gain_012':{'kv':.12},
 'combined':{'kv':.08,'q_gain':.10,'q_hz':.5},
 'reversed_slow_pitch':{'q_gain':-.10,'q_hz':.5},
}
if a.two_pole:cases.update({'two_pole_010':{'q_gain':.1,'q_hz':a.two_pole_hz,'q2_hz':a.two_pole_hz}})
if a.validation_only:cases={k:cases[k] for k in (['baseline','two_pole_010'] if a.two_pole else ['baseline','slow_pitch_010'])}
if a.gain_check:cases={f'two_pole_{int(g*1000):03d}':{'q_gain':g,'q_hz':.75,'q2_hz':.75} for g in (.15,.2)}
if a.cutoff_check:cases={f'two_pole_hz_{int(hz*100):03d}':{'q_gain':.1,'q_hz':hz,'q2_hz':hz} for hz in (1.,1.5)}
if a.candidates:cases=json.loads(a.candidates.read_text())
SETTINGS['seed']=a.seed
SETTINGS['rms_ned_m_s']=[v*a.scale for v in SETTINGS['rms_ned_m_s']]
rows=[];histories={};dt=.02
for name,cfg in cases.items():
 b=Backend();b.sim.substeps=16;b.released=True;base=b.command.copy();gust=Gusts()
 if cfg.get('remove034'):gust.amplitude[:,3]=0
 lat=Turn(dt=dt,p=.005,i=.002,rdamp=.003,dff=0,filter_hz=4)
 lat.integral=base[2]/(np.pi*55/360*.1*1500)
 rateff=0.;integral=base[0]-.45;last=.45;qf=0;qf2=0;kv=cfg.get('kv',.6/35);trace=[]
 for j in range(6001):
  t=j*dt;y=b.state;rot=Rotation.from_quat(y[3:7]);phi,theta,_=rot.as_euler('xyz');rates=rot.inv().apply(y[10:13]);wind=gust.at(t+.5*dt) if t<80 else np.zeros(3)
  climb=-y[9];height=-y[2];target=100+a.height_ramp*np.clip((t-2)/2,0,1);targetrate=a.height_ramp/2 if 2<t<4 else 0.;rateff+=dt/(dt+.5)*(targetrate-rateff);vdes=np.clip((target-height)/10+rateff,-2.5,1);err=vdes-climb
  ff=.45+vdes*(.15 if vdes>=0 else .18);pd=np.clip(ff+kv*err,0,.6);pd=np.clip(pd,last-.6*dt,last+.6*dt);last=pd
  integral=np.clip(integral+.3*kv*err*dt,np.clip(-pd-.1,-.3,.3),np.clip(.6-pd+.1,-.3,.3))
  hz=cfg.get('q_hz',.5);qf+=dt/(dt+1/(2*np.pi*hz))*(rates[1]-qf)
  qf2+=dt/(dt+1/(2*np.pi*cfg.get('q2_hz',.75)))*(qf-qf2)
  qdamper=qf2 if cfg.get('q2_hz') else qf
  throttle=float(np.clip(pd+integral+np.clip(-cfg.get('q_gain',0)*qdamper,-.25,.25),0,.6))
  if cfg.get('fixed'):throttle=base[0]
  speed=np.linalg.norm(y[7:10]-wind);sv=np.clip(5/max(speed,2),.25,2.5)
  heading=(rates[1]*np.sin(phi)+rates[2]*np.cos(phi))/np.cos(theta)*180/np.pi
  lat.feedforward=lambda r:.009*r*sv
  u=lat.update(0,heading,rates[0]*180/np.pi,speed)
  cmd=np.array([throttle,max(-u,0)*np.pi*55/360*.1*1300,max(u,0)*np.pi*55/360*.1*1500])
  trace.append([t,height-100,climb,theta*180/np.pi,rates[1]*180/np.pi,throttle,*wind,u])
  b.state,b.rpm,_=b.sim.step_actuated(y,b.rpm,b.actuator,cmd,voltage=22.2,wind=wind,dt=dt)
  if not np.isfinite(b.state).all():raise RuntimeError(name+' nonfinite')
 data=np.array(trace);use=(data[:,0]>=15)&(data[:,0]<80);calm=data[:,0]>105
 row=dict(case=name,config=cfg,height_rms=float(np.sqrt(np.mean(data[use,1]**2))),climb_rms=float(np.sqrt(np.mean(data[use,2]**2))),pitch_std=float(np.std(data[use,3])),pitch_rate_rms=float(np.sqrt(np.mean(data[use,4]**2))),throttle_std=float(np.std(data[use,5])),throttle_min=float(data[use,5].min()),throttle_max=float(data[use,5].max()),saturation_fraction=float(np.mean((data[use,5]<=0)|(data[use,5]>=.6))),recovery_pitch_std=float(np.std(data[calm,3])))
 rows.append(row);histories[name]=data;np.savez_compressed(a.output/(name+'.npz'),trace=data);(a.output/'summary.json').write_text(json.dumps(dict(gusts=SETTINGS,rows=rows),indent=2));print(json.dumps(row),flush=True)
fig,axes=plt.subplots(4,1,figsize=(12,11),sharex=True,layout='constrained')
for name in histories:
 if name not in histories:continue
 d=histories[name]
 for ax,col in zip(axes,[3,2,1,5]):ax.plot(d[:,0],d[:,col],label=name,lw=1)
for ax,label in zip(axes,['Payload pitch (deg)','Climb (m/s)','Height error (m)','Throttle']):ax.set_ylabel(label);ax.grid(alpha=.25);ax.axvline(80,color='black',ls='--')
axes[0].legend(ncol=3);axes[-1].set_xlabel('Time (s); gusts stop at 80 s');fig.suptitle('Matched nonlinear gust screen — truth sensing; native algebra, not native SITL')
fig.savefig(a.output/'screen.png',dpi=140)
