"""Matched pitch-sign diagnostic using the active 9-DOF plant; not native SITL.
Raw states stay in --raw; final animation and evidence go to --output.
"""
import argparse,json,sys,subprocess
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation,FFMpegWriter
p=argparse.ArgumentParser();p.add_argument('--model',type=Path,required=True);p.add_argument('--raw',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--states-only',action='store_true');a=p.parse_args()
a.raw.mkdir(parents=True,exist_ok=True);a.output.mkdir(parents=True,exist_ok=True);sys.path.insert(0,str(a.model))
from json_backend import Backend
cases=[('Damper off',0,10,False),('Original sign',.1,10,False),('Reversed sign',-.1,10,False),('Current filter',.1,1.5,True)]
dt=.02;duration=24;hist=[];summary=[]
for name,g,hz,two in cases:
 b=Backend();b.sim.substeps=16;b.released=True;base=b.command.copy();q1=q2=0.;rows=[];states=[];frames=[]
 for j in range(int(duration/dt)+1):
  t=j*dt
  if j==100:
   rot=R.from_quat(b.state[3:7]);delta=rot.apply([0,np.deg2rad(5),0]);b.state[10:13]+=delta;b.state[7:10]-=np.cross(delta,rot.apply(b.sim.geometry['hinge']))
  rot=R.from_quat(b.state[3:7]);q=rot.inv().apply(b.state[10:13])[1]
  alpha=dt/(dt+1/(2*np.pi*hz));q1+=alpha*(q-q1);q2+=alpha*(q1-q2)
  cmd=base.copy();cmd[0]=np.clip(base[0]+np.clip(-g*(q2 if two else q1),-.25,.25),0,.6)
  b.time=t;b.command=cmd.copy();frames.append(b.viewer_state())
  rows.append([t,rot.as_euler('xyz')[1]*180/np.pi,R.from_quat(b.state[16:20]).as_euler('xyz')[1]*180/np.pi,q*180/np.pi,cmd[0]*100]);states.append(b.state.copy())
  b.state,b.rpm,_=b.sim.step_actuated(b.state,b.rpm,b.actuator,cmd,voltage=22.2,dt=dt)
 (a.raw/(name.replace(' ','-')+'.jsonl')).write_text(''.join(json.dumps(f)+'\n' for f in frames))
 d=np.array(rows);hist.append(d);np.savez_compressed(a.raw/(name.replace(' ','-')+'.npz'),trace=d,state=states)
 summary.append(dict(case=name,gain=g,filter_hz=hz,stages=2 if two else 1,pitch_rate_rms_last_10s=float(np.sqrt(np.mean(d[d[:,0]>14,3]**2))),pitch_range_deg=[float(d[:,1].min()),float(d[:,1].max())]))
 print(summary[-1],flush=True)
meta=dict(model_commit=subprocess.check_output(['git','-C',str(a.model),'rev-parse','HEAD'],text=True).strip(),mass_kg=1.758,voltage=22.2,brakes_mm=base[1:].tolist(),trim_throttle=float(base[0]),kick_time_s=2,kick_deg_s=5,description='Standalone truth-rate feedback on current plant; fixed brake trim and base throttle, calm air, no height or lateral controller. Angles are unamplified. Not a replay of the real flight.',cases=summary)
(a.output/'pitch-sign.json').write_text(json.dumps(meta,indent=2))
if a.states_only:sys.exit(0)
plt.rcParams.update({'text.color':'#e6edf3','axes.labelcolor':'#e6edf3','xtick.color':'#b0becb','ytick.color':'#b0becb','axes.edgecolor':'#627080','font.size':10})
fig,axes=plt.subplots(3,4,figsize=(16,8),gridspec_kw={'height_ratios':[2.2,1,1]},facecolor='#0b1520');fig.subplots_adjust(top=.86,bottom=.11,hspace=.36,wspace=.27)
fig.text(.5,.96,'9-DOF pitch-feedback comparison',ha='center',fontsize=18)
fig.text(.5,.92,'Same +5°/s payload pitch-rate disturbance at 2 s • calm air • fixed brake trim • 1.758 kg',ha='center')
fig.text(.5,.03,'Standalone diagnostic, not native AUTO or flight replay. Side views follow the joint; true angles, schematic geometry. Playback 1×.',ha='center',fontsize=10)
artists=[];pitchlim=max(10,np.ceil(max(np.max(np.abs(d[:,1:3])) for d in hist)/5)*5)
for col,((name,g,hz,two),d) in enumerate(zip(cases,hist)):
 ax=axes[0,col];ax.set_facecolor('#0b1520');ax.set_aspect('equal');ax.set_xlim(-1.25,1.25);ax.set_ylim(-1.15,1.15);ax.axis('off');ax.set_title(name,color='#e6edf3',fontsize=14)
 law='ΔT = 0' if not g else ('ΔT = +0.10 qf' if g<0 else 'ΔT = −0.10 qf')
 ax.text(.5,.97,law+'\n'+('2 × 1.5 Hz LPF' if two else '10 Hz LPF' if g else ''),transform=ax.transAxes,ha='center',va='top',fontsize=10)
 ax.axhline(0,color='#44515f',lw=1,ls=':');payload,=ax.plot([],[],color='#ffb46a',lw=7);canopy,=ax.plot([],[],color='#70d8c6',lw=5);lines,=ax.plot([],[],color='#8b9cad',lw=1);label=ax.text(.5,.02,'',transform=ax.transAxes,ha='center',fontsize=10)
 for row,field,lim,ylabel in [(1,1,(-pitchlim,pitchlim),'Pitch (°)'),(2,4,(0,60),'Throttle (%)')]:
  aa=axes[row,col];aa.set_facecolor('#122130');aa.set_xlim(0,duration);aa.set_ylim(*lim);aa.grid(alpha=.15);aa.axvline(2,color='#8393a4',ls=':',lw=1)
  if col==0:aa.set_ylabel(ylabel)
  if row==2:aa.set_xlabel('Time (s)')
 pp,=axes[1,col].plot([],[],color='#ffb46a');cp,=axes[1,col].plot([],[],color='#70d8c6');th,=axes[2,col].plot([],[],color='#a5bfff')
 artists.append((payload,canopy,lines,label,pp,cp,th))
fig.text(.5,.885,'Orange: payload pitch    Green: canopy pitch    qf: filtered payload body pitch rate (rad/s)',ha='center',fontsize=10)
clock=fig.text(.93,.96,'',ha='right')
def update(k):
 idx=min(int(k/25/dt),len(hist[0])-1);clock.set_text(f'{idx*dt:4.1f} s')
 for d,art in zip(hist,artists):
  p,c,l,txt,pp,cp,th=art;theta,can=np.deg2rad(d[idx,1:3]);v=np.array([np.sin(theta),-np.cos(theta)]);center=.55*v;forward=np.array([np.cos(theta),np.sin(theta)])
  pts=np.array([center-.24*forward,center+.24*forward]);p.set_data(pts[:,0],pts[:,1]);wingcenter=np.array([0,.55]);cf=np.array([np.cos(can),np.sin(can)]);pts=np.array([wingcenter-.65*cf,wingcenter+.65*cf]);c.set_data(pts[:,0],pts[:,1]);l.set_data([pts[0,0],0,pts[1,0]],[pts[0,1],0,pts[1,1]])
  txt.set_text(f'Pitch {d[idx,1]:+.1f}°  |  q {d[idx,3]:+.1f}°/s');pp.set_data(d[:idx+1,0],d[:idx+1,1]);cp.set_data(d[:idx+1,0],d[:idx+1,2]);th.set_data(d[:idx+1,0],d[:idx+1,4])
update(500);fig.savefig(a.output/'pitch-sign.jpg',dpi=100)
anim=FuncAnimation(fig,update,frames=int(duration*25)+1,interval=40);anim.save(a.output/'pitch-sign.mp4',writer=FFMpegWriter(fps=25,codec='libx264',extra_args=['-crf','21','-pix_fmt','yuv420p','-movflags','+faststart']))
