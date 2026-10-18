"""Matched +5 m altitude command; standalone truth-sensed controller diagnostic."""
import argparse,json,sys,subprocess
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
p=argparse.ArgumentParser();p.add_argument('--model',type=Path,required=True);p.add_argument('--raw',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
sys.path.insert(0,str(a.model));from json_backend import Backend
from controllers import Turn
for path in [a.raw,a.output]:path.mkdir(parents=True,exist_ok=True)
cases=[('Damper off',0,10,False),('Original sign',.1,10,False),('Reversed sign',-.1,10,False),('Current filter',.1,1.5,True)]
rows=[];dt=.02
for name,g,hz,two in cases:
 b=Backend();b.sim.substeps=16;b.released=True;base=b.command.copy();q1=q2=0.;integral=base[0]-.45;last=base[0];vf=0;kv=.6/35;frames=[];trace=[]
 lat=Turn(dt=dt,p=.005,i=.002,rdamp=.003,dff=0,filter_hz=4);lat.integral=base[2]/(np.pi*55/360*.1*1500)
 for j in range(2501):
  t=j*dt;y=b.state;rot=Rotation.from_quat(y[3:7]);phi,theta,_=rot.as_euler('xyz');rates=rot.inv().apply(y[10:13]);height=-y[2];climb=-y[9];target=100 if t<5 else 105
  vdes=np.clip((target-height)/10,-2.5,1);vf+=dt/(.5+dt)*(vdes-vf);err=vdes-climb
  ff=.45+vf*(.15 if vf>=0 else .18);pd=np.clip(ff+kv*err,0,.6);pd=np.clip(pd,last-.6*dt,last+.6*dt);last=pd
  integral=np.clip(integral+.3*kv*err*dt,np.clip(-pd-.1,-.3,.3),np.clip(.6-pd+.1,-.3,.3))
  alpha=dt/(dt+1/(2*np.pi*hz));q1+=alpha*(rates[1]-q1);q2+=alpha*(q1-q2)
  throttle=float(np.clip(pd+integral+np.clip(-(g if t>=5 else 0)*(q2 if two else q1),-.25,.25),0,.6))
  speed=np.linalg.norm(y[7:10]);sv=np.clip(5/max(speed,2),.25,2.5);heading=(rates[1]*np.sin(phi)+rates[2]*np.cos(phi))/np.cos(theta)*180/np.pi
  lat.feedforward=lambda r:.009*r*sv;u=lat.update(0,heading,rates[0]*180/np.pi,speed)
  cmd=np.array([throttle,max(-u,0)*np.pi*55/360*.1*1300,max(u,0)*np.pi*55/360*.1*1500])
  b.time=t;b.command=cmd.copy();f=b.viewer_state();f.update(target_altitude_m=target,payload_pitch_deg=float(theta*180/np.pi),canopy_pitch_deg=float(Rotation.from_quat(y[16:20]).as_euler('xyz')[1]*180/np.pi));frames.append(f)
  trace.append([t,height,target,climb,f['payload_pitch_deg'],f['canopy_pitch_deg'],throttle,rates[1]*180/np.pi])
  b.state,b.rpm,_=b.sim.step_actuated(y,b.rpm,b.actuator,cmd,voltage=22.2,dt=dt)
  if not np.isfinite(b.state).all():raise RuntimeError(name+' nonfinite')
 (a.raw/(name.replace(' ','-')+'.jsonl')).write_text(''.join(json.dumps(f)+'\n' for f in frames));d=np.array(trace);np.savez_compressed(a.raw/(name.replace(' ','-')+'.npz'),trace=d)
 result=dict(case=name,gain=g,filter_hz=hz,stages=2 if two else 1,final_altitude_m=float(d[-1,1]),altitude_minmax_m=[float(d[:,1].min()),float(d[:,1].max())],pitch_rate_rms_last_10s=float(np.sqrt(np.mean(d[d[:,0]>=40,7]**2))),throttle_saturation_fraction=float(np.mean(d[d[:,0]>=5,6]>=.5999)));rows.append(result);print(result,flush=True)
(a.output/'altitude-comparison.json').write_text(json.dumps(dict(model_commit=subprocess.check_output(['git','-C',str(a.model),'rev-parse','HEAD'],text=True).strip(),description='Standalone height-loop algebra with truth sensing, not native SITL. Same lateral controller, initial trim, 22.2 V, 1.758 kg, calm air. Height step 100 to 105 m at 5 s. Pitch damping enabled at that instant in all nonzero cases. No artificial pitch kick.',height_time_constant_s=10,feedforward_filter_s=.5,climb_limit_m_s=1,sink_limit_m_s=2.5,throttle_limit=.6,cases=rows),indent=2))
