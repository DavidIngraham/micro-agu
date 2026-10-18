"""Audit matched landing controller runs and publish bounded comparison assets.

Raw logs remain under --root/--previous; only figures, summaries and replay excerpts are exported.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from pymavlink import mavutil
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--root',type=Path,required=True)
p.add_argument('--previous',type=Path,required=True)
p.add_argument('--output',type=Path,required=True)
a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
expected=json.loads((a.root/'mission-lateral-1hz.json').read_text())['parameters']
cache={}
def read(directory):
 landing=json.loads((directory/'landing.json').read_text())
 frames=json.loads((directory/'viewer.json').read_text())
 log=mavutil.mavlink_connection(str(directory/'flight.BIN'));pg=[];flare=None;params={};active=0
 while True:
  msg=log.recv_match(type=['PARM','PGYR','PGFL'])
  if msg is None:break
  if msg.get_type()=='PARM':params[msg.Name]=msg.Value
  elif msg.get_type()=='PGYR':pg.append([msg.TimeUS*1e-6,msg.Tgt,msg.Act,msg.Out,msg.RD])
  elif msg.Active:
   active+=1
   if flare is None:flare=msg.TimeUS*1e-6
   assert 0<=msg.Common<=60.001 and 0<=msg.Left<=100 and 0<=msg.Right<=100
 assert flare is not None and active
 trace=np.array(pg);trace=trace[(trace[:,0]>=flare-20)&(trace[:,0]<flare)]
 subset=[f for f in frames if flare-20<=f['time_s']<flare]
 times=np.array([f['time_s'] for f in subset]);states=np.array([f['state'] for f in subset])
 rot=Rotation.from_quat(states[:,3:7]);angles=rot.as_euler('xyz');rates=rot.inv().apply(states[:,10:13])
 truth=np.rad2deg((rates[:,1]*np.sin(angles[:,0])+rates[:,2]*np.cos(angles[:,0]))/np.cos(angles[:,1]))
 target=np.interp(times,trace[:,0],trace[:,1])
 result=dict(miss_m=landing['radial_error_m'],sink_m_s=landing['contact']['velocity'][2],groundspeed_m_s=float(np.linalg.norm(landing['contact']['velocity'][:2])),truth_rate_rmse_deg_s=float(np.sqrt(np.mean((target-truth)**2))),saturation_pct=float(np.mean(abs(trace[:,3])>=.98)*100),command_travel_per_s=float(abs(np.diff(trace[:,3])).sum()/(trace[-1,0]-trace[0,0])),parameters={k:v for k,v in params.items() if k.startswith(("PG_TURN_","TECS_","LAND_","PG_FLARE"))},flare_time_s=flare,flare_samples=active,log_sha256=hashlib.sha256((directory/'flight.BIN').read_bytes()).hexdigest())
 cache[str(directory)]=(landing,frames,trace,times,truth,result)
 return result
rows=[]
for trial in range(12):
 before=a.previous/f'moderate-{trial:02d}';after=a.root/f'landing-1hz-{trial:02d}'
 old=read(before);new=read(after)
 assert 'PASSED STEP: test.Plane.ParagliderLandingStudy' in (after/'run.log').read_text()
 for key,value in expected.items():assert abs(new['parameters'][key]-value)<1e-5,(trial,key)
 for key,value in old['parameters'].items():
  if key.startswith(('TECS_','LAND_','PG_FLARE')):assert new['parameters'][key]==value,(trial,key)
 assert cache[str(before)][0]['config']==cache[str(after)][0]['config']
 rows.append(dict(trial=trial,before=old,after=new))
errors=np.array([r['after']['miss_m'] for r in rows])
summary=dict(attempted=12,completed=12,within_5m=int(sum(errors<=5)),median_m=float(np.median(errors)),p90_m=float(np.percentile(errors,90)),worst_m=float(max(errors)),median_sink_m_s=float(np.median([r['after']['sink_m_s'] for r in rows])),median_groundspeed_m_s=float(np.median([r['after']['groundspeed_m_s'] for r in rows])),parameters=expected,selection='1 Hz selected before inspecting trials 4–11; trials 0–3 used for diagnosis',trials=rows)
(a.output/'results.json').write_text(json.dumps(summary,indent=2))
fig,axes=plt.subplots(1,3,figsize=(14,4.3))
for key,label,color in [('before','Original landing preset','#cc7444'),('after','Mission gains + 1 Hz','#218c9b')]:
 for ax,metric in zip(axes,['truth_rate_rmse_deg_s','saturation_pct','command_travel_per_s']):ax.plot(range(12),[r[key][metric] for r in rows],'o-',color=color,label=label)
for ax,title in zip(axes,['Truth heading-rate RMSE (deg/s)','Full differential command (%)','Command total variation / second']):
 ax.set(title=title,xlabel='Matched condition index');ax.axvline(3.5,color='.6',ls='--');ax.grid(alpha=.2)
axes[0].legend();fig.suptitle('Final 20 seconds before flare · dashed line separates development and reserved cases')
fig.tight_layout();fig.savefig(a.output/'landing-control-comparison.png',dpi=150);plt.close(fig)
fig,axes=plt.subplots(1,2,figsize=(11,4.5))
for radius in [1,2,5]:axes[0].add_patch(plt.Circle((0,0),radius,fill=False,color='.75'))
for row in rows:
 land=cache[str(a.root/f"landing-1hz-{row['trial']:02d}")][0]
 axes[0].scatter(land['cross_track_error_m'],land['along_track_error_m'],color='#218c9b')
axes[0].set_aspect('equal',adjustable='datalim');axes[0].set(xlabel='Cross-track error (m)',ylabel='Along-track error (m)',title='Selected configuration: 12 landings')
axes[1].scatter(errors,[r['after']['sink_m_s'] for r in rows],color='#218c9b');axes[1].set(xlabel='Distance from target (m)',ylabel='Sink speed at crossing (m/s)',title='Placement and arrival speed');axes[1].grid(alpha=.2)
fig.tight_layout();fig.savefig(a.output/'landing-final.png',dpi=150);plt.close(fig)
# The previously published trial, fixed before this tuning work, prevents cherry-picking.
trial=1
fig,axes=plt.subplots(3,1,figsize=(11,9),sharex=True)
for key,directory,label in [('before',a.previous/'moderate-01','Original landing preset'),('after',a.root/'landing-1hz-01','Mission gains + 1 Hz')]:
 land,frames,trace,times,truth,result=cache[str(directory)];flare=result['flare_time_s'];end=land['contact']['time']
 axes[0].plot(times-flare,truth,label=label);axes[0].plot(trace[:,0]-flare,trace[:,1],ls='--',alpha=.5)
 axes[1].plot(trace[:,0]-flare,trace[:,3],label=label)
 axes[2].plot(trace[:,0]-flare,trace[:,4],label=label)
 left=max((f for f in frames if f['time_s']<=end),key=lambda f:f['time_s']);right=min((f for f in frames if f['time_s']>=end),key=lambda f:f['time_s'])
 fraction=(end-left['time_s'])/(right['time_s']-left['time_s']) if right['time_s']>left['time_s'] else 0
 endpoint=dict(left);endpoint['time_s']=end
 state=np.array(left['state'])+fraction*(np.array(right['state'])-left['state'])
 for offset in [3,16]:state[offset:offset+4]=Slerp([0,1],Rotation.from_quat([left['state'][offset:offset+4],right['state'][offset:offset+4]]))([fraction]).as_quat()[0]
 endpoint['state']=state.tolist()
 for k in ['rpm','throttle']:endpoint[k]=left[k]+fraction*(right[k]-left[k])
 endpoint['brake_mm']=(np.array(left['brake_mm'])+fraction*(np.array(right['brake_mm'])-left['brake_mm'])).tolist()
 clipped=[f for f in frames if end-22<=f['time_s']<end]+[endpoint]
 replay=dict(frames=clipped,target_ned_m=land['target_ned_m'],result=land,selected_trial=trial,selection='Previously published trial 1, fixed before tuning',label=('Before · original landing preset' if key=='before' else 'After · mission gains, 1 Hz feedback'),flare_time_s=flare,flare_event_source='First active native PGFL sample; SITL simulation timestamp',flare_log_sha256=result['log_sha256'])
 (a.output/('replay-before.json' if key=='before' else 'replay.json')).write_text(json.dumps(replay,separators=(',',':')))
for ax,label in zip(axes,['Truth heading rate (deg/s)','Differential brake command','Roll damping contribution']):ax.set_ylabel(label);ax.grid(alpha=.25);ax.legend()
axes[0].set_title('Previously published landing, matched conditions · dashed lines: heading-rate target');axes[-1].set_xlabel('Seconds before native flare entry')
fig.tight_layout();fig.savefig(a.output/'matched-response.png',dpi=150)
print({k:v for k,v in summary.items() if k not in ['trials','parameters']})
