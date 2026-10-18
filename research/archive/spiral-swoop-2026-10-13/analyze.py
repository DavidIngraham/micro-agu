"""Audit and plot native swoop results, including unsuccessful development cases."""
import json,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from pymavlink import mavutil
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
raw=Path('/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/spiral-swoop-2026-10-13')
out=Path(__file__).parent
rows={}
for d in sorted(raw.iterdir()):
 if not d.is_dir() or not (d/'landing.json').exists():continue
 land=json.loads((d/'landing.json').read_text());frames=json.loads((d/'viewer.json').read_text())
 t=np.array([f['time_s'] for f in frames]);s=np.array([f['state'] for f in frames]);a=np.rad2deg(Rotation.from_quat(s[:,3:7]).as_euler('xyz'));heading=np.rad2deg(np.unwrap(np.deg2rad(a[:,2])))
 params={};flare=None;pg=[];common=[]
 log=mavutil.mavlink_connection(str(d/'flight.BIN'))
 while (m:=log.recv_match(type=['PARM','PGFL','PGYR'])) is not None:
  if m.get_type()=='PARM':params[m.Name]=m.Value
  elif m.get_type()=='PGYR':pg.append([m.TimeUS/1e6,m.Tgt,m.Act,m.Out])
  elif m.get_type()=='PGFL':
   common.append([m.TimeUS/1e6,m.Common])
   assert 0<=m.Left<=100 and 0<=m.Right<=100 and 0<=m.Common<=min(land['config'].get('flare_pct',75),100-land['config'].get('flare_reserve',10))+.001
   if m.Active and flare is None:flare=m.TimeUS/1e6
 if land['contact'] is None:continue
 expected=json.loads(Path('/home/david/repos/ardupilot-high-dof/Tools/simulation/paraglider/mission-controller-candidate.json').read_text())['parameters']
 for key,value in expected.items():
  if key=='PG_TURN_RMAX':value=25 if land.get('events') else land['config'].get('turn_rate_limit_dps',25)
  if key=='NAVL1_PERIOD':value=land['config'].get('final_l1_period',10) if land.get('events') else land['config'].get('turn_l1_period',10)
  assert abs(params[key]-value)<1e-5,(d.name,key,params[key],value)
 assert flare is not None
 assert 'PASSED STEP: test.Plane.ParagliderSpiralStudy' in (d/'run.log').read_text()
 end=land['contact']['time']
 left=max((f for f in frames if f['time_s']<=end),key=lambda f:f['time_s']);right=min((f for f in frames if f['time_s']>=end),key=lambda f:f['time_s'])
 ratio=(end-left['time_s'])/(right['time_s']-left['time_s']) if right['time_s']>left['time_s'] else 0
 endpoint=np.array(left['state'])+ratio*(np.array(right['state'])-left['state'])
 for offset in [3,16]:endpoint[offset:offset+4]=Slerp([0,1],Rotation.from_quat([left['state'][offset:offset+4],right['state'][offset:offset+4]]))([ratio]).as_quat()[0]
 endpoint_rotation=Rotation.from_quat(endpoint[3:7]);endpoint_angles=endpoint_rotation.as_euler('xyz',degrees=True)
 joint=endpoint_rotation.inv()*Rotation.from_quat(endpoint[16:20])
 keep=t<=end;t=t[keep];s=s[keep];a=a[keep];heading=heading[keep]
 frames=[f for f,k in zip(frames,keep) if k];ground=np.linalg.norm(s[:,7:9],axis=1);air=np.linalg.norm(s[:,7:10]-np.array([f['wind_ned_m_s'] for f in frames]),axis=1)
 ix=-1;pg=np.array(pg);mask=(t>=flare) if flare else np.zeros(len(t),bool)
 row=dict(miss=land['radial_error_m'],horizontal=float(np.linalg.norm(land['contact']['velocity'][:2])),sink=land['contact']['velocity'][2],bank=float(endpoint_angles[0]),pitch=float(endpoint_angles[1]),heading=float(endpoint_angles[2]),peak_bank=float(max(abs(a[:,0]))),peak_airspeed=float(max(air)),flare_time=flare,flare_height=float(np.interp(flare,t,-s[:,2])) if flare else None,min_flare_sink=float(min(s[mask,9])) if mask.any() else None,turn_sweep=float(heading[-1]-heading[0]),peak_heading_rate=float(np.max(np.abs(pg[:,2]))),saturation_pct=float(np.mean(np.abs(pg[:,3])>=.99)*100),joint_rotvec_deg=np.rad2deg(joint.as_rotvec()).tolist(),joint_rate_dps=np.rad2deg(endpoint_rotation.inv().apply(endpoint[23:26]-endpoint[10:13])).tolist(),body_rate_dps=np.rad2deg(endpoint_rotation.inv().apply(endpoint[10:13])).tolist(),config=land['config'],parameters={k:v for k,v in params.items() if k.startswith(('PG_','NAVL1','LAND_'))})
 common=np.array(common);available=1-np.interp(pg[:,0],common[:,0],common[:,1])/100
 pre=pg[:,0]<flare;post=(pg[:,0]>=flare)&(pg[:,0]<=end)
 row['preflare_command_saturation_pct']=float(np.mean(abs(pg[pre,3])>=.99)*100)
 row['flare_available_command_saturation_pct']=float(np.mean(abs(pg[post,3])>=.98*available[post])*100)
 brake=np.array([f['brake_mm'] for f in frames]);row['physical_travel_limit_pct']=float(np.mean(np.any(brake>=np.array([62.3955,71.9948])*.99,axis=1))*100)
 row['schedule_events']=land.get('events',[])
 event=land['events'][0]['time'] if land.get('events') else end
 turnmask=(t>=t[0]+3)&(t<event)
 finalmask=(t>=event)&(t<flare)
 row['spiral_peak_airspeed']=float(max(air[turnmask])) if turnmask.any() else None
 row['spiral_mean_airspeed']=float(np.mean(air[turnmask])) if turnmask.any() else None
 row['rollout_airspeed']=float(np.interp(event,t,air))
 row['flare_airspeed']=float(np.interp(flare,t,air))
 row['spiral_heading_change']=float(np.interp(event,t,heading)-heading[0])
 row['rollout_height']=float(np.interp(event,t,-s[:,2]))
 row['final_peak_airspeed']=float(max(air[finalmask])) if finalmask.any() else None
 if d.name.startswith('dramatic2') or d.name=='dev08':
  assert len(land['events'])==1
  event_time=land['events'][0]['time']
  assert event_time>t[0]+2
  assert 40<np.interp(event_time,t,heading)<110
 rows[d.name]=row
 if len(sys.argv)>1 and d.name!=sys.argv[1]:continue
 fig,axs=plt.subplots(3,2,figsize=(13,11));target=np.array(land['target_ned_m']);corner=np.array(land['corner_ned_m'])
 axs[0,0].plot(s[:,1]-target[1],s[:,0]-target[0]);axs[0,0].scatter(0,0,marker='x',c='r');axs[0,0].scatter(corner[1]-target[1],corner[0]-target[0],marker='+',c='gray');axs[0,0].set_aspect('equal',adjustable='datalim');axs[0,0].set(xlabel='East of target (m)',ylabel='North of target (m)',title='Ground track: + corner, × target')
 axs[0,1].plot(t-t[0],-s[:,2]);axs[0,1].set(ylabel='Height (m)')
 axs[1,0].plot(t-t[0],air,label='Airspeed');axs[1,0].plot(t-t[0],ground,label='Ground speed');axs[1,0].plot(t-t[0],s[:,9],label='Downward speed');axs[1,0].set(ylabel='m/s');axs[1,0].legend()
 axs[1,1].plot(t-t[0],a[:,0],label='Bank');axs[1,1].plot(t-t[0],a[:,1],label='Pitch');axs[1,1].plot(t-t[0],heading-heading[0],label='Heading change');axs[1,1].set(ylabel='Degrees');axs[1,1].legend()
 axs[2,0].plot(t-t[0],np.array([f['brake_mm'] for f in frames]));axs[2,0].set(ylabel='Applied brake (mm)')
 axs[2,1].plot(t-t[0],[f['throttle'] for f in frames]);axs[2,1].set(ylabel='Throttle')
 for ax in axs.flat:ax.grid(alpha=.2)
 for ax in [axs[0,1],*axs[1],*axs[2]]:
  ax.set_xlabel('Time from release (s)')
  if flare:ax.axvline(flare-t[0],ls='--',color='gray')
 fig.suptitle(f'{d.name}: miss {row["miss"]:.2f} m, horizontal {row["horizontal"]:.2f} m/s, sink {row["sink"]:.2f} m/s')
 fig.tight_layout();fig.savefig(d/'response.png',dpi=130);plt.close(fig)
 print(d.name,{k:round(v,3) if isinstance(v,float) else v for k,v in row.items() if k not in ['config','parameters','joint_rotvec_deg','joint_rate_dps','body_rate_dps']})
(out/'results.json').write_text(json.dumps(rows,indent=2))