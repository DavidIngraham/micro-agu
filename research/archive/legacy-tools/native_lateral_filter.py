"""Matched native 4/1 Hz mission comparison; no controller or blog changes."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.signal import butter, sosfiltfilt
from pymavlink import mavutil
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import argparse
p=argparse.ArgumentParser();p.add_argument('root',type=Path);p.add_argument('--output',type=Path,required=True);args=p.parse_args();root=args.root;out=args.output;out.mkdir(parents=True,exist_ok=True)
results={}
fig,axs=plt.subplots(3,3,figsize=(15,11),layout='constrained')
rms=lambda x:float(np.sqrt(np.mean(np.asarray(x)**2)))
for ci,condition in enumerate(['calm','gust','crosswind']):
 for hz,color in [(4,'#da7434'),(1,'#237cac')]:
  run=root/f'{condition}-{hz}'
  if not (run/'auto-mission.json').exists():continue
  mission=json.loads((run/'ParagliderAutoMission.csv.json').read_text())
  csv=np.genfromtxt(run/'ParagliderAutoMission.csv',delimiter=',',names=True)
  frames=[json.loads(s) for s in (run/'truth.jsonl').read_text().splitlines()]
  t=np.array([f['time_s'] for f in frames]); y=np.array([f['state'] for f in frames])
  euler=Rotation.from_quat(y[:,3:7]).as_euler('xyz')
  rate=np.gradient(np.unwrap(euler[:,2]),t)*180/np.pi
  bank=euler[:,0]*180/np.pi
  throttle=np.array([f['throttle'] for f in frames])*100
  log=mavutil.mavlink_connection(str(run/'flight.BIN')); pg=[];params={}
  while True:
   m=log.recv_match(type=['PGYR','PARM'])
   if m is None:break
   if m.get_type()=='PARM':params[m.Name]=float(m.Value)
   else:pg.append([m.TimeUS*1e-6,m.Tgt,m.Act,m.Out])
  log.close();pg=np.array(pg)
  assert mission['turn_controller_parameters']['PG_TURN_FILT']==hz
  assert params['PG_TURN_FILT']==hz
  expected=json.loads((root/f'filter-{hz}.json').read_text())['parameters']
  for key,value in expected.items():assert np.isclose(params[key],value,rtol=1e-6,atol=1e-6),(key,params[key],value)
  r={'test_passed':'PASSED STEP: test.Plane.ParagliderAutoMission' in (run/'run.log').read_text(),'filter_hz':hz,'loiters':{}}
  r['max_waypoint_height_error_m']=max(abs(v['altitude_at_closest_m']-mission['route'][int(k)-1][3]) for k,v in mission['metrics'].items() if v['altitude_at_closest_m'] is not None)
  r['max_corner_overshoot_m']=max(v['next_track_overshoot_m'] or 0 for v in mission['metrics'].values())
  r['mission_end_s']=float(csv['time_s'].max())
  for seq in [3,8]:
   rows=csv[csv['seq']==seq];near=rows[rows['target_distance_m']<60];start=float(near['time_s'][0])+20;end=float(rows['time_s'][-1]);s=(t>=start)&(t<=end);p=(pg[:,0]>=start)&(pg[:,0]<=end)
   target=mission['route'][seq-1][3]; radius=abs(mission['loiter_radius_m'][str(seq)])
   cr=rows[rows['time_s']>=start]
   out=pg[p,3];dt=np.median(np.diff(pg[p,0]));fast=sosfiltfilt(butter(3,1,fs=1/dt,btype='highpass',output='sos'),out)
   truth=np.interp(pg[p,0],t,rate)
   err=truth-pg[p,1]; eh=sosfiltfilt(butter(3,1,fs=1/dt,btype='highpass',output='sos'),err); el=sosfiltfilt(butter(3,1,fs=1/dt,btype='lowpass',output='sos'),err)
   r['loiters'][str(seq)]={'duration_s':end-start,'radial_error_rms_m':rms(cr['target_distance_m']-radius),'height_error_rms_m':rms(-y[s,2]-target),'truth_heading_rate_error_rms_deg_s':rms(err),'truth_rate_error_above_1hz_rms_deg_s':rms(eh),'truth_rate_error_below_1hz_rms_deg_s':rms(el),'truth_heading_rate_mean_deg_s':float(truth.mean()),'target_heading_rate_mean_deg_s':float(pg[p,1].mean()),'bank_std_deg':float(bank[s].std()),'brake_output_std':float(out.std()),'brake_output_above_1hz_rms':rms(fast),'output_saturation_pct':float(np.mean(abs(out)>=.99)*100),'throttle_std_pct':float(throttle[s].std())}
   if seq==3:
    plot=(t>=start-20)&(t<=min(end,start+75));ps=(pg[:,0]>=start-20)&(pg[:,0]<=min(end,start+75));tx=t[plot]-(start-20)
    axs[ci,0].plot(tx,rate[plot],color=color,label=f'{hz} Hz',lw=.8)
    axs[ci,1].plot(pg[ps,0]-(start-20),pg[ps,3],color=color,lw=.8)
    axs[ci,2].plot(tx,-y[plot,2]-target,color=color,lw=.8)
  results[f'{condition}-{hz}']=r
 for j,label in enumerate(['Truth heading rate (deg/s)','Differential brake command','Height error (m)']):
  axs[ci,j].set_title(condition+' · '+label);axs[ci,j].grid(alpha=.25);axs[ci,j].set_xlabel('Seconds from first-loiter approach')
 axs[ci,0].legend()
fig.suptitle('Native AUTO: 4 Hz versus 1 Hz measurement filtering\nMatched gains, 1.758 kg Trout Lake model, 22.2 V; independently aligned loiter entry')
fig.savefig(out/'comparison.png',dpi=150)
(out/'results.json').write_text(json.dumps(results,indent=2))
print(json.dumps(results,indent=2))
rows=[]
for name,r in results.items():
 for seq,m in r['loiters'].items():
  cells=[name,'right' if seq=='3' else 'left',f"{m['radial_error_rms_m']:.2f}",f"{m['height_error_rms_m']:.2f}",f"{m['truth_heading_rate_error_rms_deg_s']:.2f}",f"{100*m['brake_output_above_1hz_rms']:.2f}",f"{m['output_saturation_pct']:.2f}"]
  rows.append('<tr>'+''.join('<td>'+c+'</td>' for c in cells)+'</tr>')
html='''<!doctype html><meta charset="utf-8"><title>Native lateral filter trial</title><style>body{max-width:1200px;margin:40px auto;background:#101923;color:#e3edf5;font:17px system-ui}img{width:100%}table{border-collapse:collapse;width:100%}td,th{padding:10px;border-bottom:1px solid #456;text-align:right}a{color:#87cafa}</style><h1>Native lateral filter trial: 4 Hz versus 1 Hz</h1><p>Same native controller gains, estimator, navigation and 9-DOF Trout Lake plant. Only PG_TURN_FILT changes. This filters both heading-rate feedback and roll-rate damping; feedforward bypasses it.</p><img src="comparison.png"><h2>Settled loiters</h2><table><tr><th>Case</th><th>Direction</th><th>Radial RMS (m)</th><th>Height RMS (m)</th><th>Truth rate error RMS (deg/s)</th><th>Brake above 1 Hz RMS (%)</th><th>Saturation (%)</th></tr>'''+''.join(rows)+'''</table><p>Windows start 20 s after coming within 60 m of the loiter centre. Truth rate error uses unfiltered payload heading rate, not PGYR.Act. One run per condition: repeatable gust forcing, but arrival phase changes with the trajectory. These results do not establish flight-validated robustness margins.</p><p><a href="results.json">Full results</a> · <a href="README.md">Method</a> · <a href="analyze.py">Analysis script</a></p>'''
(out/'report.html').write_text(html)
fig2,ax2=plt.subplots(1,3,figsize=(14,4.5),layout='constrained')
labels=[]
for condition in ['calm','gust','crosswind']:
 for seq in ['3','8']:
  if all(f'{condition}-{hz}' in results for hz in (4,1)):labels.append((condition,seq))
for j,(key,title,scale) in enumerate([('truth_heading_rate_error_rms_deg_s','Truth heading-rate RMS error (deg/s)',1),('radial_error_rms_m','Radial RMS error (m)',1),('brake_output_above_1hz_rms','Brake command above 1 Hz RMS (%)',100)]):
 for hz,color,offset in [(4,'#da7434',-.18),(1,'#237cac',.18)]:
  ax2[j].bar(np.arange(len(labels))+offset,[results[f'{c}-{hz}']['loiters'][q][key]*scale for c,q in labels],width=.36,color=color,label=f'{hz} Hz')
 ax2[j].set_xticks(np.arange(len(labels)),[c+'\n'+('right' if q=='3' else 'left') for c,q in labels]);ax2[j].set_title(title);ax2[j].grid(axis='y',alpha=.25)
ax2[0].legend();fig2.suptitle('Native AUTO settled loiters: measured tradeoffs');fig2.savefig(out/'metrics.png',dpi=160)
p=out/'report.html';p.write_text(p.read_text().replace('<h2>Settled loiters</h2>','<img src="metrics.png"><h2>Settled loiters</h2>'))