from study import OUT,RAW
from pymavlink import mavutil
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
rows=[]
for path in sorted((OUT/'configs').glob('*.json')):
 name=path.stem;p=RAW/name
 if not (p/'landing.json').exists():continue
 r=json.loads((p/'landing.json').read_text());frames=json.loads((p/'viewer.json').read_text());t=np.array([x['time_s'] for x in frames]);y=np.array([x['state'] for x in frames]);log=mavutil.mavlink_connection(str(p/'flight.BIN'));first=None;land=[]
 while True:
  m=log.recv_match(type=['PGFL','LAND'])
  if m is None:break
  d=m.to_dict()
  if m.get_type()=='LAND':land.append(d)
  elif first is None and m.Active and m.Common>0:first=d
 ft=first['TimeUS']*1e-6;at=np.array([np.interp(ft,t,y[:,k]) for k in range(26)]);target=np.array(r['target_ned_m']);corner=np.array(r['corner_ned_m']);direction=target[:2]-corner[:2];direction/=np.linalg.norm(direction)
 row=dict(name=name,miss=r['radial_error_m'],horizontal=float(np.linalg.norm(r['contact']['velocity'][:2])),sink=r['contact']['velocity'][2],flare_time=ft,actual_flare_height=float(-at[2]),configured_flare_height=r['config']['flare_alt_m'],past_target_at_flare_m=float((at[:2]-target[:2])@direction),flare_entry_speed=float(np.linalg.norm(at[7:10])),flare_duration=r['contact']['time']-ft,returncode=json.loads((p/'completed.json').read_text())['returncode']);rows.append(row)
(OUT/'native-results.json').write_text(json.dumps(rows,indent=2));print(json.dumps(rows,indent=2))
fig,axs=plt.subplots(1,3,figsize=(14,5),layout='constrained');labels=[x['name'] for x in rows]
for ax,key,label in zip(axs,['horizontal','sink','miss'],['Touchdown horizontal speed (m/s)','Touchdown sink (m/s)','Target miss (m)']):
 ax.barh(labels,[x[key] for x in rows]);ax.set_xlabel(label);ax.invert_yaxis();ax.grid(axis='x',alpha=.2)
fig.suptitle('Native AUTO: actual approach and flare onset vary between runs');fig.savefig(OUT/'native-results.png',dpi=140);plt.close(fig)
