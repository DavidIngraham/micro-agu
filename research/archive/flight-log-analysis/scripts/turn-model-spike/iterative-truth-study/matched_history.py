from pathlib import Path
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).parent;O=R.parent;a=np.load(O/'2026-02-15-causal-input.npz');b=np.load(O/'2026-02-15.npz');prop=np.load(O/'2026-02-15-propulsion-9x45.npz');fig,ax=plt.subplots(4,2,figsize=(14,11));out=[]
for col,start in enumerate([840,2910]):
 i=np.searchsorted(a['t'],start-20);j=i+1251;sl=slice(i,j);t=a['t'][sl]-start;y=a['Y'][sl];u=a['U'][sl];mm=u[:,1:]*72;chi=np.unwrap(np.arctan2(y[:,7],y[:,6]));conv=.04799655443
 ax[0,col].plot(t,800+mm[:,0]/conv,label='C3 PWM');ax[0,col].plot(t,2400-mm[:,1]/conv,label='C4 PWM');ax[0,col].set_ylabel('PWM us')
 ax[1,col].plot(t,mm[:,0],label='left');ax[1,col].plot(t,mm[:,1],label='right');ax[1,col].set_ylabel('Brake mm')
 ax[2,col].plot(t,u[:,0],label='throttle');ax[2,col].set_ylabel('Throttle')
 ax[3,col].plot(t,np.rad2deg(chi-chi[500]),label='ground course');ax[3,col].set_ylabel('Course deg');ax[3,col].set_xlabel('Seconds relative to scored segment')
 for aa in ax[:,col]:aa.axvspan(-20,0,color='gray',alpha=.1);aa.legend();aa.grid(alpha=.2)
 ax[0,col].set_title(f'TL {start}s, with 20s history (PWM reconstructed from audited mapping)')
 v=mm[500:];out.append(dict(start=start,brake_quantiles=np.percentile(v,[25,50,75,95],axis=0).tolist(),duty_above10mm=(v>10).mean(axis=0).tolist(),neutral_duty=(v<1).mean(axis=0).tolist(),effective_means={str(d):np.maximum(v-d,0).mean(axis=0).tolist() for d in [0,2,5,10]},mode_values=np.unique(np.interp(a['t'][sl],b['t'],b['mode'])).tolist()))
params=json.loads((O.parent/'2026-02-15 15-38-50/PARM.json').read_text());last={};changes=[];servo={}
for r in params:
 name=r['Name'];val=r['Value']
 if name.startswith(('SERVO1_','SERVO3_','SERVO4_')):servo.setdefault(name,[]).append(dict(time=r['TimeUS']/1e6,value=val))
 if name in last and val!=last[name]:changes.append(dict(name=name,time=r['TimeUS']/1e6,old=last[name],new=val))
 last[name]=val
(R/'matched-history-audit.json').write_text(json.dumps(dict(episodes=out,servo_parameter_records=servo,changed_parameters=changes),indent=2));fig.tight_layout();fig.savefig(R/'matched-history-audit.png',dpi=140);print(json.dumps(dict(episodes=out,changes=changes),indent=2))
