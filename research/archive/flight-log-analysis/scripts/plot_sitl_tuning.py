from pathlib import Path
import json,datetime
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path('/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/longitudinal-2026-10-09');O=R/'system-identification'
d=json.loads((R/'telemetry/SITL__FIXED_WING__1__2026-02-14 13-43-43.tlog.json').read_text())
lo=datetime.datetime.fromisoformat('2026-02-15T11:20:00-08:00').timestamp();hi=lo+1800
at=[x for x in d['series']['ATTITUDE'] if lo<=x['t']<=hi];t=np.array([x['t'] for x in at]);roll=np.rad2deg([x['roll'] for x in at]);yaw=np.rad2deg(np.unwrap([x['yaw'] for x in at]));rate=(np.interp(t+1,t,yaw)-np.interp(t-1,t,yaw))/2
fig,ax=plt.subplots(4,1,figsize=(12,9),sharex=True,layout='constrained')
for k,label in [('RLL_RATE_FF','FF'),('RLL_RATE_D_FF','D_FF'),('RLL2SRV_TCONST','Angle time constant (s)')]:
 p=d['params'][k];before=[x for x in p if x['t']<=lo];sel=([dict(t=lo,value=before[-1]['value'])] if before else [])+[x for x in p if lo<x['t']<=hi];sel.append(dict(t=hi,value=sel[-1]['value']))
 ax[0].step([(x['t']-lo)/60 for x in sel],[x['value'] for x in sel],where='post',label=label)
 print(k,sel)
ax[0].set_ylabel('Parameter value');ax[0].legend(ncol=3,fontsize=9)
ax[1].plot((t-lo)/60,roll,lw=.7);ax[1].set_ylabel('Simulated bank (deg)')
ax[2].plot((t-lo)/60,rate,lw=.7);ax[2].set_ylabel('Heading rate (deg/s)')
mo=[x for x in d['modes'] if x['t']<=hi];before=[x for x in mo if x['t']<=lo];mo=([dict(before[-1],t=lo)] if before else [])+[x for x in mo if x['t']>lo];mo.append(dict(mo[-1],t=hi))
ax[3].step([(x['t']-lo)/60 for x in mo],[x['mode'] for x in mo],where='post');ax[3].set_yticks([0,10,15],['MANUAL','AUTO','GUIDED']);ax[3].set_ylabel('Mode');ax[3].set_xlabel('Minutes after 11:20 a.m. PST, February 15')
for a in ax:a.grid(alpha=.2)
fig.suptitle('Simulation tuning on the morning before Trout Lake')
fig.savefig(O/'sitl-lateral-tuning.png',dpi=160)
(O/'sitl-selected-attitude.json').write_text(json.dumps(at))
print('modes',mo)
