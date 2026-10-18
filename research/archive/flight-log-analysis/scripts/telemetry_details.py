from pathlib import Path
import json,numpy as np,datetime
from zoneinfo import ZoneInfo
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).parent
out=[]
for fn in ['FIXED_WING__1__2026-02-22 14-53-24.tlog.json','FIXED_WING__1__2026-02-15 15-28-24.tlog.json','FIXED_WING__1__2026-01-03 14-46-27.tlog.json']:
 d=json.loads((R/'telemetry'/fn).read_text());gp=d['series'].get('GLOBAL_POSITION_INT',[]);at=d['series'].get('ATTITUDE',[]);vf=d['series'].get('VFR_HUD',[])
 if not gp:continue
 t=np.array([x['t'] for x in gp]);h=np.array([x['relative_alt']/1000 for x in gp]);sp=np.array([np.hypot(x['vx'],x['vy'])/100 for x in gp]);boot=np.array([x['time_boot_ms']/1000 for x in gp]);t0=t[0];active=(h>3)&(sp>2)
 ix=np.flatnonzero(np.diff(np.r_[False,active,False]));intervals=[]
 for a,b in zip(ix[::2],ix[1::2]):
  if t[b-1]-t[a]<10:continue
  intervals.append(dict(start=datetime.datetime.fromtimestamp(t[a],ZoneInfo('America/Los_Angeles')).isoformat(),end=datetime.datetime.fromtimestamp(t[b-1],ZoneInfo('America/Los_Angeles')).isoformat(),boot_start=float(boot[a]),boot_end=float(boot[b-1]),duration=float(t[b-1]-t[a]),hmax=float(max(h[a:b]))))
 key=fn.replace('.tlog.json','');fig,ax=plt.subplots(4,1,figsize=(14,10),sharex=True,layout='constrained');ax[0].plot((t-t0)/60,h);ax[0].set_ylabel('Relative altitude (m)');ax[1].plot((t-t0)/60,sp);ax[1].set_ylabel('Groundspeed (m/s)')
 ax[2].plot([(x['t']-t0)/60 for x in at],[x['pitch']*180/np.pi for x in at],lw=.6,label='Pitch');ax[2].set_ylabel('Pitch (deg)');ax[3].plot([(x['t']-t0)/60 for x in vf],[x['throttle'] for x in vf],lw=.6);ax[3].set_ylabel('Throttle (%)');ax[3].set_xlabel('Minutes from first position message')
 for x in ax:x.grid(alpha=.2)
 fig.suptitle(d['file']+' | telemetry coverage; gaps may hide activity');fig.savefig(R/(key+'.png'),dpi=130);plt.close(fig)
 r=dict(file=d['file'],intervals=intervals,parameter_changes={k:v for k,v in d['params'].items() if (k.startswith('TECS_') or k in ['THR_MAX','THR_SLEWRATE','TRIM_THROTTLE']) and len(v)>1},events=[x for x in d['events'] if any(s in x.get('text','').lower() for s in ['crash','failsafe','throttle','reached','mission','ekf'])])
 out.append(r);print(json.dumps({k:v for k,v in r.items() if k!='events'}),flush=True)
(R/'telemetry-flight-details.json').write_text(json.dumps(out,indent=2))
