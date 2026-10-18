from pathlib import Path
import json,numpy as np
from scipy.spatial.transform import Rotation as Rot
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).parent;O=R.parent;events=json.loads((R/'raw-brake-events.json').read_text())['events'];lag=np.arange(-25,101)*.04;groups={};cache={}
for event in events:
 day=event['day'];start=int(event['time']//30*30);i=round((event['time']-start)/.04)
 if day not in cache:cache[day]=np.load(O/f'{day}-causal-input.npz')
 a=cache[day];ii=np.searchsorted(a['t'],start);physical=a['brake_mm'][ii:ii+751];delta=physical[i+5:i+15].mean(axis=0)-physical[i-15:i-5].mean(axis=0);k=int(np.argmax(abs(delta)))
 if abs(delta[1-k])>2 or not 5<=abs(delta[k])<=20:continue
 kind='pull' if delta[k]>0 else 'release';side='left' if k==0 else 'right';sign=np.sign(delta[1]-delta[0]);truth=a['Y'][ii:ii+751];bt=np.rad2deg(truth[:,0]);tr=[(bt[i-25:i+101]-bt[i-10:i].mean())*sign]
 for name in ['slack12','slack6','slack0']:
  z=np.load(R/f'slack-atlas-{day}-{start}-{name}.npz');bank=Rot.from_quat(z['state'][:,3:7]).as_euler('xyz',degrees=True)[:,0];tr.append((bank[i-25:i+101]-bank[i-10:i].mean())*sign)
 groups.setdefault((day,side,kind),[]).append(tr)
fig,axs=plt.subplots(2,4,figsize=(15,8));summary=[]
for di,day in enumerate(['2026-01-03','2026-02-15']):
 for j,(side,kind) in enumerate([('left','pull'),('left','release'),('right','pull'),('right','release')]):
  ts=groups.get((day,side,kind),[])
  if ts:
   x=np.array(ts)
   for n,name in enumerate(['flight','slack12','slack6','slack0']):axs[di,j].plot(lag,np.median(x[:,n],axis=0),label=name,color='k' if n==0 else None)
   summary.append(dict(day=day,side=side,kind=kind,n=len(ts),median_1s={name:float(np.median(x[:,n,50])) for n,name in enumerate(['flight','slack12','slack6','slack0'])}))
  axs[di,j].set(title=f'{day} {side} {kind}, n={len(ts)}',xlabel='Seconds since detection',ylabel='Signed bank change (deg)');axs[di,j].axvline(0,color='gray',lw=.5);axs[di,j].grid(alpha=.2);axs[di,j].legend(fontsize=7)
fig.suptitle('Fixed raw-PWM event times, single-side5-20mm changes; same events for all slack models; closed-loop diagnostic');fig.tight_layout();fig.savefig(R/'fixed-event-slack.png',dpi=160);(R/'fixed-event-slack.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
