from pathlib import Path
import json,numpy as np
from scipy.spatial.transform import Rotation as Rot
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).parent;events=json.loads((R/'brake-events.json').read_text())['events'];lag=np.arange(-25,101)*.04;groups={};records=[]
for event in events:
 day=event['day'];start=int(event['time']//30*30);f=R/f'atlas-{day}-{start}-combined_small.npz'
 if not f.exists():continue
 z=np.load(f);u=z['controls'];i=round((event['time']-start)/.04);delta=(u[i+5:i+15,1:3].mean(axis=0)-u[i-15:i-5,1:3].mean(axis=0))*1000;k=int(np.argmax(abs(delta)))
 if abs(delta[1-k])>2 or not 5<=abs(delta[k])<=20:continue
 kind='pull' if delta[k]>0 else 'release';side='left' if k==0 else 'right';sign=np.sign(delta[1]-delta[0]);truth=z['truth'];y=z['state'];bank=Rot.from_quat(y[:,3:7]).as_euler('xyz',degrees=True)[:,0];bt=np.rad2deg(truth[:,0]);ob=(bt[i-25:i+101]-bt[i-10:i].mean())*sign;mo=(bank[i-25:i+101]-bank[i-10:i].mean())*sign;inp=(u[i-25:i+101,k+1]-u[i-15:i-5,k+1].mean())*1000*np.sign(delta[k]);groups.setdefault((day,kind),[]).append((inp,ob,mo));records.append(event|dict(kind=kind,side=side,side_delta_mm=delta.tolist()))
fig,axs=plt.subplots(2,4,figsize=(14,8));summary=[]
for di,day in enumerate(['2026-01-03','2026-02-15']):
 for ki,kind in enumerate(['pull','release']):
  ts=groups.get((day,kind),[])
  if ts:
   x=np.array(ts)
   axs[di,ki*2].plot(lag,np.median(x[:,0],axis=0),'k');axs[di,ki*2+1].plot(lag,np.median(x[:,1],axis=0),'k',label='flight');axs[di,ki*2+1].plot(lag,np.median(x[:,2],axis=0),label='model');axs[di,ki*2+1].fill_between(lag,*np.percentile(x[:,1],[25,75],axis=0),color='gray',alpha=.2)
   summary.append(dict(day=day,kind=kind,count=len(ts),flight_bank_delta_1s_median=float(np.median(x[:,1,50])),model_bank_delta_1s_median=float(np.median(x[:,2,50]))))
  axs[di,ki*2].set_title(f'{day} {kind}, n={len(ts)}\nCommand magnitude change (mm)');axs[di,ki*2+1].set_title('Direction-aligned bank change (deg)');axs[di,ki*2+1].legend()
 for a in axs[di]:a.set_xlabel('Seconds from detection');a.axvline(0,color='gray',lw=.5);a.grid(alpha=.2)
fig.suptitle('Single-side 5–20mm changes, pull versus release; closed-loop association, not controlled excitation');fig.tight_layout();fig.savefig(R/'brake-events-separated.png',dpi=160);(R/'brake-events-separated.json').write_text(json.dumps(dict(summary=summary,events=records),indent=2));print(json.dumps(summary,indent=2))
