from pathlib import Path
import json,numpy as np
from scipy.spatial.transform import Rotation as Rot
from scipy.ndimage import gaussian_filter1d
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).parent;rows=json.loads((R/'atlas-results.json').read_text());events=[];fig,axs=plt.subplots(2,4,figsize=(15,8));lag=np.arange(-25,101)*.04
for di,day in enumerate(['2026-01-03','2026-02-15']):
 traces=[]
 for row in [r for r in rows if r['day']==day and r['variant']=='combined_small']:
  z=np.load(R/f"atlas-{day}-{int(row['start'])}-combined_small.npz");truth=z['truth'];u=z['controls'];y=z['state'];diff=(u[:,2]-u[:,1])*1000
  change=diff-np.r_[np.repeat(diff[0],10),diff[:-10]];candidates=np.where((abs(change)>5)&(np.arange(len(diff))>50)&(np.arange(len(diff))<len(diff)-101))[0];last=-999
  rot=Rot.from_quat(y[:,3:7]);pb=rot.as_euler('xyz',degrees=True)[:,0];pr=rot.inv().apply(y[:,10:13])[:,0]*180/np.pi
  observed=np.c_[truth[:,0]*180/np.pi,truth[:,3]*180/np.pi];modeled=np.c_[pb,pr]
  for i in candidates:
   if i-last<125:continue
   if np.ptp(u[i-10:i+26,0])>.05:continue
   before=diff[i-15:i-5].mean();after=diff[i+5:i+15].mean();delta=after-before
   if abs(delta)<5:continue
   if np.std(diff[i-20:i-5])>2 or np.std(diff[i+5:i+20])>3:continue
   sign=np.sign(delta);sl=slice(i-25,i+101);ob=(observed[sl]-observed[i-10:i, :].mean(axis=0))*sign;mo=(modeled[sl]-modeled[i-10:i,:].mean(axis=0))*sign;inp=(diff[sl]-before)*sign
   traces.append((inp,ob,mo));events.append(dict(day=day,time=float(row['start']+i*.04),delta_mm=float(delta),before_mm=float(before),after_mm=float(after),throttle_range=float(np.ptp(u[i-10:i+26,0]))));last=i
 if traces:
  inp=np.array([x[0] for x in traces]);ob=np.array([x[1] for x in traces]);mo=np.array([x[2] for x in traces]);axs[di,0].plot(lag,inp.T,color='gray',alpha=.12);axs[di,0].plot(lag,np.median(inp,axis=0),'k',lw=2)
  for j in range(2):
   axs[di,j+1].plot(lag,np.median(ob[:,:,j],axis=0),'k',label='flight median');axs[di,j+1].fill_between(lag,*np.percentile(ob[:,:,j],[25,75],axis=0),color='gray',alpha=.2);axs[di,j+1].plot(lag,np.median(mo[:,:,j],axis=0),label='model median');axs[di,j+1].fill_between(lag,*np.percentile(mo[:,:,j],[25,75],axis=0),alpha=.15)
  axs[di,3].scatter(ob[:,50,0],mo[:,50,0]);lim=max(np.max(abs(ob[:,50,0])),np.max(abs(mo[:,50,0])),1);axs[di,3].plot([-lim,lim],[-lim,lim],'k--');axs[di,3].set(xlabel='Flight bank change at +1s (deg)',ylabel='Model bank change (deg)')
 for j,label in enumerate(['Signed differential change (mm)','Signed bank response (deg)','Signed roll-rate response (deg/s)','Per-event comparison']):axs[di,j].set_title(f'{day}: {len(traces)} events\n{label}');axs[di,j].grid(alpha=.2)
 for j in range(3):axs[di,j].axvline(0,color='gray',lw=.5);axs[di,j].set_xlabel('Seconds from detected change')
 axs[di,1].legend(fontsize=8)
fig.suptitle('Isolated brake-command changes: sign aligned, median and interquartile spread; closed-loop observations, not causal step tests');fig.tight_layout();fig.savefig(R/'brake-events.png',dpi=160);(R/'brake-events.json').write_text(json.dumps(dict(selection='30s ordinary atlas. Differential change >5mm; stable short before/after plateaus; throttle range <0.05; events spaced5s. Times align to 0.4s detection window and may lag true onset. Effective brake includes fitted slack. No causal inference.',events=events),indent=2));print('Events:',{d:sum(e['day']==d for e in events) for d in ['2026-01-03','2026-02-15']})
