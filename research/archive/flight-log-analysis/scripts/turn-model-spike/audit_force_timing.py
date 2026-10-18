from pathlib import Path
import json,numpy as np
from scipy.ndimage import gaussian_filter1d
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
O=Path(__file__).parent;R=O.parent
fig,axes=plt.subplots(2,2,figsize=(13,8));out=[]
for row,(day,name,window) in enumerate([('2026-01-03','2026-01-03 15-41-42',(440,458)),('2026-02-15','2026-02-15 15-38-50',(3383,3398))]):
 a=np.load(O/(day+'-physical-input.npz'));t=a['t'];y=a['Y'];dt=np.median(np.diff(t))
 raw=json.loads((R/name/'IMU.json').read_text());raw=[r for r in raw if r.get('I')==0];rt=np.array([r['TimeUS']/1e6 for r in raw]);acc=np.column_stack([np.interp(t,rt,[r[k] for r in raw]) for k in ['AccX','AccY','AccZ']])
 ph,th,ps=y[:,:3].T;sp,cp=np.sin(ph),np.cos(ph);st,ct=np.sin(th),np.cos(th);ss,cs=np.sin(ps),np.cos(ps)
 rot=np.zeros((len(t),3,3));rot[:,0,:]=np.column_stack([ct*cs,sp*st*cs-cp*ss,cp*st*cs+sp*ss]);rot[:,1,:]=np.column_stack([ct*ss,sp*st*ss+cp*cs,cp*st*ss-sp*cs]);rot[:,2,:]=np.column_stack([-st,sp*ct,cp*ct])
 force=np.einsum('nij,nj->ni',rot,acc)
 force=gaussian_filter1d(force,5,axis=0);derived=np.gradient(gaussian_filter1d(y[:,6:9],5,axis=0),dt,axis=0);derived[:,2]-=9.80665
 meta=np.load(O/(day+'.npz'));valid=meta['airborne'].astype(bool)&np.all(np.isfinite(force+derived),axis=1)
 # Exclude all samples near raw telemetry gaps and flight-mask transitions.
 for k in np.where(np.diff(rt)>.3)[0]:valid&=~((t>rt[k]-1)&(t<rt[k+1]+1))
 valid&=np.convolve(valid.astype(int),np.ones(51,dtype=int),mode='same')==51
 lags=np.arange(-.4,.401,.02);errors=[]
 for lag in lags:
  shifted=np.column_stack([np.interp(t+lag,t,force[:,k]) for k in range(3)]);errors.append(np.sqrt(np.mean((shifted[valid]-derived[valid])**2)))
 best=int(np.argmin(errors));sel=(t>=window[0])&(t<=window[1]);ax=axes[row,0]
 for k,c in enumerate(['tab:blue','tab:orange','tab:green']):
  ax.plot(t[sel]-window[0],derived[sel,k],color=c,label=['N','E','D'][k]+' velocity derivative');ax.plot(t[sel]-window[0],force[sel,k],color=c,ls='--',alpha=.8,label=['N','E','D'][k]+' rotated IMU')
 ax.set(title=day+' high-bank turn',xlabel='Seconds from '+str(window[0])+' s',ylabel='Specific force (m/s²)');ax.grid(alpha=.2)
 axes[row,1].plot(lags,errors);axes[row,1].axvline(0,color='gray',ls=':');axes[row,1].set(title='Whole-flight lag diagnostic',xlabel='IMU sample-time shift (s)',ylabel='3-axis component RMS difference (m/s²)');axes[row,1].grid(alpha=.2)
 out.append(dict(day=day,valid_samples=int(valid.sum()),best_shift_s=float(lags[best]),best_rms_m_s2=float(errors[best]),zero_shift_rms_m_s2=float(errors[20]),raw_gaps=[dict(start_s=float(rt[k]),end_s=float(rt[k+1])) for k in np.where(np.diff(rt)>.3)[0]],caveat='Centered 0.2 s Gaussian smoothing; diagnostic only, not causal validation. EKF velocity and IMU are not independent sensors.'))
axes[0,0].legend(fontsize=7,ncol=2);fig.suptitle('Checking force reconstruction before changing the aerodynamics');fig.tight_layout();fig.savefig(O/'iteration-force-timing.png',dpi=160)
(O/'force-timing-audit.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
