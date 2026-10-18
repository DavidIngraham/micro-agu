from pathlib import Path
import json,numpy as np
from scipy.signal import welch
from scipy.spatial.transform import Rotation as Rot
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).parent;rows=[];fig,ax=plt.subplots(2,3,figsize=(14,8),sharex=True)
for di,day in enumerate(['2026-01-03','2026-02-15']):
 obs=[];pred=[]
 for p in sorted(R.glob(f'atlas-{day}-*-combined_small.npz')):
  d=np.load(p);y=d['state'];actual=np.rad2deg(d['truth'][:,3:6]);model=np.rad2deg(Rot.from_quat(y[:,3:7]).inv().apply(y[:,10:13]));sl=slice(125,None);f,a=welch(actual[sl],fs=25,nperseg=256,axis=0);_,b=welch(model[sl],fs=25,nperseg=256,axis=0);obs.append(a);pred.append(b)
  bands={}
  for lo,hi in [(.1,.5),(.5,1.5),(1.5,4),(4,10)]:
   mask=(f>=lo)&(f<hi);bands[f'{lo}-{hi}Hz']=dict(flight_rms=np.sqrt(np.trapz(a[mask],f[mask],axis=0)).tolist(),model_rms=np.sqrt(np.trapz(b[mask],f[mask],axis=0)).tolist())
  rows.append(dict(day=day,episode=p.name,bands=bands))
 for j,name in enumerate(['Roll rate','Pitch rate','Yaw rate']):
  for values,label,color in [(obs,'flight','k'),(pred,'candidate','C1')]:
   values=np.array(values)[:,:,j];med=np.median(values,axis=0);ax[di,j].loglog(f[1:],med[1:],label=label,color=color);ax[di,j].fill_between(f[1:],np.quantile(values,.25,axis=0)[1:],np.quantile(values,.75,axis=0)[1:],color=color,alpha=.12)
  ax[di,j].set(title=f'{day}: {name}',xlabel='Frequency (Hz)',ylabel='PSD ((deg/s)^2/Hz)',xlim=(.1,10));ax[di,j].grid(alpha=.25)
ax[0,0].legend();fig.suptitle('Ordinary replay angular-rate spectra: episode median and interquartile spread, first 5s omitted');fig.tight_layout();fig.savefig(R/'rate-spectra.png',dpi=150);(R/'rate-spectra.json').write_text(json.dumps(rows,indent=2))
