from pathlib import Path
import json,numpy as np
from scipy.signal import butter,sosfiltfilt
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).parent;O=R.parent;A=O.parent;fig,ax=plt.subplots(2,3,figsize=(15,8))
for di,(day,folder,start) in enumerate([('2026-01-03','2026-01-03 15-41-42',480),('2026-02-15','2026-02-15 15-38-50',2790)]):
 raw=json.loads((A/folder/'IMU.json').read_text());raw=[r for r in raw if start-5<=r['TimeUS']/1e6<start+20 and r.get('I',0)==0];ti=np.array([r['TimeUS']/1e6 for r in raw]);q=np.rad2deg(np.array([[r[k] for k in ['GyrY','GyrZ']] for r in raw]));grid=np.arange(start-4,start+19,.02);v=np.column_stack([np.interp(grid,ti,q[:,j]) for j in range(2)]);mask=(grid>=start)&(grid<start+4)
 for j,label in enumerate(['Pitch rate','Yaw rate']):ax[di,0].plot(grid[mask]-start,v[mask,j],label=label)
 for col,(lo,hi,name) in enumerate([(1.3,1.9,'Low ridge'),(3.0,4.5,'High ridge')],1):
  b=sosfiltfilt(butter(3,[lo,hi],fs=50,btype='bandpass',output='sos'),v,axis=0);ax[di,col].plot(b[mask,0],b[mask,1],alpha=.7);ax[di,col].set(xlabel='Bandpassed pitch rate (deg/s)',ylabel='Bandpassed yaw rate (deg/s)',title=f'{day}: {name} {lo}–{hi} Hz');ax[di,col].grid(alpha=.25)
 ax[di,0].set(title=f'{day}: native gyro, {start}s',xlabel='Time (s)',ylabel='Body rate (deg/s)');ax[di,0].legend();ax[di,0].grid(alpha=.25)
fig.suptitle('The measured pitch/yaw motion traces a loop: spectral bands are visualization, not independent modes');fig.tight_layout();fig.savefig(R/'observed-pitch-yaw-loops.png',dpi=150)
