from pathlib import Path
import json,numpy as np
from scipy.signal import welch
from scipy.spatial.transform import Rotation as Rot
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).parent;O=R.parent;A=O.parent
fig,ax=plt.subplots(2,3,figsize=(14,8));result=[]
for di,(day,folder,start) in enumerate([('2026-01-03','2026-01-03 15-41-42',480),('2026-02-15','2026-02-15 15-38-50',2790)]):
 raw=json.loads((A/folder/'IMU.json').read_text());raw=[r for r in raw if start<=r['TimeUS']/1e6<start+30 and r.get('I',0)==0];ti=np.array([r['TimeUS']/1e6 for r in raw]);unique=np.r_[True,np.diff(ti)>0];ti=ti[unique];gyro=np.array([[r[k] for k in ['GyrX','GyrY','GyrZ']] for r in raw])[unique];dt=np.median(np.diff(ti));grid=np.arange(ti[0],ti[-1],dt);native=np.column_stack([np.interp(grid,ti,gyro[:,i]) for i in range(3)])
 att=json.loads((A/folder/'ATT.json').read_text());att=[r for r in att if start<=r['TimeUS']/1e6<start+30];ta=np.array([r['TimeUS']/1e6 for r in att]);rots=Rot.from_euler('xyz',np.array([[r[k] for k in ['Roll','Pitch','Yaw']] for r in att]),degrees=True);delta=np.diff(ta);rates=(rots[:-1].inv()*rots[1:]).as_rotvec()/delta[:,None];at=.5*(ta[1:]+ta[:-1]);adt=np.median(delta);ag=np.arange(at[0],at[-1],adt);ar=np.column_stack([np.interp(ag,at,rates[:,i]) for i in range(3)])
 d=np.load(R/f'atlas-{day}-{start}-combined_small.npz');y=d['state'];model=Rot.from_quat(y[:,3:7]).inv().apply(y[:,10:13]);datasets=[(native,1/dt,'native IMU','k'),(ar,1/adt,'ATT increments','C2'),(d['truth'][:,3:6],25,'25Hz held IMU','C0'),(model,25,'candidate','C1')]
 peaks={}
 for values,fs,label,color in datasets:
  f,p=welch(np.rad2deg(values),fs=fs,nperseg=min(len(values),round(fs*10)),axis=0);mask=(f>=.7)&(f<=4);peaks[label]=f[mask][np.argmax(p[mask],axis=0)].tolist()
  for j in range(3):ax[di,j].loglog(f[1:],p[1:,j],label=label,color=color,alpha=.8)
 result.append(dict(day=day,start=start,native_imu_dt_s=float(dt),native_att_dt_s=float(adt),imu_gap_max=float(np.diff(ti).max()),peaks_07_4Hz=peaks))
 for j,name in enumerate(['Roll','Pitch','Yaw']):ax[di,j].set(title=f'{day} {start}s: {name}',xlabel='Hz',ylabel='PSD ((deg/s)^2/Hz)',xlim=(.1,10));ax[di,j].grid(alpha=.25)
ax[0,0].legend();fig.tight_layout();fig.savefig(R/'native-rate-spectra.png',dpi=150);(R/'native-rate-spectra.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
