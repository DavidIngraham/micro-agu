from pathlib import Path
import json,numpy as np
from scipy.signal import welch,coherence,csd,resample_poly
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).parent;O=R.parent;A=O.parent;fig,axs=plt.subplots(2,3,figsize=(14,8));results=[]
for di,(day,folder) in enumerate([('2026-01-03','2026-01-03 15-41-42'),('2026-02-15','2026-02-15 15-38-50')]):
 imu=json.loads((A/folder/'IMU.json').read_text());imu=[r for r in imu if r.get('I',0)==0];ti=np.array([r['TimeUS']/1e6 for r in imu]);gyro=np.array([[r[k] for k in ['GyrX','GyrY','GyrZ']] for r in imu]);rc=json.loads((A/folder/'RCOU.json').read_text());tr=np.array([r['TimeUS']/1e6 for r in rc]);commands=np.array([[(r['C1']-1000)/1000,(r['C3']-800)*np.pi*55/360*.1,(2400-r['C4'])*np.pi*55/360*.1] for r in rc]);allcoh=[];allps=[];allcross=[]
 for path in sorted(R.glob(f'atlas-{day}-*-combined_small.npz')):
  start=int(path.name.split('-')[-2]);grid=np.arange(start,start+30,.02);g=np.column_stack([np.interp(grid,ti,gyro[:,j]) for j in range(3)]);u=commands[np.clip(np.searchsorted(tr,grid,side='right')-1,0,len(tr)-1)];g=resample_poly(g,1,2,axis=0);u=resample_poly(u,1,2,axis=0);diff=u[:,2]-u[:,1];f,pg=welch(np.rad2deg(g),fs=25,nperseg=250,axis=0);_,pu=welch(np.c_[diff,u[:,0]],fs=25,nperseg=250,axis=0);_,c=coherence(diff,g[:,2],fs=25,nperseg=250);_,ct=coherence(u[:,0],g[:,1],fs=25,nperseg=250);_,cyz=coherence(g[:,1],g[:,2],fs=25,nperseg=250);_,s=csd(g[:,1],g[:,2],fs=25,nperseg=250);band=(f>=1.3)&(f<=1.9);k=np.where(band)[0][np.argmax(pg[band,2])];row=dict(day=day,start=start,brake_diff_std_mm=float(np.std(diff)),throttle_std=float(np.std(u[:,0])),yaw_peak_hz=float(f[k]),brake_yaw_coherence_at_peak=float(c[k]),throttle_pitch_coherence_at_peak=float(ct[k]),pitch_yaw_coherence_at_peak=float(cyz[k]),pitch_yaw_phase_deg=float(np.angle(s[k],deg=True)),yaw_psd_at_peak=float(pg[k,2]));results.append(row);allcoh.append(np.c_[c,ct,cyz]);allps.append(np.c_[pg[:,2],pu]);allcross.append(s)
 for j,label in enumerate(['Brake / yaw coherence','Throttle / pitch coherence','Pitch / yaw coherence']):axs[di,0].plot(f,np.median(np.array(allcoh)[:,:,j],axis=0),label=label)
 for j,label in enumerate(['Yaw PSD (deg/s)^2/Hz','Differential brake PSD mm^2/Hz','Throttle PSD fraction^2/Hz']):axs[di,1].semilogy(f,np.median(np.array(allps)[:,:,j],axis=0),label=label)
 quiet=sorted([r for r in results if r['day']==day],key=lambda r:r['brake_diff_std_mm'])[:max(3,len(allps)//4)];axs[di,2].scatter([r['brake_diff_std_mm'] for r in results if r['day']==day],[r['yaw_psd_at_peak'] for r in results if r['day']==day]);axs[di,2].set(xlabel='Differential brake standard deviation (mm)',ylabel='Yaw peak PSD at 1.3–1.9 Hz');axs[di,2].set_yscale('log')
 for j in range(2):axs[di,j].set(xlim=(.1,4),xlabel='Frequency (Hz)');axs[di,j].legend(fontsize=7)
 for j in range(3):axs[di,j].set_title(day);axs[di,j].grid(alpha=.25)
 print(day,'quietest',quiet[:3],flush=True)
fig.suptitle('Native IMU/input spectral consistency; filtered 50→25Hz, closed-loop association only');fig.tight_layout();fig.savefig(R/'input-motion-spectra.png',dpi=150);(R/'input-motion-spectra.json').write_text(json.dumps(results,indent=2))
