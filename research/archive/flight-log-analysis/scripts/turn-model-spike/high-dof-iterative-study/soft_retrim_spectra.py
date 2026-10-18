from pathlib import Path
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.signal import welch,resample_poly
from scipy.spatial.transform import Rotation as Rot
R=Path(__file__).parent;O=R.parent;A=O.parent;fig,axs=plt.subplots(2,3,figsize=(14,8));rows=[]
for di,(day,folder) in enumerate([('2026-01-03','2026-01-03 15-41-42'),('2026-02-15','2026-02-15 15-38-50')]):
 imu=[r for r in json.loads((A/folder/'IMU.json').read_text()) if r.get('I',0)==0];ti=np.array([r['TimeUS']/1e6 for r in imu]);g=np.array([[r[k] for k in ['GyrX','GyrY','GyrZ']] for r in imu]);spectra={k:[] for k in ['Flight','Retained','Soft + retrim']}
 for path in sorted(R.glob(f'atlas-{day}-*-combined_small.npz')):
  start=int(path.name.split('-')[-2]);grid=np.arange(start,start+30,.02);native=np.column_stack([np.interp(grid,ti,g[:,j]) for j in range(3)]);native=resample_poly(native,1,2,axis=0)[125:];f,p=welch(np.rad2deg(native),fs=25,nperseg=250,axis=0);spectra['Flight'].append(p)
  for name,file in [('Soft + retrim',R/f'soft_v3_retrim-{day}-{start}.npz'),('Retained',R/(f'preload_6.0-{day}-{start}.npz' if di==0 else f'candidate_v2-{day}-{start}.npz'))]:
   y=np.load(file)['state'];body=Rot.from_quat(y[:,3:7]).inv().apply(y[:,10:13]);_,ps=welch(np.rad2deg(body[125:750]),fs=25,nperseg=250,axis=0);spectra[name].append(ps)
  for name in spectra:
   ps=spectra[name][-1];bands={label:np.sqrt(np.trapz(ps[(f>=lo)&(f<=hi)],f[(f>=lo)&(f<=hi)],axis=0)).tolist() for label,lo,hi in [('low_mode',1.1,2.1),('high_mode',2.7,4.7)]};rows.append(dict(day=day,start=start,name=name,**bands))
 for name,color in [('Flight','black'),('Retained','#197d9e'),('Soft + retrim','#d88735')]:
  ps=np.median(spectra[name],axis=0)
  for j in range(3):axs[di,j].semilogy(f,ps[:,j],label=name,color=color,lw=1.5)
 for j,label in enumerate(['Roll','Pitch','Yaw']):axs[di,j].set(xlim=(.1,5),xlabel='Frequency (Hz)',ylabel='Rate PSD ((deg/s)²/Hz)',title=f'{day}: {label}');axs[di,j].grid(alpha=.2)
axs[0,0].legend();fig.suptitle('A softer joint improves roll participation but does not recover the observed roll spectrum');fig.tight_layout();fig.savefig(R/'soft-retrim-spectra.png',dpi=150);(R/'soft-retrim-spectra.json').write_text(json.dumps(rows,indent=2))
