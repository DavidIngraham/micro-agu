from pathlib import Path
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.signal import welch,resample_poly
from scipy.spatial.transform import Rotation as Rot
R=Path(__file__).parent;O=R.parent;A=O.parent;fig,axs=plt.subplots(2,3,figsize=(14,8));rows=[]
for di,(day,folder) in enumerate([('2026-01-03','2026-01-03 15-41-42'),('2026-02-15','2026-02-15 15-38-50')]):
 imu=[r for r in json.loads((A/folder/'IMU.json').read_text()) if r.get('I',0)==0];ti=np.array([r['TimeUS']/1e6 for r in imu]);g=np.array([[r[k] for k in ['GyrX','GyrY','GyrZ']] for r in imu]);spectra={k:[] for k in ['Flight','Aero correction','Modal + rigging']}
 for path in sorted(R.glob(f'atlas-{day}-*-combined_small.npz')):
  start=int(path.name.split('-')[-2]);grid=np.arange(start,start+30,.02);native=np.column_stack([np.interp(grid,ti,g[:,j]) for j in range(3)]);native=resample_poly(native,1,2,axis=0)[125:];f,p=welch(np.rad2deg(native),fs=25,nperseg=250,axis=0);spectra['Flight'].append(p)
  for name,file in [('Aero correction',path),('Modal + rigging',R/f'candidate_v3-{day}-{start}.npz')]:
   d=np.load(file);y=d['state'];body=Rot.from_quat(y[:,3:7]).inv().apply(y[:,10:13]);_,ps=welch(np.rad2deg(body[125:750]),fs=25,nperseg=250,axis=0);spectra[name].append(ps)
  for name in spectra:
   ps=spectra[name][-1];bands={label:np.sqrt(np.trapz(ps[(f>=lo)&(f<=hi)],f[(f>=lo)&(f<=hi)],axis=0)).tolist() for label,lo,hi in [('slow',.1,1),('low_mode',1.1,2.1),('high_mode',2.7,4.7)]};rows.append(dict(day=day,start=start,name=name,**bands))
 for name,color in [('Flight','black'),('Aero correction','#8c8c8c'),('Modal + rigging','#197d9e')]:
  ps=np.median(spectra[name],axis=0)
  for j in range(3):axs[di,j].semilogy(f,ps[:,j],label=name,color=color,lw=1.5)
 for j,label in enumerate(['Roll','Pitch','Yaw']):axs[di,j].set(xlim=(.1,5),xlabel='Frequency (Hz)',ylabel='Rate PSD ((deg/s)²/Hz)',title=f'{day}: {label}');axs[di,j].grid(alpha=.2)
axs[0,0].legend();fig.suptitle('Median motion spectra across ordinary replay windows; first five seconds excluded');fig.tight_layout();fig.savefig(R/'final-rate-spectra.png',dpi=150);(R/'final-rate-spectra.json').write_text(json.dumps(rows,indent=2))
for day in ['2026-01-03','2026-02-15']:
 for name in spectra:
  a=[r for r in rows if r['day']==day and r['name']==name];print(day,name,{band:np.median([r[band] for r in a],axis=0).tolist() for band in ['slow','low_mode','high_mode']},flush=True)
