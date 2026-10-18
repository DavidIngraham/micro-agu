from pathlib import Path
import json,numpy as np
from scipy.signal import csd,resample_poly
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).parent;O=R.parent;A=O.parent;obs=json.loads((R/'rpm-mode-evidence.json').read_text());rows=[]
for day,folder in [('2026-01-03','2026-01-03 15-41-42'),('2026-02-15','2026-02-15 15-38-50')]:
 raw=json.loads((A/folder/'IMU.json').read_text());raw=[r for r in raw if r.get('I',0)==0];ti=np.array([r['TimeUS']/1e6 for r in raw]);gy=np.array([[r[k] for k in ['GyrX','GyrY','GyrZ']] for r in raw])
 for o in [r for r in obs if r['day']==day]:
  start=o['start'];grid=np.arange(start,start+30,.02);g=resample_poly(np.column_stack([np.interp(grid,ti,gy[:,j]) for j in range(3)]),1,2,axis=0);spectra=np.zeros((251,3,3),complex)
  for i in range(3):
   for j in range(3):f,spectra[:,i,j]=csd(g[:,j],g[:,i],fs=25,nperseg=500)
  for branch in ['low','high']:
   k=np.argmin(abs(f-o[branch+'_hz']));e,v=np.linalg.eigh(spectra[k]);vec=v[:,-1];vec*=np.exp(-1j*np.angle(vec[1]));rows.append(dict(day=day,start=start,branch=branch,hz=float(f[k]),amplitude=abs(vec).tolist(),phase_relative_pitch=np.angle(vec,deg=True).tolist(),rank1_fraction=float(e[-1]/sum(e)),first_second_ratio=float(e[-1]/max(e[-2],1e-20)),roll_pitch_coherence=float(abs(spectra[k,0,1])**2/(spectra[k,0,0].real*spectra[k,1,1].real)),roll_yaw_coherence=float(abs(spectra[k,0,2])**2/(spectra[k,0,0].real*spectra[k,2,2].real))))
fig,ax=plt.subplots(2,2,figsize=(12,8))
for di,day in enumerate(['2026-01-03','2026-02-15']):
 for j,branch in enumerate(['low','high']):
  a=np.array([r['amplitude'] for r in rows if r['day']==day and r['branch']==branch]);ax[di,j].boxplot(a,labels=['Roll','Pitch','Yaw']);ax[di,j].set(title=f'{day}: {branch} spectral ridge',ylabel='Normalized body-rate amplitude');ax[di,j].grid(alpha=.25)
fig.suptitle('Three-axis spectral mode shape: checking that the fitted branch is the observed motion');fig.tight_layout();fig.savefig(R/'observed-mode-shapes.png',dpi=150);(R/'observed-mode-shapes.json').write_text(json.dumps(rows,indent=2));print([(d,b,np.median([r['amplitude'] for r in rows if r['day']==d and r['branch']==b],axis=0).tolist()) for d in ['2026-01-03','2026-02-15'] for b in ['low','high']])

