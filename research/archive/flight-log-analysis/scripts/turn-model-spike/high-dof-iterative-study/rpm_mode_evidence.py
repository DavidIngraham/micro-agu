from pathlib import Path
import json,numpy as np
from scipy.signal import welch,resample_poly,csd,coherence
from scipy.stats import spearmanr
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).parent;O=R.parent;A=O.parent;rows=[];fig,ax=plt.subplots(2,2,figsize=(12,9))
for di,(day,folder) in enumerate([('2026-01-03','2026-01-03 15-41-42'),('2026-02-15','2026-02-15 15-38-50')]):
 raw=json.loads((A/folder/'IMU.json').read_text());raw=[r for r in raw if r.get('I',0)==0];ti=np.array([r['TimeUS']/1e6 for r in raw]);gy=np.array([[r[k] for k in ['GyrY','GyrZ']] for r in raw]);prop=np.load(O/f'{day}-propulsion-9x45.npz')
 for path in sorted(R.glob(f'atlas-{day}-*-combined_small.npz')):
  start=int(path.name.split('-')[-2]);grid=np.arange(start,start+30,.02);g=resample_poly(np.column_stack([np.interp(grid,ti,gy[:,j]) for j in range(2)]),1,2,axis=0);f,p=welch(g,fs=25,nperseg=500,axis=0);_,cross=csd(g[:,0],g[:,1],fs=25,nperseg=500);_,coh=coherence(g[:,0],g[:,1],fs=25,nperseg=500);rpm=float(np.mean(np.interp(grid,prop['t'],prop['rpm'])));row=dict(day=day,start=start,rpm=rpm)
  for name,lo,hi in [('low',1.1,2.1),('high',2.7,4.7)]:
   ids=np.where((f>=lo)&(f<=hi))[0];k=ids[np.argmax(np.sqrt(p[ids,0]*p[ids,1]))];row[name+'_hz']=float(f[k]);row[name+'_phase']=float(np.angle(cross[k],deg=True));row[name+'_coherence']=float(coh[k])
  rows.append(row)
 d=[r for r in rows if r['day']==day]
 for j,name in enumerate(['low','high']):
  sc=ax[di,j].scatter([r['rpm'] for r in d],[r[name+'_hz'] for r in d],c=[r[name+'_phase'] for r in d],cmap='coolwarm',vmin=-180,vmax=180);ax[di,j].set(title=f'{day}: {name} pitch/yaw peak',xlabel='Mean logged RPM',ylabel='Frequency (Hz)');ax[di,j].grid(alpha=.25);fig.colorbar(sc,ax=ax[di,j],label='Pitch→yaw cross-phase (deg)')
 print(day,{n:spearmanr([r['rpm'] for r in d],[r[n+'_hz'] for r in d]).statistic for n in ['low','high']},flush=True)
fig.tight_layout();fig.savefig(R/'rpm-mode-evidence.png',dpi=150);(R/'rpm-mode-evidence.json').write_text(json.dumps(rows,indent=2))
