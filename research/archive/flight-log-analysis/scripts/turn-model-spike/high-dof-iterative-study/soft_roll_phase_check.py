from pathlib import Path
import sys,json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.signal import welch,csd,coherence,resample_poly
from scipy.spatial.transform import Rotation as Rot
R=Path(__file__).parent;O=R.parent;A=O.parent;sys.path.insert(0,str(R/'candidate-v3'));from simulator import Simulator
from equilibrium import solve,perturb
rows=[];summary=[];fig,axs=plt.subplots(2,3,figsize=(14,8))
for di,(day,folder) in enumerate([('2026-01-03','2026-01-03 15-41-42'),('2026-02-15','2026-02-15 15-38-50')]):
 raw=[r for r in json.loads((A/folder/'IMU.json').read_text()) if r.get('I',0)==0];ti=np.array([r['TimeUS']/1e6 for r in raw]);g=np.array([[r[k] for k in ['GyrX','GyrY','GyrZ']] for r in raw])
 for path in sorted(R.glob(f'atlas-{day}-*-combined_small.npz')):
  start=int(path.name.split('-')[-2]);grid=np.arange(start,start+30,.02);observed=resample_poly(np.column_stack([np.interp(grid,ti,g[:,j]) for j in range(3)]),1,2,axis=0)[125:];signals={'Flight':observed}
  for name,stem in [('Nominal','candidate_v3'),('Soft/retrim','soft_v3_retrim')]:
   y=np.load(R/f'{stem}-{day}-{start}.npz')['state'];signals[name]=Rot.from_quat(y[125:750,3:7]).inv().apply(y[125:750,10:13])
  for name,b in signals.items():
   f,p=welch(b,fs=25,nperseg=250,axis=0);ids=np.where((f>=2.7)&(f<=4.7))[0];k=ids[np.argmax(np.sqrt(p[ids,1]*p[ids,2]))];kr=ids[np.argmax(p[ids,0])];row=dict(day=day,start=start,name=name,high_pitch_yaw_peak_hz=float(f[k]),high_roll_peak_hz=float(f[kr]))
   for label,i,j in [('roll_pitch',0,1),('roll_yaw',0,2),('pitch_yaw',1,2)]:
    _,s=csd(b[:,i],b[:,j],fs=25,nperseg=250);_,co=coherence(b[:,i],b[:,j],fs=25,nperseg=250);row[label+'_phase_deg']=float(np.angle(s[kr],deg=True));row[label+'_coherence']=float(co[kr])
   rows.append(row)
 for name,color in [('Nominal','#197d9e'),('Soft/retrim','#d88735')]:
  rr=[r for r in rows if r['day']==day and r['name']==name];oo=[next(o for o in rows if o['day']==day and o['name']=='Flight' and o['start']==r['start']) for r in rr];axs[di,0].scatter([o['high_roll_peak_hz'] for o in oo],[r['high_roll_peak_hz'] for r in rr],label=name,c=color,alpha=.7)
  q=dict(day=day,name=name,windows=len(rr),roll_peak_mae_hz=float(np.mean([abs(r['high_roll_peak_hz']-o['high_roll_peak_hz']) for r,o in zip(rr,oo)])),pitch_yaw_peak_mae_hz=float(np.mean([abs(r['high_pitch_yaw_peak_hz']-o['high_pitch_yaw_peak_hz']) for r,o in zip(rr,oo)])))
  for j,label in enumerate(['roll_pitch','roll_yaw']):
   pairs=[(r,o) for r,o in zip(rr,oo) if min(r[label+'_coherence'],o[label+'_coherence'])>.5];delta=[float((r[label+'_phase_deg']-o[label+'_phase_deg']+180)%360-180) for r,o in pairs];q[label]=dict(coherent_windows=len(pairs),median_abs_phase_error_deg=float(np.median(abs(np.array(delta)))));axs[di,j+1].scatter([o[label+'_phase_deg'] for r,o in pairs],[r[label+'_phase_deg'] for r,o in pairs],label=name,c=color,alpha=.7)
  summary.append(q);print(q,flush=True)
 axs[di,0].plot([2.7,4.7],[2.7,4.7],'k--',lw=.8);axs[di,0].set(xlabel='Observed high-band roll peak (Hz)',ylabel='Predicted high-band roll peak (Hz)',title=day)
 for j,label in enumerate(['Roll→pitch phase','Roll→yaw phase']):axs[di,j+1].plot([-180,180],[-180,180],'k--',lw=.8);axs[di,j+1].set(xlabel='Observed '+label+' (deg)',ylabel='Predicted phase (deg)',title=day,xlim=(-180,180),ylim=(-180,180))
 for ax in axs[di]:ax.grid(alpha=.2)
axs[0,0].legend();fig.suptitle('High-band structure: phase at each signal’s roll spectral ridge; coherence >0.5 for both');fig.tight_layout();fig.savefig(R/'soft-roll-ridge-phase-check.png',dpi=150);(R/'soft-roll-ridge-phase-check.json').write_text(json.dumps(dict(summary=summary,windows=rows,method='5-second initial transient removed; Welch250 samples/25Hz, 50% overlap (~4 segments). Phase evaluated at each signal high-band roll ridge; driven spectral comparison, not independent modal identification.'),indent=2))
