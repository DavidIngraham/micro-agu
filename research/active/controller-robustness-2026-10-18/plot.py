import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
P=Path(__file__).resolve().parent;rows=json.loads((P/'metrics.json').read_text());fig,axes=plt.subplots(2,2,figsize=(10,7),layout='constrained')
for column,phase in enumerate([-1,1]):
 for scheme,color in [('baseline','#547ba5'),('lqi','#c07127')]:
  rr=[next(r for r in rows if r['id']==f'force-{i}-{scheme}') for i in ([0,2,4,6] if phase==-1 else [1,3,5,7])]
  f=[r['spec']['experiment']['hz'] for r in rr]
  for axis,ref in [(0,8),(1,20)]:
   axes[axis,column].plot(f,[r['harmonic_rms'][axis+1]/ref for r in rr],'-o',color=color,label='Classical' if scheme=='baseline' else 'LQI')
   axes[axis,column].axhline(1,color='gray',ls=':',lw=.7);axes[axis,column].set(xlabel='Forcing frequency (Hz)',ylabel=('Pitch' if axis==0 else 'Yaw')+' RMS / fixed-control RMS',title=('Pitch lags yaw 90°' if phase==-1 else 'Pitch leads yaw 90°'));axes[axis,column].legend()
fig.savefig(P/'forcing-response.png',dpi=170)
fig,axes=plt.subplots(2,1,figsize=(11,6),layout='constrained')
from scipy.spatial.transform import Rotation
for scheme,color in [('baseline','#547ba5'),('lqi','#c07127')]:
 d=json.loads((P/'runs'/f'force-2-{scheme}.json').read_text());ss=d['samples'];t=np.array([s['time'] for s in ss]);y=np.array([s['state'] for s in ss]);r=Rotation.from_quat(y[:,3:7]).inv().apply(y[:,10:13])*180/np.pi
 axes[0].plot(t,r[:,1],color=color,lw=.8,label=scheme);axes[1].plot(t,-y[:,2]-np.array([s['targetAlt'] for s in ss]),color=color,lw=.8,label=scheme)
for ax in axes:ax.axvspan(10,50,color='gray',alpha=.12);ax.set(xlabel='Campaign time (s)');ax.legend()
axes[0].set(ylabel='Pitch rate (deg/s)',title='Observed-phase 1.6 Hz torque forcing; shaded interval is forcing')
axes[1].set(ylabel='Altitude error (m)');fig.savefig(P/'observed-phase.png',dpi=170)

fig,axes=plt.subplots(2,2,figsize=(11,7),layout='constrained')
for column,case in enumerate(['boundary--0.18','mode-2.1-0.3']):
 for scheme,color in [('baseline','#547ba5'),('lqi','#c07127')]:
  d=json.loads((P/'runs'/f'{case}-{scheme}.json').read_text());ss=d['samples'];t=np.array([s['time'] for s in ss]);y=np.array([s['state'] for s in ss]);r=Rotation.from_quat(y[:,3:7]).inv().apply(y[:,10:13])*180/np.pi
  axes[0,column].plot(t,r[:,2],color=color,lw=.7,label=scheme)
  axes[1,column].plot(t,np.array([s['brakes'][1]-s['brakes'][0] for s in ss]),color=color,lw=.7,label=scheme)
  if not d['complete']:axes[0,column].axvline(t[-1],color=color,ls='--',label='LQI stopped by guard')
 for row in range(2):axes[row,column].set(xlabel='Campaign time (s)',ylabel='Yaw rate (deg/s)' if row==0 else 'Differential brake (mm)');axes[row,column].legend(fontsize=8)
 axes[0,column].set_title('1.6 Hz, gain -0.18; fixed-control unstable' if column==0 else '2.1 Hz, gain +0.30; fixed-control settles')
fig.savefig(P/'responsive-mode.png',dpi=170)
