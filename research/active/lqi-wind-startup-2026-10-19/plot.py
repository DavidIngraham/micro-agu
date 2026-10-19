import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
P=Path(__file__).resolve().parent
fig,axes=plt.subplots(3,2,figsize=(11,9),layout='constrained')
for col,case in enumerate(['natural-head','delayed-cross']):
 for strategy,color in [('gate','#547ba5'),('zero','#c07127'),('weighted','#399578')]:
  name=f'{case}-{strategy}'+('-73' if col==0 else '')
  name=('smooth-'+name) if strategy=='weighted' else name
  d=json.loads((P/'runs'/f'{name}.json').read_text());s=d['samples'];t=np.array([x['elapsed'] for x in s]);y=np.array([x['state'] for x in s]);att=Rotation.from_quat(y[:,3:7]).as_euler('xyz',degrees=True);mask=(t>=5.8)&(t<35)
  axes[0,col].plot(t[mask],-y[mask,2],color=color,label=strategy)
  axes[1,col].plot(t[mask],att[mask,0],color=color)
  axes[2,col].plot(t[mask],np.array([x['active'] for x in s])[mask],color=color,label=strategy)
 for row in range(3):axes[row,col].set(xlabel='Time since inflation start (s)')
 axes[0,col].set(ylabel='Height (m)',title='Natural headwind 1.5 m/s' if col==0 else 'Crosswind 1.5 m/s; wind withheld 12 s');axes[0,col].legend()
 axes[1,col].set(ylabel='True roll (deg)')
 axes[2,col].set(ylabel='LQI active',yticks=[0,1])
fig.savefig(P/'startup-comparison.png',dpi=160)
fig,axes=plt.subplots(2,1,figsize=(10,6),layout='constrained')
for name,color in [('correct','#399578'),('reversed','#c07127'),('rotated','#aa5070'),('too-strong','#547ba5')]:
 d=json.loads((P/'runs'/f'smooth-prior-{name}.json').read_text());s=d['samples'];t=np.array([x['elapsed'] for x in s]);y=np.array([x['state'] for x in s]);att=Rotation.from_quat(y[:,3:7]).as_euler('xyz',degrees=True);mask=(t>=5.8)&(t<35)
 axes[0].plot(t[mask],att[mask,0],color=color,label=name);axes[1].plot(t[mask],np.array([x['startup'].get('TKWQ',1) for x in s])[mask],color=color,label=name)
axes[0].set(ylabel='True roll (deg)',title='Weighted startup: correct and incorrect wind priors');axes[0].legend()
axes[1].set(ylabel='Heuristic confidence',xlabel='Time since inflation start (s)')
fig.savefig(P/'wind-priors.png',dpi=160)

fig,axes=plt.subplots(3,1,figsize=(10,7),layout='constrained')
for strategy,color in [('zero','#c07127'),('weighted','#399578')]:
 d=json.loads((P/'runs'/f'late-valid-{strategy}.json').read_text());s=d['samples'];t=np.array([x['elapsed'] for x in s]);mask=(t>=47)&(t<=57)
 for ax,key in [(axes[0],'TKWN'),(axes[1],'TKWQ')]:
  ax.plot(t[mask],np.array([x['startup'].get(key,0) for x in s])[mask],color=color,label=strategy)
 axes[2].plot(t[mask],np.array([x['active'] for x in s])[mask],color=color,label=strategy)
axes[0].set(title='EKF wind made available just before assumption timeout',ylabel='Controller north wind (m/s)')
axes[1].set(ylabel='Heuristic confidence')
axes[2].set(ylabel='LQI active',yticks=[0,1],xlabel='Time since inflation start (s)')
for ax in axes:ax.legend()
fig.savefig(P/'late-handover.png',dpi=160)
