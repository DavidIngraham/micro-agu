import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
P=Path(__file__).resolve().parent
fig,axes=plt.subplots(3,2,figsize=(11,9),layout='constrained')
for col,case in enumerate(['calm','crosswind']):
 for variant,color in [('deployed','#547ba5'),('candidate','#c07127')]:
  d=json.loads((P/'runs'/f'{variant}-{case}.json').read_text());s=d['samples'];t=np.array([x['elapsed'] for x in s]);y=np.array([x['state'] for x in s]);mask=(t>=5.8)&(t<=25);att=Rotation.from_quat(y[:,3:7]).as_euler('xyz',degrees=True)
  axes[0,col].plot(t[mask],-y[mask,2],color=color,label='Classical takeoff' if variant=='deployed' else 'LQI takeoff')
  axes[1,col].plot(t[mask],att[mask,0],color=color)
  axes[2,col].plot(t[mask],[x['active'] for x,m in zip(s,mask) if m],color=color)
 for row in range(3):axes[row,col].set(xlabel='Time since inflation start (s)')
 axes[0,col].set(title=case,ylabel='Height (m)');axes[0,col].legend()
 axes[1,col].set(ylabel='True roll (deg)')
 axes[2,col].set(ylabel='LQI active',yticks=[0,1])
fig.savefig(P/'takeoff-comparison.png',dpi=160)
