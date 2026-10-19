import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
OUT=Path(__file__).resolve().parent
rows=json.loads((OUT/'checks.json').read_text())
base=Path('/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/longitudinal-2026-10-09/turn-model-spike')
a=np.load(base/'2026-02-15-causal-input.npz')
fig,axes=plt.subplots(2,1,figsize=(11,6),layout='constrained')
for ax,start,duration in zip(axes,[3366,3459],[14,6]):
 i=np.argmin(abs(a['t']-start));n=round(duration/.04)+1;t=np.arange(n)*.04
 ax.plot(t,np.rad2deg(a['Y'][i:i+n,5]),color='black',lw=1,label='Measured yaw rate')
 for label,kind,title in [('k1.0-c1.0-j1.0','recorded32','Baseline, recorded brakes'),('k1.0-c0.05-j1.3','recorded32','Low damping, recorded brakes'),('k1.0-c0.05-j1.3','constant_brakes','Low damping, constant brakes')]:
  r=next(x for x in rows if x['name']==f'trout_lake-{start}' and x['variant']==label and x['kind']==kind)
  ax.plot(t,np.array(r['trace'])[:,2],lw=1,label=title)
 ax.axvspan(0,2,color='gray',alpha=.15)
 ax.set(xlabel='Seconds from window start',ylabel='Yaw rate (deg/s)',title=f'Trout Lake, log time {start} s; shaded initialization excluded from metrics')
 axes[0].legend(fontsize=8,ncol=2)
fig.savefig(OUT/'replay.png',dpi=170)
