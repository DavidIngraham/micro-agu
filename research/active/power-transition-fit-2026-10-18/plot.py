import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
P=Path(__file__).resolve().parent;tr=np.load(P/'traces.npz');base=Path('/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/longitudinal-2026-10-09/turn-model-spike')
a=np.load(base/'2026-02-15-causal-input.npz');fig,axes=plt.subplots(2,1,figsize=(11,6),layout='constrained')
for ax,start,end,cut in zip(axes,[3350,3443],[3380,3465],[3363.13,3456.45]):
 i=np.argmin(abs(a['t']-start));n=round((end-start)/.04)+1;t=a['t'][i:i+n]
 ax.plot(t,np.rad2deg(a['Y'][i:i+n,5]),color='black',lw=1,label='Measured yaw rate')
 for label in ['baseline','constant']:ax.plot(t,tr[f'trout_lake-{start}-{label}'][:,2],lw=1,label=label)
 ax.axvline(cut,color='gray',ls='--',label='Throttle cut');ax.set(xlabel='Log time (s)',ylabel='Yaw rate (deg/s)',title='Fit transition' if start==3350 else 'Transfer transition: no refitting');ax.legend(fontsize=8,ncol=4)
fig.savefig(P/'transitions.png',dpi=160)
