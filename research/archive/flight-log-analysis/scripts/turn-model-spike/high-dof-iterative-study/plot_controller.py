from pathlib import Path
import numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).parent
fig,ax=plt.subplots(4,2,figsize=(14,11),sharex=True)
for j,(flight,slack) in enumerate([('hood_river','12.15'),('trout_lake','0.00')]):
 for ff,color in [('fixed','C0'),('envelope','C1')]:
  p=R/f'controller-{flight}-{slack}-{ff}-payload.npz'
  if not p.exists():continue
  d=np.load(p);t=d['t'];a=d['data'];ax[0,j].plot(t,a[:,2],color=color,label=ff,alpha=.7);ax[1,j].plot(t,a[:,3],color=color);ax[2,j].plot(t,a[:,4],color=color);ax[3,j].plot(t,a[:,7],color=color)
 ax[0,j].plot(t,a[:,0],'k--',label='requested');ax[1,j].plot(t,a[:,0],'k--');ax[3,j].plot(t,a[:,1],'k--');ax[0,j].set_title(f'{flight}, effective left offset {slack} mm');ax[0,j].legend();ax[3,j].set_xlabel('seconds')
for i,label in enumerate(['Payload heading rate (deg/s)','Air-course rate (deg/s)','Signed brake fraction','Climb (m/s)']):
 for j in range(2):ax[i,j].set_ylabel(label);ax[i,j].grid(alpha=.25)
fig.suptitle('Archived turn gains on the higher-DOF candidate: closed-loop diagnostic');fig.tight_layout();fig.savefig(R/'controller-comparison.png',dpi=140)
