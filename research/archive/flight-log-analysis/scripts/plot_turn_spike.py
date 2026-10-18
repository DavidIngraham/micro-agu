from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
O=Path('/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/longitudinal-2026-10-09/turn-model-spike')
z=dict(np.load(O/'2026-01-03.npz'));t=z['t'];m=(t>=430)&(t<=465);tt=t[m];fig,ax=plt.subplots(4,2,figsize=(13,11),layout='constrained')
for a,keys,lab in [(ax[0,0],['roll'],'Payload bank (deg)'),(ax[1,0],['yaw'],'Unwrapped heading (deg)'),(ax[2,0],['throttle_pwm'],'Throttle fraction'),(ax[3,0],['left','right'],'Brake travel fraction'),(ax[1,1],['h'],'Height above home (m)'),(ax[2,1],['climb'],'Climb (m/s)'),(ax[3,1],['spd'],'Groundspeed (m/s)')]:
 for k in keys:a.plot(tt,z[k][m],label=k)
 a.set_ylabel(lab);a.axvspan(447.16,452.20,color='green',alpha=.13);a.legend();a.grid(alpha=.2);a.set_xlabel('Boot seconds')
ax[0,1].plot(z['PE'][m],z['PN'][m]);ax[0,1].scatter(z['PE'][m][0],z['PN'][m][0],label='Start');ax[0,1].axis('equal');ax[0,1].set_xlabel('East (m)');ax[0,1].set_ylabel('North (m)');ax[0,1].grid(alpha=.2)
fig.suptitle('Hood River: high-bank full-turn candidate, MANUAL');fig.savefig(O/'hood-river-high-bank.png',dpi=145)
# Course rotation confirms actual trajectory curvature independently of yaw.
for day,lo,hi in [('2026-01-03',447.16,452.20),('2026-01-03',508.16,515.08),('2026-02-15',3387.37,3392.77)]:
 z=dict(np.load(O/(day+'.npz')));m=(z['t']>=lo)&(z['t']<=hi);course=np.rad2deg(np.unwrap(np.arctan2(z['VE'][m],z['VN'][m])));print(day,lo,'course delta',course[-1]-course[0],'speed median',np.median(z['spd'][m]),'bank',np.median(z['roll'][m]))
